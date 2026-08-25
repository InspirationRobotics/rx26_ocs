"""bench_stage1 — Gate G3 Part A, start to finish, on one machine.

    .venv/Scripts/python bench_stage1.py

Runs the whole start-of-run sequence against a broker in this process, with a
stand-in RoboCommand and a stand-in USV, and checks every criterion in the G3
Part A matrix. No docker, no mosquitto, no network, no boat.

WHY THIS EXISTS ALONGSIDE RoboNation's stub. Theirs is the authority on the wire
format and you must pass against it before you believe anything. But it needs
docker, it is another machine's worth of setup, and it cannot be run from a unit
test. This can run on the laptop of whoever is standing on the dock. Use both:
this one to know the bridge's own logic is sound, theirs to know we agree with
RoboNation about what the bytes mean.

The broker is amqtt, in-process. Note the plugin config below -- amqtt 0.12
refuses every connection unless anonymous auth is declared explicitly, and the
refusal surfaces as a paho client that simply never connects.
"""
from __future__ import annotations

import asyncio
import math
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

import paho.mqtt.client as mqtt

sys.path.insert(0, str(Path(__file__).resolve().parent))

from rx_bridge.bridge import Bridge                                  # noqa: E402
from rx_bridge.config import load                                    # noqa: E402
from rx_bridge.proto import common_pb2, rx_commands_pb2              # noqa: E402
from rx_bridge.proto import rx_common_pb2, rx_course_pb2             # noqa: E402
from rx_bridge.proto import rx_reports_pb2, rx_requests_pb2          # noqa: E402
from rx_bridge.runstate import RunState                              # noqa: E402

HOST, PORT = "127.0.0.1", 11883
TEAM, VEHICLE = "ASTA", "USV1"

_results: list[tuple[str, bool, str]] = []


def check(tag: str, ok: bool, detail: str = "") -> bool:
    _results.append((tag, ok, detail))
    print("  %-4s %s  %s" % (tag, "PASS" if ok else "FAIL", detail), flush=True)
    return ok


def settle(seconds: float = 1.2) -> None:
    time.sleep(seconds)


# --------------------------------------------------------------------------
# the broker
# --------------------------------------------------------------------------

def start_broker() -> None:
    from amqtt.broker import Broker

    cfg = {
        "listeners": {"default": {"type": "tcp", "bind": "%s:%d" % (HOST, PORT),
                                  "max_connections": 50}},
        "sys_interval": 0,
        # amqtt 0.12 rejects everything without this, silently from the client's side.
        "plugins": {
            "amqtt.plugins.authentication.AnonymousAuthPlugin": {"allow_anonymous": True},
        },
    }
    ready = threading.Event()

    def run() -> None:
        async def main() -> None:
            broker = Broker(cfg)          # must be constructed inside the loop
            await broker.start()
            ready.set()
            await asyncio.Event().wait()
        asyncio.run(main())

    threading.Thread(target=run, daemon=True).start()
    if not ready.wait(20):
        raise SystemExit("broker never started")
    settle(0.6)


# --------------------------------------------------------------------------
# a stand-in RoboCommand
# --------------------------------------------------------------------------

class StubRoboCommand:
    """Publishes the course, receives our traffic, issues RunStart."""

    def __init__(self) -> None:
        self.requests: list[rx_requests_pb2.RxRequest] = []
        self.reports: list[rx_reports_pb2.RxReport] = []
        self.seq = 0
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="stub-rc")
        self.c.on_message = self._on_message
        self.c.connect(HOST, PORT, 15)
        self.c.loop_start()
        settle(0.8)
        self.c.subscribe([("robocommand/robotx/%s/request" % TEAM, 1),
                          ("robocommand/robotx/%s/+/report" % TEAM, 0)])
        settle(0.5)

    def _on_message(self, _c, _u, msg) -> None:
        if msg.topic.endswith("/request"):
            r = rx_requests_pb2.RxRequest()
            r.ParseFromString(msg.payload)
            self.requests.append(r)
        else:
            r = rx_reports_pb2.RxReport()
            r.ParseFromString(msg.payload)
            self.reports.append(r)

    def publish_course(self) -> None:
        course = rx_course_pb2.RxCourse()
        course.course_id = "ALPHA"
        course.pinger_freq_hz = 25000
        for lat, lon in ((27.3350, -82.5325), (27.3350, -82.5290),
                         (27.3378, -82.5290), (27.3378, -82.5325)):
            course.corners.add(latitude=lat, longitude=lon)
        course.sent_at.FromNanoseconds(time.time_ns())
        self.c.publish("robocommand/robotx/course", course.SerializeToString(),
                       qos=1, retain=True)

    def send_run_start(self, declaration_seq: int, run_id: int) -> None:
        self.seq += 1
        cmd = rx_commands_pb2.RxCommand()
        cmd.team_id = TEAM
        cmd.seq = self.seq
        cmd.sent_at.FromNanoseconds(time.time_ns())
        cmd.run_start.declaration_seq = declaration_seq
        cmd.run_start.run_id = run_id
        self.c.publish("robocommand/robotx/%s/command" % TEAM,
                       cmd.SerializeToString(), qos=1)

    def heartbeats(self) -> list[rx_reports_pb2.RxReport]:
        return [r for r in self.reports if r.WhichOneof("body") == "heartbeat"]


# --------------------------------------------------------------------------
# what the vehicles hear
# --------------------------------------------------------------------------

class CommandTap:
    """Everything the OCS sends onto the team command namespace.

    This is what a vehicle's reporter would subscribe to, split out into its
    own client for one reason: it must be connected and subscribed BEFORE the
    retained RxCourse is published. amqtt hands a newly-subscribed client that
    retained message even when the topic does not match, and that delivery
    leaves the subscription dead -- so a tap created later silently receives
    nothing. Mosquitto does not do this; the ordering costs nothing and makes
    the bench independent of which broker it is run against.
    """

    def __init__(self) -> None:
        self.commands: list[rx_commands_pb2.RxCommand] = []
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="cmd-tap")
        self.c.on_message = self._on_message
        self.c.connect(HOST, PORT, 15)
        self.c.loop_start()
        settle(0.8)
        self.c.subscribe("team/robotx/%s/+/command" % TEAM, 0)
        settle(0.5)

    def _on_message(self, _c, _u, msg) -> None:
        if not msg.topic.endswith("/command"):
            return
        cmd = rx_commands_pb2.RxCommand()
        try:
            cmd.ParseFromString(msg.payload)
        except Exception:  # noqa: BLE001
            return
        self.commands.append((msg.topic, cmd))

    def run_starts(self):
        return [(t, c) for t, c in self.commands
                if c.WhichOneof("body") == "run_start"]


# --------------------------------------------------------------------------
# a stand-in USV
# --------------------------------------------------------------------------

class FakeUSV:
    """2 Hz heartbeats onto the team namespace, with the two fault modes."""

    def __init__(self) -> None:
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake-usv")
        self.c.connect(HOST, PORT, 15)
        self.c.loop_start()
        settle(0.8)
        self.topic = "team/robotx/%s/%s/report" % (TEAM, VEHICLE)
        self._stop = threading.Event()
        self.sent = 0
        self.nan = False
        self.unknown_task = False
        self._t0 = time.time()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self) -> None:
        while not self._stop.is_set():
            self.c.publish(self.topic, self._frame(), qos=0)
            self.sent += 1
            self._stop.wait(0.5)          # 2 Hz, as the handbook requires

    def _frame(self) -> bytes:
        theta = (time.time() - self._t0) * 0.05
        r = rx_reports_pb2.RxReport()
        r.team_id, r.vehicle_id = TEAM, VEHICLE
        r.sent_at.FromNanoseconds(time.time_ns())
        hb = r.heartbeat
        hb.state = common_pb2.RobotState.Value("STATE_AUTO")
        hb.position.latitude = 27.3364 + 0.0004 * math.sin(theta)
        hb.position.longitude = -82.5307 + 0.0004 * math.cos(theta)
        hb.spd_mps = 1.4
        hb.heading_deg = float("nan") if self.nan else (math.degrees(theta) % 360.0)
        hb.roll_deg, hb.pitch_deg = 3.0 * math.sin(theta * 7), 1.5 * math.sin(theta * 5)
        hb.altitude_hae_m, hb.depth_m = -24.6, 0.0
        hb.vehicle_type = rx_common_pb2.VehicleType.TYPE_USV
        if not self.unknown_task:
            hb.current_task = rx_common_pb2.RxTask.TASK_NONE
        return r.SerializeToString()

    def stop(self) -> None:
        self._stop.set()
        self.c.loop_stop()
        self.c.disconnect()


# --------------------------------------------------------------------------

CONFIG = """
team_id = "{team}"
uav_geofence = []

[[vehicle]]
id = "{vehicle}"
type = "TYPE_USV"

[team]
host = "{host}"
port = {port}

[robocommand]
host = "{host}"
port = {port}

[limits]
# 1, not the handbook's 2: this rig is one vehicle by design, and it
# exercises the sequence mechanics rather than producing a PoR
# submission. The real two-vehicle declaration is bench_stage3.
min_declared = 1

[tiers]
task1 = "TIER_CORE"
task2 = "TIER_NONE"
task3 = "TIER_NONE"
task4 = "TIER_NONE"

[paths]
seq_store = "{seq}"
wire_log = "{wire}"
"""


def run_start_bytes(declaration_seq: int, run_id: int) -> bytes:
    """A RunStart as RoboCommand would send it, for the re-declaration checks."""
    cmd = rx_commands_pb2.RxCommand()
    cmd.seq = 900 + run_id % 100
    cmd.run_start.declaration_seq = declaration_seq
    cmd.run_start.run_id = run_id
    return cmd.SerializeToString()


def main() -> int:
    work = Path(tempfile.mkdtemp(prefix="g3-stage1-"))
    cfg_path = work / "bridge.toml"
    cfg_path.write_text(
        CONFIG.format(team=TEAM, vehicle=VEHICLE, host=HOST, port=PORT,
                      seq=(work / "seq.json").as_posix(),
                      wire=(work / "wire").as_posix()),
        encoding="utf-8",
    )

    print("G3 Part A -- one machine, no docker, no boat")
    print("workspace: %s\n" % work)

    start_broker()
    print("broker up on %s:%d\n" % (HOST, PORT))

    tap = CommandTap()          # before the retained course; see CommandTap
    rc = StubRoboCommand()
    cfg = load(cfg_path)
    logs: list[str] = []
    bridge = Bridge(cfg, log=lambda m: logs.append(str(m)))
    bridge.start()
    settle(2.0)

    print("A1-A4  connect, course, ordering")
    check("A1", bridge.machine.state is RunState.CONNECTED,
          "state=%s" % bridge.machine.state.name)
    check("A3", not bridge.declare().startswith("declared"),
          "declare refused before the course arrives")

    rc.publish_course()
    settle(1.5)
    check("A2", bridge.machine.state is RunState.COURSE_RX,
          "state=%s course=%s" % (bridge.machine.state.name,
                                  getattr(bridge.machine.course, "course_id", "?")))

    usv = FakeUSV()
    settle(2.0)
    check("A4", bridge.counters.forwarded == 0,
          "forwarded=%d while COURSE_RX (silence before declaring is correct)"
          % bridge.counters.forwarded)

    print("\nA5-A7  declare, then heartbeats at the mandated rate")
    msg = bridge.declare()
    settle(1.2)
    check("A5", bridge.machine.state is RunState.DECLARED and len(rc.requests) == 1,
          "%s; RoboCommand received %d request(s)" % (msg, len(rc.requests)))

    if rc.requests:
        d = rc.requests[0].run_declaration
        check("A5b", list(d.vehicle_ids) == [VEHICLE]
              and d.task1_tier == common_pb2.TaskTier.Value("TIER_CORE"),
              "vehicles=%s task1=%s" % (list(d.vehicle_ids),
                                        common_pb2.TaskTier.Name(d.task1_tier)))

    before = bridge.counters.forwarded
    n0 = len(rc.heartbeats())
    settle(6.0)
    fwd = bridge.counters.forwarded - before
    got = len(rc.heartbeats()) - n0
    check("A6", fwd > 0 and got > 0, "forwarded=%d, RoboCommand saw %d" % (fwd, got))
    check("A7", 10 <= fwd <= 14, "%d frames in 6 s (expect ~12 at 2 Hz)" % fwd)

    print("\nA8-A9  the faults that cannot be provoked on a bench")
    usv.nan = True
    # Drain before snapshotting. At the instant the fault is switched on a
    # CLEAN frame may already be in flight -- published, queued, or partway
    # through the hot path -- and it lands after the snapshot, reading as a
    # bad frame that got through. One heartbeat interval of settle removes
    # the race without weakening the check: zero clean frames may be
    # forwarded across the three seconds that follow.
    settle(1.0)
    inv0, fwd0 = bridge.counters.dropped_invalid, bridge.counters.forwarded
    settle(3.0)
    check("A8", bridge.counters.dropped_invalid > inv0
          and bridge.counters.forwarded == fwd0,
          "NaN heading: %d dropped, %d forwarded"
          % (bridge.counters.dropped_invalid - inv0,
             bridge.counters.forwarded - fwd0))
    usv.nan = False

    usv.unknown_task = True
    settle(1.0)                     # same race as A8; see the note there
    inv1, fwd1 = bridge.counters.dropped_invalid, bridge.counters.forwarded
    settle(3.0)
    check("A9", bridge.counters.dropped_invalid > inv1
          and bridge.counters.forwarded == fwd1,
          "TASK_UNKNOWN: %d dropped, %d forwarded"
          % (bridge.counters.dropped_invalid - inv1,
             bridge.counters.forwarded - fwd1))
    usv.unknown_task = False
    settle(1.5)

    print("\nA10-A11  RunStart, and refusing one that is not ours")
    ours = bridge.machine.declaration_seq or 1
    rc.send_run_start(ours + 99, 999)
    settle(1.5)
    check("A11", bridge.machine.state is RunState.DECLARED
          and bridge.machine.run_id is None,
          "mismatched declaration_seq refused; state=%s" % bridge.machine.state.name)
    check("A11b", tap.run_starts() == [],
          "a RunStart answering someone else's declaration is NOT relayed "
          "(%d seen by the vehicle)" % len(tap.run_starts()))

    rc.send_run_start(ours, 42)
    settle(1.5)
    check("A10", bridge.machine.state is RunState.RUNNING
          and bridge.machine.run_id == 42,
          "state=%s run_id=%s" % (bridge.machine.state.name, bridge.machine.run_id))

    # The whole point of the OCS from the vehicle's side. The fleet is
    # position-holding in autonomous mode and cannot reach RoboCommand; if this
    # relay does not happen, nothing on the water ever learns the run began.
    relayed = tap.run_starts()
    # The counter is reported alongside the receipt on purpose: it separates
    # "the OCS never published" from "the vehicle never received", which are
    # different faults with different fixes and look identical from the
    # vehicle's end.
    check("A10b", len(relayed) == 1
          and relayed[0][0] == "team/robotx/%s/%s/command" % (TEAM, VEHICLE)
          and relayed[0][1].run_start.run_id == 42
          and relayed[0][1].run_start.declaration_seq == ours,
          "RunStart relayed to the vehicle: %d received (bridge routed %d), "
          "run_id=%s decl_seq=%s"
          % (len(relayed), bridge.counters.commands_routed,
             relayed[0][1].run_start.run_id if relayed else None,
             relayed[0][1].run_start.declaration_seq if relayed else None))

    print("\nA12-A13  durability")
    seq_before = bridge.seqs.snapshot()["reports"].get(VEHICLE, 0)
    state_before = bridge.machine.state
    decl_before = bridge.machine.declaration_seq
    wire = bridge.wire.path
    bridge.stop()
    settle(1.0)

    reborn = Bridge(cfg, log=lambda m: logs.append("reborn: " + str(m)))
    reborn.start()
    settle(3.0)
    seq_after = reborn.seqs.snapshot()["reports"].get(VEHICLE, 0)
    check("A12a", seq_after >= seq_before,
          "report seq %d -> %d across a process restart (must not go backwards)"
          % (seq_before, seq_after))
    check("A12b", reborn.machine.state is state_before
          and reborn.machine.declaration_seq == decl_before,
          "run state after restart: %s (declaration_seq=%s), was %s (%s)"
          % (reborn.machine.state.name, reborn.machine.declaration_seq,
             state_before.name, decl_before))

    print("\nA14  a second run in the same timeslot")
    # End, declare again, and prove the second declaration is distinguishable
    # from the first. The unit tests cover RunMachine and SeqStore separately;
    # this is the one that proves declare() wires them together correctly.
    first_seq = reborn.machine.declaration_seq
    reborn.end()
    n_req = len(rc.requests)
    again = reborn.declare()
    settle(1.5)
    second_seq = reborn.machine.declaration_seq
    check("A14a", reborn.machine.state is RunState.DECLARED
          and len(rc.requests) == n_req + 1,
          "%s -- RoboCommand received a second declaration" % again)
    check("A14b", second_seq is not None and second_seq != first_seq,
          "declaration_seq %s -> %s: the second run does NOT reuse the first's "
          "sequence number" % (first_seq, second_seq))
    # The teeth: a RunStart answering run one must not start run two.
    reborn._handle_command(run_start_bytes(first_seq, 999))
    settle(0.8)
    check("A14c", reborn.machine.state is RunState.DECLARED
          and reborn.machine.run_id != 999,
          "a stale RunStart for run 1 did NOT start run 2 (state=%s run_id=%s)"
          % (reborn.machine.state.name, reborn.machine.run_id))
    reborn._handle_command(run_start_bytes(second_seq, 1000))
    settle(0.8)
    check("A14d", reborn.machine.state is RunState.RUNNING
          and reborn.machine.run_id == 1000,
          "the matching RunStart started run 2: state=%s run_id=%s"
          % (reborn.machine.state.name, reborn.machine.run_id))

    lines = wire.read_text(encoding="utf-8").strip().splitlines() if wire.exists() else []
    check("A13", len(lines) > 10 and '"b64"' in "".join(lines[:40]),
          "%d frames in %s" % (len(lines), wire.name))

    usv.stop()
    reborn.stop()

    passed = sum(1 for _, ok, _ in _results if ok)
    print("\n%d/%d checks passed" % (passed, len(_results)))
    failed = [t for t, ok, _ in _results if not ok]
    if failed:
        print("FAILED: %s" % ", ".join(failed))
    # os._exit skips atexit AND the stdio flush. Piped, stdout is block-
    # buffered, so without this the summary line is simply lost -- which
    # is exactly the line CI and TESTING.md tell you to look for.
    sys.stdout.flush()
    os._exit(0 if not failed else 1)


if __name__ == "__main__":
    main()
