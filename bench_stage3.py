"""bench_stage3 — the Communications Proof of Readiness rehearsal, on one machine.

    .venv/Scripts/python bench_stage3.py

Handbook 3.1 asks for one thing: rc-test logs showing a 3.4.10 run start
sequence with AT LEAST TWO VEHICLES in the run_declaration. Stage 1 proves the
sequence with one vehicle over MQTT; stage 2 proves the TCP link. This proves
the shape the submission actually needs -- two vehicles connected, a third
configured but absent, and a declaration that names exactly the two that are
really there.

WHY THE THIRD VEHICLE MATTERS. RoboCommand publishes RunStart only once EVERY
declared vehicle reports STATE_AUTO, with no timeout and no error. UUV1 has no
reporter. Declare it and the run never starts, and the symptom is
indistinguishable from a broken link to their server -- which is the wrong thing
to be debugging with a timeslot running down. The active set is what stops that,
so this bench checks that it does.

Set RX26_ASV if the vehicle repo is not a sibling of this one.
"""
from __future__ import annotations

import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from bench_stage1 import HOST, PORT, TEAM                              # noqa: E402
from bench_stage1 import StubRoboCommand, check, settle, start_broker  # noqa: E402
from bench_stage1 import run_start_bytes                               # noqa: E402
from bench_stage2 import _PKG                                          # noqa: E402
from rx_bridge.bridge import Bridge                                    # noqa: E402
from rx_bridge.config import load                                      # noqa: E402
from rx_bridge.runstate import RunState                                # noqa: E402

from crusader_groundstation.ocs_link import OcsLink, fake_report       # noqa: E402

TCP_PORT = 37598          # its own port; never collide with a real OCS

CONFIG = """
team_id = "{team}"
uav_geofence = [
    [1.28010, 103.85520],
    [1.28010, 103.85625],
    [1.28110, 103.85625],
    [1.28010, 103.85520],
]

[[vehicle]]
id = "USV1"
type = "TYPE_USV"

[[vehicle]]
id = "UUV1"
type = "TYPE_UUV"

[[vehicle]]
id = "UAV1"
type = "TYPE_UAV"

[team]
host = "{host}"
port = {port}

[tcp]
host = "127.0.0.1"
port = {tcp}

[robocommand]
host = "{host}"
port = {port}

[tiers]
task1 = "TIER_CORE"

[limits]
# The real floor, unlike stages 1 and 2: this bench exists to rehearse the
# submission, and the submission requires two.
min_declared = 2
rate = 5.0
heartbeat_reserve = 2.0
stale_ms = 1000.0

[paths]
seq_store = "{seq}"
wire_log  = "{wire}"
"""


class Vehicle:
    """One vehicle's client, streaming at the mandated 2 Hz."""

    def __init__(self, vid: str, vtype: str, logs: list) -> None:
        self.vid = vid
        self.directives: list = []
        self.link = OcsLink("127.0.0.1", TCP_PORT,
                            on_command=self._on_msg,
                            log=lambda m, v=vid: logs.append("%s: %s" % (v, m)))
        self.link.start()
        self._stop = threading.Event()
        self._t0 = time.time()
        self._type = vtype
        threading.Thread(target=self._run, daemon=True).start()

    def _on_msg(self, msg):
        if "ocs_directive" in msg:
            self.directives.append(msg["ocs_directive"])

    def _run(self):
        while not self._stop.is_set():
            r = fake_report(self.vid, TEAM, self._t0)
            r["heartbeat"]["vehicle_type"] = self._type
            if self._type == "TYPE_UAV":
                # A UAV may leave nothing UNKNOWN; the OCS refuses it otherwise.
                r["heartbeat"]["flight_phase"] = "FLIGHT_PHASE_GROUNDED"
            self.link.publish(r)
            self._stop.wait(0.5)

    def stop(self):
        self._stop.set()
        self.link.stop()


def main() -> int:
    work = Path(tempfile.mkdtemp(prefix="por-stage3-"))
    cfg_path = work / "bridge.toml"
    cfg_path.write_text(
        CONFIG.format(team=TEAM, host=HOST, port=PORT, tcp=TCP_PORT,
                      seq=(work / "seq.json").as_posix(),
                      wire=(work / "wire").as_posix()),
        encoding="utf-8")

    print("PoR rehearsal -- 3.4.10 with two vehicles, no docker, no boat")
    print("boat client: %s" % (_PKG / "crusader_groundstation" / "ocs_link.py"))
    print("workspace:   %s\n" % work)

    start_broker()
    rc = StubRoboCommand()
    cfg = load(cfg_path)
    logs: list[str] = []
    bridge = Bridge(cfg, log=lambda m: logs.append(str(m)))
    bridge.start()
    settle(2.0)

    print("C1  3.4.10 steps 1-3: connect, subscribe, receive and validate the course")
    rc.publish_course()
    settle(1.5)
    check("C1a", bridge.machine.state is RunState.COURSE_RX,
          "state=%s after the retained course" % bridge.machine.state.name)
    check("C1b", bridge.course_findings == [],
          "course validated clean: %s" % (bridge.course_findings or "no findings"))

    print("\nC2  only the vehicles that are really there")
    check("C2a", bridge.declare().startswith("cannot declare"),
          "declare refused with nothing connected")
    usv = Vehicle("USV1", "TYPE_USV", logs)
    settle(2.0)
    check("C2b", bridge.active_ids() == ("USV1",),
          "one vehicle connected -> active=%s" % (bridge.active_ids(),))
    msg = bridge.declare()
    check("C2c", "at least 2" in msg,
          "one vehicle is refused, naming the handbook floor: %r" % msg[:70])

    uav = Vehicle("UAV1", "TYPE_UAV", logs)
    settle(2.0)
    check("C2d", bridge.active_ids() == ("USV1", "UAV1"),
          "both connected -> active=%s (UUV1 absent, never declared)"
          % (bridge.active_ids(),))

    print("\nC3  3.4.10 step 4: the declaration names exactly the active two")
    n_req = len(rc.requests)
    msg = bridge.declare()
    settle(1.5)
    check("C3a", bridge.machine.state is RunState.DECLARED
          and len(rc.requests) == n_req + 1, msg)
    declared = list(rc.requests[-1].run_declaration.vehicle_ids) if rc.requests else []
    check("C3b", declared == ["USV1", "UAV1"],
          "RoboCommand received vehicle_ids=%s" % declared)
    check("C3c", len(declared) >= 2,
          "%d vehicles -- handbook 3.1 requires at least 2" % len(declared))
    check("C3d", "UUV1" not in declared,
          "UUV1 has no reporter and is NOT declared -- this is what stops the "
          "run hanging forever waiting for it")

    print("\nC4  the standby-auto directive reaches the declared vehicles")
    settle(1.0)
    check("C4a", len(usv.directives) >= 1 and len(uav.directives) >= 1,
          "USV1 got %d, UAV1 got %d" % (len(usv.directives), len(uav.directives)))
    if usv.directives:
        d = usv.directives[0]
        check("C4b", d.get("action") == "standby_auto"
              and d.get("declaration_seq") == bridge.machine.declaration_seq,
              "action=%r declaration_seq=%s (ours is %s)"
              % (d.get("action"), d.get("declaration_seq"),
                 bridge.machine.declaration_seq))
        check("C4c", sorted(d.get("vehicles", [])) == ["UAV1", "USV1"],
              "it names the declared set: %s" % d.get("vehicles"))

    print("\nC5  3.4.10 steps 5-6: heartbeats, and every declared vehicle in AUTO")
    before = bridge.counters.forwarded
    settle(4.0)
    fwd = bridge.counters.forwarded - before
    check("C5a", fwd > 0 and len(rc.heartbeats()) > 0,
          "forwarded %d, RoboCommand saw %d" % (fwd, len(rc.heartbeats())))
    waiting = bridge.awaiting_auto()
    check("C5b", "USV1" not in waiting and "UAV1" not in waiting,
          "declared vehicles reporting STATE_AUTO; still waiting on %s "
          "(UUV1 is undeclared, so it does not hold the run)" % (waiting,))
    hz = {v: round(bridge.hb_rate.hz(v, time.time()), 2) for v in ("USV1", "UAV1")}
    check("C5c", all(1.5 <= v <= 2.5 for v in hz.values()),
          "published rate per vehicle: %s (the handbook mandates 2 Hz)" % hz)

    print("\nC6  3.4.10 steps 7-9: RunStart, matched, and the run_id recorded")
    seq = bridge.machine.declaration_seq
    bridge._handle_command(run_start_bytes(seq + 500, 4242))
    settle(0.8)
    check("C6a", bridge.machine.state is RunState.DECLARED,
          "a RunStart for somebody else's declaration is refused (state=%s)"
          % bridge.machine.state.name)
    bridge._handle_command(run_start_bytes(seq, 7))
    settle(0.8)
    check("C6b", bridge.machine.state is RunState.RUNNING
          and bridge.machine.run_id == 7,
          "RunStart matched declaration_seq=%s -> RUNNING run_id=%s"
          % (seq, bridge.machine.run_id))

    print("\nC7  the operator can override the active set")
    bridge.end()
    bridge.set_active("UUV1", True)
    check("C7a", bridge.active_ids() == ("USV1", "UUV1", "UAV1"),
          "UUV1 forced in -> %s" % (bridge.active_ids(),))
    bridge.set_active("UUV1", None)
    bridge.set_active("UAV1", False)
    check("C7b", bridge.active_ids() == ("USV1",),
          "UAV1 forced out despite reporting -> %s" % (bridge.active_ids(),))
    check("C7c", bridge.declare().startswith("cannot declare"),
          "and the floor still holds with one active")

    usv.stop()
    uav.stop()
    bridge.stop()

    import bench_stage1 as _s1
    passed = sum(1 for _, ok, _ in _s1._results if ok)
    total = len(_s1._results)
    print("\n%d/%d checks passed" % (passed, total))
    if passed != total:
        print("\n--- bridge log ---")
        for line in logs[-40:]:
            print("   " + line)
    sys.stdout.flush()
    return 0 if passed == total else 1


if __name__ == "__main__":
    raise SystemExit(main())
