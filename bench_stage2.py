"""bench_stage2 — the vehicle link, start to finish, on one machine.

    .venv/Scripts/python bench_stage2.py

Stage 1 proves the bridge's rules against a stand-in RoboCommand over MQTT. This
proves the other half: that a real vehicle, running the real client from the boat
repo, can reach the OCS over TCP and have its telemetry come out the far side.

IT IMPORTS THE BOAT'S OWN ocs_link.py rather than reimplementing a client here.
That is the entire point. The framing is duplicated across the two repos by
necessity -- a ROS package cannot import from the OCS -- so the one failure this
bench exists to catch is those two copies drifting apart. A test that spoke the
OCS's own framing back to itself would agree with itself perfectly while the boat
sent garbage.

Set RX26_ASV if the vehicle repo is not a sibling of this one.
"""
from __future__ import annotations

import os
import socket
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from bench_stage1 import StubRoboCommand, check, settle, start_broker  # noqa: E402
from bench_stage1 import HOST, PORT, TEAM                              # noqa: E402
from rx_bridge.bridge import Bridge                                    # noqa: E402
from rx_bridge.config import load                                      # noqa: E402
from rx_bridge.proto import rx_commands_pb2                            # noqa: E402
from rx_bridge.runstate import RunState                                # noqa: E402

TCP_PORT = 37599        # not 37564: never collide with a real OCS on this box

_DEFAULT_ASV = Path(__file__).resolve().parent.parent / "rx26_asv"
_ASV = Path(os.environ.get("RX26_ASV", _DEFAULT_ASV))
_PKG = _ASV / "crusader_groundstation"
if not (_PKG / "crusader_groundstation" / "ocs_link.py").is_file():
    raise SystemExit(
        "cannot find the boat's ocs_link.py under %s\n"
        "Set RX26_ASV to the rx26_asv checkout." % _PKG
    )
sys.path.insert(0, str(_PKG))

from crusader_groundstation.ocs_link import (      # noqa: E402
    FrameReader, OcsLink, encode, fake_report, json_safe, rfc3339,
)

CONFIG = """
team_id = "{team}"
uav_geofence = []

[[vehicle]]
id = "USV1"
type = "TYPE_USV"

[[vehicle]]
id = "UUV1"
type = "TYPE_UUV"

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
rate = 5.0
heartbeat_reserve = 2.0
stale_ms = 1000.0

[paths]
seq_store = "{seq}"
wire_log  = "{wire}"
"""


class RawClient:
    """A socket we can misuse deliberately -- glued writes, split writes.

    OcsLink will not produce those on demand (it sends one whole frame at a
    time, correctly), so the framing edge cases need a client that can be told
    to behave badly.
    """

    def __init__(self, port: int) -> None:
        self.s = socket.create_connection(("127.0.0.1", port), timeout=5.0)
        self.reader = FrameReader()
        self.got: list[bytes] = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        self.s.settimeout(0.2)
        while True:
            try:
                chunk = self.s.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not chunk:
                return
            try:
                self.got.extend(self.reader.feed(chunk))
            except ValueError:
                return

    def send_raw(self, blob: bytes) -> None:
        self.s.sendall(blob)

    def close(self) -> None:
        try:
            self.s.close()
        except OSError:
            pass


def report(vehicle: str, *, fault: str = "") -> bytes:
    """One heartbeat, JSON-encoded exactly the way the boat encodes it."""
    import json
    r = fake_report(vehicle, TEAM, time.time() - 3.0, fault)
    # fake_report belongs to the USV client, so it hardcodes TYPE_USV. Sent
    # as UUV1 that trips the mistype net BEFORE validation, and the fault
    # actually under test never gets looked at.
    r["heartbeat"]["vehicle_type"] = ("TYPE_UUV" if vehicle.startswith("UUV")
                                      else "TYPE_USV")
    return json.dumps(json_safe(r), allow_nan=False).encode("utf-8")


def main() -> int:
    work = Path(tempfile.mkdtemp(prefix="g3-stage2-"))
    cfg_path = work / "bridge.toml"
    cfg_path.write_text(
        CONFIG.format(team=TEAM, host=HOST, port=PORT, tcp=TCP_PORT,
                      seq=(work / "seq.json").as_posix(),
                      wire=(work / "wire").as_posix()),
        encoding="utf-8",
    )

    print("G3 Part B -- the vehicle link, no docker, no boat")
    print("boat client: %s" % (_PKG / "crusader_groundstation" / "ocs_link.py"))
    print("workspace:   %s\n" % work)

    start_broker()
    rc = StubRoboCommand()
    cfg = load(cfg_path)
    logs: list[str] = []
    bridge = Bridge(cfg, log=lambda m: logs.append(str(m)))
    bridge.start()
    settle(2.0)
    rc.publish_course()
    settle(1.5)

    print("B1-B2  the boat's own client connects and is identified")
    link = OcsLink("127.0.0.1", TCP_PORT, log=lambda m: logs.append("link: " + m))
    link.start()
    t0 = time.time()
    stop = threading.Event()

    def stream() -> None:
        while not stop.is_set():
            link.publish(fake_report("USV1", TEAM, t0))
            stop.wait(0.5)

    threading.Thread(target=stream, daemon=True).start()
    settle(2.0)
    check("B1", link.connected, "client reports connected")
    check("B2", "USV1" in bridge.tcp.peers(),
          "OCS identified the socket as USV1: peers=%s" % bridge.tcp.peers())

    print("\nB3-B4  telemetry reaches RoboCommand through the TCP path")
    check("B3", bridge.counters.forwarded == 0,
          "forwarded=%d before declaring (silence is correct)"
          % bridge.counters.forwarded)
    msg = bridge.declare()
    settle(0.5)
    before, n0 = bridge.counters.forwarded, len(rc.heartbeats())
    settle(5.0)
    fwd = bridge.counters.forwarded - before
    got = len(rc.heartbeats()) - n0
    check("B4", fwd > 0 and got > 0 and bridge.machine.state is RunState.DECLARED,
          "%s -- forwarded %d over TCP, RoboCommand saw %d" % (msg, fwd, got))
    check("B4b", 8 <= fwd <= 14, "%d frames in 5 s (expect ~10 at 2 Hz)" % fwd)
    check("B4c", bridge.counters.last_via.get("USV1") == "tcp",
          "the bridge records the transport: last_via=%r"
          % bridge.counters.last_via.get("USV1"))

    # Silence the 2 Hz stream. Everything below counts `forwarded` deltas,
    # and a background streamer racing them makes the numbers meaningless.
    # The link stays connected -- B9 onwards still needs it.
    stop.set()
    settle(1.0)

    print("\nB5-B6  the fault nets still fire on the TCP path")
    raw = RawClient(TCP_PORT)
    settle(0.5)
    inv0 = bridge.counters.dropped_invalid
    fwd0 = bridge.counters.forwarded
    raw.send_raw(encode(report("UUV1", fault="nan")))
    settle(1.0)
    check("B5", bridge.counters.dropped_invalid == inv0 + 1,
          "NaN heading refused: invalid %d -> %d"
          % (inv0, bridge.counters.dropped_invalid))
    inv1 = bridge.counters.dropped_invalid
    raw.send_raw(encode(report("UUV1", fault="unknown_task")))
    settle(1.0)
    check("B6", bridge.counters.dropped_invalid == inv1 + 1,
          "TASK_UNKNOWN refused: invalid %d -> %d"
          % (inv1, bridge.counters.dropped_invalid))

    print("\nB7-B8  framing: glued and split writes")
    fwd1 = bridge.counters.forwarded
    raw.send_raw(encode(report("UUV1")) + encode(report("UUV1")))
    settle(1.2)
    check("B7", bridge.counters.forwarded == fwd1 + 2,
          "two frames in ONE write arrived as two: forwarded %d -> %d"
          % (fwd1, bridge.counters.forwarded))

    fwd2 = bridge.counters.forwarded
    blob = encode(report("UUV1"))
    cut = 9                                   # mid-payload, past the header
    raw.send_raw(blob[:cut])
    settle(0.6)
    mid = bridge.counters.forwarded
    raw.send_raw(blob[cut:])
    settle(1.0)
    check("B8", mid == fwd2 and bridge.counters.forwarded == fwd2 + 1,
          "one frame split across two writes arrived once, not twice or never "
          "(%d -> %d -> %d)" % (fwd2, mid, bridge.counters.forwarded))

    print("\nB9-B10  commands route back down the same socket")
    cmd = rx_commands_pb2.RxCommand()
    cmd.seq = 5
    cmd.vehicle_id = "USV1"
    cmd.keep_out_zone.SetInParent()
    bridge._route_command(cmd, "keep_out_zone")
    settle(1.0)
    check("B9", link.counts[1] >= 1,
          "the boat's client received %d command(s)" % link.counts[1])
    check("B9b", not raw.got,
          "a command addressed to USV1 did NOT go to UUV1's socket (%d seen)"
          % len(raw.got))

    bcast = rx_commands_pb2.RxCommand()
    bcast.seq = 6
    bcast.all_clear.SetInParent()
    n_link, n_raw = link.counts[1], len(raw.got)
    bridge._route_command(bcast, "all_clear")
    settle(1.0)
    check("B10", link.counts[1] > n_link and len(raw.got) > n_raw,
          "an unaddressed command reached BOTH vehicles (%d, %d)"
          % (link.counts[1] - n_link, len(raw.got) - n_raw))

    print("\nB11-B12  link loss: latest-wins, and reconnect")
    raw.close()
    bridge.tcp.stop()                      # the OCS goes away underneath it
    settle(1.0)
    for _ in range(20):                    # 20 heartbeats into a dead link
        link.publish(fake_report("USV1", TEAM, t0))
    settle(0.5)
    check("B11", not link.connected, "client noticed the OCS had gone")

    sent_before = link.counts[0]
    bridge.tcp.start()                     # OCS comes back
    settle(7.0)                            # past the client's 5 s retry
    sent_after = link.counts[0] - sent_before
    check("B12", link.connected and "USV1" in bridge.tcp.peers(),
          "client reconnected and re-identified: peers=%s" % bridge.tcp.peers())
    check("B12b", sent_after <= 2,
          "20 reports queued during the outage produced %d frames on reconnect "
          "-- latest-wins, not a backlog" % sent_after)

    link.stop()
    bridge.stop()

    passed = sum(1 for _, ok, _ in _results if ok)
    print("\n%d/%d checks passed" % (passed, len(_results)))
    if passed != len(_results):
        print("\n--- bridge log ---")
        for line in logs[-40:]:
            print("   " + line)
    return 0 if passed == len(_results) else 1


# check() records into bench_stage1's list; borrow it so the summary counts ours.
import bench_stage1 as _s1          # noqa: E402
_results = _s1._results


if __name__ == "__main__":
    raise SystemExit(main())
