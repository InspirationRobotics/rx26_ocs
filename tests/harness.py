"""Fixtures shared by the vehicle-link tests: a real Bridge with no network but its own TCP port.

The Bridge's two MQTT clients are replaced by a recorder, so nothing here ever
connects to a broker; its TCP server binds port 0 (the OS picks), so nothing here
can collide with a real OCS on 37564. Clients are real sockets speaking the real
framing -- the point is the link, not a mock of it.

    from tests.harness import BridgeCase, VehicleClient
"""
from __future__ import annotations

import json
import socket
import sys
import tempfile
import threading
import time
import unittest
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge.bridge import Bridge                       # noqa: E402
from rx_bridge.config import load                         # noqa: E402
from rx_bridge.framing import FrameReader, encode         # noqa: E402
from rx_bridge.tcpserver import Conn                      # noqa: E402
from rx_bridge.proto import rx_commands_pb2, rx_course_pb2  # noqa: E402

TEAM = "ASTA"

#: Ekko prefers its radio; the boat has no preference; the sub is configured but
#: never connected, as in bench_stage3. There is no static fence: the UAV geofence
#: comes from the course (rx_bridge/geofence.py), and a test that wants a
#: `[geofence]` section or a `[[geofence.course]]` entry sets `BridgeCase.geofence`.
CONFIG = """
team_id = "{team}"

[[vehicle]]
id = "USV1"
type = "TYPE_USV"
{usv_extra}

[[vehicle]]
id = "UUV1"
type = "TYPE_UUV"

[[vehicle]]
id = "UAV1"
type = "TYPE_UAV"
prefer = "radio"
{uav_extra}

[team]
host = "127.0.0.1"
port = 1

[robocommand]
host = "127.0.0.1"
port = 1

[tcp]
host = "127.0.0.1"
port = 0

[tiers]
task1 = "TIER_CORE"

[limits]
min_declared = 1
{limits}

[paths]
seq_store = "{seq}"
wire_log = "{wire}"
{geofence}
"""


class FakeMqtt:
    """Stands in for a paho client: records what would have been published."""

    def __init__(self) -> None:
        self.published: list[tuple[str, bytes, int]] = []

    def publish(self, topic, payload, qos=0, **_kw):
        self.published.append((topic, payload, qos))

    def loop_stop(self):
        pass

    def disconnect(self):
        pass


def rfc3339(when: float | None = None) -> str:
    dt = datetime.fromtimestamp(when if when is not None else time.time(), tz=timezone.utc)
    return dt.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def heartbeat(vid: str, *, seq: int = 0, state: str = "STATE_AUTO",
              task: str = "TASK_NONE") -> dict:
    """A heartbeat in the JSON the vehicle clients send, valid for its type."""
    vtype = {"USV1": "TYPE_USV", "UUV1": "TYPE_UUV", "UAV1": "TYPE_UAV"}[vid]
    hb = {"state": state,
          "position": {"latitude": 1.2806, "longitude": 103.8557},
          "spd_mps": 1.0, "heading_deg": 90.0, "roll_deg": 0.0, "pitch_deg": 0.0,
          "vehicle_type": vtype, "current_task": task}
    if vtype == "TYPE_UAV":
        hb["flight_phase"] = "FLIGHT_PHASE_AIRBORNE"
    out = {"team_id": TEAM, "vehicle_id": vid, "sent_at": rfc3339(), "heartbeat": hb}
    if seq:
        out["seq"] = seq
    return out


def hello_frame(vid: str, link: str = "radio", agent: str = "test",
                synthesized=None) -> dict:
    """The `ocs_hello` a client opens with. `synthesized` is left out unless given,
    because a hello without it is the contract's default case."""
    h = {"vehicle_id": vid, "link": link, "agent": agent}
    if synthesized is not None:
        h["synthesized"] = synthesized
    return {"ocs_hello": h}


def readiness(vid: str, *, command_seq: int, seq: int = 0) -> dict:
    out = {"team_id": TEAM, "vehicle_id": vid, "sent_at": rfc3339(),
           "readiness": {"command_seq": command_seq}}
    if seq:
        out["seq"] = seq
    return out


def incident_ack(vid: str, *, command_seq: int) -> dict:
    return {"team_id": TEAM, "vehicle_id": vid, "sent_at": rfc3339(),
            "incident_ack": {"command_seq": command_seq}}


#: The harness's course: a ~117 m x 111 m rectangle at Singapore, convex, so the
#: derived fence is its 2 m inset.
BOX = [(1.28010, 103.85520), (1.28010, 103.85625),
       (1.28110, 103.85625), (1.28110, 103.85520)]

#: RoboNation's own test course, as their server publishes it: a rotated rectangle,
#: closed (first corner repeated at the end).
ROBONATION_TEST_COURSE = [(1.280329, 103.8556869), (1.2804716, 103.8554529),
                          (1.2808993, 103.8557337), (1.2807567, 103.8559743),
                          (1.280329, 103.8556869)]


def course_bytes(cid: str = "ALPHA", freq: int = 25000, corners=None) -> bytes:
    c = rx_course_pb2.RxCourse()
    c.course_id = cid
    c.pinger_freq_hz = freq
    for lat, lon in (BOX if corners is None else corners):
        c.corners.add(latitude=lat, longitude=lon)
    return c.SerializeToString()


def fence_frame(vid: str, polygon, *, verified: bool = True, state: str = "done",
                at="now", reason: str | None = None, lockout=None) -> dict:
    """The `ocs_fence` the radio adapter sends: what the autopilot was read back
    holding. `polygon` is the OPEN polygon (no closing vertex), [] for "cleared", or
    None. `at` is the read-back time as epoch seconds, "now", or None."""
    when = time.time() if at == "now" else at
    return {"ocs_fence": {
        "vehicle_id": vid, "state": state,
        "polygon": None if polygon is None else [list(p) for p in polygon],
        "verified": verified,
        "verified_at": None if when is None else rfc3339(when),
        "reason": reason if reason is not None else (
            "written and read back" if verified else "not read back"),
        "lockout": lockout}}


def open_fence(info_fence) -> list | None:
    """An `ocs_info.uav_geofence` (closed, [{latitude, longitude}]) as the OPEN
    [[lat, lon], ...] the adapter writes and reports back."""
    if not info_fence:
        return None
    return [[p["latitude"], p["longitude"]] for p in info_fence[:-1]]


def confirm_command(*, vehicle: str, report_seq: int, seq: int = 90,
                    addressed_to: str = "") -> rx_commands_pb2.RxCommand:
    cmd = rx_commands_pb2.RxCommand()
    cmd.team_id = TEAM
    cmd.seq = seq
    cmd.vehicle_id = addressed_to
    cmd.readiness_confirm.report_seq = report_seq
    cmd.readiness_confirm.vehicle_id = vehicle
    return cmd


def wait_for(cond, timeout: float = 3.0, every: float = 0.01) -> bool:
    """Poll `cond` until true or timeout -- never a fixed sleep on the happy path."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(every)
    return bool(cond())


class VehicleClient:
    """A real socket speaking the vehicle-link framing, collecting what it is sent."""

    def __init__(self, port: int) -> None:
        self.sock = socket.create_connection(("127.0.0.1", port), timeout=5.0)
        self.frames: list[dict] = []
        self.closed = threading.Event()
        self._adapter_vid: str | None = None
        self._reader = FrameReader()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self) -> None:
        self.sock.settimeout(0.1)
        while True:
            try:
                chunk = self.sock.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            for payload in self._reader.feed(chunk):
                frame = json.loads(payload.decode("utf-8"))
                self.frames.append(frame)
                self._answer(frame)
        self.closed.set()

    def _answer(self, frame: dict) -> None:
        vid = self._adapter_vid
        info = frame.get("ocs_info") if vid else None
        if info is None or "uav_geofence" not in info:
            return
        fence = info["uav_geofence"]
        try:
            if fence:
                self.fence(vid, open_fence(fence))
            elif info.get("course") is None:
                self.fence(vid, [], reason="cleared and read back empty")
        except OSError:
            pass

    def send(self, obj: dict) -> None:
        self.sock.sendall(encode(json.dumps(obj).encode("utf-8")))

    def send_raw(self, payload: bytes) -> None:
        self.sock.sendall(encode(payload))

    def fence(self, vid: str, polygon, **kw) -> None:
        """Send an `ocs_fence` for `vid` (see fence_frame)."""
        self.send(fence_frame(vid, polygon, **kw))

    def adapter_for(self, vid: str) -> "VehicleClient":
        """Behave like the radio adapter from now on: answer every `ocs_info` that
        carries a fence with an `ocs_fence` (verified, the OPEN polygon it was
        given, read back now), and one that says there is no course at all with a
        verified EMPTY polygon -- "cleared". A course whose fence was refused is
        answered with nothing: the adapter leaves the old fence standing. Returns self."""
        self._adapter_vid = vid
        return self

    def hello(self, vid: str, link: str = "radio", agent: str = "test",
              synthesized=None) -> None:
        self.send(hello_frame(vid, link, agent, synthesized))

    def of_kind(self, key: str) -> list[dict]:
        """Frames whose top-level key is `key` (ocs_info, ocs_directive, run_start...)."""
        return [f for f in list(self.frames) if key in f]

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


class BridgeCase(unittest.TestCase):
    """A Bridge on an ephemeral TCP port with recording stand-ins for both brokers."""

    usv_extra = ""
    uav_extra = ""
    limits = ""
    #: Text appended at the very end of the config, after [paths]: a `[geofence]`
    #: section and any `[[geofence.course]]` entries. Bare keys here would belong to
    #: the last table, which is why it is a block of its own.
    geofence = ""
    #: Vehicle ids for the bridge's bench dry-run switch (empty: off).
    bench_auto: tuple = ()

    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.logs: list[str] = []
        self.bridge = self.make_bridge()
        self.port = self.bridge.tcp.bound_port
        self.clients: list[VehicleClient] = []

    def make_bridge(self) -> Bridge:
        tmp = Path(self._tmp.name)
        cfg_path = tmp / "bridge.toml"
        cfg_path.write_text(CONFIG.format(
            team=TEAM, usv_extra=self.usv_extra, uav_extra=self.uav_extra,
            limits=self.limits, geofence=self.geofence,
            seq=(tmp / "seq.json").as_posix(),
            wire=(tmp / "wire").as_posix()), encoding="utf-8")
        bridge = Bridge(load(cfg_path), log=lambda m: self.logs.append(str(m)),
                        bench_auto=self.bench_auto)
        bridge.rc, bridge.team = FakeMqtt(), FakeMqtt()
        bridge.tcp.start()
        self.addCleanup(self._stop, bridge)
        return bridge

    def _stop(self, bridge: Bridge) -> None:
        for c in self.clients:
            c.close()
        bridge.tcp.stop()
        bridge.wire.close()

    # ---- helpers ------------------------------------------------------------

    def client(self) -> VehicleClient:
        c = VehicleClient(self.bridge.tcp.bound_port)
        self.clients.append(c)
        return c

    def connect(self, vid: str, link: str | None, *, agent: str = "test",
                synthesized=None) -> VehicleClient:
        """A client that has identified itself and is filed under (vid, link).

        link=None sends no hello -- a report is the first frame, as today's boat
        and Ekko clients do. The vehicle is waited for, not slept for.
        """
        c = self.client()
        if link is None:
            c.send(heartbeat(vid))
            link = "wifi"
        else:
            c.hello(vid, link, agent, synthesized)
        self.assertTrue(wait_for(lambda: link in self.bridge.tcp.links_of(vid)),
                        "%s never filed on %s" % (vid, link))
        return c

    def feed(self, obj: dict, link: str = "wifi", *, key: str | None = None) -> str | None:
        """Hand one vehicle frame straight to the bridge, as if it came off a socket.

        No socket and no thread, so the hot-path tests are deterministic. Frames
        fed with the same `key` share one Conn (so a hello sticks, as it would on
        a real connection); the default key is the link.
        """
        conns = self.__dict__.setdefault("_fed", {})
        conn = conns.get(key or link)
        if conn is None:
            conn = conns[key or link] = Conn(None, "fed:%s" % (key or link))
            conn.link = link
        return self.bridge._on_tcp_frame(json.dumps(obj).encode("utf-8"), conn)

    def confirm_fence(self, vid: str = "UAV1", polygon="fence", *, link: str = "radio",
                      **kw) -> None:
        """Play the radio adapter: identify as `vid` (once) and send an `ocs_fence`.

        `polygon` "fence" (the default) is the OPEN form of the fence the bridge holds
        for the course in hand -- the adapter's whole job, done right. Pass a list,
        [] or None to say something else. kw goes to fence_frame (verified, at, ...)."""
        # Its own fed connection (key), never the one a test feeds its heartbeats
        # down: a test that opens with its own hello on `link` must still be the
        # first to speak on that socket.
        key = "fence:%s:%s" % (vid, link)
        conn = self.__dict__.get("_fed", {}).get(key)
        if conn is None:
            self.feed(hello_frame(vid, link, "fleet_link"), link, key=key)
            conn = self._fed[key]
        if vid not in conn.vids:
            # A fed frame has no TcpServer behind it to file the socket under the
            # vehicle it names; do what TcpServer._register does, by hand.
            conn.vids.append(vid)
        if polygon == "fence":
            polygon = self.bridge._fence.open_polygon
        self.feed(fence_frame(vid, polygon, **kw), link, key=key)

    def declared(self, active=("USV1", "UAV1"), *, confirm: bool = True) -> None:
        """Walk the machine to DECLARED with `active` forced in. Each aircraft going
        in is first confirmed to hold the course's fence, as the adapter would
        (`confirm=False` leaves that to the test: the gate refuses an unconfirmed one)."""
        b = self.bridge
        b.machine.on_connect()
        b._handle_course(course_bytes())
        for vid in b.cfg.vehicle_ids:
            b.set_active(vid, vid in active)
        if confirm:
            for vid in active:
                if b.cfg.vehicle(vid).is_uav:
                    self.confirm_fence(vid)
        self.assertIn("declared", b.declare())

    def reports_to_rc(self, vid: str | None = None) -> list:
        """RxReports the bridge published to RoboCommand, parsed."""
        from rx_bridge.proto import rx_reports_pb2
        out = []
        for topic, payload, _qos in self.bridge.rc.published:
            if "/report" in topic:
                r = rx_reports_pb2.RxReport()
                r.ParseFromString(payload)
                if vid is None or r.vehicle_id == vid:
                    out.append(r)
        return out

    def wire_lines(self) -> list[dict]:
        self.bridge.wire._fh.flush()
        text = self.bridge.wire.path.read_text(encoding="utf-8")
        return [json.loads(line) for line in text.splitlines() if line.strip()]
