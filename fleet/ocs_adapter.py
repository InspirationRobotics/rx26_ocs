"""ocs_adapter -- the OCS half of Ekko's radio link to the RoboCommand bridge.

    Ekko's companion  --TUNNEL-->  RFD900 / COM8  -->  fleet_link  -->  rx_bridge  --> RoboCommand
                                                          ^ this file
    RoboCommand  -->  rx_bridge  -->  fleet_link  --TUNNEL-->  COM8  -->  Ekko's companion

The bridge (inside ocs_web, TCP 127.0.0.1:37564) stays transport-agnostic: for each radio
vehicle this opens ONE ordinary vehicle-link connection to it -- 4-byte length + JSON, the
framing rx_bridge/framing.py owns -- says who it is in a first `ocs_hello` frame, and then
translates. The wire format of the radio side is rxl_codec's; the JSON on the bridge side is
exactly what the vehicles' own ocs_link.py send and what the bridge sends them.

WHAT FLOWS
  uplink    a TUNNEL from the companion sysid carrying an RXL report (0x10-0x17) -> RxReport
            JSON -> bridge. INCIDENT_ACK and READINESS are answered with an RXL ACK, but only
            once the bridge has taken them: an ACK is a promise, and the drone's retry is the
            safety net if the bridge was down. Every report carries a `seq` of this adapter's
            own (see RadioVehicle._forward): the bridge hands it back as the report_seq of a
            ReadinessConfirm, and the adapter turns it into the cmd_seq Ekko used.
  heartbeat until Ekko's companion sends its own RXL HEARTBEAT, one is built from the
            autopilot's MAVLink on the radio (ocs_heartbeat.py). A fresh companion heartbeat
            always takes precedence -- even one that cannot be forwarded, because the vehicle
            saying "I do not know my position" must not be contradicted by us guessing one.
  downlink  RxCommand JSON and `ocs_info` (course, UAV geofence) from the bridge -> RXL TUNNEL to
            the companion sysid, on the configured link only, retried and repeated per the
            catalogue (POLICIES below).

NEVER BLOCK, NEVER INVENT, ASK THE AUTOPILOT FOR TWO THINGS. Routing is the hub's and is
untouched: this runs as a hub observer (a deque append under the hub lock) plus its own threads.
The stand-in heartbeat needs EXTENDED_SYS_STATE (flight phase), which ArduCopter 4.7 puts in no
stream group and sends only when asked per message. Stream rates on Ekko are latched at boot,
but MAV_CMD_SET_MESSAGE_INTERVAL works per port at once, so the adapter sends exactly that, and
nothing else -- no REQUEST_DATA_STREAM, no other message: sysid 250 -> autopilot (1, 1), param1
245, param2 1e6 us. It asks only while the stand-in is the one speaking, only when the message
has not been heard for 10 s, and at most once per 10 s -- the rule and the 1 Hz interval of
gcs_radio.py's WANTED list, so the two can never fight over the rate. The autopilot's own radio
port budget (~1.3 kB/s of the mesh) is not ours to spend; our air use is a handful of small
frames, counted in /fleet (tx, and heartbeat.flight_state_request).
  The second, only on a radio vehicle with `fence_upload = true`: the declared geofence from
`ocs_info` is written to the autopilot as an inclusion fence (fence_upload.py -- read three
FENCE_* parameters, download the fence, write it if it differs, read it back; never while armed,
never a parameter written). A few dozen frames once per polygon per run of fleet_link, counted in
/fleet (ocs.vehicles.<id>.fence_upload).

CONCURRENCY. Three kinds of thread meet here: the hub's (observer: append to a deque, nothing
else), the tick thread (drains the deque, builds the heartbeat, services retries) and one reader
thread per bridge connection (commands in). One RLock guards all adapter state. Lock order is
always adapter -> hub, never the reverse (the observer takes no adapter lock), so the hub can
never wait on this file.
"""
from __future__ import annotations

import collections
import contextlib
import json
import logging
import select
import socket
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Deque, Dict, Iterable, List, Optional, Sequence, Tuple

from . import fence_upload as fu
from . import ocs_translate as tr
from . import rxl_codec as rx
from .hub import Hub
from .mavframe import (MAV_CMD_SET_MESSAGE_INTERVAL, MSG_TUNNEL, Frame, build_command_long_frame,
                       build_tunnel_frame, pack_frame, parse_tunnel)
from .ocs_heartbeat import (AUTOPILOT_COMPID, CURRENT_TASK, MSG_EXTENDED_SYS_STATE, REQUEST_INTERVAL_US, REQUEST_STALE_S,
                            HeartbeatSynth, Timeouts)
from .ocs_heartbeat import MESSAGE_IDS as AUTOPILOT_IDS
from .vehicles import RateMeter

try:
    from rx_bridge import framing                  # the vehicle link's frame format: stdlib only
except ImportError:                                # run from a folder that is not OCS/
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from rx_bridge import framing

log = logging.getLogger("fleet")

OCS_DIR = Path(__file__).resolve().parents[1]
DEFAULT_BRIDGE_PORT = 37564
#: Where an autopilot's fence is saved before fence_upload replaces it (gitignored, like run/tunnel).
DEFAULT_FENCE_BACKUP_DIR = "run/fence"
INBOX_MAX = 2048
POLL_S = 0.05
DEDUP_HORIZON_S = 30.0                  # catalogue 4.5: the (msg, content) seen-set horizon
ATTEMPT_SEQS_KEPT = 16
READINESS_SEQS_KEPT = 256               # relayed READINESS reports a ReadinessConfirm may still name
MAX_REFUSAL_REASONS = 32                # distinct reasons counted in /fleet before the rest share one bucket
_UNKNOWN_FRAME_LOG_EVERY = 100
#: The RxReport body kinds (the oneof's field names) this link's reports may be rebuilt from a
#: side channel rather than reported by the vehicle. Declared in the hello, so the bridge never
#: lets them outrank a real report of the same kind that arrives on another link.
SYNTHESIZED_KINDS = ("heartbeat",)


class OcsConfigError(ValueError):
    pass


# ------------------------------------------------------------------------- config

@dataclass(frozen=True)
class RadioVehicleCfg:
    vehicle_id: str
    vehicle_type: str = "TYPE_UAV"
    node: int = rx.NODE_UAV                  # the catalogue's node id, not a sysid
    companion_sysid: int = 200
    autopilot_sysid: int = 1
    link: str = "rfd"
    rate_hz: float = 2.0                     # uav_params.yaml ocs_client.rate_hz: the mandated rate
    geoid_separation_m: Optional[float] = None
    airborne_alt_m: float = 1.0
    phase_fallback: bool = False
    companion_fresh_s: float = 2.0           # a companion heartbeat this young takes precedence
    timeouts: Timeouts = Timeouts()
    fence_upload: bool = False               # write ocs_info's geofence to the autopilot (fence_upload.py)
    fence_may_block_arming: bool = False     # ...even when that leaves it unable to arm where it is
    fence_clear_without_course: bool = False  # bridge has no course -> clear the autopilot's fence


@dataclass(frozen=True)
class OcsCfg:
    enabled: bool = True
    bridge_host: str = "127.0.0.1"
    bridge_port: int = DEFAULT_BRIDGE_PORT
    sysid: int = 250                          # QGC on this laptop is 255, gcs_radio 254
    compid: int = 190
    vehicles: Tuple[RadioVehicleCfg, ...] = ()
    reconnect_min_s: float = 1.0
    reconnect_max_s: float = 10.0
    zone_ttl_s: float = 3600.0                # safety expiry; the real end is ALL_CLEAR or the next run
    moving_ttl_s: float = 30.0                # a moving object not updated for this long is not repeated
    retry_give_up_s: float = 600.0
    max_zones: int = 8
    fence_backup_dir: str = str(OCS_DIR / DEFAULT_FENCE_BACKUP_DIR)


def parse_hostport(value: str, where: str) -> Tuple[str, int]:
    host, sep, port = str(value).rpartition(":")
    if not sep or not host or not port.isdigit() or not 1 <= int(port) <= 65535:
        raise OcsConfigError("%s: %r is not HOST:PORT" % (where, value))
    return host, int(port)


def _byte(table: dict, key: str, default: int, where: str, lo: int = 0) -> int:
    v = table.get(key, default)
    if isinstance(v, bool) or not isinstance(v, int) or not lo <= v <= 255:
        raise OcsConfigError("%s: %s must be an integer %d-255, not %r" % (where, key, lo, v))
    return v


def _positive(table: dict, key: str, default: float, where: str) -> float:
    v = table.get(key, default)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not v > 0:
        raise OcsConfigError("%s: %s must be a positive number, not %r" % (where, key, v))
    return float(v)


def parse_radio_vehicle(t: dict, index: int) -> RadioVehicleCfg:
    where = "[[ocs.radio_vehicle]] #%d" % (index + 1)
    if not isinstance(t.get("vehicle_id"), str) or not t["vehicle_id"]:
        raise OcsConfigError("%s: missing vehicle_id" % where)
    vtype = str(t.get("type", "TYPE_UAV"))
    if vtype not in rx.NODE_BY_VEHICLE_TYPE:
        raise OcsConfigError("%s: type %r is not one of %s" % (where, vtype, ", ".join(rx.NODE_BY_VEHICLE_TYPE)))
    geoid = t.get("geoid_separation_m")
    if geoid is not None and (isinstance(geoid, bool) or not isinstance(geoid, (int, float))
                              or not -120.0 <= geoid <= 120.0):
        raise OcsConfigError("%s: geoid_separation_m must be a number within +/-120 m" % where)
    node = _byte(t, "node", rx.NODE_BY_VEHICLE_TYPE[vtype], where, lo=1)
    if node in (rx.NODE_OCS, rx.NODE_BROADCAST):
        raise OcsConfigError("%s: node %d is the OCS / broadcast" % (where, node))
    flags = {}
    for key in ("fence_upload", "fence_may_block_arming", "fence_clear_without_course"):
        flags[key] = t.get(key, False)
        if not isinstance(flags[key], bool):
            raise OcsConfigError("%s: %s must be true or false, not %r" % (where, key, flags[key]))
    base = Timeouts()
    return RadioVehicleCfg(
        vehicle_id=t["vehicle_id"], vehicle_type=vtype, node=node,
        companion_sysid=_byte(t, "companion_sysid", 200, where, lo=1),
        autopilot_sysid=_byte(t, "autopilot_sysid", 1, where, lo=1),
        link=str(t.get("link", "rfd")), rate_hz=_positive(t, "rate_hz", 2.0, where),
        geoid_separation_m=None if geoid is None else float(geoid),
        airborne_alt_m=_positive(t, "airborne_alt_m", 1.0, where),
        phase_fallback=bool(t.get("phase_fallback", False)),
        companion_fresh_s=_positive(t, "companion_fresh_s", 2.0, where),
        timeouts=Timeouts(pose_s=_positive(t, "pose_timeout_s", base.pose_s, where),
                          attitude_s=_positive(t, "attitude_timeout_s", base.attitude_s, where),
                          status_s=_positive(t, "status_timeout_s", base.status_s, where),
                          flight_s=_positive(t, "flight_timeout_s", base.flight_s, where)),
        **flags)


def parse_ocs_config(raw: dict) -> OcsCfg:
    """The `[ocs]` table (with its `[[ocs.radio_vehicle]]` array) -> OcsCfg. An absent table
    is a disabled adapter. Raises OcsConfigError."""
    if not raw:
        return OcsCfg(enabled=False)
    where = "[ocs]"
    host, port = parse_hostport(raw.get("bridge", "127.0.0.1:%d" % DEFAULT_BRIDGE_PORT), where + " bridge")
    vehicles = tuple(parse_radio_vehicle(t, i) for i, t in enumerate(raw.get("radio_vehicle", [])))
    sysid, compid = _byte(raw, "sysid", 250, where, lo=1), _byte(raw, "compid", 190, where)
    ids = [v.vehicle_id for v in vehicles]
    if len(set(ids)) != len(ids):
        raise OcsConfigError("%s: a vehicle_id is listed twice in radio_vehicle" % where)
    used: Dict[int, str] = {}
    for v in vehicles:
        for sysid_in_use in (v.companion_sysid, v.autopilot_sysid):
            if sysid_in_use == sysid:
                raise OcsConfigError("%s: sysid %d is the OCS's own and is also %s's: the radio could not tell "
                                     "who spoke" % (where, sysid, v.vehicle_id))
        if v.companion_sysid == v.autopilot_sysid:
            raise OcsConfigError("%s: %s has the same sysid for companion and autopilot" % (where, v.vehicle_id))
        for s in (v.companion_sysid, v.autopilot_sysid):
            if used.setdefault(s, v.vehicle_id) != v.vehicle_id:
                raise OcsConfigError("%s: sysid %d belongs to both %s and %s" % (where, s, used[s], v.vehicle_id))
    return OcsCfg(enabled=bool(raw.get("enabled", True)), bridge_host=host, bridge_port=port,
                  sysid=sysid, compid=compid, vehicles=vehicles,
                  reconnect_min_s=_positive(raw, "reconnect_min_s", 1.0, where),
                  reconnect_max_s=_positive(raw, "reconnect_max_s", 10.0, where),
                  zone_ttl_s=_positive(raw, "zone_ttl_s", 3600.0, where),
                  moving_ttl_s=_positive(raw, "moving_ttl_s", 30.0, where),
                  retry_give_up_s=_positive(raw, "retry_give_up_s", 600.0, where),
                  max_zones=int(_positive(raw, "max_zones", 8, where)),
                  fence_backup_dir=str(OCS_DIR / str(raw.get("fence_backup_dir", DEFAULT_FENCE_BACKUP_DIR))))


# ----------------------------------------------------------------- bridge connection

class BridgeLink:
    """ONE ordinary vehicle-link connection to the bridge, kept alive in the background.

    Dials, says `ocs_hello` first (nothing else can be sent before it: the socket is only
    published after the hello is out), reads frames until the peer goes, then backs off and
    dials again. `ocs_web` not running is the normal state, not an error."""

    def __init__(self, vehicle_id: str, host: str, port: int, on_frame: Callable[[dict], None], *,
                 agent: str = "fleet_link", synthesized: Sequence[str] = SYNTHESIZED_KINDS,
                 reconnect_min_s: float = 1.0, reconnect_max_s: float = 10.0,
                 connect_timeout_s: float = 2.0, send_timeout_s: float = 0.25,
                 clock: Callable[[], float] = time.monotonic):
        self.vehicle_id, self.host, self.port = vehicle_id, host, port
        self.on_frame = on_frame
        self.agent = agent
        self.synthesized = tuple(synthesized)
        self.reconnect_min_s, self.reconnect_max_s = reconnect_min_s, reconnect_max_s
        self.connect_timeout_s, self.send_timeout_s = connect_timeout_s, send_timeout_s
        self._clock = clock
        self._sock: Optional[socket.socket] = None
        self._tx_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.connects = self.disconnects = self.frames_in = self.frames_out = 0
        self.undecodable_in = self.send_failures = 0
        self.last_error: Optional[str] = None
        self._connected_at: Optional[float] = None
        self._retry_at: Optional[float] = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="ocs-bridge-" + self.vehicle_id, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._close()
        if self._thread is not None:
            self._thread.join(3.0)

    def hello(self) -> dict:
        return {"ocs_hello": {"vehicle_id": self.vehicle_id, "link": "radio", "agent": self.agent,
                              "synthesized": list(self.synthesized)}}

    def send_json(self, obj: dict) -> bool:
        """Frame and send one object. False when not connected or the write failed (and the
        connection is then dropped, so the reader thread redials)."""
        try:
            data = framing.encode(json.dumps(obj, allow_nan=False, separators=(",", ":")).encode("utf-8"))
        except (TypeError, ValueError) as e:
            self.last_error = "unserialisable frame dropped: %s" % e
            return False
        with self._tx_lock:
            sock = self._sock
            if sock is None:
                return False
            try:
                sock.sendall(data)
            except OSError as e:
                self.send_failures += 1
                self._drop("send failed: %s" % e, sock)
                return False
            self.frames_out += 1
            return True

    # ---- the reader thread

    def _run(self) -> None:
        delay = self.reconnect_min_s
        while not self._stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout_s)
                sock.settimeout(self.send_timeout_s)
                sock.sendall(framing.encode(json.dumps(self.hello(), separators=(",", ":")).encode("utf-8")))
            except OSError as e:
                if self.last_error is None or not self.last_error.startswith("cannot reach"):
                    log.info("ocs bridge %s:%d (%s): not reachable yet - retrying every %.0f-%.0f s (%s)",
                             self.host, self.port, self.vehicle_id, self.reconnect_min_s, self.reconnect_max_s, e)
                self.last_error = "cannot reach %s:%d: %s" % (self.host, self.port, e)
                self._retry_at = self._clock() + delay
                if self._stop.wait(delay):
                    return
                delay = min(delay * 2.0, self.reconnect_max_s)
                continue
            delay = self.reconnect_min_s
            self._retry_at = None
            self.frames_out += 1
            self.connects += 1
            self._connected_at = self._clock()
            self.last_error = None
            self._sock = sock                       # published only now: the hello is already out
            log.info("ocs bridge %s:%d: connected as %s (radio)", self.host, self.port, self.vehicle_id)
            self._read(sock)
            self._drop(self.last_error or "closed", sock)
            self._retry_at = self._clock() + delay
            if self._stop.wait(delay):
                return

    def _read(self, sock: socket.socket) -> None:
        reader = framing.FrameReader()
        while not self._stop.is_set() and self._sock is sock:
            try:
                ready, _, _ = select.select([sock], [], [], 0.2)
                if not ready:
                    continue
                chunk = sock.recv(65536)
            except (OSError, ValueError) as e:             # ValueError: the fd was closed under us
                if self._sock is sock:
                    self.last_error = "read failed: %s" % e
                return
            if not chunk:
                self.last_error = "closed by the bridge"
                return
            try:
                frames = reader.feed(chunk)
            except ValueError as e:
                self.last_error = "%s - reconnecting" % e
                return
            for payload in frames:
                self.frames_in += 1
                try:
                    obj = json.loads(payload.decode("utf-8"))
                except (UnicodeDecodeError, ValueError) as e:
                    self.undecodable_in += 1
                    log.warning("ocs bridge: undecodable frame ignored (%s)", e)
                    continue
                try:
                    self.on_frame(obj)
                except Exception:                                    # noqa: BLE001
                    log.exception("ocs bridge: handler raised on a frame; carrying on")

    def _drop(self, reason: str, sock: socket.socket) -> None:
        with self._tx_lock:
            if self._sock is sock:
                self._sock = None
                self.disconnects += 1
                self.last_error = reason
                self._connected_at = None
                log.warning("ocs bridge %s:%d (%s): disconnected - %s", self.host, self.port,
                            self.vehicle_id, reason)
        try:
            sock.close()
        except OSError:
            pass

    def _close(self) -> None:
        sock = self._sock
        if sock is not None:
            self._drop("stopping", sock)

    def status(self) -> dict:
        now = self._clock()
        return {"address": "%s:%d" % (self.host, self.port), "connected": self.connected,
                "synthesized": list(self.synthesized),
                "connected_age_s": None if self._connected_at is None else round(now - self._connected_at, 1),
                "connects": self.connects, "disconnects": self.disconnects,
                "retry_in_s": None if self._retry_at is None else max(0.0, round(self._retry_at - now, 1)),
                "last_error": self.last_error, "frames_in": self.frames_in, "frames_out": self.frames_out,
                "undecodable_in": self.undecodable_in, "send_failures": self.send_failures}


# ---------------------------------------------------------------- downlink policies

@dataclass(frozen=True)
class Policy:
    """How one downlink message is repeated. `fast_n` sends `fast_s` apart, then one every
    `slow_s` (None: the burst is all there is). `ack` says what an RXL ACK (0x38) does:
    "finish" ends it, "burst" ends only the fast part, "" ignores it. `answer` does the same
    for the application-level answer, an INCIDENT_ACK quoting the command's cmd_seq."""
    fast_n: int
    fast_s: float
    slow_s: Optional[float]
    ack: str = ""
    answer: str = ""

    def interval(self, sent: int) -> Optional[float]:
        return self.fast_s if sent < self.fast_n else self.slow_s

    @property
    def retried(self) -> bool:
        return self.ack == "finish" and self.slow_s is not None


#: Catalogue 4.3 / 4.5, with every deviation written down in docs/EKKO_RADIO_LINK.md:
#:   RUN_START 4.3 row: "5x at 2 Hz, then 0.1 Hz. ACK"        (4.5 says 3x then 0.2 Hz; the row is
#:                      specific to the message, so it wins)
#:   ALL_CLEAR 4.5:      "2 Hz for 3 attempts, then 0.2 Hz until ACKed or superseded"
#:   ASSISTANCE_REQUEST  has no ACK mark; the vehicle's INCIDENT_ACK is the answer, so it is
#:                       retried like a state transfer until that arrives
#:   KEEP_OUT_ZONE       "repeated at 0.2 Hz for the life of the zone"; a 3x 2 Hz burst first
#:                       (a lost first zone is a safety event; the burst is 3 x 31 B)
#:   MOVING_OBJECT_ALERT "1 Hz while active"
#:   READINESS_CONFIRM   no ACK mark: 3x at 2 Hz and done (idempotent, same report_seq)
#:   COURSE / GEOFENCE   "sent on receipt, then 0.1 Hz"
POLICIES: Dict[int, Policy] = {
    rx.RUN_START: Policy(5, 0.5, 10.0, ack="finish"),
    rx.ASSISTANCE_REQUEST: Policy(3, 0.5, 5.0, ack="finish", answer="finish"),
    rx.KEEP_OUT_ZONE: Policy(3, 0.5, 5.0, ack="burst", answer="burst"),
    rx.ALL_CLEAR: Policy(3, 0.5, 5.0, ack="finish", answer="finish"),
    rx.MOVING_OBJECT_ALERT: Policy(1, 1.0, 1.0),
    rx.READINESS_CONFIRM: Policy(3, 0.5, None, ack="finish"),
    rx.COURSE: Policy(1, 10.0, 10.0),
    rx.GEOFENCE_INFO: Policy(1, 10.0, 10.0),
}
#: Commands that name a domain: only sent to a vehicle of that type (UNKNOWN matches all).
DOMAIN_COMMANDS = frozenset({rx.ASSISTANCE_REQUEST, rx.KEEP_OUT_ZONE, rx.ALL_CLEAR, rx.MOVING_OBJECT_ALERT})
#: Keys that belong to one run and are dropped when RoboCommand starts another.
RUN_SCOPED = frozenset({"zone", "moving", "assist", "clear", "confirm"})


def types_overlap(a: Iterable[int], b: Iterable[int]) -> bool:
    """Do two lists of VehicleType values name a vehicle in common? An empty list or one that
    holds TYPE_UNKNOWN (0) means EVERY type, on either side. The one rule for "does this
    command reach that vehicle" and "does this ALL_CLEAR end that zone": the two must not
    disagree about what UNKNOWN means."""
    a, b = set(a), set(b)
    return not a or not b or 0 in a or 0 in b or bool(a & b)


@dataclass
class _Pending:
    out: tr.Outgoing
    policy: Policy
    created: float
    next_due: float
    expires_at: Optional[float] = None
    give_up_at: Optional[float] = None
    sent: int = 0
    last_sent: Optional[float] = None
    seqs: Deque[int] = field(default_factory=lambda: collections.deque(maxlen=ATTEMPT_SEQS_KEPT))

    @property
    def name(self) -> str:
        return rx.NAMES[self.out.msg_id]


class Sender:
    """Builds and writes the adapter's frames. One MAVLink sequence counter for the OCS
    component (sysid/compid), shared by every vehicle and every message type, as the MAVLink
    header expects."""

    def __init__(self, hub: Hub, cfg: OcsCfg):
        self.hub, self.cfg = hub, cfg
        self._seq = 0
        self.meter = RateMeter()
        self.frames = self.bytes = 0

    def send(self, link: str, data: bytes, ptype: int, target_sysid: int, origin: str, now: float) -> Optional[int]:
        """One RXL TUNNEL. The MAVLink sequence the frame went out with, or None if the link refused it."""
        return self._send(link, origin, now, lambda seq: build_tunnel_frame(
            self.hub.dialect, data, payload_type=ptype, sysid=self.cfg.sysid, compid=self.cfg.compid,
            seq=seq, target_system=target_sysid))

    def send_command_long(self, link: str, command: int, params: Sequence[float], target_sysid: int,
                          target_compid: int, origin: str, now: float) -> Optional[int]:
        """One COMMAND_LONG. As send()."""
        return self._send(link, origin, now, lambda seq: build_command_long_frame(
            self.hub.dialect, command, params, sysid=self.cfg.sysid, compid=self.cfg.compid, seq=seq,
            target_system=target_sysid, target_component=target_compid))

    def send_message(self, link: str, msgid: int, payload: bytes, origin: str, now: float) -> Optional[int]:
        """Any other message, its payload already packed (the target is inside it). As send()."""
        return self._send(link, origin, now, lambda seq: pack_frame(
            self.hub.dialect, msgid, payload, sysid=self.cfg.sysid, compid=self.cfg.compid, seq=seq))

    def _send(self, link: str, origin: str, now: float, build: Callable[[int], bytes]) -> Optional[int]:
        seq, self._seq = self._seq, (self._seq + 1) & 0xFF
        raw = build(seq)
        if not self.hub.transmit(link, raw, origin=origin):
            return None
        self.frames += 1
        self.bytes += len(raw)
        self.meter.add(len(raw), now)
        return seq


# ---------------------------------------------------------------- one radio vehicle

class RadioVehicle:
    """Everything the adapter knows and does for one vehicle on the radio. No threads of its
    own: the adapter's tick thread, the vehicle's bridge reader thread and the /fleet server
    call in, always holding `lock`."""

    def __init__(self, cfg: RadioVehicleCfg, ocs: OcsCfg, sender: Sender, bridge, lock: threading.RLock,
                 clock: Callable[[], float], wall: Callable[[], float], hub: Hub):
        self.cfg, self.ocs, self.sender, self.bridge, self.lock = cfg, ocs, sender, bridge, lock
        self.clock, self.wall, self.hub = clock, wall, hub
        self.ctx = tr.UplinkContext(cfg.vehicle_id, cfg.vehicle_type, cfg.node)
        self.synth = HeartbeatSynth(autopilot_sysid=cfg.autopilot_sysid, geoid_separation_m=cfg.geoid_separation_m,
                                    airborne_alt_m=cfg.airborne_alt_m, phase_fallback=cfg.phase_fallback,
                                    vehicle_type=cfg.vehicle_type, timeouts=cfg.timeouts)
        # downlink
        self.pending: Dict[tuple, _Pending] = {}
        self.run_id: Optional[int] = None
        self.sent_by_type: collections.Counter = collections.Counter()
        self.sends = self.retries = self.acks_matched = self.acks_unmatched = self.acks_sent = 0
        self.give_ups = self.superseded = self.expired = self.duplicates = self.cancelled = 0
        self.tx_refused = self.tx_no_route = self.not_for_vehicle = self.unknown_frames = 0
        self.encode_errors = 0
        self.last_encode_error: Optional[str] = None
        self.refusals: collections.Counter = collections.Counter()      # reason (no values) -> count
        self.directives: collections.Counter = collections.Counter()
        # uplink
        self.uplink_by_type: collections.Counter = collections.Counter()
        self.forwarded: collections.Counter = collections.Counter()
        self.decode_errors = self.refused = self.foreign = self.not_for_us = self.duplicates_in = 0
        self.not_forwarded_no_bridge = 0
        self.last_decode_error: Optional[str] = None
        self.last_refusal: Optional[str] = None
        self.last_uplink: Optional[Tuple[float, str]] = None
        self._seen: Dict[Tuple[int, bytes], float] = {}
        self._report_seq = 0
        #: our report seq -> the cmd_seq Ekko put in the RXL READINESS we relayed under it
        self._readiness: "collections.OrderedDict[int, int]" = collections.OrderedDict()
        # heartbeat
        self._hb_due = 0.0
        self.hb_source = "none"
        self.hb_reason: Optional[str] = "nothing heard from the autopilot yet"
        self.hb_sent = self.hb_suppressed = 0
        self.hb_last_sent: Optional[float] = None
        self.phase_source: Optional[str] = None
        self._companion_t: Optional[float] = None
        self._companion_slot: Optional[dict] = None
        self._companion_refusal: Optional[str] = None
        self._unknown_logged = 0
        # asking the autopilot for EXTENDED_SYS_STATE
        self.fs_state = "not checked yet"
        self.fs_sent = self.fs_refused = 0
        self.fs_last_sent: Optional[float] = None
        self._fs_tried: Optional[float] = None
        self._fs_next = 0.0
        # the declared geofence onto the autopilot; None when this vehicle has fence_upload off
        self.fence: Optional[fu.FenceUploader] = None
        self.fence_reports = 0
        self._fence_sent: Optional[tuple] = None         # (bridge connection, body) last reported
        if cfg.fence_upload:
            self.fence = fu.FenceUploader(label=cfg.vehicle_id, target_sysid=cfg.autopilot_sysid,
                                          target_compid=AUTOPILOT_COMPID, our_sysid=ocs.sysid,
                                          send=self._send_to_autopilot, armed=self.synth.armed,
                                          position=self.synth.position, route_ok=self._route_ok,
                                          backup_dir=ocs.fence_backup_dir,
                                          may_block_arming=cfg.fence_may_block_arming, wall=wall)

    # ------------------------------------------------------------- what to listen to

    def interested(self, link: str, frame: Frame) -> bool:
        if link != self.cfg.link:
            return False
        if frame.msgid == MSG_TUNNEL:
            return frame.sysid == self.cfg.companion_sysid
        if frame.sysid != self.cfg.autopilot_sysid:
            return False
        return frame.msgid in AUTOPILOT_IDS or (self.fence is not None and frame.msgid in fu.MESSAGE_IDS)

    def consume(self, frame: Frame, now: float) -> None:
        if frame.msgid == MSG_TUNNEL:
            self._on_tunnel(frame, now)
            return
        self.synth.note(frame, now)
        if self.fence is not None:
            self.fence.note(frame, now)

    # ---------------------------------------------------------------------- uplink

    def _on_tunnel(self, frame: Frame, now: float) -> None:
        t = parse_tunnel(frame)
        cid = rx.catalogue_id(t.payload_type)
        if not t.length_ok or cid is None or cid not in rx.NAMES:
            self.foreign += 1                    # boat_link traffic, inter-vehicle messages, junk
            return
        if t.target_system not in (0, self.ocs.sysid):
            self.not_for_us += 1                 # a multipoint radio: we hear what is addressed to others
            return
        try:
            name, msg = rx.decode(t.payload_type, t.data)
        except rx.RxlError as e:
            self.decode_errors += 1
            self.last_decode_error = str(e)
            return
        self.last_uplink = (now, name)
        self.uplink_by_type[name] += 1
        if cid == rx.ACK:
            self._on_ack(msg, now)
            return
        if name not in tr.UPLINK_BODIES:
            return                               # a downlink id heard back (our own echo, another OCS)
        duplicate = self._seen_recently(cid, t.data, now) if cid in rx.RECEIVER_ACKS else False
        if cid == rx.HEARTBEAT:
            self._companion_heartbeat(name, msg, now)
            return
        if duplicate:
            self.duplicates_in += 1
            self._ack(cid, frame.seq, now)       # the first ACK may be what was lost
            return
        try:
            report = tr.report_from_rxl(name, msg, self.ctx, now_to_wall(self.wall, self.clock, now))
        except tr.Refusal as e:
            self.refused += 1
            self.last_refusal = "%s: %s" % (name, e)
            return
        if not self._forward(report, name, rxl_seq=msg["cmd_seq"] if cid == rx.READINESS else None):
            return                               # not ACKed: the sender's retry is the net
        if cid == rx.INCIDENT_ACK:
            self._answered(msg["cmd_seq"])
        if cid in rx.RECEIVER_ACKS:
            self._seen[(cid, t.data)] = now      # only now: a report the bridge refused is not "already seen"
            self._ack(cid, frame.seq, now)

    def _seen_recently(self, cid: int, data: bytes, now: float) -> bool:
        """Has this exact message already been taken by the bridge within the dedup horizon?
        (Messages are remembered in _on_tunnel once forwarded: a retry of one the bridge could
        not take must go through, not be ACKed as a duplicate of something never delivered.)"""
        self._seen = {k: t for k, t in self._seen.items() if now - t <= DEDUP_HORIZON_S}
        return (cid, data) in self._seen

    def _companion_heartbeat(self, name: str, msg: dict, now: float) -> None:
        self._companion_t = now
        try:
            self._companion_slot = tr.report_from_rxl(name, msg, self.ctx, now_to_wall(self.wall, self.clock, now))
            self._companion_refusal = None
        except tr.Refusal as e:
            self._companion_slot = None
            self._companion_refusal = "companion heartbeat not forwarded: %s" % e
            self.refused += 1
            self.last_refusal = "HEARTBEAT: %s" % e

    def _forward(self, report: dict, name: str, rxl_seq: Optional[int] = None) -> bool:
        """Give the bridge a report, stamped with this link's own `seq` (1, 2, 3 ...: the bridge
        keeps it as the report's own number and puts it back in the report_seq of a
        ReadinessConfirm). `rxl_seq` is the cmd_seq of the RXL READINESS the report was made
        from; it is remembered against the seq only once the bridge has taken the report."""
        self._report_seq = self._report_seq % 0xFFFFFFFF + 1
        seq = self._report_seq
        if not self.bridge.send_json(tr.stamp_seq(report, seq)):
            self.not_forwarded_no_bridge += 1
            return False
        self.forwarded[name] += 1
        if rxl_seq is not None:
            self._readiness[seq] = rxl_seq
            while len(self._readiness) > READINESS_SEQS_KEPT:
                self._readiness.popitem(last=False)
        return True

    def _companion_fresh(self, now: float) -> bool:
        return self._companion_t is not None and now - self._companion_t <= self.cfg.companion_fresh_s

    # -------------------------------------------------------------------- heartbeat

    def heartbeat_tick(self, now: float) -> None:
        if now < self._hb_due:
            return
        self._hb_due = now + 1.0 / self.cfg.rate_hz
        if self._companion_fresh(now):
            self.hb_source, self.hb_reason = "companion", self._companion_refusal
            self.phase_source = None
            report, self._companion_slot = self._companion_slot, None
            if report is None:
                return                           # nothing new (or nothing forwardable) this period
        else:
            hb, info = self.synth.build(now)
            if hb is None:
                self.hb_source, self.hb_reason = "none", info
                self.hb_suppressed += 1
                return
            self.hb_source, self.hb_reason, self.phase_source = "synthesized", None, info
            # sent_at is when the DATA was in hand (the newest autopilot message it was built from):
            # the bridge measures staleness against it, so `now` would make old data look fresh.
            report = tr.report_from_heartbeat(hb, self.cfg.vehicle_id,
                                              now_to_wall(self.wall, self.clock, self.synth.data_t))
        if self._forward(report, "HEARTBEAT"):
            self.hb_sent += 1
            self.hb_last_sent = now
        else:
            self.hb_reason = "built, but not forwarded: the bridge link is down"

    # ------------------------------------------- asking for the stand-in's missing input

    def request_flight_state(self, now: float) -> None:
        """Ask the autopilot for EXTENDED_SYS_STATE (the flight phase), the one input of the
        stand-in heartbeat that ArduCopter 4.7 never sends unasked.

        The rule is gcs_radio.py's: ask only for a message that is NOT arriving (unheard for
        REQUEST_STALE_S, or never), never re-ask for one that is -- and at most once per
        REQUEST_STALE_S. It is further limited to when it is of any use: the autopilot is alive
        on the link (a HEARTBEAT within the status timeout, and a route to it), and the
        stand-in is the one speaking (a fresh companion heartbeat needs none of it)."""
        c = self.cfg
        if self._companion_fresh(now):
            self.fs_state = "not needed: the companion's own heartbeat is being relayed"
            return
        age = self.synth.flight_age(now)
        if age is not None and age <= REQUEST_STALE_S:
            self.fs_state = "EXTENDED_SYS_STATE is arriving (last %.0f s ago)" % age
            return
        alive = self.synth.status_age(now)
        if alive is None or alive > c.timeouts.status_s or not self._route_ok():
            self.fs_state = ("waiting: the autopilot (sysid %d) is not alive on %s - no HEARTBEAT in the last "
                             "%.1f s, so nothing is asked of it" % (c.autopilot_sysid, c.link, c.timeouts.status_s))
            return
        if now < self._fs_next:
            self.fs_state = "requested %.0f s ago and still not heard; asking again in %.0f s" % (
                now - self._fs_tried, self._fs_next - now)
            return
        self._fs_tried, self._fs_next = now, now + REQUEST_STALE_S
        seq = self.sender.send_command_long(
            c.link, MAV_CMD_SET_MESSAGE_INTERVAL, (MSG_EXTENDED_SYS_STATE, REQUEST_INTERVAL_US),
            c.autopilot_sysid, AUTOPILOT_COMPID, "adapter:" + c.vehicle_id, now)
        if seq is None:
            self.fs_refused += 1
            self.fs_state = "the %s link refused the request (down or full); trying again in %.0f s" % (
                c.link, REQUEST_STALE_S)
            return
        self.fs_sent += 1
        self.fs_last_sent = now
        self.fs_state = "requested now"
        (log.info if self.fs_sent == 1 else log.debug)(
            "ocs %s: asked the autopilot for EXTENDED_SYS_STATE at %.0f Hz (not heard for %s) [%d request(s)]",
            c.vehicle_id, 1e6 / REQUEST_INTERVAL_US,
            "ever" if age is None else "%.0f s" % age, self.fs_sent)

    # ------------------------------------------------------------ the geofence, onto the autopilot

    def fence_tick(self, now: float) -> None:
        if self.fence is not None:
            self.fence.tick(now)
            self._report_fence()

    def _send_to_autopilot(self, msgid: int, payload: bytes, now: float) -> bool:
        return self.sender.send_message(self.cfg.link, msgid, payload, "fence:" + self.cfg.vehicle_id,
                                        now) is not None

    # --------------------------------------------------------------------- downlink

    def on_bridge_frame(self, obj) -> None:
        """One JSON object from the bridge (called on the bridge reader thread)."""
        with self.lock:
            now = self.clock()
            try:
                self._dispatch(obj, now)
            except (tr.Refusal, rx.RxlError) as e:                   # a refusal nothing below claimed
                self._refuse("frame", e)
            except Exception:                                        # noqa: BLE001
                self.unknown_frames += 1
                log.exception("ocs %s: bridge frame raised", self.cfg.vehicle_id)
            self.service(now)                    # a first send now, not on the next tick

    def _dispatch(self, obj, now: float) -> None:
        if not isinstance(obj, dict):
            return self._unknown(obj)
        if "ocs_directive" in obj:
            action = str((obj["ocs_directive"] or {}).get("action", "?")) if isinstance(obj["ocs_directive"], dict) else "?"
            if self.directives[action] == 0:
                log.info("ocs %s: directive %r has no RXL message and is not sent on the radio (the "
                         "vehicle holds until RUN_START)", self.cfg.vehicle_id, action)
            self.directives[action] += 1
            return
        if "ocs_info" in obj:
            return self._info(obj["ocs_info"], now)
        try:
            found = tr.command_body(obj)
        except tr.Refusal as e:
            return self._refuse("COMMAND", e)
        if found is None:
            return self._unknown(obj)
        with self._refusing(found[0].upper()):
            self._command(obj, now)

    def _command(self, obj: dict, now: float) -> None:
        out = tr.command_to_outgoing(obj, default_vehicle_id=self.cfg.vehicle_id,
                                     report_seq_of=self._readiness.get)
        if not self._applies(out):
            self.not_for_vehicle += 1
            return
        if out.msg_id == rx.RUN_START:
            self._on_run_start(out.info["run_id"])
        elif out.msg_id == rx.ALL_CLEAR:
            self._cancel_covered(out.info["affected"])
        self._add(out, now)

    @contextlib.contextmanager
    def _refusing(self, label: str):
        """Whatever inside is refused (translation, a field that does not fit the wire, too many
        zones) is counted and logged against `label`, and the caller carries on."""
        try:
            yield
        except (tr.Refusal, rx.RxlError, fu.FenceError) as e:
            self._refuse(label, e)

    def _refuse(self, label: str, exc: BaseException) -> None:
        """A downlink message that cannot be put on the radio truthfully. Loud, not repaired:
        the bridge has already logged it as routed to this link, so this WARNING and the
        per-reason counter in /fleet are the only place the operator learns it never went."""
        key = tr.reason_key(exc)
        if key not in self.refusals and len(self.refusals) >= MAX_REFUSAL_REASONS:
            key = "(other reasons)"
        self.refusals[key] += 1
        self.encode_errors += 1
        self.last_encode_error = "%s: %s" % (label, exc)
        log.warning("ocs %s: %s NOT sent on the radio - refused, not truncated or repaired: %s",
                    self.cfg.vehicle_id, label, exc)

    def _unknown(self, obj) -> None:
        self.unknown_frames += 1
        if self.unknown_frames <= 3 or self.unknown_frames % _UNKNOWN_FRAME_LOG_EVERY == 0:
            keys = sorted(obj) if isinstance(obj, dict) else type(obj).__name__
            log.warning("ocs %s: unrecognised frame from the bridge ignored (%s) [%d so far]",
                        self.cfg.vehicle_id, keys, self.unknown_frames)

    def _applies(self, out: tr.Outgoing) -> bool:
        """A command that names a domain is sent only to a vehicle of that domain (UNKNOWN
        matches everything): nobody waits for an answer Ekko does not owe."""
        if out.msg_id not in DOMAIN_COMMANDS:
            return True
        return types_overlap(out.info.get("affected", []), [rx.VEHICLE_TYPE_BY_NAME[self.cfg.vehicle_type]])

    def _on_run_start(self, run_id: int) -> None:
        if self.run_id is not None and run_id != self.run_id:
            gone = [k for k in self.pending if k[0] in RUN_SCOPED]
            for k in gone:
                del self.pending[k]
            self.cancelled += len(gone)
            if gone:
                log.info("ocs %s: run %d -> %d: dropped %d message(s) of the previous run",
                         self.cfg.vehicle_id, self.run_id, run_id, len(gone))
        self.run_id = run_id

    def _cancel_covered(self, types: List[int]) -> None:
        """ALL_CLEAR for a vehicle type ends the zones and moving-object alerts that affect it
        (TYPE_UNKNOWN meaning every type, as in _applies)."""
        for key in [k for k, p in self.pending.items() if k[0] in ("zone", "moving")
                    and types_overlap(types, p.out.info.get("affected", []))]:
            del self.pending[key]
            self.cancelled += 1

    def _info(self, info, now: float) -> None:
        """`ocs_info`: the course and the UAV geofence. Each is its own message with its own
        refusal -- a course that does not fit must not cost the vehicle its geofence."""
        if not isinstance(info, dict):
            return self._unknown(info)
        if "course" in info:
            with self._refusing("COURSE"):
                self._set_course(info["course"], now)
        fence_key = next((k for k in ("uav_geofence", "uavGeofence") if k in info), None)
        if fence_key is not None:
            with self._refusing("GEOFENCE_INFO"):
                self._set_fence(info[fence_key], now)
            if self.fence is not None:       # its own refusal: the RXL message's limits are not the autopilot's
                with self._refusing("FENCE_UPLOAD"):
                    self._fence_from_info(info, info[fence_key], now)

    def _fence_from_info(self, info: dict, fence, now: float) -> None:
        """What the autopilot's fence should be, from one `ocs_info` (docs/VEHICLE_LINK.md):
          a polygon                               -> that polygon (the current course's);
          no polygon AND no course ("course": null) -> nothing at all, if fence_clear_without_course:
                                                     the bridge has heard no course since it started,
                                                     so this is a bench without RoboCommand;
          no polygon but a course                 -> left as it is: the course's fence was refused
                                                     (the bridge says why), and a leftover fence from
                                                     another course stops the aircraft arming here,
                                                     which a cleared one would not."""
        if fence is not None:
            self.fence.set_polygon(tr.fence_points(fence), now)
        elif "course" in info and info["course"] is None and self.cfg.fence_clear_without_course:
            self.fence.set_clear(now)
        else:
            self.fence.set_polygon(None, now)

    def _report_fence(self) -> None:
        """`ocs_fence` to the bridge: what the autopilot was last confirmed to hold. Sent on every
        new bridge connection and whenever it changes (a re-check refreshes verified_at once a
        minute); the bridge's pre-run gate declares no UAV run without a fresh one."""
        c = self.fence.confirmation()
        body = dict(c, verified_at=None if c["verified_at"] is None else tr.rfc3339(c["verified_at"]))
        key = (getattr(self.bridge, "connects", None), json.dumps(body, sort_keys=True))
        if key == self._fence_sent or not self.bridge.connected:
            return
        if self.bridge.send_json({"ocs_fence": body}):
            self._fence_sent = key
            self.fence_reports += 1

    def _set_course(self, course, now: float) -> None:
        if course is None:
            self.cancelled += self.pending.pop(("course",), None) is not None
            return
        payload = tr.course_payload(course)
        self._add(tr.Outgoing(rx.COURSE, payload, ("course",), None,
                              {"epoch": payload[0], "corners": len(payload) // 8}), now)

    def _set_fence(self, fence, now: float) -> None:
        old = [k for k in self.pending if k[0] == "fence"]
        parts = [] if fence is None else tr.geofence_payloads(fence)
        new_payloads = {("fence", i): p for i, p in enumerate(parts)}
        if {k: self.pending[k].out.payload for k in old} == new_payloads:
            self.duplicates += 1
            return
        for k in old:
            del self.pending[k]
            self.superseded += 1
        for i, p in enumerate(parts):
            self._add(tr.Outgoing(rx.GEOFENCE_INFO, p, ("fence", i), None,
                                  {"epoch": p[0], "part": i, "parts": len(parts)}), now + 0.25 * i)

    def _add(self, out: tr.Outgoing, now: float) -> None:
        old = self.pending.get(out.key)
        if old is not None:
            if old.out.payload == out.payload:
                self.duplicates += 1
                if out.msg_id == rx.MOVING_OBJECT_ALERT:
                    old.expires_at = now + self.ocs.moving_ttl_s
                return
            self.superseded += 1
            if out.msg_id == rx.MOVING_OBJECT_ALERT:          # latest wins, but stays at 1 Hz
                old.out, old.expires_at = out, now + self.ocs.moving_ttl_s
                old.next_due = max(now, (old.last_sent or -1e9) + old.policy.fast_s)
                return
            del self.pending[out.key]
        if out.msg_id == rx.KEEP_OUT_ZONE and out.key not in self.pending and \
                sum(1 for k in self.pending if k[0] == "zone") >= self.ocs.max_zones:
            raise tr.Refusal("%d keep-out zones are already active; this one was not sent"
                             % self.ocs.max_zones)
        policy = POLICIES[out.msg_id]
        p = _Pending(out, policy, now, now)
        if out.msg_id == rx.KEEP_OUT_ZONE:
            p.expires_at = now + self.ocs.zone_ttl_s
        elif out.msg_id == rx.MOVING_OBJECT_ALERT:
            p.expires_at = now + self.ocs.moving_ttl_s
        elif policy.retried:
            p.give_up_at = now + self.ocs.retry_give_up_s
        self.pending[out.key] = p

    def _route_ok(self) -> bool:
        """The reverse path's own rule (hub.py): transmit only toward a vehicle heard on this
        link within the route age, so nothing is keyed into an empty radio."""
        for sysid in (self.cfg.companion_sysid, self.cfg.autopilot_sysid):
            age = self.hub.heard_age(self.cfg.link, sysid)
            if age is not None and age <= self.hub.route_age_s:
                return True
        return False

    def _send_rxl(self, msg_id: int, payload: bytes, now: float) -> Optional[int]:
        """One RXL message to the companion, if there is anyone to hear it. The MAVLink sequence
        it went out with, or None -- counted as tx_no_route (nobody has been heard on the link
        lately) or tx_refused (the link would not take it)."""
        if not self._route_ok():
            self.tx_no_route += 1
            return None
        seq = self.sender.send(self.cfg.link, payload, rx.payload_type(msg_id), self.cfg.companion_sysid,
                               "adapter:" + self.cfg.vehicle_id, now)
        if seq is None:
            self.tx_refused += 1
        return seq

    def service(self, now: float) -> None:
        """Send whatever is due; retire what is finished, expired or given up on."""
        for key in list(self.pending):
            p = self.pending[key]
            if p.expires_at is not None and now >= p.expires_at:
                del self.pending[key]
                self.expired += 1
                log.info("ocs %s: %s %s expired", self.cfg.vehicle_id, p.name, key[1:] or "")
                continue
            if p.give_up_at is not None and now >= p.give_up_at:
                del self.pending[key]
                self.give_ups += 1
                log.warning("ocs %s: gave up on %s %s after %.0f s without an ACK", self.cfg.vehicle_id,
                            p.name, key[1:] or "", now - p.created)
                continue
            if now < p.next_due:
                continue
            seq = self._send_rxl(p.out.msg_id, p.out.payload, now)
            if seq is None:
                p.next_due = now + 0.5
                continue
            if p.sent and p.policy.retried:
                self.retries += 1
            p.seqs.append(seq)
            p.sent += 1
            p.last_sent = now
            self.sends += 1
            self.sent_by_type[p.name] += 1
            interval = p.policy.interval(p.sent)
            if interval is None:
                del self.pending[key]
            else:
                p.next_due = now + interval

    # ------------------------------------------------------------------ ACKs, answers

    def _ack(self, cid: int, seq: int, now: float) -> None:
        """Acknowledge an uplink message the bridge has taken."""
        if self._send_rxl(rx.ACK, rx.pack_ack(msg_id=cid, seq=seq, src=self.cfg.node), now) is not None:
            self.acks_sent += 1
            self.sent_by_type["ACK"] += 1

    def _on_ack(self, ack: dict, now: float) -> None:
        """The companion acknowledges one of our frames: msg_id and the MAVLink sequence of
        the attempt it received (any of the last few attempts will do)."""
        for key, p in self.pending.items():
            if p.out.msg_id == ack["msg_id"] and ack["seq"] in p.seqs and ack["src"] == rx.NODE_OCS:
                self.acks_matched += 1
                self._settle(key, p, p.policy.ack)
                return
        self.acks_unmatched += 1

    def _answered(self, cmd_seq: int) -> None:
        """An INCIDENT_ACK quoting a command's cmd_seq answers that command."""
        for key, p in list(self.pending.items()):
            if p.out.cmd_seq == cmd_seq and p.policy.answer:
                self._settle(key, p, p.policy.answer)

    def _settle(self, key: tuple, p: _Pending, how: str) -> None:
        if how == "finish":
            self.pending.pop(key, None)
        elif how == "burst":
            p.sent = max(p.sent, p.policy.fast_n)
            p.next_due = max(p.next_due, (p.last_sent or 0.0) + (p.policy.slow_s or 0.0))

    # ------------------------------------------------------------------------ status

    def status(self, now: float, tunnel_log: Optional[str]) -> dict:
        def age(t):
            return None if t is None else round(now - t, 2)
        last = self.last_uplink
        queue = [{"msg": p.name, "key": list(k[1:]), "sent": p.sent,
                  "next_in_s": round(max(0.0, p.next_due - now), 1),
                  "age_s": round(now - p.created, 1)} for k, p in self.pending.items()]
        return {
            "vehicle_id": self.cfg.vehicle_id, "type": self.cfg.vehicle_type, "node": self.cfg.node,
            "link": self.cfg.link, "companion_sysid": self.cfg.companion_sysid,
            "autopilot_sysid": self.cfg.autopilot_sysid,
            "bridge": self.bridge.status() if hasattr(self.bridge, "status") else {"connected": self.bridge.connected},
            "uplink": {"last_report_type": None if last is None else last[1],
                       "last_report_age_s": None if last is None else age(last[0]),
                       "by_type": dict(self.uplink_by_type), "forwarded": dict(self.forwarded),
                       "not_forwarded_no_bridge": self.not_forwarded_no_bridge,
                       "decode_errors": self.decode_errors, "last_decode_error": self.last_decode_error,
                       "refused": self.refused, "last_refusal": self.last_refusal,
                       "duplicates": self.duplicates_in,
                       "report_seq": self._report_seq, "readiness_seqs_held": len(self._readiness),
                       "ignored": {"foreign_tunnel": self.foreign, "not_addressed_to_ocs": self.not_for_us}},
            "heartbeat": {"source": self.hb_source, "reason": self.hb_reason, "rate_hz": self.cfg.rate_hz,
                          "sent": self.hb_sent, "suppressed": self.hb_suppressed,
                          "last_sent_age_s": age(self.hb_last_sent), "phase_source": self.phase_source,
                          "current_task": CURRENT_TASK if self.hb_source == "synthesized" else None,
                          "companion_age_s": age(self._companion_t),
                          "phase_fallback": self.cfg.phase_fallback,
                          "inputs": self.synth.inputs_status(now),
                          "flight_state_request": {
                              "state": self.fs_state, "sent": self.fs_sent, "refused": self.fs_refused,
                              "last_sent_age_s": age(self.fs_last_sent), "message_id": MSG_EXTENDED_SYS_STATE,
                              "interval_us": REQUEST_INTERVAL_US, "retry_after_s": REQUEST_STALE_S}},
            "downlink": {"queue": queue, "queue_depth": len(queue), "sent": dict(self.sent_by_type),
                         "sends": self.sends, "retries": self.retries, "acks_matched": self.acks_matched,
                         "acks_unmatched": self.acks_unmatched, "acks_sent": self.acks_sent,
                         "give_ups": self.give_ups, "superseded": self.superseded, "expired": self.expired,
                         "cancelled": self.cancelled, "duplicates_from_bridge": self.duplicates,
                         "tx_refused": self.tx_refused, "tx_no_route": self.tx_no_route,
                         "not_for_this_vehicle": self.not_for_vehicle, "unknown_frames": self.unknown_frames,
                         "encode_errors": self.encode_errors, "last_encode_error": self.last_encode_error,
                         "refusals_by_reason": dict(self.refusals),
                         "directives_ignored": dict(self.directives), "run_id": self.run_id,
                         "course": self._known("course"), "geofence": self._known("fence")},
            "fence_upload": (dict(self.fence.status(now), reports_to_bridge=self.fence_reports)
                             if self.fence is not None else {"enabled": False}),
            "tunnel_log": tunnel_log,
        }

    def _known(self, kind: str) -> dict:
        found = [p for k, p in self.pending.items() if k[0] == kind]
        if not found:
            return {"known": False}
        info = found[0].out.info
        return {"known": True, "epoch": info.get("epoch"),
                "parts": len(found) if kind == "fence" else None,
                "corners": info.get("corners"), "repeats_every_s": found[0].policy.slow_s}


def now_to_wall(wall: Callable[[], float], clock: Callable[[], float], at: float) -> float:
    """Wall time of a past monotonic instant `at`: the data's own age is kept, not
    flattened to now."""
    return wall() - (clock() - at)


# ------------------------------------------------------------------------ the adapter

class OcsAdapter:
    """The adapter as fleet_link runs it: an observer on the hub, a tick thread, one bridge
    connection per radio vehicle, and a `status()` for /fleet."""

    def __init__(self, cfg: OcsCfg, hub: Hub, *, clock: Optional[Callable[[], float]] = None,
                 wall: Callable[[], float] = time.time, tunnel_log=None,
                 bridge_factory: Optional[Callable[[RadioVehicleCfg, Callable[[dict], None]], object]] = None):
        for v in cfg.vehicles:
            if v.link not in hub.links:
                raise OcsConfigError("[[ocs.radio_vehicle]] %s: link %r is not defined (links: %s)"
                                     % (v.vehicle_id, v.link, ", ".join(hub.links)))
        self.cfg, self.hub = cfg, hub
        self.clock = clock or hub.clock
        self.wall = wall
        self.tunnel_log = tunnel_log
        self.lock = threading.RLock()
        self.sender = Sender(hub, cfg)
        self.inbox: Deque[Tuple[RadioVehicle, Frame, float]] = collections.deque(maxlen=INBOX_MAX)
        self.inbox_overflow = 0
        self.vehicles: Dict[str, RadioVehicle] = {}
        for v in cfg.vehicles:
            holder: List[RadioVehicle] = []

            def on_frame(obj, holder=holder):
                holder[0].on_bridge_frame(obj)
            bridge = (bridge_factory(v, on_frame) if bridge_factory else
                      BridgeLink(v.vehicle_id, cfg.bridge_host, cfg.bridge_port, on_frame,
                                 reconnect_min_s=cfg.reconnect_min_s, reconnect_max_s=cfg.reconnect_max_s))
            vehicle = RadioVehicle(v, cfg, self.sender, bridge, self.lock, self.clock, wall, hub)
            holder.append(vehicle)
            self.vehicles[v.vehicle_id] = vehicle
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._poll_errors = 0
        hub.observe(self._observe, AUTOPILOT_IDS | {MSG_TUNNEL} | fu.MESSAGE_IDS)
        hub.add_status_section("ocs", self.status)

    # the hub's thread: a deque append and nothing else
    def _observe(self, event: str, where: str, frame: Frame, now: float, info: dict) -> None:
        if event != "rx":
            return
        for v in self.vehicles.values():
            if v.interested(where, frame):
                if len(self.inbox) == INBOX_MAX:
                    self.inbox_overflow += 1
                self.inbox.append((v, frame, now))

    def start(self) -> None:
        for v in self.vehicles.values():
            if hasattr(v.bridge, "start"):
                v.bridge.start()
        self._thread = threading.Thread(target=self._run, name="ocs-adapter", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(3.0)
        for v in self.vehicles.values():
            if hasattr(v.bridge, "stop"):
                v.bridge.stop()

    def _run(self) -> None:
        while not self._stop.wait(POLL_S):
            try:
                self.poll()
            except Exception:                                        # noqa: BLE001
                self._poll_errors += 1
                if self._poll_errors <= 3 or self._poll_errors % 1000 == 0:
                    log.exception("ocs adapter poll raised (%d so far)", self._poll_errors)

    def poll(self, now: Optional[float] = None) -> None:
        """Drain the radio frames the hub handed over, then do everything that is due."""
        with self.lock:
            now = self.clock() if now is None else now
            while self.inbox:
                v, frame, t = self.inbox.popleft()
                try:
                    v.consume(frame, t)
                except Exception:                                    # noqa: BLE001
                    log.exception("ocs %s: frame handler raised", v.cfg.vehicle_id)
            for v in self.vehicles.values():
                v.heartbeat_tick(now)
                v.request_flight_state(now)
                v.fence_tick(now)
                v.service(now)

    def status(self) -> dict:
        with self.lock:
            now = self.clock()
            log_path = None if self.tunnel_log is None else self.tunnel_log.status().get("path")
            return {
                "enabled": True,
                "bridge": "%s:%d" % (self.cfg.bridge_host, self.cfg.bridge_port),
                "sysid": self.cfg.sysid, "compid": self.cfg.compid,
                "tx": {"frames": self.sender.frames, "bytes": self.sender.bytes,
                       "bytes_per_s": round(self.sender.meter.rate(now), 2)},
                "inbox_overflow": self.inbox_overflow,
                "vehicles": {vid: v.status(now, log_path) for vid, v in self.vehicles.items()},
                "tunnel_log": None if self.tunnel_log is None else self.tunnel_log.status(),
            }


def disabled_status(reason: str) -> dict:
    return {"enabled": False, "reason": reason}
