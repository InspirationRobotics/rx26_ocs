"""fence_upload -- the declared UAV geofence onto the aircraft's autopilot, over the radio.

    bridge.toml uav_geofence --ocs_info--> ocs_adapter --MISSION_* (type FENCE)--> COM8 --> ArduCopter
                                                         ^ this file drives that dialog

The bridge declares the polygon to RoboCommand (RunDeclaration.uav_geofence) and hands the same
polygon to the vehicle as `ocs_info.uav_geofence`; the adapter relays it to Ekko's companion as
RXL GEOFENCE_INFO. With `fence_upload = true` on the radio vehicle (fleet.toml) the adapter ALSO
writes it to the autopilot as an ArduPilot inclusion fence, so the fence the autopilot enforces
is the fence that was declared -- and QGC shows it (Plan view, Download).

THE REFERENCE IS EKKO'S OWN UPLOADER -- rx26_uav uav_common/fence_core.py, behind the
/uav/fence_upload service -- and its rules are kept, not reinvented:
  * never while armed, and never while the armed state is unknown;
  * never sets FENCE_ENABLE or any other parameter: enabling a fence is a ground-station act;
  * the closing vertex is stripped (the declaration is closed, ArduPilot closes it itself), and
    every vertex carries param1 = the polygon's vertex count;
  * a fence the autopilot does not echo back does not exist: every write is read back and
    compared -- count, vertex count, and every vertex within 1 m.

WHAT IT ADDS, because it runs over a lossy radio and on its own initiative rather than when a
person calls a service:
  * READ FIRST. The autopilot's fence is downloaded before anything is written. Already this
    polygon: nothing is written. Anything else: it is saved to run/fence/ (every item, raw)
    BEFORE it is replaced -- a field fence drawn in QGC is never lost to an automatic upload --
    and if it cannot be saved, it is not replaced.
  * NEVER LOCK THE AIRCRAFT OUT. An inclusion polygon the aircraft is outside of fails pre-arm
    ("Vehicle breaching") when the fence is enabled, and breaches straight after takeoff when it
    auto-enables (AC_Fence: an auto-enabled fence is only checked once enabled). So FENCE_TYPE,
    FENCE_ENABLE and FENCE_AUTOENABLE are READ each attempt (never written), and when the polygon
    can engage the write waits until the aircraft is known to be inside it. A parameter that
    does not answer counts as "can engage". With polygon fencing off the polygon is written
    wherever the aircraft is: the bench case, a Singapore fence shown in QGC in Sarasota.
    `may_block_arming` (fleet.toml fence_may_block_arming) is the operator saying "I accept it
    will not arm": then a polygon is written over FENCE_ENABLE 1 even with the aircraft outside --
    pre-arm refuses, so nothing flies (Copter runs the fence check while disarmed precisely so
    pre-arm sees the breach). It never covers an aircraft that could still ARM and only meet the
    fence after takeoff (FENCE_ENABLE 0 with FENCE_AUTOENABLE on, or FENCE_ENABLE unanswered).
  * STEP RETRIES. Every request is re-sent if its answer has not come within a step timeout
    (ArduPilot itself re-requests a missing upload item about once a second); an attempt that
    still fails is retried whole RETRY_S later, at most MAX_ATTEMPTS times per polygon.
  * ONE WRITE PER POLYGON PER RUN. Once verified it is left alone for the rest of fleet_link's
    run: a fence the operator later changes in QGC is theirs, and this does not fight it. A new
    polygon from the bridge starts over; a withdrawn one (None) stops, and the autopilot's fence
    is left exactly as it is -- this never clears a fence.

No threads, no I/O but the backup file: the adapter feeds it the autopilot's frames (note) and
ticks it (tick), both under the adapter's lock, and every frame goes out through `send`. Frames
are packed here by hand (mavframe.pack_frame adds the header and CRC); the tests cross-check each
layout against pymavlink in both directions.
"""
from __future__ import annotations

import json
import logging
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from .mavframe import Frame

log = logging.getLogger("fleet")

MSG_PARAM_REQUEST_READ = 20
MSG_PARAM_VALUE = 22
MSG_MISSION_REQUEST = 40              # deprecated, same layout as _INT; answered all the same
MSG_MISSION_REQUEST_LIST = 43
MSG_MISSION_COUNT = 44
MSG_MISSION_ACK = 47
MSG_MISSION_REQUEST_INT = 51
MSG_MISSION_ITEM_INT = 73
#: What the uploader reads off the radio -- from the autopilot only (see note()).
MESSAGE_IDS = frozenset((MSG_PARAM_VALUE, MSG_MISSION_REQUEST, MSG_MISSION_COUNT, MSG_MISSION_ACK,
                         MSG_MISSION_REQUEST_INT, MSG_MISSION_ITEM_INT))

MISSION_TYPE_FENCE = 1                # MAV_MISSION_TYPE_FENCE
MAV_FRAME_GLOBAL = 0                  # what fence_core sends, as QGC does
CMD_FENCE_RETURN_POINT = 5000
CMD_FENCE_POLYGON_VERTEX_INCLUSION = 5001
CMD_FENCE_POLYGON_VERTEX_EXCLUSION = 5002
CMD_FENCE_CIRCLE_INCLUSION = 5003
CMD_FENCE_CIRCLE_EXCLUSION = 5004
_ITEM_NAMES = {CMD_FENCE_RETURN_POINT: "return point",
               CMD_FENCE_POLYGON_VERTEX_INCLUSION: "inclusion-polygon vertex",
               CMD_FENCE_POLYGON_VERTEX_EXCLUSION: "exclusion-polygon vertex",
               CMD_FENCE_CIRCLE_INCLUSION: "inclusion circle",
               CMD_FENCE_CIRCLE_EXCLUSION: "exclusion circle"}

ACK_ACCEPTED = 0
MISSION_RESULTS = {0: "ACCEPTED", 1: "ERROR", 2: "UNSUPPORTED_FRAME", 3: "UNSUPPORTED", 4: "NO_SPACE",
                   5: "INVALID", 6: "INVALID_PARAM1", 7: "INVALID_PARAM2", 8: "INVALID_PARAM3",
                   9: "INVALID_PARAM4", 10: "INVALID_PARAM5_X", 11: "INVALID_PARAM6_Y", 12: "INVALID_PARAM7",
                   13: "INVALID_SEQUENCE", 14: "DENIED", 15: "OPERATION_CANCELLED"}

#: Read, never written. FENCE_TYPE bit 2 (value 4) is "inclusion/exclusion circles + polygons".
PARAMS = ("FENCE_TYPE", "FENCE_ENABLE", "FENCE_AUTOENABLE")
FENCE_TYPE_POLYGON = 4

#: fence_core.DEFAULT_TOLERANCE_M: the int32 1e-7 deg wire loses about a centimetre.
TOLERANCE_M = 1.0
_EARTH_R_M = 6371008.8

# ------------------------------------------------------------------------- wire layouts
# MAVLink puts fields on the wire sorted by size (stable), then the extension fields in
# declaration order; trailing extensions may be left off. tests/test_fence_upload.py checks every
# one of these against pymavlink's own encoder and decoder.
_PARAM_REQUEST_READ = struct.Struct("<hBB16s")      # param_index, target_system, target_component, param_id
_PARAM_VALUE = struct.Struct("<fHH16sB")            # param_value, param_count, param_index, param_id, param_type
_MISSION_REQUEST_LIST = struct.Struct("<BBB")       # target_system, target_component | mission_type
_MISSION_COUNT = struct.Struct("<HBBB")             # count, target_system, target_component | mission_type
_MISSION_REQUEST = struct.Struct("<HBBB")           # seq, target_system, target_component | mission_type (40 and 51)
_MISSION_ACK = struct.Struct("<BBBB")               # target_system, target_component, type | mission_type
_MISSION_ITEM_INT = struct.Struct("<4fiifHHBBBBBB")  # param1-4, x, y, z, seq, command, target_system,
#                                                      target_component, frame, current, autocontinue | mission_type


class FenceError(ValueError):
    """A polygon that cannot be put on an autopilot as it stands."""


def _fields(s: struct.Struct, frame: Frame) -> tuple:
    """A v2 sender trims trailing zero bytes: pad back to the layout before reading."""
    body = frame.payload
    if len(body) < s.size:
        body = body + bytes(s.size - len(body))
    return s.unpack_from(body, 0)


def pack_param_request_read(name: str, target_system: int, target_component: int) -> bytes:
    return _PARAM_REQUEST_READ.pack(-1, target_system, target_component, name.encode("ascii"))


def pack_mission_request_list(target_system: int, target_component: int) -> bytes:
    return _MISSION_REQUEST_LIST.pack(target_system, target_component, MISSION_TYPE_FENCE)


def pack_mission_count(count: int, target_system: int, target_component: int) -> bytes:
    return _MISSION_COUNT.pack(count, target_system, target_component, MISSION_TYPE_FENCE)


def pack_mission_request_int(seq: int, target_system: int, target_component: int) -> bytes:
    return _MISSION_REQUEST.pack(seq, target_system, target_component, MISSION_TYPE_FENCE)


def pack_mission_ack(result: int, target_system: int, target_component: int) -> bytes:
    return _MISSION_ACK.pack(target_system, target_component, result, MISSION_TYPE_FENCE)


def pack_fence_vertex(seq: int, lat: float, lon: float, vertex_count: int,
                      target_system: int, target_component: int) -> bytes:
    """One inclusion-polygon vertex as MISSION_ITEM_INT -- fence_core.MavFenceTransport.send_item:
    frame GLOBAL, current 0, autocontinue 0, param1 = the vertex count of THIS polygon."""
    return _MISSION_ITEM_INT.pack(float(vertex_count), 0.0, 0.0, 0.0,
                                  int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0,
                                  seq, CMD_FENCE_POLYGON_VERTEX_INCLUSION, target_system, target_component,
                                  MAV_FRAME_GLOBAL, 0, 0, MISSION_TYPE_FENCE)


def param_value(frame: Frame) -> Tuple[str, float]:
    value, _count, _index, raw_id, _ptype = _fields(_PARAM_VALUE, frame)
    return raw_id.split(b"\x00", 1)[0].decode("ascii", "replace"), value


def mission_item(frame: Frame) -> dict:
    """A downloaded fence item, every field kept: it is what a backup is made of."""
    p1, p2, p3, p4, x, y, z, seq, command, tsys, _tcomp, mframe, _cur, _auto, mtype = \
        _fields(_MISSION_ITEM_INT, frame)
    return {"seq": seq, "command": command, "frame": mframe, "param1": p1, "param2": p2, "param3": p3,
            "param4": p4, "lat": x * 1e-7, "lon": y * 1e-7, "alt": z,
            "target_system": tsys, "mission_type": mtype}


# ------------------------------------------------------------------------------ geometry

def open_polygon(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    """The closed declaration -> the open polygon ArduPilot wants (fence_core.items_from_polygon)."""
    pts = []
    for i, p in enumerate(points):
        lat, lon = float(p[0]), float(p[1])
        if not (math.isfinite(lat) and math.isfinite(lon)) or not (-90 <= lat <= 90 and -180 <= lon <= 180):
            raise FenceError("geofence vertex %d (%r, %r) is not a latitude/longitude" % (i, p[0], p[1]))
        pts.append((lat, lon))
    if len(pts) >= 2 and pts[0] == pts[-1]:
        pts = pts[:-1]                     # ArduPilot closes a polygon itself
    if len(pts) < 3:
        raise FenceError("geofence has %d distinct vertices after removing the closing point; a polygon "
                         "needs at least 3" % len(pts))
    return pts


def offset_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Great-circle distance a-b in metres (haversine). Right at a metre and across an ocean: the
    first live run printed 23738 km from California to Singapore with a flat-earth formula that did
    not wrap longitude through +/-180."""
    la1, la2 = math.radians(a[0]), math.radians(b[0])
    h = (math.sin((la2 - la1) / 2.0) ** 2
         + math.cos(la1) * math.cos(la2) * math.sin(math.radians(b[1] - a[1]) / 2.0) ** 2)
    return 2.0 * _EARTH_R_M * math.asin(min(1.0, math.sqrt(h)))


def inside(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> bool:
    """Ray casting on lat/lon -- a polygon a few hundred metres across is flat enough."""
    y, x = point
    hit = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        yi, xi = polygon[i]
        yj, xj = polygon[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            hit = not hit
        j = i
    return hit


def matches(items: Sequence[dict], polygon: Sequence[Tuple[float, float]]) -> Optional[str]:
    """None when the downloaded items are exactly this one inclusion polygon (fence_core's
    readback_verify), else what differs."""
    n = len(polygon)
    if len(items) != n:
        return "the autopilot holds %d fence item(s), the polygon has %d vertices" % (len(items), n)
    for item, want in zip(items, polygon):
        if item["command"] != CMD_FENCE_POLYGON_VERTEX_INCLUSION:
            return "item %d is a %s" % (item["seq"], _ITEM_NAMES.get(item["command"], "command %d" % item["command"]))
        if int(item["param1"]) != n:
            return "item %d says its polygon has %d vertices, not %d" % (item["seq"], int(item["param1"]), n)
        d = offset_m(want, (item["lat"], item["lon"]))
        if d > TOLERANCE_M:
            return "vertex %d is %.1f m from the declared one" % (item["seq"], d)
    return None


def shapes(items: Sequence[dict]) -> List[dict]:
    """Downloaded fence items -> what a map draws: {"type": "polygon", "inclusion", "points"} and
    {"type": "circle", "inclusion", "center", "radius_m"}. Polygon vertices are grouped by their
    param1 count, as ArduPilot groups them; a group that does not add up is left out rather than
    drawn as a wrong shape. The breach return point is not a boundary and is skipped."""
    out: List[dict] = []
    k = 0
    while k < len(items):
        item = items[k]
        cmd = item["command"]
        if cmd in (CMD_FENCE_CIRCLE_INCLUSION, CMD_FENCE_CIRCLE_EXCLUSION):
            out.append({"type": "circle", "inclusion": cmd == CMD_FENCE_CIRCLE_INCLUSION,
                        "center": [item["lat"], item["lon"]], "radius_m": item["param1"]})
            k += 1
        elif cmd in (CMD_FENCE_POLYGON_VERTEX_INCLUSION, CMD_FENCE_POLYGON_VERTEX_EXCLUSION):
            n = int(item["param1"])
            group = items[k:k + n]
            if n >= 3 and len(group) == n and all(g["command"] == cmd and int(g["param1"]) == n for g in group):
                out.append({"type": "polygon", "inclusion": cmd == CMD_FENCE_POLYGON_VERTEX_INCLUSION,
                            "points": [[g["lat"], g["lon"]] for g in group]})
                k += n
            else:
                k += 1
        else:
            k += 1
    return out


def describe(items: Sequence[dict]) -> str:
    """What an autopilot's fence is, in words for the log and /fleet."""
    if not items:
        return "no fence items"
    counts: Dict[str, int] = {}
    for item in items:
        name = _ITEM_NAMES.get(item["command"], "command %d" % item["command"])
        counts[name] = counts.get(name, 0) + 1
    def plural(name: str, n: int) -> str:
        return name if n == 1 else name[:-2] + "ices" if name.endswith("vertex") else name + "s"
    return ", ".join("%d %s" % (n, plural(name, n)) for name, n in counts.items())


# ------------------------------------------------------------------------- the uploader

@dataclass(frozen=True)
class Timing:
    step_s: float = 1.5               # a request unanswered this long is sent again
    write_step_s: float = 3.0         # an upload waits longer: ArduPilot re-requests on its own each ~1 s
    tries: int = 4                    # sends of one request before the attempt fails
    retry_s: float = 30.0             # a failed attempt is retried this much later
    max_attempts: int = 5             # failed attempts per polygon before giving up
    hold_recheck_s: float = 30.0      # held: the FENCE_* params are asked again this often
    recheck_s: float = 60.0           # done: the autopilot's fence is read back again this often
    clear_grace_s: float = 20.0       # "no course": wait this long for one before clearing


class FenceUploader:
    """One radio vehicle's autopilot fence. Phases of an attempt: params -> (gate) -> read ->
    write -> verify. `state` is what /fleet and the log say; `_phase` is the dialog in flight.

    `armed(now)` and `position(now)` are the autopilot's own, fresh, or None; `route_ok()` says the
    radio has heard the vehicle lately. They are asked at every tick AND before every frame of a
    write: a HEARTBEAT that says armed and the autopilot's next MISSION_REQUEST can arrive in the
    same batch, and the vertex must not go out in between."""

    def __init__(self, *, label: str, target_sysid: int, target_compid: int, our_sysid: int,
                 send: Callable[[int, bytes, float], bool], armed: Callable[[float], Optional[bool]],
                 position: Callable[[float], Optional[Tuple[float, float]]], route_ok: Callable[[], bool],
                 backup_dir: Optional[str], may_block_arming: bool = False,
                 wall: Callable[[], float] = time.time, timing: Timing = Timing()):
        self.label, self.ts, self.tc, self.our_sysid = label, target_sysid, target_compid, our_sysid
        self._send_fn, self.backup_dir, self.wall, self.t = send, backup_dir, wall, timing
        self._armed, self._position_of, self._route_ok = armed, position, route_ok
        self.may_block_arming = may_block_arming
        #: Set when the polygon was cleared to go only because may_block_arming accepts that the
        #: aircraft will not arm where it is: what /fleet and the map say about it.
        self.lockout: Optional[str] = None
        self._read_items: Optional[List[dict]] = None   # the last download, as the autopilot held it
        self._read_at: Optional[float] = None
        self.polygon: Optional[List[Tuple[float, float]]] = None
        self.state, self.reason = "off", "no geofence from the bridge yet"
        self.attempts = self.failures = self.writes = self.frames = self.send_refused = 0
        self.last_error: Optional[str] = None
        self.backup: Optional[str] = None
        self.autopilot_fence: Optional[str] = None     # describe() of the last download
        self.done_at: Optional[float] = None
        #: When the autopilot was last READ holding exactly `polygon` (monotonic, and wall for the
        #: bridge): what the `ocs_fence` confirmation and the bridge's pre-run gate rest on.
        self.verified_at: Optional[float] = None
        self.verified_wall: Optional[float] = None
        self.rechecks = 0
        self._recheck_at = math.inf
        self._next_at = 0.0
        #: name -> (value, when heard). Heard at any time -- ArduPilot broadcasts a parameter that
        #: is set -- but only a value heard since `_params_since` counts for the gate.
        self._params: Dict[str, Tuple[float, float]] = {}
        self._params_since = math.inf
        self._phase: Optional[str] = None               # "params" | "read" | "write" | "verify" | "recheck"
        self._resend: Optional[Callable[[float], None]] = None
        self._step_s = timing.step_s
        self._deadline = math.inf
        self._tries = 0
        self._count: Optional[int] = None               # download: the autopilot's item count
        self._items: List[dict] = []
        self._requested: set = set()                    # upload: seqs the autopilot asked for

    # ------------------------------------------------------------------------ inputs

    def set_polygon(self, points: Optional[Sequence[Tuple[float, float]]], now: float) -> None:
        """The geofence from the bridge's `ocs_info` (closed), or None. Raises FenceError."""
        if points is None:
            if self.polygon is not None:
                self._abort()
                self.polygon = None
                self._set("off", "the bridge withdrew the geofence; the autopilot's fence is left as it is",
                          logging.INFO)
            return
        polygon = open_polygon(points)
        if polygon == self.polygon:
            return
        self._retarget(polygon, now)
        log.info("fence %s: %d-vertex geofence from the bridge; will put it on the autopilot when it is "
                 "disarmed", self.label, len(polygon))
        self._set("waiting", "starting")

    def set_clear(self, now: float) -> None:
        """The bridge has no course (it has heard none from RoboCommand since it started): the
        autopilot should hold NO fence items, so the team can fly without a RoboCommand server.
        Not at once: an OCS that has just started gets its (retained) course within seconds, and
        that must not cost the aircraft a clear-then-write cycle -- so only after clear_grace_s
        with nothing newer from the bridge. Then exactly like a write: disarmed only, the FENCE_*
        params read, the fence read first and saved, an empty fence written and read back empty."""
        if self.polygon == []:
            return
        self._retarget([], now + self.t.clear_grace_s)
        log.info("fence %s: no course from RoboCommand; the autopilot's fence will be cleared in %.0f s "
                 "unless a course arrives first", self.label, self.t.clear_grace_s)
        self._set("waiting", "no course from RoboCommand: the autopilot's fence is cleared once it is "
                             "disarmed, unless a course arrives first")

    def _retarget(self, polygon: List[Tuple[float, float]], start_at: float) -> None:
        self._abort()
        self.polygon = polygon
        self.attempts = self.failures = 0
        self.last_error = self.backup = self.done_at = self.lockout = None
        self.verified_at = self.verified_wall = None
        self._recheck_at = math.inf
        self._next_at = start_at

    def note(self, frame: Frame, now: float) -> None:
        """One frame off the radio. Only the autopilot's, only CRC-checked ones, and only mission
        traffic addressed to us: QGC fetching its own copy over the same radio is not our answer."""
        if frame.sysid != self.ts or frame.compid != self.tc or not frame.validated:
            return
        if frame.msgid == MSG_PARAM_VALUE:
            name, value = param_value(frame)
            if name in PARAMS:
                self._on_param(name, value, now)
            return
        if frame.msgid in (MSG_MISSION_REQUEST, MSG_MISSION_REQUEST_INT):
            seq, tsys, _tc, mtype = _fields(_MISSION_REQUEST, frame)
            if tsys == self.our_sysid and mtype == MISSION_TYPE_FENCE:
                self._on_request(seq, now)
        elif frame.msgid == MSG_MISSION_COUNT:
            count, tsys, _tc, mtype = _fields(_MISSION_COUNT, frame)
            if tsys == self.our_sysid and mtype == MISSION_TYPE_FENCE:
                self._on_count(count, now)
        elif frame.msgid == MSG_MISSION_ITEM_INT:
            item = mission_item(frame)
            if item["target_system"] == self.our_sysid and item["mission_type"] == MISSION_TYPE_FENCE:
                self._on_item(item, now)
        elif frame.msgid == MSG_MISSION_ACK:
            tsys, _tc, result, mtype = _fields(_MISSION_ACK, frame)
            if tsys == self.our_sysid and mtype == MISSION_TYPE_FENCE:
                self._on_ack(result, now)

    def tick(self, now: float) -> None:
        """Called every adapter poll."""
        if self.polygon is None or self.state == "gave up":
            return
        if self.state == "done":
            return self._recheck_tick(now)
        if not self._clear_to_talk(now):
            return
        if self._phase is not None:
            if now >= self._deadline:
                if self._tries < self.t.tries:
                    self._fire(now)
                else:
                    self._timed_out(now)
            return
        if self.state == "held":
            why = self._gate(self._position_of(now))
            if why is None:
                log.info("fence %s: no longer held - the polygon cannot lock the aircraft out now", self.label)
                return self._start_read(now)
            self.reason = why
            if now >= self._next_at:
                self._start_params(now)      # ask the params again: QGC may have changed them
            return
        if now < self._next_at:
            return
        self.attempts += 1
        self._start_params(now)

    # ------------------------------------------------------------------------- phases

    def _start_params(self, now: float) -> None:
        self._params_since = now
        self._begin("params", now, self.t.step_s, self._ask_params)
        self._set_state_quietly("reading the FENCE_* parameters")

    def _ask_params(self, now: float) -> None:
        for name in PARAMS:
            if self._param(name) is None:
                self._send(MSG_PARAM_REQUEST_READ, pack_param_request_read(name, self.ts, self.tc), now)

    def _on_param(self, name: str, value: float, now: float) -> None:
        old = self._params.get(name)
        self._params[name] = (value, now)
        if old is not None and old[0] != value:
            log.info("fence %s: %s changed %g -> %g on the autopilot", self.label, name, old[0], value)
            if self.state == "gave up":      # e.g. the polygon bit was just set in QGC
                self.attempts = self.failures = 0
                self._next_at = now
                self._set("waiting", "%s changed; trying again" % name, logging.INFO)
        if self._phase == "params" and all(self._param(n) is not None for n in PARAMS):
            self._params_done(now)

    def _params_done(self, now: float) -> None:
        self._phase, self._deadline = None, math.inf
        why = self._gate(self._position_of(now))
        if why is None:
            return self._start_read(now)
        # Held: tick() re-runs the gate with the live position, and asks the params again
        # every hold_recheck_s.
        self._next_at = now + self.t.hold_recheck_s
        self._set("held", why, logging.WARNING)

    def _start_read(self, now: float) -> None:
        self._start_download("read", now, "reading the autopilot's fence")

    def _start_verify(self, now: float) -> None:
        self._start_download("verify", now, "reading the fence back")

    def _start_download(self, phase: str, now: float, doing: Optional[str]) -> None:
        self._count, self._items = None, []
        self._begin(phase, now, self.t.step_s, self._ask_list)
        if doing is not None:
            self._set_state_quietly(doing)

    def _start_write(self, now: float) -> None:
        self._requested = set()
        self._begin("write", now, self.t.write_step_s, self._ask_count)
        self._set_state_quietly("writing %d vertices" % len(self.polygon) if self.polygon
                                else "clearing the autopilot's fence")

    def _recheck_tick(self, now: float) -> None:
        """Done: read the fence back again every recheck_s while the aircraft is disarmed and heard,
        so a confirmation is never older than that and a fence changed behind our back (QGC, the
        companion's own /uav/fence_upload) is seen. A re-check that cannot run or gets no answer
        changes nothing but its age -- reads are not urgent; the bridge's gate judges the age."""
        if self._phase == "recheck":
            if self._armed(now) is not False or not self._route_ok():
                self._abort()
                self._recheck_at = now + self.t.recheck_s
            elif now >= self._deadline:
                if self._tries < self.t.tries:
                    self._fire(now)
                else:
                    self._abort()
                    self._recheck_at = now + self.t.recheck_s
                    log.info("fence %s: re-check got no answer; the confirmation is now %.0f s old",
                             self.label, now - self.verified_at)
            return
        if now >= self._recheck_at and self._armed(now) is False and self._route_ok():
            self.rechecks += 1
            self._start_download("recheck", now, None)

    def _ask_list(self, now: float) -> None:
        if self._count is None:
            self._send(MSG_MISSION_REQUEST_LIST, pack_mission_request_list(self.ts, self.tc), now)
        else:
            self._send(MSG_MISSION_REQUEST_INT, pack_mission_request_int(len(self._items), self.ts, self.tc), now)

    def _ask_count(self, now: float) -> None:
        # Re-sending the count restarts the upload, which ArduPilot allows from the same sender.
        if not self._clear_to_talk(now):
            return
        self._requested = set()
        self._send(MSG_MISSION_COUNT, pack_mission_count(len(self.polygon), self.ts, self.tc), now)

    # ------------------------------------------------------------- autopilot answers

    def _on_count(self, count: int, now: float) -> None:
        if self._phase not in DOWNLOADS or self._count is not None:
            return                           # a late or repeated answer
        self._count = count
        if count == 0:
            return self._downloaded(now)     # a zero-length list is complete at the COUNT: no ACK
        self._begin(self._phase, now, self.t.step_s, self._ask_list)

    def _on_item(self, item: dict, now: float) -> None:
        if self._phase not in DOWNLOADS or self._count is None or item["seq"] != len(self._items):
            return
        self._items.append(item)
        if len(self._items) < self._count:
            return self._begin(self._phase, now, self.t.step_s, self._ask_list)
        self._send(MSG_MISSION_ACK, pack_mission_ack(ACK_ACCEPTED, self.ts, self.tc), now)
        self._downloaded(now)

    def _on_request(self, seq: int, now: float) -> None:
        if self._phase != "write" or not self._clear_to_talk(now):
            return
        if not 0 <= seq < len(self.polygon):
            return self._fail(now, "the autopilot asked for vertex %d of %d" % (seq, len(self.polygon)))
        lat, lon = self.polygon[seq]
        self._send(MSG_MISSION_ITEM_INT, pack_fence_vertex(seq, lat, lon, len(self.polygon), self.ts, self.tc), now)
        self._requested.add(seq)
        self._deadline = now + self._step_s  # progress: the autopilot is driving, give it a full step

    def _on_ack(self, result: int, now: float) -> None:
        if self._phase == "recheck":                     # not a failure: tried again at the next re-check
            self._abort()
            self._recheck_at = now + self.t.recheck_s
            return
        if self._phase in DOWNLOADS:
            return self._fail(now, "the autopilot refused the fence download: %s" % _result(result))
        if self._phase != "write":
            return
        if result != ACK_ACCEPTED:
            return self._fail(now, "the autopilot rejected the fence: %s%s" % (_result(result), self._nack_hint()))
        missing = sorted(set(range(len(self.polygon))) - self._requested)
        if missing:                          # fence_core: a premature ACK is not a success
            return self._fail(now, "ACCEPTED with vertices %s never requested" % missing)
        self.writes += 1
        self._start_verify(now)

    def _downloaded(self, now: float) -> None:
        phase, items = self._phase, self._items
        self._phase, self._deadline = None, math.inf
        self.autopilot_fence = describe(items)
        self._read_items, self._read_at = list(items), now
        diff = matches(items, self.polygon)
        if phase == "recheck":
            self._recheck_at = now + self.t.recheck_s
            if diff is None:
                self.verified_at, self.verified_wall = now, self.wall()
                return
            self.attempts = self.failures = 0
            self.done_at = self.verified_at = self.verified_wall = None
            self._next_at = now
            return self._set("waiting", "the autopilot's fence changed since it was verified (it holds %s: %s); "
                                        "putting the declared one back" % (self.autopilot_fence, diff), logging.WARNING)
        if phase == "verify":
            if diff is not None:
                return self._fail(now, "read back wrong: %s" % diff)
            if not self.polygon:
                return self._done(now, "cleared and read back empty")
            return self._done(now, "written and read back: %d vertices, each within %.0f m"
                              % (len(self.polygon), TOLERANCE_M))
        if diff is None:
            return self._done(now, "the autopilot already holds this polygon; nothing written" if self.polygon
                              else "the autopilot holds no fence items; nothing to clear")
        if items:
            try:
                self.backup = self._save_backup(items, now)
            except OSError as e:
                return self._fail(now, "could not save the autopilot's current fence (%s) before replacing "
                                       "it, so it was not replaced: %s" % (self.autopilot_fence, e))
            log.warning("fence %s: replacing the autopilot's fence (%s); saved to %s",
                        self.label, self.autopilot_fence, self.backup)
        self._start_write(now)

    # ------------------------------------------------------------------- the rules

    def _gate(self, position: Optional[Tuple[float, float]]) -> Optional[str]:
        """None when the polygon may be written now, else why not -- see the module docstring."""
        ftype, enable, auto = (self._param(n) for n in PARAMS)
        self.lockout = None
        if not self.polygon:                 # clearing: there is no polygon to be outside of
            if ftype is not None and int(ftype) == FENCE_TYPE_POLYGON and enable:
                self.lockout = ("%s will NOT ARM with its fence cleared: FENCE_TYPE 4 is polygons only and "
                                "FENCE_ENABLE is %g (pre-arm: Fences enabled, but none selected)" % (self.label, enable))
            return None
        polygon_on = ftype is None or int(ftype) & FENCE_TYPE_POLYGON
        switched = (enable is None or enable != 0) or (auto is None or auto != 0)
        if not (polygon_on and switched):
            return None
        if position is not None and inside(position, self.polygon):
            return None
        why = ", ".join(("%s %g" % (n, v)) if v is not None else "%s unanswered" % n
                        for n, v in zip(PARAMS, (ftype, enable, auto)))
        where = ("the aircraft's position is unknown" if position is None else
                 "the aircraft is outside it (%.1f km from vertex 0)" % (offset_m(position, self.polygon[0]) / 1000.0))
        if self.may_block_arming and enable is not None and enable != 0:
            # Enabled now: pre-arm sees the breach and refuses, so the aircraft stays on the ground.
            self.lockout = ("%s will NOT ARM while it is outside this polygon: polygon fencing is on (%s) and %s "
                            "(pre-arm: Vehicle breaching). Written anyway because fence_may_block_arming = true"
                            % (self.label, why, where))
            return None
        if self.may_block_arming:
            hint = (" fence_may_block_arming does not cover this: with FENCE_ENABLE %s the aircraft can still "
                    "arm, and the fence would engage after takeoff" % ("unanswered" if enable is None else "%g" % enable))
        else:
            hint = " Or set fence_may_block_arming = true in fleet.toml to write it anyway and accept that it will not arm."
        return ("NOT written: polygon fencing can engage (%s) and %s. An inclusion polygon the aircraft is "
                "outside of blocks arming, or breaches on takeoff. Set FENCE_ENABLE 0 and FENCE_AUTOENABLE 0 in "
                "QGC to load it for viewing, or bring the aircraft inside -- this never sets parameters.%s"
                % (why, where, hint))

    def _clear_to_talk(self, now: float) -> bool:
        """Disarmed for certain and heard on the radio; otherwise the dialog in flight is dropped
        -- an interruption, not a failure: the attempt starts over once it is."""
        armed = self._armed(now)
        if armed is False and self._route_ok():
            return True
        self._abort()
        self._set("waiting", "the aircraft is armed: nothing is written to an armed autopilot" if armed else
                  "the autopilot is not heard on the radio (no fresh HEARTBEAT), so its armed state is unknown",
                  logging.INFO)
        return False

    def _param(self, name: str) -> Optional[float]:
        got = self._params.get(name)
        return got[0] if got is not None and got[1] >= self._params_since else None

    def _nack_hint(self) -> str:
        ftype = self._param("FENCE_TYPE")
        if ftype is not None and not int(ftype) & FENCE_TYPE_POLYGON:
            return (" -- FENCE_TYPE %g has no polygon bit (4), and the autopilot NACKs a polygon without it "
                    "(rx26_uav docs/BRINGUP.md)" % ftype)
        return ""

    # ------------------------------------------------------------- step machinery

    def _begin(self, phase: str, now: float, step_s: float, resend: Callable[[float], None]) -> None:
        self._phase, self._resend, self._step_s, self._tries = phase, resend, step_s, 0
        self._fire(now)

    def _fire(self, now: float) -> None:
        self._tries += 1
        self._deadline = now + self._step_s
        self._resend(now)

    def _timed_out(self, now: float) -> None:
        if self._phase == "params":
            # Whatever answered counts; whatever did not stays unknown, which the gate reads as
            # "can engage". Never a failure: a firmware without one of them still gets a verdict.
            missing = [n for n in PARAMS if self._param(n) is None]
            log.info("fence %s: no answer for %s after %d request(s)", self.label, ", ".join(missing), self._tries)
            return self._params_done(now)
        what = {"read": "the fence download", "verify": "the fence read-back", "write": "the fence upload"}
        self._fail(now, "no answer during %s after %d request(s), %.1f s apart"
                   % (what[self._phase], self._tries, self._step_s))

    def _abort(self) -> None:
        """Drop the dialog in flight. ArduPilot commits an upload only when it is complete, so a
        half-sent polygon times out on the autopilot and the old fence stays."""
        self._phase, self._deadline, self._resend = None, math.inf, None

    def _fail(self, now: float, error: str) -> None:
        self._abort()
        self.failures += 1
        self.last_error = error
        if self.failures >= self.t.max_attempts:
            return self._set("gave up", "%d attempts failed; the last: %s. A new polygon from the bridge, a "
                                        "changed FENCE_* parameter or a fleet_link restart tries again"
                             % (self.failures, error), logging.WARNING)
        self._next_at = now + self.t.retry_s
        self._set("waiting", "attempt %d failed (%s); retrying in %.0f s" % (self.attempts, error, self.t.retry_s),
                  logging.WARNING)

    def _done(self, now: float, how: str) -> None:
        self._abort()
        self.done_at = now
        self.verified_at, self.verified_wall = now, self.wall()
        self._recheck_at = now + self.t.recheck_s
        if self.lockout:
            log.warning("fence %s: %s", self.label, self.lockout)
        self._set("done", how + ". FENCE_ENABLE is not touched; QGC shows it after Plan view > Download",
                  logging.INFO)

    def _set(self, state: str, reason: str, level: Optional[int] = None) -> None:
        """Change what /fleet says; log it at `level` when it is news (a repeat stays quiet)."""
        if level is not None and (state, reason) != (self.state, self.reason):
            log.log(level, "fence %s: %s - %s", self.label, state.upper(), reason)
        self.state, self.reason = state, reason

    def _set_state_quietly(self, reason: str) -> None:
        self.state, self.reason = "busy", reason

    def _send(self, msgid: int, payload: bytes, now: float) -> None:
        if self._send_fn(msgid, payload, now):
            self.frames += 1
        else:
            self.send_refused += 1           # the step timer re-sends; nothing else to do

    def _save_backup(self, items: List[dict], now: float) -> str:
        if not self.backup_dir:
            raise OSError("no backup directory is configured")
        wall = self.wall()
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(wall))
        path = Path(self.backup_dir) / ("%s-fence-%s.json" % (self.label, stamp))
        path.parent.mkdir(parents=True, exist_ok=True)
        doc = {"vehicle": self.label, "saved_at_utc": stamp, "autopilot_sysid": self.ts,
               "why": "the autopilot's fence before fleet_link replaced it with the declared geofence",
               "summary": describe(items), "items": items,
               "replaced_by": [list(p) for p in self.polygon]}
        path.write_text(json.dumps(doc, indent=1), encoding="utf-8")
        return str(path)

    # ------------------------------------------------------------------------ status

    def status(self, now: float) -> dict:
        return {"enabled": True, "state": self.state, "reason": self.reason,
                "vertices": None if self.polygon is None else len(self.polygon),
                "attempts": self.attempts, "failures": self.failures, "writes": self.writes,
                "last_error": self.last_error, "autopilot_fence": self.autopilot_fence, "backup": self.backup,
                "params": {n: (None if n not in self._params else self._params[n][0]) for n in PARAMS},
                "params_age_s": {n: round(now - self._params[n][1], 1) for n in PARAMS if n in self._params},
                "done_age_s": None if self.done_at is None else round(now - self.done_at, 1),
                "lockout": self.lockout, "may_block_arming": self.may_block_arming,
                "verified": self.verified_at is not None,
                "verified_age_s": None if self.verified_at is None else round(now - self.verified_at, 1),
                "rechecks": self.rechecks, "clearing": self.polygon == [],
                # for the OCS map: the polygon this was told to put on the autopilot, and what the
                # autopilot held at the last read (not re-read after done: its age says how old)
                "polygon": None if self.polygon is None else [list(p) for p in self.polygon],
                "autopilot_shapes": None if self._read_items is None else shapes(self._read_items),
                "autopilot_read_age_s": None if self._read_at is None else round(now - self._read_at, 1),
                "frames": self.frames, "send_refused": self.send_refused}


    def confirmation(self) -> dict:
        """The body of the `ocs_fence` frame the adapter sends the bridge (docs/VEHICLE_LINK.md):
        `verified` only while the autopilot was last READ holding exactly `polygon` ([] = cleared).
        verified_at is wall time (the adapter formats it)."""
        return {"vehicle_id": self.label, "state": self.state,
                "polygon": None if self.polygon is None else [list(p) for p in self.polygon],
                "verified": self.verified_at is not None, "verified_at": self.verified_wall,
                "reason": self.reason, "lockout": self.lockout}


#: The dialogs that download the fence (and so take MISSION_COUNT / MISSION_ITEM_INT answers).
DOWNLOADS = ("read", "verify", "recheck")


def _result(code: int) -> str:
    return "%s (%d)" % (MISSION_RESULTS.get(code, "?"), code)
