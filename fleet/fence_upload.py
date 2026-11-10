"""fence_upload -- the aircraft's autopilot fence over the radio: READ all of it, WRITE only RoboCommand's polygon.

    bridge ocs_info.uav_geofence --> ocs_adapter --MISSION_* (type FENCE)--> radio --> ArduCopter
      (RoboCommand's course, inset)
    bridge <-- ocs_fence -------- ocs_adapter <-- every fence item, the FENCE_* params, home HAE

THE WORKFLOW (Chase, 7 Nov 2026). RoboCommand sends the course. The bridge turns it into RoboCommand's
fence -- the course joined with its UAV transit area, inset by `[geofence] boundary_margin_m` -- and this
file puts that on the autopilot as an inclusion polygon, so QGC shows it (Plan view, Download). The pilot
then draws Ekko's OWN fence inside it in QGC and uploads both. The bridge declares the pilot's polygon as
read back from here, never one it made up, and the ceiling it declares is FENCE_ALT_MAX above home, read
here too. Apart from that one polygon the OCS is read-only.

ArduPilot treats two inclusion polygons as an INTERSECTION -- unless FENCE_OPTIONS bit 1 ("union of
inclusion areas") is set -- so with the pilot's polygon inside RoboCommand's, the aircraft is held to the
pilot's. FENCE_OPTIONS is read and reported so the page can say when that is not so.

READ, ALWAYS. Every `watch_s` while the aircraft is disarmed and heard, whatever the bridge has sent and
with fence_upload off: FENCE_TYPE, FENCE_ENABLE, FENCE_AUTOENABLE, FENCE_ALT_MAX and FENCE_OPTIONS are
asked for, and every fence item is downloaded. A read that ArduPilot refuses because QGC is uploading at
that moment (it DENIES a download mid-upload) is tried again a few seconds later.

WRITE, ONCE PER POLYGON, NEVER OVER THE PILOT'S FENCE. With `fence_upload = true` (fleet.toml) a polygon
from the bridge is written only when a read shows the autopilot needs it:
  * it already holds that polygon among its inclusion polygons, whatever else is beside it -> nothing
    is written;
  * it holds an inclusion polygon lying inside that polygon (within KEEP_TOLERANCE_M) -- the pilot's
    fence for this course, drawn before the course arrived or kept after RoboCommand's was deleted ->
    nothing is written and nothing replaced. A polygon this OCS wrote itself in this run (an earlier
    course's, or one the autopilot read back mangled) is never taken for the pilot's; after a fleet_link
    restart that is not known, which matters only if one course's polygon lay inside another's;
  * otherwise (no fence, or a fence for another course or a test) -> it is saved to run/fence/ and
    REPLACED by that one polygon, then read back.
Once there it is never written again in this run of fleet_link, whatever QGC does to it afterwards. A
course change (a new polygon from the bridge) starts over; the bridge withdrawing its polygon stops, and
the autopilot's fence is left exactly as it is -- this never clears a fence.

THE REFERENCE IS EKKO'S OWN UPLOADER -- rx26_uav uav_common/fence_core.py, behind the /uav/fence_upload
service -- and its rules are kept, not reinvented:
  * never while armed, and never while the armed state is unknown;
  * never sets FENCE_ENABLE or any other parameter: enabling a fence is a ground-station act;
  * the closing vertex is stripped (the declaration is closed, ArduPilot closes it itself), and every
    vertex carries param1 = the polygon's vertex count;
  * a fence the autopilot does not echo back does not exist: every write is read back and compared --
    every vertex within 1 m.

WHAT IT ADDS, because it runs over a lossy radio and on its own initiative:
  * NOTHING IS LOST. Whatever the autopilot held is saved to run/fence/ (every item, raw) BEFORE it is
    replaced, and if it cannot be saved it is not replaced.
  * NEVER LOCK THE AIRCRAFT OUT. An inclusion polygon the aircraft is outside of fails pre-arm ("Vehicle
    breaching") when the fence is enabled, and breaches straight after takeoff when it auto-enables
    (AC_Fence: an auto-enabled fence is only checked once enabled). So FENCE_TYPE, FENCE_ENABLE and
    FENCE_AUTOENABLE are READ each attempt (never written), and when the polygon can engage the write
    waits until the aircraft is known to be inside it. A parameter that does not answer counts as "can
    engage". With polygon fencing off the polygon is written wherever the aircraft is: the bench case, a
    Singapore fence shown in QGC in Sarasota. `may_block_arming` (fleet.toml fence_may_block_arming) is
    the operator saying "I accept it will not arm": then a polygon is written over FENCE_ENABLE 1 even
    with the aircraft outside -- pre-arm refuses, so nothing flies. It never covers an aircraft that could
    still ARM and only meet the fence after takeoff (FENCE_ENABLE 0 with FENCE_AUTOENABLE on, or
    FENCE_ENABLE unanswered).
  * STEP RETRIES. Every request is re-sent if its answer has not come within a step timeout (ArduPilot
    itself re-requests a missing upload item about once a second); an attempt that still fails is retried
    whole RETRY_S later, at most MAX_ATTEMPTS times per polygon.

No threads, no I/O but the backup file: the adapter feeds it the autopilot's frames (note) and ticks it
(tick), both under the adapter's lock, and every frame goes out through `send`. Frames are packed here by
hand (mavframe.pack_frame adds the header and CRC); the tests cross-check each layout against pymavlink in
both directions.
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

#: Read, never written: what the write gate needs. FENCE_TYPE bit 2 (value 4) is "inclusion/exclusion
#: circles + polygons".
PARAMS = ("FENCE_TYPE", "FENCE_ENABLE", "FENCE_AUTOENABLE")
#: Read, never written, and reported to the bridge: the gate's three, FENCE_ALT_MAX (the ceiling, metres
#: above home, enforced with FENCE_TYPE bit 0) and FENCE_OPTIONS (bit 1: inclusion areas are a union).
REPORTED_PARAMS = PARAMS + ("FENCE_ALT_MAX", "FENCE_OPTIONS")
FENCE_TYPE_POLYGON = 4

#: fence_core.DEFAULT_TOLERANCE_M: the int32 1e-7 deg wire loses about a centimetre.
TOLERANCE_M = 1.0
#: How far outside the bridge's polygon a vertex of another inclusion polygon may lie and that polygon
#: still count as drawn inside it (the pilot's own fence, which the write never replaces). The bridge's
#: polygon is RoboCommand's course inset by 2 m, so a vertex placed by hand on its line is well inside this.
KEEP_TOLERANCE_M = 5.0
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


def outside_by_m(point: Tuple[float, float], polygon: Sequence[Tuple[float, float]]) -> float:
    """0 when `point` is inside the (open) polygon, else metres to its nearest edge. Flat metres around
    the point: right to well under a metre at course scale, which is all a tolerance of metres needs."""
    if inside(point, polygon):
        return 0.0
    k_lat = math.radians(1.0) * _EARTH_R_M
    k_lon = k_lat * math.cos(math.radians(point[0]))
    xy = [((lon - point[1]) * k_lon, (lat - point[0]) * k_lat) for lat, lon in polygon]
    best = math.inf
    for (x1, y1), (x2, y2) in zip(xy, xy[1:] + xy[:1]):
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 == 0 else max(0.0, min(1.0, -(x1 * dx + y1 * dy) / L2))
        best = min(best, math.hypot(x1 + t * dx, y1 + t * dy))
    return best


def same_ring(a: Sequence[Tuple[float, float]], b: Sequence[Tuple[float, float]],
              tol_m: float = TOLERANCE_M) -> bool:
    """The same open polygon, every vertex within `tol_m`: from any starting vertex, in either direction --
    QGC keeps the order it downloaded, but a hand-made copy need not."""
    n = len(a)
    if n == 0 or n != len(b):
        return False
    for ring in (list(b), list(reversed(b))):
        for k in range(n):
            if all(offset_m(a[i], ring[(i + k) % n]) <= tol_m for i in range(n)):
                return True
    return False


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


def inclusion_polygons(items: Sequence[dict]) -> List[List[Tuple[float, float]]]:
    """Every inclusion polygon the downloaded items make, open, in the autopilot's order."""
    return [[(p[0], p[1]) for p in s["points"]] for s in shapes(items)
            if s["type"] == "polygon" and s["inclusion"]]


def contains(items: Sequence[dict], polygon: Sequence[Tuple[float, float]]) -> bool:
    """Does the autopilot hold this (open) polygon as one of its inclusion polygons -- whatever else it
    holds beside it?"""
    return any(same_ring(p, polygon) for p in inclusion_polygons(items))


def drawn_inside(items: Sequence[dict], polygon: Sequence[Tuple[float, float]],
                 ignore: Sequence[Sequence[Tuple[float, float]]] = ()) -> Optional[int]:
    """The vertex count of the first inclusion polygon lying inside `polygon` within KEEP_TOLERANCE_M --
    the pilot's own fence for this course -- or None when there is none. `polygon` itself, and anything
    in `ignore` (what this OCS wrote, e.g. an earlier course's polygon), is never the pilot's."""
    for p in inclusion_polygons(items):
        if same_ring(p, polygon) or any(same_ring(p, q) for q in ignore):
            continue
        if all(outside_by_m(v, polygon) <= KEEP_TOLERANCE_M for v in p):
            return len(p)
    return None


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
    watch_s: float = 30.0             # the autopilot's fence and FENCE_* params are read this often
    denied_retry_s: float = 5.0       # a read refused mid-QGC-upload is tried again this soon


class FenceUploader:
    """One radio vehicle's autopilot fence: always READ, and WRITE the bridge's polygon only where the
    module docstring says. Phases of a write attempt: params -> (gate) -> read -> write -> verify; the
    watch is one more download ("watch") every watch_s. `state` is the WRITE side's, for /fleet and the
    log ("off" when there is nothing to write or writing is off); `_phase` is the dialog in flight.

    `armed(now)` and `position(now)` are the autopilot's own, fresh, or None; `route_ok()` says the
    radio has heard the vehicle lately; `home_hae(now)` is the home altitude above the WGS84 ellipsoid,
    or None. They are asked at every tick AND before every frame of a write: a HEARTBEAT that says armed
    and the autopilot's next MISSION_REQUEST can arrive in the same batch, and the vertex must not go out
    in between."""

    def __init__(self, *, label: str, target_sysid: int, target_compid: int, our_sysid: int,
                 send: Callable[[int, bytes, float], bool], armed: Callable[[float], Optional[bool]],
                 position: Callable[[float], Optional[Tuple[float, float]]], route_ok: Callable[[], bool],
                 backup_dir: Optional[str], may_block_arming: bool = False, write: bool = True,
                 home_hae: Optional[Callable[[float], Optional[float]]] = None,
                 wall: Callable[[], float] = time.time, timing: Timing = Timing()):
        self.label, self.ts, self.tc, self.our_sysid = label, target_sysid, target_compid, our_sysid
        self._send_fn, self.backup_dir, self.wall, self.t = send, backup_dir, wall, timing
        self._armed, self._position_of, self._route_ok = armed, position, route_ok
        self._home_hae = home_hae or (lambda now: None)
        self.may_block_arming = may_block_arming
        #: False: the bridge's polygon is only looked for on the autopilot, never written (fence_upload off).
        self.write = write
        #: Set when the polygon was cleared to go only because may_block_arming accepts that the
        #: aircraft will not arm where it is: what /fleet and the map say about it.
        self.lockout: Optional[str] = None
        # the last download, as the autopilot held it: items, when (monotonic and wall), home HAE then
        self._read_items: Optional[List[dict]] = None
        self._read_at: Optional[float] = None
        self._read_wall: Optional[float] = None
        self.read_home_hae_m: Optional[float] = None
        self.reads = self.read_refusals = 0
        self._watch_at = 0.0                            # the first read goes out at once
        self.polygon: Optional[List[Tuple[float, float]]] = None
        #: Polygons that are this OCS's own doing in this run -- each one it wrote, and whatever it read
        #: back straight after writing -- so a polygon it put there for an earlier course, or mangled,
        #: is never mistaken for the pilot's fence and left in place.
        self._ours: List[List[Tuple[float, float]]] = []
        self.state, self.reason = "off", "no geofence from the bridge yet"
        self.attempts = self.failures = self.writes = self.frames = self.send_refused = 0
        self.last_error: Optional[str] = None
        self.backup: Optional[str] = None
        self.autopilot_fence: Optional[str] = None     # describe() of the last download
        self.done_at: Optional[float] = None
        #: When the autopilot was last READ holding `polygon` among its inclusion polygons (monotonic,
        #: and wall for the bridge). None while it does not, or nobody has looked.
        self.verified_at: Optional[float] = None
        self.verified_wall: Optional[float] = None
        self._next_at = 0.0
        #: name -> (value, when heard). Heard at any time -- ArduPilot broadcasts a parameter that
        #: is set -- but only a value heard since `_params_since` counts for the gate.
        self._params: Dict[str, Tuple[float, float]] = {}
        self._params_since = math.inf
        self._phase: Optional[str] = None               # "params" | "read" | "write" | "verify" | "watch"
        self._resend: Optional[Callable[[float], None]] = None
        self._step_s = timing.step_s
        self._deadline = math.inf
        self._tries = 0
        self._count: Optional[int] = None               # download: the autopilot's item count
        self._items: List[dict] = []
        self._requested: set = set()                    # upload: seqs the autopilot asked for

    # ------------------------------------------------------------------------ inputs

    def set_polygon(self, points: Optional[Sequence[Tuple[float, float]]], now: float) -> None:
        """RoboCommand's polygon from the bridge's `ocs_info` (closed), or None. Raises FenceError."""
        if points is None:
            if self.polygon is not None:
                self._abort()
                self.polygon = None
                self.verified_at = self.verified_wall = None
                self._set("off", "the bridge withdrew RoboCommand's polygon; the autopilot's fence is left as it is",
                          logging.INFO)
            return
        polygon = open_polygon(points)
        if polygon == self.polygon:
            return
        self._retarget(polygon, now)
        if not self.write:
            self._set("off", "fence_upload is off: RoboCommand's %d-vertex polygon is looked for on the autopilot, "
                             "never written" % len(polygon), logging.INFO)
            return self._judge_presence()
        log.info("fence %s: RoboCommand's %d-vertex polygon from the bridge; will put it on the autopilot "
                 "when it is disarmed, unless it is there already", self.label, len(polygon))
        self._set("waiting", "starting")

    def _retarget(self, polygon: List[Tuple[float, float]], start_at: float) -> None:
        self._abort()
        self.polygon = polygon
        self.attempts = self.failures = 0
        self.last_error = self.backup = self.done_at = self.lockout = None
        self.verified_at = self.verified_wall = None
        self._next_at = start_at

    def note(self, frame: Frame, now: float) -> None:
        """One frame off the radio. Only the autopilot's, only CRC-checked ones, and only mission
        traffic addressed to us: QGC fetching its own copy over the same radio is not our answer."""
        if frame.sysid != self.ts or frame.compid != self.tc or not frame.validated:
            return
        if frame.msgid == MSG_PARAM_VALUE:
            name, value = param_value(frame)
            if name in REPORTED_PARAMS:
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
        """Called every adapter poll: a dialog in flight first, then a write that is due, then the watch."""
        if self._phase is not None:
            return self._dialog_tick(now)
        if self._writing():
            self._write_tick(now)
            if self._phase is not None:
                return
        self._watch_tick(now)

    def _writing(self) -> bool:
        return self.write and self.polygon is not None and self.state in ("waiting", "held")

    def _dialog_tick(self, now: float) -> None:
        if self._phase == "watch":
            if self._armed(now) is not False or not self._route_ok():
                self._abort()                 # an interruption: read again once disarmed and heard
                return
            if now >= self._deadline:
                if self._tries < self.t.tries:
                    return self._fire(now)
                self._abort()
                self._watch_at = now + self.t.watch_s
                log.info("fence %s: no answer to a fence read; the last read is now %s old", self.label,
                         "never" if self._read_at is None else "%.0f s" % (now - self._read_at))
            return
        if not self._clear_to_talk(now):
            return
        if now >= self._deadline:
            if self._tries < self.t.tries:
                self._fire(now)
            else:
                self._timed_out(now)

    def _write_tick(self, now: float) -> None:
        if not self._clear_to_talk(now):
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

    def _watch_tick(self, now: float) -> None:
        """Read the autopilot's fence (and ask for the FENCE_* params) every watch_s while it is disarmed
        and heard. Reads are not urgent: one that cannot run waits, and the bridge judges the age."""
        if now < self._watch_at or self._armed(now) is not False or not self._route_ok():
            return
        for name in REPORTED_PARAMS:
            self._send(MSG_PARAM_REQUEST_READ, pack_param_request_read(name, self.ts, self.tc), now)
        self._start_download("watch", now, None)

    # ------------------------------------------------------------------------- phases

    def _start_params(self, now: float) -> None:
        self._params_since = now
        self._begin("params", now, self.t.step_s, self._ask_params)
        self._set_state_quietly("reading the FENCE_* parameters")

    def _ask_params(self, now: float) -> None:
        for name in REPORTED_PARAMS:
            if self._param(name) is None:
                self._send(MSG_PARAM_REQUEST_READ, pack_param_request_read(name, self.ts, self.tc), now)

    def _on_param(self, name: str, value: float, now: float) -> None:
        old = self._params.get(name)
        self._params[name] = (value, now)
        if old is not None and old[0] != value:
            log.info("fence %s: %s changed %g -> %g on the autopilot", self.label, name, old[0], value)
            if self.state == "gave up" and name in PARAMS:      # e.g. the polygon bit was just set in QGC
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
        self._ours.append(list(self.polygon))
        self._begin("write", now, self.t.write_step_s, self._ask_count)
        self._set_state_quietly("writing %d vertices" % len(self.polygon))

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
        if self._phase == "watch":
            # Not a failure: ArduPilot DENIES a download while QGC is uploading. Soon again.
            self._abort()
            self.read_refusals += 1
            self._watch_at = now + self.t.denied_retry_s
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
        self._read_items, self._read_at, self._read_wall = list(items), now, self.wall()
        self.autopilot_fence = describe(items)
        self.read_home_hae_m = self._home_hae(now)
        self.reads += 1
        self._watch_at = now + self.t.watch_s
        if phase == "watch":
            return self._judge_presence()
        held = contains(items, self.polygon)
        if phase == "verify":
            self._ours.extend(inclusion_polygons(items))    # straight after our write: all of it is ours
            if not held:
                return self._fail(now, "read back wrong: the polygon just written is not on the autopilot (it "
                                       "holds %s)" % self.autopilot_fence)
            return self._done(now, "written and read back: %d vertices, each within %.0f m"
                              % (len(self.polygon), TOLERANCE_M))
        if held:
            beside = len(inclusion_polygons(items)) - 1
            return self._done(now, "the autopilot already holds RoboCommand's polygon%s; nothing written"
                              % ("" if not beside else " and %d polygon(s) beside it" % beside))
        kept = drawn_inside(items, self.polygon, ignore=self._ours)
        if kept is not None:
            # The pilot's fence for this course is there already; RoboCommand's polygon is not. Writing
            # would replace the pilot's -- the one polygon this OCS exists to declare.
            return self._done(now, "NOT written: the autopilot holds a %d-vertex polygon inside RoboCommand's "
                                   "(the pilot's, drawn in QGC), and writing would replace it. It holds %s"
                              % (kept, self.autopilot_fence))
        if items:
            try:
                self.backup = self._save_backup(items, now)
            except OSError as e:
                return self._fail(now, "could not save the autopilot's current fence (%s) before replacing "
                                       "it, so it was not replaced: %s" % (self.autopilot_fence, e))
            log.warning("fence %s: replacing the autopilot's fence (%s), which has neither RoboCommand's polygon "
                        "nor one inside it; saved to %s", self.label, self.autopilot_fence, self.backup)
        self._start_write(now)

    def _judge_presence(self) -> None:
        """After a read: is the bridge's polygon (still) among the autopilot's inclusion polygons? Only
        `verified` moves. Not there any more after it was is the pilot's doing (QGC) and is never put back."""
        if self.polygon is None or self._read_items is None:
            return
        if contains(self._read_items, self.polygon):
            self.verified_at, self.verified_wall = self._read_at, self._read_wall
            return
        if self.verified_at is not None:
            log.warning("fence %s: RoboCommand's polygon is no longer on the autopilot (it holds %s); not put "
                        "back -- the fence is the pilot's now", self.label, self.autopilot_fence)
            if self.state == "done":
                self.reason = ("RoboCommand's polygon is no longer on the autopilot (it holds %s): taken off in "
                               "QGC? Not put back" % self.autopilot_fence)
        self.verified_at = self.verified_wall = None

    # ------------------------------------------------------------------- the rules

    def _gate(self, position: Optional[Tuple[float, float]]) -> Optional[str]:
        """None when the polygon may be written now, else why not -- see the module docstring."""
        ftype, enable, auto = (self._param(n) for n in PARAMS)
        self.lockout = None
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

    def latest_param(self, name: str) -> Optional[float]:
        """The value last heard from the autopilot, however long ago (it broadcasts a change), or None."""
        got = self._params.get(name)
        return None if got is None else got[0]

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
        self._judge_presence()
        if self.lockout:
            log.warning("fence %s: %s", self.label, self.lockout)
        self._set("done", how + ". Never written again for this course; FENCE_ENABLE is not touched", logging.INFO)

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
               "why": "the autopilot's fence before fleet_link replaced it with RoboCommand's polygon for the course",
               "summary": describe(items), "items": items,
               "replaced_by": [list(p) for p in self.polygon]}
        path.write_text(json.dumps(doc, indent=1), encoding="utf-8")
        return str(path)

    # ------------------------------------------------------------------------ status

    def status(self, now: float) -> dict:
        return {"enabled": True, "write": self.write, "state": self.state, "reason": self.reason,
                "vertices": None if self.polygon is None else len(self.polygon),
                "attempts": self.attempts, "failures": self.failures, "writes": self.writes,
                "last_error": self.last_error, "autopilot_fence": self.autopilot_fence, "backup": self.backup,
                "params": {n: self.latest_param(n) for n in REPORTED_PARAMS},
                "params_age_s": {n: round(now - self._params[n][1], 1) for n in REPORTED_PARAMS if n in self._params},
                "done_age_s": None if self.done_at is None else round(now - self.done_at, 1),
                "lockout": self.lockout, "may_block_arming": self.may_block_arming,
                "verified": self.verified_at is not None,
                "verified_age_s": None if self.verified_at is None else round(now - self.verified_at, 1),
                "reads": self.reads, "read_refusals": self.read_refusals,
                "home_hae_m": self.read_home_hae_m,
                # for the OCS map: RoboCommand's polygon this was told to put on the autopilot, and what
                # the autopilot held at the last read (its age says how old)
                "polygon": None if self.polygon is None else [list(p) for p in self.polygon],
                "autopilot_shapes": None if self._read_items is None else shapes(self._read_items),
                "autopilot_read_age_s": None if self._read_at is None else round(now - self._read_at, 1),
                "frames": self.frames, "send_refused": self.send_refused}

    def confirmation(self) -> dict:
        """The body of the `ocs_fence` frame the adapter sends the bridge (docs/VEHICLE_LINK.md). What the
        autopilot held at the last read -- every shape, the FENCE_* params, home HAE -- is what the bridge
        declares from; `verified` says only whether RoboCommand's polygon was among it. Times are wall time
        (the adapter formats them)."""
        return {"vehicle_id": self.label, "state": self.state,
                "polygon": None if self.polygon is None else [list(p) for p in self.polygon],
                "verified": self.verified_at is not None, "verified_at": self.verified_wall,
                "reason": self.reason, "lockout": self.lockout,
                "read_at": self._read_wall,
                "shapes": None if self._read_items is None else shapes(self._read_items),
                "params": {n: self.latest_param(n) for n in REPORTED_PARAMS},
                "home_hae_m": None if self.read_home_hae_m is None else round(self.read_home_hae_m, 2)}


#: The dialogs that download the fence (and so take MISSION_COUNT / MISSION_ITEM_INT answers).
DOWNLOADS = ("read", "verify", "watch")


def _result(code: int) -> str:
    return "%s (%d)" % (MISSION_RESULTS.get(code, "?"), code)
