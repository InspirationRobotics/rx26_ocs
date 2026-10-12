"""rxl_codec -- the RXL catalogue messages as MAVLink TUNNEL payloads. Pure functions,
stdlib `struct` only: no pymavlink, no protobuf, no sockets, no clock.

WHAT THIS IS. The team's inter-vehicle catalogue (Boat/RXL_Inter-Vehicle_Message_
Catalogue.pdf, sized in Boat/RXL_byte_budget.py) defines a binary frame of its own.
On Ekko's radio the frame is MAVLink and the catalogue message travels inside a
TUNNEL (msg 385):

    TUNNEL.payload_type = 0x8000 | catalogue_id      (0x8011 = SAFE_PASSAGE, ...)
    TUNNEL.payload      = the catalogue PAYLOAD only, little-endian, packed

The catalogue's own 12-byte header does not travel: MAVLink already carries the
source (sysid/compid), the destination (target_system) and a sequence, so
SRC|DST, SEQ and CRC are the MAVLink header's job. T_MS (the time of the data) has
no MAVLink header equivalent; messages whose data has an age carry their own
`time_boot_ms` exactly as the Mesh bench does (Mesh/rxl.py).

Everything crossing to or from RoboCommand names its fields after RoboNation's and
its enums carry RoboNation's VALUES -- read from the pinned generated protobuf
(OCS/rx_bridge/gen/robotx/*_pb2.py, revision f6457fa), never from memory or the
spreadsheet. tests/test_rxl_codec.py re-checks every table below against that
generated code whenever the OCS .venv is present.

EVERY DECODER LENGTH-CHECKS. A radio delivers garbage as confidently as data, and
Ekko's own RX loop once died on a 10-byte frame. Each `unpack_*` raises RxlError (a
ValueError) with a plain message on a wrong length, an out-of-range count or an
enum value outside the pinned table; it never raises struct.error or IndexError and
never returns a partly-read dict. Every `pack_*` refuses a value that does not fit
its field (a run_id above 65535, a radius above 6553.5 m) rather than truncating it:
a silently wrapped number is a wrong number that looks right.

ID TABLE (payload_type = 0x8000 | id). The ids and names are the catalogue's, except
0x27 which is ours (see GEOFENCE_INFO).

    id    name                dir          payload
    0x10  HEARTBEAT           UAV -> OCS      24
    0x11  SAFE_PASSAGE        UAV -> OCS      21 + 9n   (n <= 10: max 111)
    0x12  PIPELINE_SURVEY     -> OCS          11
    0x13  RESOURCE_DELIVERY   UAV -> OCS       3
    0x14  DOCKING             -> OCS           1
    0x15  FIREFIGHTING        -> OCS           1
    0x16  INCIDENT_ACK        UAV -> OCS       2   (receiver ACKs it)
    0x17  READINESS           UAV -> OCS       2   (receiver ACKs it)
    0x20  RUN_START           OCS -> UAV       3   (UAV ACKs it)
    0x21  ASSISTANCE_REQUEST  OCS -> UAV      11
    0x22  KEEP_OUT_ZONE       OCS -> UAV      14   (catalogue 13: zone_id u8 -> cmd_seq u16)
    0x23  ALL_CLEAR           OCS -> UAV       3   (UAV ACKs it)
    0x24  MOVING_OBJECT_ALERT OCS -> UAV      13
    0x25  READINESS_CONFIRM   OCS -> UAV       3
    0x26  COURSE              OCS -> UAV      4 + 8n (n <= 15: max 124)
    0x27  GEOFENCE_INFO       OCS -> UAV      5 + 8n (n <= 15 per part; parts i of n)
    0x38  ACK                 either way       3

PAYLOAD TYPES THAT ARE NOT OURS and must never be reused here: 32768 (boat), 32769-
32771 (retired), 32772-32774 (POSITIONS / LIGHTS / GNSS), 33022 (TEST) -- all in
Drone/rx26_uav/uav_common/uav_common/boat_link.py, and the Mesh bench's 0x8011,
0x8013, 0x8018, 0x8019, 0x8034 (see KNOWN_CLASHES for the one that is a real clash).
"""
from __future__ import annotations

import math
import struct
from typing import Callable, Dict, List, NamedTuple, Optional, Sequence, Tuple


class RxlError(ValueError):
    """A payload that cannot be decoded, or a value that cannot be encoded.

    `key` is an optional value-free label ("RUN_START.run_id does not fit u16") so a counter can
    group like refusals without one bucket per value; the message keeps the value."""

    def __init__(self, message: str = "", key: Optional[str] = None):
        super().__init__(message)
        self.key = key


# ----------------------------------------------------------------------- identity

TUNNEL_BASE = 0x8000

# Catalogue node ids (section 2.1). A node id is NOT a MAVLink sysid: the sysid of
# Ekko's companion is 200, its RXL node is 3.
NODE_OCS = 0x1
NODE_USV = 0x2
NODE_UAV = 0x3
NODE_UUV = 0x4
NODE_BROADCAST = 0xF
NODE_BY_VEHICLE_TYPE = {"TYPE_USV": NODE_USV, "TYPE_UUV": NODE_UUV, "TYPE_UAV": NODE_UAV}

HEARTBEAT = 0x10
SAFE_PASSAGE = 0x11
PIPELINE_SURVEY = 0x12
RESOURCE_DELIVERY = 0x13
DOCKING = 0x14
FIREFIGHTING = 0x15
INCIDENT_ACK = 0x16
READINESS = 0x17
RUN_START = 0x20
ASSISTANCE_REQUEST = 0x21
KEEP_OUT_ZONE = 0x22
ALL_CLEAR = 0x23
MOVING_OBJECT_ALERT = 0x24
READINESS_CONFIRM = 0x25
COURSE = 0x26
#: Ours. The catalogue has no message for the UAV geofence: RunDeclaration carries it
#: to RoboCommand and nothing carries it to Ekko. 0x27 is the first id after COURSE;
#: it is free in the catalogue (which uses 0x10-0x17, 0x18 acoustic-only, 0x20-0x26,
#: 0x30, 0x31, 0x33-0x39 and keeps 0x32 empty on purpose), in the Mesh bench (0x11,
#: 0x13, 0x18, 0x19, 0x34), in the dialect (native 42027 is unused) and in
#: boat_link.py (0x8000-0x8006, 0x80FE).
GEOFENCE_INFO = 0x27
ACK = 0x38

NAMES: Dict[int, str] = {
    HEARTBEAT: "HEARTBEAT", SAFE_PASSAGE: "SAFE_PASSAGE", PIPELINE_SURVEY: "PIPELINE_SURVEY",
    RESOURCE_DELIVERY: "RESOURCE_DELIVERY", DOCKING: "DOCKING", FIREFIGHTING: "FIREFIGHTING",
    INCIDENT_ACK: "INCIDENT_ACK", READINESS: "READINESS", RUN_START: "RUN_START",
    ASSISTANCE_REQUEST: "ASSISTANCE_REQUEST", KEEP_OUT_ZONE: "KEEP_OUT_ZONE",
    ALL_CLEAR: "ALL_CLEAR", MOVING_OBJECT_ALERT: "MOVING_OBJECT_ALERT",
    READINESS_CONFIRM: "READINESS_CONFIRM", COURSE: "COURSE", GEOFENCE_INFO: "GEOFENCE_INFO",
    ACK: "ACK",
}
#: Catalogue ids this file names but does not decode (inter-vehicle, not OCS traffic).
#: They are logged by name with their raw bytes.
INTER_VEHICLE_NAMES: Dict[int, str] = {
    0x18: "USV_REACHED_GATE", 0x19: "NEXT_BUOY_SET", 0x30: "PASSAGE_PLAN",
    0x31: "PASSAGE_DELTA", 0x33: "DELIVERY_TASKING", 0x34: "SURVEY_START",
    0x35: "FLEET_HOLD", 0x36: "TIME_SYNC", 0x37: "LINK_STATS", 0x39: "LINK_PING",
}
#: TUNNEL payload types other code on the air already owns (boat_link.py:66-71).
FOREIGN_PAYLOAD_TYPES: Dict[int, str] = {
    32768: "BOAT", 32769: "retired v1 whole map", 32770: "retired v2 positions",
    32771: "retired v2 lights", 32772: "POSITIONS", 32773: "LIGHTS", 32774: "GNSS",
    33022: "TEST",
}
#: Where the Mesh bench and the catalogue use one id for two different payloads.
#: 0x13: the catalogue's RESOURCE_DELIVERY is RoboCommand's ResourceDeliveryRequest
#: (task, resource_color, delivery_circle_color); Mesh/rxl.py and the native
#: RXL_RESOURCE_DELIVERY (42013) carry (platform_id, tin_color, state). Both are three
#: bytes, so a receiver cannot tell them apart by length. This codec decodes the
#: catalogue form; the tunnel log marks the id with this note.
KNOWN_CLASHES: Dict[int, str] = {
    RESOURCE_DELIVERY: "0x13 is also the Mesh bench / native RXL_RESOURCE_DELIVERY "
                       "(platform_id, tin_color, state): same size, different meaning",
}

#: The receiver of these answers with an ACK (0x38) -- catalogue 4.1/4.3/4.5.
#: INCIDENT_ACK and READINESS go UAV -> OCS; RUN_START and ALL_CLEAR go OCS -> UAV.
RECEIVER_ACKS = frozenset({INCIDENT_ACK, READINESS, RUN_START, ALL_CLEAR})


def payload_type(catalogue_id: int) -> int:
    return TUNNEL_BASE | catalogue_id


def catalogue_id(ptype: int) -> Optional[int]:
    """The catalogue id inside a TUNNEL payload_type, or None if it is not in the
    0x8000 | id form for an id this file knows (so a foreign type never aliases)."""
    cid = ptype ^ TUNNEL_BASE
    if ptype & TUNNEL_BASE and 0 < cid <= 0xFF and (
            cid in NAMES or cid in INTER_VEHICLE_NAMES):
        return cid
    return None


def payload_type_name(ptype: int) -> str:
    cid = catalogue_id(ptype)
    if cid is not None:
        return NAMES.get(cid) or INTER_VEHICLE_NAMES[cid]
    return FOREIGN_PAYLOAD_TYPES.get(ptype, "UNKNOWN")


def is_ours(ptype: int) -> bool:
    """A payload type this codec decodes (OCS traffic), as opposed to one it only names."""
    cid = catalogue_id(ptype)
    return cid is not None and cid in NAMES


# --------------------------------------------------------------------------- enums
# Values and names copied from the generated protobuf (see the module docstring).

ROBOT_STATE = {0: "STATE_UNKNOWN", 1: "STATE_KILLED", 2: "STATE_MANUAL", 3: "STATE_AUTO"}
FLIGHT_PHASE = {0: "FLIGHT_PHASE_UNKNOWN", 1: "FLIGHT_PHASE_GROUNDED", 2: "FLIGHT_PHASE_AIRBORNE"}
RX_TASK = {0: "TASK_UNKNOWN", 1: "TASK_NONE", 2: "TASK_SAFE_PASSAGE",
           3: "TASK_INFRA_SURVEY_REPAIR", 4: "TASK_COORDINATED_LOGISTICS",
           5: "TASK_DYNAMIC_INCIDENT"}
VEHICLE_TYPE = {0: "TYPE_UNKNOWN", 1: "TYPE_USV", 2: "TYPE_UUV", 3: "TYPE_UAV"}
VEHICLE_TYPE_BY_NAME = {name: number for number, name in VEHICLE_TYPE.items()}
BEACON_STATE = {0: "BEACON_STATE_UNKNOWN", 1: "BEACON_STATE_OFF", 2: "BEACON_STATE_FLASHING_RED",
                3: "BEACON_STATE_FLASHING_GREEN", 4: "BEACON_STATE_FLASHING_BLUE",
                5: "BEACON_STATE_STEADY_BLUE"}
PIPELINE_SEGMENT = {0: "PIPELINE_SEGMENT_UNKNOWN", 1: "PIPELINE_SEGMENT_INTACT",
                    2: "PIPELINE_SEGMENT_DAMAGED"}
COLOR = {0: "COLOR_UNKNOWN", 1: "COLOR_RED", 2: "COLOR_GREEN", 3: "COLOR_BLUE", 4: "COLOR_ANY"}

#: Ours (HEARTBEAT.phase): the sub-state RoboNation has no field for. The catalogue
#: lists SEARCH / APPROACH / EXECUTE / COMPLETE / HOLD without numbers; they are
#: numbered here in that order from 0, and 0xFF means "no sub-state". The OCS does not
#: use it.
PHASE_NAMES = {0: "SEARCH", 1: "APPROACH", 2: "EXECUTE", 3: "COMPLETE", 4: "HOLD"}
PHASE_NONE = 0xFF

STATE_UNKNOWN, STATE_KILLED, STATE_MANUAL, STATE_AUTO = 0, 1, 2, 3
PHASE_GROUNDED, PHASE_AIRBORNE = 1, 2
TASK_NONE = 1
TYPE_USV, TYPE_UUV, TYPE_UAV = 1, 2, 3

#: HEARTBEAT.hdg_cdeg when GPS yaw is unresolved -- never substitute a plausible value.
HDG_UNRESOLVED = 0xFFFF

MAX_BUOYS = 10                 # handbook 3.3.2
MAX_SEGMENTS = 8               # two bits each in a u16
MAX_COURSE_CORNERS = 15        # (128 - 4) // 8
MAX_FENCE_PER_PART = 15        # (128 - 5) // 8
MAX_FENCE_PARTS = 16
MAX_FENCE_VERTICES = MAX_FENCE_PER_PART * MAX_FENCE_PARTS
TUNNEL_MAX = 128


# ------------------------------------------------------------------- field helpers

def _need(name: str, data: bytes, size: int) -> None:
    if len(data) != size:
        raise RxlError("%s: payload is %d bytes, expected exactly %d" % (name, len(data), size))


def _need_at_least(name: str, data: bytes, size: int) -> None:
    if len(data) < size:
        raise RxlError("%s: payload is %d bytes, shorter than its %d-byte header"
                       % (name, len(data), size))


def _uint(name: str, field: str, value, bits: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value < (1 << bits):
        raise RxlError("%s.%s: %r does not fit an unsigned %d-bit field" % (name, field, value, bits),
                       key="%s.%s does not fit u%d" % (name, field, bits))
    return value


def _enum(name: str, field: str, value, table: Dict[int, str], *, allow_unknown=True) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value not in table:
        raise RxlError("%s.%s: %r is not a value of the pinned enum (%s)"
                       % (name, field, value, ", ".join(str(k) for k in table)))
    if not allow_unknown and value == 0:
        raise RxlError("%s.%s: UNKNOWN (0) is never transmitted" % (name, field))
    return value


def _e7(name: str, field: str, degrees, limit: float) -> int:
    if isinstance(degrees, bool) or not isinstance(degrees, (int, float)) or not math.isfinite(degrees) \
            or abs(degrees) > limit:
        raise RxlError("%s.%s: %r is not a coordinate within +/-%g degrees" % (name, field, degrees, limit))
    return int(round(degrees * 1e7))


def _lat(name: str, value) -> int:
    return _e7(name, "lat", value, 90.0)


def _lon(name: str, value) -> int:
    return _e7(name, "lon", value, 180.0)


def _deg_from_e7(name: str, field: str, e7: int, limit: float) -> float:
    if abs(e7) > limit * 1e7:
        raise RxlError("%s.%s: %d degE7 is outside +/-%g degrees" % (name, field, e7, limit))
    return e7 / 1e7


def _scaled(name: str, field: str, value, scale: float, bits: int) -> int:
    """value * scale, rounded, into an unsigned field. Refuses NaN, negatives, overflow."""
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise RxlError("%s.%s: %r is not a finite number" % (name, field, value))
    return _uint(name, field, int(round(value * scale)), bits)


# ------------------------------------------------------------------------- HEARTBEAT

_HB = struct.Struct("<BBiiHHhhiBB")      # 24 bytes: the catalogue's table, offset for offset
HEARTBEAT_SIZE = _HB.size


def pack_heartbeat(*, vehicle: int, robot_state: int, flight_phase: int, pos_valid: bool,
                   att_valid: bool, alt_valid: bool, dead_reckoned: bool = False,
                   lat: float = 0.0, lon: float = 0.0, spd_mps: float = 0.0,
                   hdg_deg: Optional[float] = None, roll_deg: float = 0.0, pitch_deg: float = 0.0,
                   alt_m: float = 0.0, task: int = TASK_NONE, phase: int = PHASE_NONE) -> bytes:
    """`hdg_deg=None` is the unresolved marker (0xFFFF). Fields whose valid flag is
    clear are packed as given and mean nothing to the receiver."""
    n = "HEARTBEAT"
    flags = (_enum(n, "robot_state", robot_state, ROBOT_STATE)
             | _enum(n, "flight_phase", flight_phase, FLIGHT_PHASE) << 2
             | bool(pos_valid) << 4 | bool(att_valid) << 5 | bool(alt_valid) << 6
             | bool(dead_reckoned) << 7)
    hdg = HDG_UNRESOLVED if hdg_deg is None else _heading_cdeg(n, "hdg_deg", hdg_deg)
    roll, pitch = (_signed(n, f, v, 100, 16) for f, v in (("roll_deg", roll_deg), ("pitch_deg", pitch_deg)))
    return _HB.pack(_uint(n, "vehicle", vehicle, 8), flags, _lat(n, lat), _lon(n, lon),
                    _scaled(n, "spd_mps", spd_mps, 100, 16), hdg, roll, pitch,
                    _signed(n, "alt_m", alt_m, 100, 32), _uint(n, "task", task, 8),
                    _uint(n, "phase", phase, 8))


def _heading_cdeg(name: str, field: str, degrees) -> int:
    """Degrees to centidegrees in [0, 36000): wraps (a heading of 360 is 0) and refuses
    NaN/inf, so a heading can never land on HEARTBEAT's 0xFFFF 'unresolved' marker."""
    if isinstance(degrees, bool) or not isinstance(degrees, (int, float)) or not math.isfinite(degrees):
        raise RxlError("%s.%s: %r is not a finite heading" % (name, field, degrees))
    return int(round((degrees % 360.0) * 100)) % 36000


def _signed(name: str, field: str, value, scale: float, bits: int) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise RxlError("%s.%s: %r is not a finite number" % (name, field, value))
    n = int(round(value * scale))
    if not -(1 << (bits - 1)) <= n < (1 << (bits - 1)):
        raise RxlError("%s.%s: %r does not fit a signed %d-bit field at scale %g"
                       % (name, field, value, bits, scale))
    return n


def unpack_heartbeat(data: bytes) -> dict:
    n = "HEARTBEAT"
    _need(n, data, _HB.size)
    (vehicle, flags, lat, lon, spd, hdg, roll, pitch, alt, task, phase) = _HB.unpack(data)
    state, fphase = flags & 3, (flags >> 2) & 3
    if fphase not in FLIGHT_PHASE:
        raise RxlError("HEARTBEAT.flags: flight_phase %d is not a pinned FlightPhase value" % fphase)
    pos_valid, att_valid, alt_valid = bool(flags & 16), bool(flags & 32), bool(flags & 64)
    return {
        "vehicle": vehicle, "robot_state": state, "flight_phase": fphase,
        "pos_valid": pos_valid, "att_valid": att_valid, "alt_valid": alt_valid,
        "dead_reckoned": bool(flags & 128),
        # Values behind a clear valid flag are not decoded to numbers: the sender says
        # they mean nothing, so they are blanks here too.
        "lat": _deg_from_e7(n, "lat", lat, 90.0) if pos_valid else None,
        "lon": _deg_from_e7(n, "lon", lon, 180.0) if pos_valid else None,
        "spd_mps": spd / 100.0,
        "hdg_deg": None if hdg == HDG_UNRESOLVED else hdg / 100.0,
        "roll_deg": roll / 100.0 if att_valid else None,
        "pitch_deg": pitch / 100.0 if att_valid else None,
        "alt_m": alt / 100.0 if alt_valid else None,
        "task": task, "phase": phase,
    }


# ---------------------------------------------------------------------- SAFE_PASSAGE

_SP_HEAD = struct.Struct("<IiiiiB")      # time_boot_ms, entry lat/lon, exit lat/lon, n  (Mesh/rxl.py:211)
_SP_BUOY = struct.Struct("<iiB")         # lat, lon, BeaconState


def pack_safe_passage(*, time_boot_ms: int, entry: Optional[Tuple[float, float]],
                      exit_: Optional[Tuple[float, float]],
                      buoys: Sequence[Tuple[float, float, int]]) -> bytes:
    """entry / exit_ are (lat, lon) or None for "not known" (sent as 0,0, which the
    decoder reads back as absent)."""
    n = "SAFE_PASSAGE"
    if len(buoys) > MAX_BUOYS:
        raise RxlError("SAFE_PASSAGE: %d buoys, the message carries %d" % (len(buoys), MAX_BUOYS))

    def pt(p, field):
        return (0, 0) if p is None else (_lat(n, p[0]), _lon(n, p[1]))
    (elat, elon), (xlat, xlon) = pt(entry, "entry"), pt(exit_, "exit")
    out = bytearray(_SP_HEAD.pack(_uint(n, "time_boot_ms", time_boot_ms, 32), elat, elon, xlat, xlon,
                                  len(buoys)))
    for lat, lon, state in buoys:
        out += _SP_BUOY.pack(_lat(n, lat), _lon(n, lon), _enum(n, "state", state, BEACON_STATE))
    return bytes(out)


def unpack_safe_passage(data: bytes) -> dict:
    n = "SAFE_PASSAGE"
    _need_at_least(n, data, _SP_HEAD.size)
    boot, elat, elon, xlat, xlon, count = _SP_HEAD.unpack_from(data, 0)
    if count > MAX_BUOYS:
        raise RxlError("SAFE_PASSAGE: buoy count %d exceeds the %d the task uses" % (count, MAX_BUOYS))
    _need(n, data, _SP_HEAD.size + count * _SP_BUOY.size)
    buoys = []
    for i in range(count):
        lat, lon, state = _SP_BUOY.unpack_from(data, _SP_HEAD.size + i * _SP_BUOY.size)
        if state not in BEACON_STATE:
            raise RxlError("SAFE_PASSAGE.buoy[%d].state: %d is not a pinned BeaconState value" % (i, state))
        buoys.append({"lat": _deg_from_e7(n, "buoy.lat", lat, 90.0),
                      "lon": _deg_from_e7(n, "buoy.lon", lon, 180.0), "state": state})

    def pt(la, lo, field):
        # (0,0) is "not known". Nothing is invented for it: the field is simply absent.
        if la == 0 and lo == 0:
            return None
        return {"lat": _deg_from_e7(n, field + ".lat", la, 90.0), "lon": _deg_from_e7(n, field + ".lon", lo, 180.0)}
    return {"time_boot_ms": boot, "entry": pt(elat, elon, "entry"), "exit": pt(xlat, xlon, "exit"),
            "buoys": buoys}


# ------------------------------------------------------------------ PIPELINE_SURVEY

_PIPE = struct.Struct("<iiBH")           # active buoy lat/lon, segment count, 2 bits each


def pack_pipeline_survey(*, lat: float, lon: float, segments: Sequence[int]) -> bytes:
    n = "PIPELINE_SURVEY"
    if len(segments) > MAX_SEGMENTS:
        raise RxlError("PIPELINE_SURVEY: %d segments, the message carries %d" % (len(segments), MAX_SEGMENTS))
    packed = 0
    for i, s in enumerate(segments):
        packed |= _enum(n, "segment[%d]" % i, s, PIPELINE_SEGMENT) << (2 * i)
    return _PIPE.pack(_lat(n, lat), _lon(n, lon), len(segments), packed)


def unpack_pipeline_survey(data: bytes) -> dict:
    n = "PIPELINE_SURVEY"
    _need(n, data, _PIPE.size)
    lat, lon, count, packed = _PIPE.unpack(data)
    if count > MAX_SEGMENTS:
        raise RxlError("PIPELINE_SURVEY: segment count %d exceeds %d" % (count, MAX_SEGMENTS))
    segments = [(packed >> (2 * i)) & 3 for i in range(count)]
    for i, s in enumerate(segments):
        if s not in PIPELINE_SEGMENT:
            raise RxlError("PIPELINE_SURVEY.segment[%d]: %d is not a pinned status" % (i, s))
    if packed >> (2 * count):
        raise RxlError("PIPELINE_SURVEY: bits set beyond the %d declared segments" % count)
    return {"lat": _deg_from_e7(n, "lat", lat, 90.0), "lon": _deg_from_e7(n, "lon", lon, 180.0),
            "segments": segments}


# -------------------------------------------------- RESOURCE_DELIVERY, DOCKING, FIREFIGHTING

_RES = struct.Struct("<BBB")


def pack_resource_delivery(*, task: int, resource_color: int, delivery_circle_color: int) -> bytes:
    n = "RESOURCE_DELIVERY"
    return _RES.pack(_enum(n, "task", task, RX_TASK), _enum(n, "resource_color", resource_color, COLOR),
                     _enum(n, "delivery_circle_color", delivery_circle_color, COLOR))


def unpack_resource_delivery(data: bytes) -> dict:
    n = "RESOURCE_DELIVERY"
    _need(n, data, _RES.size)
    task, rc, dc = _RES.unpack(data)
    return {"task": _enum(n, "task", task, RX_TASK), "resource_color": _enum(n, "resource_color", rc, COLOR),
            "delivery_circle_color": _enum(n, "delivery_circle_color", dc, COLOR)}


def pack_docking(*, bay_id: int) -> bytes:
    return struct.pack("<B", _uint("DOCKING", "bay_id", bay_id, 8))


def unpack_docking(data: bytes) -> dict:
    _need("DOCKING", data, 1)
    return {"bay_id": data[0]}


def pack_firefighting(*, window_id: int) -> bytes:
    return struct.pack("<B", _uint("FIREFIGHTING", "window_id", window_id, 8))


def unpack_firefighting(data: bytes) -> dict:
    _need("FIREFIGHTING", data, 1)
    return {"window_id": data[0]}


# ------------------------------------------------------- INCIDENT_ACK, READINESS (u16 seq)

def _pack_cmd_seq(name: str, cmd_seq: int) -> bytes:
    return struct.pack("<H", _uint(name, "cmd_seq", cmd_seq, 16))


def _unpack_cmd_seq(name: str, data: bytes) -> dict:
    _need(name, data, 2)
    return {"cmd_seq": struct.unpack("<H", data)[0]}


def pack_incident_ack(*, cmd_seq: int) -> bytes:
    return _pack_cmd_seq("INCIDENT_ACK", cmd_seq)


def unpack_incident_ack(data: bytes) -> dict:
    return _unpack_cmd_seq("INCIDENT_ACK", data)


def pack_readiness(*, cmd_seq: int) -> bytes:
    return _pack_cmd_seq("READINESS", cmd_seq)


def unpack_readiness(data: bytes) -> dict:
    return _unpack_cmd_seq("READINESS", data)


# --------------------------------------------------------------------------- downlink

_RUN = struct.Struct("<HB")


def pack_run_start(*, run_id: int, flags: int = 0) -> bytes:
    return _RUN.pack(_uint("RUN_START", "run_id", run_id, 16), _uint("RUN_START", "flags", flags, 8))


def unpack_run_start(data: bytes) -> dict:
    _need("RUN_START", data, _RUN.size)
    run_id, flags = _RUN.unpack(data)
    return {"run_id": run_id, "flags": flags}


_ASSIST = struct.Struct("<HiiB")


def pack_assistance_request(*, cmd_seq: int, lat: float, lon: float, vehicle_type: int) -> bytes:
    n = "ASSISTANCE_REQUEST"
    return _ASSIST.pack(_uint(n, "cmd_seq", cmd_seq, 16), _lat(n, lat), _lon(n, lon),
                        _enum(n, "vehicle_type", vehicle_type, VEHICLE_TYPE))


def unpack_assistance_request(data: bytes) -> dict:
    n = "ASSISTANCE_REQUEST"
    _need(n, data, _ASSIST.size)
    seq, lat, lon, vt = _ASSIST.unpack(data)
    return {"cmd_seq": seq, "lat": _deg_from_e7(n, "lat", lat, 90.0), "lon": _deg_from_e7(n, "lon", lon, 180.0),
            "vehicle_type": _enum(n, "vehicle_type", vt, VEHICLE_TYPE)}


def affected_mask(vehicle_types: Sequence[int]) -> int:
    """Bit n set = VehicleType n is affected (USV = bit 1, UUV = bit 2, UAV = bit 3)."""
    mask = 0
    for vt in vehicle_types:
        mask |= 1 << _enum("affected types", "vehicle_type", vt, VEHICLE_TYPE)
    return mask


def affected_types(mask: int) -> List[int]:
    return [vt for vt in VEHICLE_TYPE if mask & (1 << vt)]


_KOZ = struct.Struct("<HiiHBB")
KOZ_ACTIVE = 0x01


def pack_keep_out_zone(*, cmd_seq: int, lat: float, lon: float, radius_m: float,
                       affected: Sequence[int], active: bool = True) -> bytes:
    n = "KEEP_OUT_ZONE"
    return _KOZ.pack(_uint(n, "cmd_seq", cmd_seq, 16), _lat(n, lat), _lon(n, lon),
                     _scaled(n, "radius_m", radius_m, 10, 16), affected_mask(affected),
                     KOZ_ACTIVE if active else 0)


def unpack_keep_out_zone(data: bytes) -> dict:
    n = "KEEP_OUT_ZONE"
    _need(n, data, _KOZ.size)
    seq, lat, lon, radius_dm, mask, flags = _KOZ.unpack(data)
    if mask & ~affected_mask(list(VEHICLE_TYPE)):
        raise RxlError("KEEP_OUT_ZONE.affected: bits set for vehicle types that do not exist")
    return {"cmd_seq": seq, "lat": _deg_from_e7(n, "lat", lat, 90.0), "lon": _deg_from_e7(n, "lon", lon, 180.0),
            "radius_m": radius_dm / 10.0, "affected": affected_types(mask), "active": bool(flags & KOZ_ACTIVE)}


_CLEAR = struct.Struct("<HB")


def pack_all_clear(*, cmd_seq: int, vehicle_type: int) -> bytes:
    n = "ALL_CLEAR"
    return _CLEAR.pack(_uint(n, "cmd_seq", cmd_seq, 16), _enum(n, "vehicle_type", vehicle_type, VEHICLE_TYPE))


def unpack_all_clear(data: bytes) -> dict:
    n = "ALL_CLEAR"
    _need(n, data, _CLEAR.size)
    seq, vt = _CLEAR.unpack(data)
    return {"cmd_seq": seq, "vehicle_type": _enum(n, "vehicle_type", vt, VEHICLE_TYPE)}


_MOV = struct.Struct("<iiHHB")


def pack_moving_object_alert(*, lat: float, lon: float, heading_deg: float, speed_mps: float,
                             affected: Sequence[int]) -> bytes:
    n = "MOVING_OBJECT_ALERT"
    return _MOV.pack(_lat(n, lat), _lon(n, lon), _heading_cdeg(n, "heading_deg", heading_deg), _scaled(n, "speed_mps", speed_mps, 100, 16),
                     affected_mask(affected))


def unpack_moving_object_alert(data: bytes) -> dict:
    n = "MOVING_OBJECT_ALERT"
    _need(n, data, _MOV.size)
    lat, lon, hdg, spd, mask = _MOV.unpack(data)
    if hdg >= 36000:
        raise RxlError("MOVING_OBJECT_ALERT.heading: %d centidegrees is not below 36000" % hdg)
    if mask & ~affected_mask(list(VEHICLE_TYPE)):
        raise RxlError("MOVING_OBJECT_ALERT.affected: bits set for vehicle types that do not exist")
    return {"lat": _deg_from_e7(n, "lat", lat, 90.0), "lon": _deg_from_e7(n, "lon", lon, 180.0),
            "heading_deg": hdg / 100.0, "speed_mps": spd / 100.0, "affected": affected_types(mask)}


_CONFIRM = struct.Struct("<HB")


def pack_readiness_confirm(*, report_seq: int, vehicle: int) -> bytes:
    n = "READINESS_CONFIRM"
    return _CONFIRM.pack(_uint(n, "report_seq", report_seq, 16), _uint(n, "vehicle", vehicle, 8))


def unpack_readiness_confirm(data: bytes) -> dict:
    _need("READINESS_CONFIRM", data, _CONFIRM.size)
    seq, vehicle = _CONFIRM.unpack(data)
    return {"report_seq": seq, "vehicle": vehicle}


# ------------------------------------------------------------------ COURSE, GEOFENCE

def crc16_ccitt_false(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF): the catalogue's frame CRC, here
    used only to derive the 8-bit epochs below. Check value for b"123456789": 0x29B1."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def course_epoch(course_id: str) -> int:
    """The catalogue's course_epoch ("hash of course_id", never defined): the low byte of
    CRC-16/CCITT-FALSE over the UTF-8 course_id. A changed epoch invalidates every
    cached plan on the vehicle."""
    return crc16_ccitt_false(course_id.encode("utf-8")) & 0xFF


_COURSE_HEAD = struct.Struct("<BHB")
_CORNER = struct.Struct("<ii")


def pack_course(*, course_id: str, pinger_freq_hz: int,
                corners: Sequence[Tuple[float, float]]) -> bytes:
    """`corners` is the closed boundary polygon as (lat, lon), first == last. The course
    id does not travel (the vehicle only needs to know when it CHANGES): its hash does."""
    n = "COURSE"
    if not course_id:
        raise RxlError("COURSE: course_id is empty, so there is no epoch to send")
    if not 3 <= len(corners) <= MAX_COURSE_CORNERS:
        raise RxlError("COURSE: %d corners; a boundary needs 3 to %d" % (len(corners), MAX_COURSE_CORNERS))
    out = bytearray(_COURSE_HEAD.pack(course_epoch(course_id), _uint(n, "pinger_freq_hz", pinger_freq_hz, 16),
                                      len(corners)))
    for lat, lon in corners:
        out += _CORNER.pack(_lat(n, lat), _lon(n, lon))
    return bytes(out)


def unpack_course(data: bytes) -> dict:
    n = "COURSE"
    _need_at_least(n, data, _COURSE_HEAD.size)
    epoch, pinger, count = _COURSE_HEAD.unpack_from(data, 0)
    if count > MAX_COURSE_CORNERS:
        raise RxlError("COURSE: corner count %d exceeds %d" % (count, MAX_COURSE_CORNERS))
    _need(n, data, _COURSE_HEAD.size + count * _CORNER.size)
    corners = []
    for i in range(count):
        lat, lon = _CORNER.unpack_from(data, _COURSE_HEAD.size + i * _CORNER.size)
        corners.append({"lat": _deg_from_e7(n, "corner.lat", lat, 90.0),
                        "lon": _deg_from_e7(n, "corner.lon", lon, 180.0)})
    return {"course_epoch": epoch, "pinger_freq_hz": pinger, "corners": corners}


_FENCE_HEAD = struct.Struct("<BBBBB")
FENCE_CLOSED = 0x01


def fence_epoch(vertices_e7: Sequence[Tuple[int, int]]) -> int:
    """Low byte of CRC-16/CCITT-FALSE over every vertex (lat i32 LE, lon i32 LE), in order.
    All parts of one fence share it; any change to the polygon changes it."""
    return crc16_ccitt_false(b"".join(_CORNER.pack(a, b) for a, b in vertices_e7)) & 0xFF


def pack_geofence_parts(vertices: Sequence[Tuple[float, float]]) -> List[bytes]:
    """The UAV geofence as one or more GEOFENCE_INFO payloads, 15 vertices to a part.
    Part i of n carries vertices [15i, 15i+15) in order; a receiver rebuilds the polygon
    when it holds all n parts of one fence_epoch. A fence that fits one part is one
    message with part=0, n_parts=1."""
    n = "GEOFENCE_INFO"
    if not 3 <= len(vertices) <= MAX_FENCE_VERTICES:
        raise RxlError("GEOFENCE_INFO: %d vertices; a fence needs 3 to %d" % (len(vertices), MAX_FENCE_VERTICES))
    e7 = [(_lat(n, la), _lon(n, lo)) for la, lo in vertices]
    epoch = fence_epoch(e7)
    closed = FENCE_CLOSED if e7[0] == e7[-1] else 0
    chunks = [e7[i:i + MAX_FENCE_PER_PART] for i in range(0, len(e7), MAX_FENCE_PER_PART)]
    parts = []
    for i, chunk in enumerate(chunks):
        body = _FENCE_HEAD.pack(epoch, closed, i, len(chunks), len(chunk))
        parts.append(body + b"".join(_CORNER.pack(a, b) for a, b in chunk))
    return parts


def unpack_geofence_info(data: bytes) -> dict:
    n = "GEOFENCE_INFO"
    _need_at_least(n, data, _FENCE_HEAD.size)
    epoch, flags, part, n_parts, count = _FENCE_HEAD.unpack_from(data, 0)
    if not 1 <= n_parts <= MAX_FENCE_PARTS or part >= n_parts:
        raise RxlError("GEOFENCE_INFO: part %d of %d is not a valid part number" % (part, n_parts))
    if not 1 <= count <= MAX_FENCE_PER_PART:
        raise RxlError("GEOFENCE_INFO: %d vertices in one part (1 to %d)" % (count, MAX_FENCE_PER_PART))
    _need(n, data, _FENCE_HEAD.size + count * _CORNER.size)
    vertices = []
    for i in range(count):
        lat, lon = _CORNER.unpack_from(data, _FENCE_HEAD.size + i * _CORNER.size)
        vertices.append({"lat": _deg_from_e7(n, "vertex.lat", lat, 90.0),
                         "lon": _deg_from_e7(n, "vertex.lon", lon, 180.0)})
    return {"fence_epoch": epoch, "closed": bool(flags & FENCE_CLOSED), "part": part,
            "n_parts": n_parts, "vertices": vertices}


class GeofenceAssembler:
    """Receiver side of the multi-part scheme (the drone implements the same rule; kept
    here so the spec has an executable form and the tests can prove the round trip).

    Parts of one fence_epoch are collected; a part with a DIFFERENT epoch discards what
    was held (a newer fence supersedes an older one -- never mix two generations).
    `add` returns the complete polygon as [(lat, lon)] once every part is in, else None.
    The assembled polygon's epoch is re-derived and compared with the one on the air,
    so a part lost-and-replaced across two fences cannot pass."""

    def __init__(self):
        self.epoch: Optional[int] = None
        self.n_parts = 0
        self.parts: Dict[int, List[Tuple[int, int]]] = {}

    def add(self, data: bytes) -> Optional[List[Tuple[float, float]]]:
        msg = unpack_geofence_info(data)
        if msg["fence_epoch"] != self.epoch or msg["n_parts"] != self.n_parts:
            self.epoch, self.n_parts, self.parts = msg["fence_epoch"], msg["n_parts"], {}
        self.parts[msg["part"]] = [(v["lat"], v["lon"]) for v in msg["vertices"]]
        if len(self.parts) < self.n_parts:
            return None
        polygon = [pt for i in range(self.n_parts) for pt in self.parts[i]]
        e7 = [(int(round(a * 1e7)), int(round(b * 1e7))) for a, b in polygon]
        if fence_epoch(e7) != self.epoch:
            self.parts = {}
            raise RxlError("GEOFENCE_INFO: the assembled polygon does not match epoch %d" % self.epoch)
        return polygon


# ---------------------------------------------------------------------------- ACK

_ACK = struct.Struct("<BBB")


def pack_ack(*, msg_id: int, seq: int, src: int) -> bytes:
    """Acknowledge the frame `msg_id` that MAVLink header sequence `seq` (0-255) carried,
    sent by RXL node `src`. Catalogue 4.5."""
    n = "ACK"
    return _ACK.pack(_uint(n, "msg_id", msg_id, 8), _uint(n, "seq", seq, 8), _uint(n, "src", src, 8))


def unpack_ack(data: bytes) -> dict:
    _need("ACK", data, _ACK.size)
    msg_id, seq, src = _ACK.unpack(data)
    return {"msg_id": msg_id, "seq": seq, "src": src}


# --------------------------------------------------------------------- the registry

class Message(NamedTuple):
    id: int
    name: str
    unpack: Callable[[bytes], dict]
    direction: str


_REGISTRY: Dict[int, Message] = {m.id: m for m in (
    Message(HEARTBEAT, "HEARTBEAT", unpack_heartbeat, "uplink"),
    Message(SAFE_PASSAGE, "SAFE_PASSAGE", unpack_safe_passage, "uplink"),
    Message(PIPELINE_SURVEY, "PIPELINE_SURVEY", unpack_pipeline_survey, "uplink"),
    Message(RESOURCE_DELIVERY, "RESOURCE_DELIVERY", unpack_resource_delivery, "uplink"),
    Message(DOCKING, "DOCKING", unpack_docking, "uplink"),
    Message(FIREFIGHTING, "FIREFIGHTING", unpack_firefighting, "uplink"),
    Message(INCIDENT_ACK, "INCIDENT_ACK", unpack_incident_ack, "uplink"),
    Message(READINESS, "READINESS", unpack_readiness, "uplink"),
    Message(RUN_START, "RUN_START", unpack_run_start, "downlink"),
    Message(ASSISTANCE_REQUEST, "ASSISTANCE_REQUEST", unpack_assistance_request, "downlink"),
    Message(KEEP_OUT_ZONE, "KEEP_OUT_ZONE", unpack_keep_out_zone, "downlink"),
    Message(ALL_CLEAR, "ALL_CLEAR", unpack_all_clear, "downlink"),
    Message(MOVING_OBJECT_ALERT, "MOVING_OBJECT_ALERT", unpack_moving_object_alert, "downlink"),
    Message(READINESS_CONFIRM, "READINESS_CONFIRM", unpack_readiness_confirm, "downlink"),
    Message(COURSE, "COURSE", unpack_course, "downlink"),
    Message(GEOFENCE_INFO, "GEOFENCE_INFO", unpack_geofence_info, "downlink"),
    Message(ACK, "ACK", unpack_ack, "either"),
)}


def decode(ptype: int, data: bytes) -> Optional[Tuple[str, dict]]:
    """(name, fields) for a TUNNEL payload this codec decodes, None for any other
    payload type (foreign, inter-vehicle, unknown). Raises RxlError for ours but bad."""
    cid = catalogue_id(ptype)
    if cid is None or cid not in _REGISTRY:
        return None
    if len(data) > TUNNEL_MAX:
        raise RxlError("%s: payload is %d bytes, a TUNNEL holds %d" % (NAMES[cid], len(data), TUNNEL_MAX))
    msg = _REGISTRY[cid]
    try:
        return msg.name, msg.unpack(bytes(data))
    except RxlError:
        raise
    except Exception as e:                                       # noqa: BLE001
        # Belt and braces: whatever the garbage did, the caller sees RxlError.
        raise RxlError("%s: undecodable payload (%s: %s)" % (msg.name, type(e).__name__, e))
