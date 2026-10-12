"""ocs_translate -- between the bridge's JSON and the RXL codec's dicts. Pure functions.

The bridge speaks the existing vehicle link: RxReport JSON up (the shape Ekko's and
Crusader's ocs_link.py send, which the bridge parses with protobuf ParseDict) and
RxCommand JSON down (the bridge's `_to_json`: MessageToJson with the proto field names,
and with every default-valued field OMITTED -- a missing `seq` is 0, a missing
`vehicle_type` is TYPE_UNKNOWN, a missing `latitude` is 0.0). The `ocs_info` frame
carries the course and the UAV geofence.

Key spellings: both the proto field names (`run_start`, `course_id`) and protobuf's
lowerCamelCase (`runStart`, `courseId`) are accepted everywhere, and enum values may be
the proto name ("TYPE_UAV") or the number, because the bridge's JSON may be produced
either way and a translator that works for one spelling only breaks silently on the other.

REFUSAL, NOT REPAIR. Anything that cannot be turned into a publishable report or an
encodable command raises Refusal with a plain reason. Nothing is defaulted: an unknown
enum, a position of 0,0 where a position is required, a value too large for its field.
The adapter counts the refusal and shows the reason; it never sends a guess.
"""
from __future__ import annotations

import math
import re
from datetime import datetime, timezone
from typing import Callable, Dict, List, Optional, Tuple

from . import rxl_codec as rx


class Refusal(ValueError):
    """The message cannot be translated truthfully. str(e) is the reason; `key` is the same
    reason with the values taken out, for counting (see reason_key)."""

    def __init__(self, message: str = "", key: Optional[str] = None):
        super().__init__(message)
        self.key = key


_NUMBER = re.compile(r"-?\d+(?:\.\d+)?")
_QUOTED = re.compile(r"'[^']*'|\"[^\"]*\"")


def from_rxl(e: rx.RxlError) -> Refusal:
    """The codec refused a value that does not fit its wire field; same reason, same key."""
    return Refusal(str(e), key=e.key)


def reason_key(exc: BaseException) -> str:
    """A refusal's reason without its values, so a counter has one bucket per KIND of refusal
    ("RUN_START.run_id does not fit u16") and not one per number that was refused."""
    key = getattr(exc, "key", None)
    if key:
        return key
    return _NUMBER.sub("#", _QUOTED.sub("'...'", str(exc)))[:100]


# ----------------------------------------------------------------------- JSON access

def pick(d: dict, *names, default=None):
    """First present key among `names` (proto name first, then camelCase)."""
    for n in names:
        if n in d:
            return d[n]
    return default


def camel(snake: str) -> str:
    head, *rest = snake.split("_")
    return head + "".join(w.capitalize() for w in rest)


def field(d: dict, name: str, default=None):
    return pick(d, name, camel(name), default=default)


def enum_value(table: Dict[int, str], raw, what: str) -> int:
    """An enum as the JSON carried it (name or number) -> its number. A missing field is
    the proto zero value, which every pinned enum calls UNKNOWN."""
    if raw is None:
        return 0
    if isinstance(raw, bool):
        raise Refusal("%s: %r is not an enum value" % (what, raw))
    if isinstance(raw, int):
        if raw in table:
            return raw
        raise Refusal("%s: %d is not a value of the pinned enum" % (what, raw))
    for number, name in table.items():
        if name == raw:
            return number
    raise Refusal("%s: %r is not a name in the pinned enum" % (what, raw))


def number(raw, what: str, *, default: Optional[float] = None) -> float:
    if raw is None:
        if default is None:
            raise Refusal("%s is missing" % what)
        return default
    if isinstance(raw, str):                      # protobuf JSON writes NaN/Infinity as strings
        try:
            raw = float(raw)
        except ValueError:
            raise Refusal("%s: %r is not a number" % (what, raw))
    if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(raw):
        raise Refusal("%s: %r is not a finite number" % (what, raw))
    return float(raw)


def integer(raw, what: str, *, default: Optional[int] = None) -> int:
    """A whole, non-negative number (a sequence, an id, a frequency). protobuf omits a zero
    uint32, so a missing one is `default` when the caller allows that."""
    value = number(raw, what, default=None if default is None else float(default))
    if value != int(value) or value < 0:
        raise Refusal("%s: %r is not a non-negative whole number" % (what, value))
    return int(value)


def latlng(raw, what: str, *, required: bool = True) -> Optional[Tuple[float, float]]:
    """(lat, lon) from {latitude, longitude} (or lat/lon, or a [lat, lon] pair). protobuf
    drops a coordinate that is exactly 0.0, so one missing half is 0.0 -- but a point with
    BOTH missing is a point that was never set, and is refused when it is required."""
    if raw is None:
        if required:
            raise Refusal("%s is missing" % what)
        return None
    if isinstance(raw, (list, tuple)) and len(raw) == 2:
        lat, lon = raw
    elif isinstance(raw, dict):
        lat, lon = pick(raw, "latitude", "lat"), pick(raw, "longitude", "lon", "lng")
        if lat is None and lon is None:
            if required:
                raise Refusal("%s has neither latitude nor longitude" % what)
            return None
    else:
        raise Refusal("%s: %r is not a position" % (what, raw))
    return number(lat, what + ".latitude", default=0.0), number(lon, what + ".longitude", default=0.0)


def seq_of(cmd: dict) -> int:
    return integer(field(cmd, "seq"), "RxCommand.seq", default=0)


def rfc3339(when: float) -> str:
    return datetime.fromtimestamp(when, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


# --------------------------------------------------------- uplink: RXL -> RxReport JSON

def _need_known(value: int, what: str) -> int:
    if value == 0:
        raise Refusal("%s is UNKNOWN: the OCS refuses to publish it, and nothing is substituted" % what)
    return value


def _point(lat_lon: Optional[dict]) -> Optional[dict]:
    return None if lat_lon is None else {"latitude": lat_lon["lat"], "longitude": lat_lon["lon"]}


def _heartbeat(m: dict, ctx: "UplinkContext") -> dict:
    if m["vehicle"] != ctx.node:
        raise Refusal("HEARTBEAT.vehicle is node %d but this link carries %s (node %d)"
                      % (m["vehicle"], ctx.vehicle_id, ctx.node))
    state = _need_known(m["robot_state"], "HEARTBEAT.robot_state")
    phase = _need_known(m["flight_phase"], "HEARTBEAT.flight_phase") if ctx.vehicle_type == "TYPE_UAV" \
        else m["flight_phase"]
    task = _need_known(m["task"], "HEARTBEAT.task")
    if m["task"] not in rx.RX_TASK:
        raise Refusal("HEARTBEAT.task %d is not an RxTask value" % m["task"])
    for flag, what in (("pos_valid", "position"), ("att_valid", "attitude"), ("alt_valid", "altitude")):
        if not m[flag]:
            raise Refusal("HEARTBEAT %s is flagged invalid by the sender; nothing is substituted" % what)
    if m["hdg_deg"] is None:
        raise Refusal("HEARTBEAT heading is unresolved (0xFFFF); the bridge refuses a NaN heading")
    hb = {"state": rx.ROBOT_STATE[state], "position": {"latitude": m["lat"], "longitude": m["lon"]},
          "spd_mps": m["spd_mps"], "heading_deg": m["hdg_deg"], "roll_deg": m["roll_deg"],
          "pitch_deg": m["pitch_deg"], "altitude_hae_m": m["alt_m"], "depth_m": 0.0,
          "current_task": rx.RX_TASK[task], "vehicle_type": ctx.vehicle_type}
    if ctx.vehicle_type == "TYPE_UAV":
        hb["flight_phase"] = rx.FLIGHT_PHASE[phase]
    return hb


def _safe_passage(m: dict, ctx: "UplinkContext") -> dict:
    out: dict = {}
    if m["entry"] is not None:
        out["entry_position"] = _point(m["entry"])
    if m["exit"] is not None:
        out["exit_position"] = _point(m["exit"])
    buoys = []
    for i, b in enumerate(m["buoys"]):
        state = _need_known(b["state"], "SAFE_PASSAGE.buoy[%d].state" % i)
        buoys.append({"position": {"latitude": b["lat"], "longitude": b["lon"]},
                      "state": rx.BEACON_STATE[state]})
    if buoys:
        out["buoys"] = buoys
    return out


def _pipeline_survey(m: dict, ctx: "UplinkContext") -> dict:
    segments = [rx.PIPELINE_SEGMENT[_need_known(s, "PIPELINE_SURVEY.segment[%d]" % i)]
                for i, s in enumerate(m["segments"])]
    return {"active_buoy_position": {"latitude": m["lat"], "longitude": m["lon"]}, "segments": segments}


def _resource_delivery(m: dict, ctx: "UplinkContext") -> dict:
    return {"task": rx.RX_TASK[_need_known(m["task"], "RESOURCE_DELIVERY.task")],
            "resource_color": rx.COLOR[_need_known(m["resource_color"], "RESOURCE_DELIVERY.resource_color")],
            "delivery_circle_color": rx.COLOR[_need_known(m["delivery_circle_color"],
                                                          "RESOURCE_DELIVERY.delivery_circle_color")]}


class UplinkContext:
    """What a report needs to know about the vehicle it is attributed to."""

    def __init__(self, vehicle_id: str, vehicle_type: str, node: int):
        self.vehicle_id, self.vehicle_type, self.node = vehicle_id, vehicle_type, node


#: RXL message name -> (RxReport body key, builder). One entry per uplink report.
UPLINK_BODIES: Dict[str, Tuple[str, Callable[[dict, UplinkContext], dict]]] = {
    "HEARTBEAT": ("heartbeat", _heartbeat),
    "SAFE_PASSAGE": ("safe_passage", _safe_passage),
    "PIPELINE_SURVEY": ("pipeline_survey", _pipeline_survey),
    "RESOURCE_DELIVERY": ("resource_delivery", _resource_delivery),
    "DOCKING": ("docking", lambda m, c: {"bay_id": m["bay_id"]}),
    "FIREFIGHTING": ("firefighting", lambda m, c: {"window_id": m["window_id"]}),
    "INCIDENT_ACK": ("incident_ack", lambda m, c: {"command_seq": m["cmd_seq"]}),
    "READINESS": ("readiness", lambda m, c: {"command_seq": m["cmd_seq"]}),
}


def report_from_rxl(name: str, msg: dict, ctx: UplinkContext, sent_at: float) -> dict:
    """The RxReport JSON for one decoded uplink message, attributed to ctx.vehicle_id.
    `sent_at` is when the data was in hand (the receive time): the bridge re-stamps it and
    keeps ours only as its staleness measurement. Raises Refusal."""
    try:
        body_key, build = UPLINK_BODIES[name]
    except KeyError:
        raise Refusal("%s is not an uplink report" % name)
    return {"vehicle_id": ctx.vehicle_id, "sent_at": rfc3339(sent_at), body_key: build(msg, ctx)}


def report_from_heartbeat(hb: dict, vehicle_id: str, sent_at: float) -> dict:
    """The RxReport JSON for a heartbeat body built by ocs_heartbeat."""
    return {"vehicle_id": vehicle_id, "sent_at": rfc3339(sent_at), "heartbeat": hb}


def stamp_seq(report: dict, seq: int) -> dict:
    """`report` carrying the vehicle-side `seq` (the bridge keeps it as the report's own number
    and hands it back in a ReadinessConfirm). A seq of 0 is the proto default and is omitted
    from the wire, which would read as "no seq": it is refused here."""
    if isinstance(seq, bool) or not isinstance(seq, int) or not 1 <= seq <= 0xFFFFFFFF:
        raise Refusal("report seq %r is not a uint32 above 0" % (seq,))
    return dict(report, seq=seq)


# -------------------------------------------------- downlink: RxCommand JSON -> RXL

COMMAND_BODIES = ("run_start", "assistance_request", "keep_out_zone", "all_clear",
                  "moving_object_alert", "readiness_confirm")


def command_body(cmd: dict) -> Optional[Tuple[str, dict]]:
    """(body name, body dict) of an RxCommand JSON, or None if it carries no body."""
    for name in COMMAND_BODIES:
        body = pick(cmd, name, camel(name))
        if body is not None:
            if not isinstance(body, dict):
                raise Refusal("RxCommand.%s is %r, not an object" % (name, body))
            return name, body
    return None


class Outgoing:
    """One RXL message the adapter must put on the radio, and how to key it."""
    __slots__ = ("msg_id", "payload", "key", "cmd_seq", "info")

    def __init__(self, msg_id: int, payload: bytes, key: tuple, cmd_seq: Optional[int] = None,
                 info: Optional[dict] = None):
        self.msg_id, self.payload, self.key, self.cmd_seq = msg_id, payload, key, cmd_seq
        self.info = info or {}


def node_of_vehicle_id(vehicle_id: str) -> int:
    """RXL node of a bridge vehicle id by its type prefix (USV1 -> 2, UAV1 -> 3, UUV1 -> 4)."""
    kind = {"USV": rx.NODE_USV, "UAV": rx.NODE_UAV, "UUV": rx.NODE_UUV}.get(str(vehicle_id)[:3].upper())
    if kind is None:
        raise Refusal("vehicle id %r has no RXL node (expected a USV/UAV/UUV prefix)" % (vehicle_id,))
    return kind


def _types(values) -> List[int]:
    return [enum_value(rx.VEHICLE_TYPE, v, "affected_vehicle_types") for v in (values or [])]


def command_to_outgoing(cmd: dict, *, default_vehicle_id: Optional[str] = None,
                        report_seq_of: Optional[Callable[[int], Optional[int]]] = None) -> Outgoing:
    """RxCommand JSON -> the RXL message that carries it. Raises Refusal.

    `default_vehicle_id` is the vehicle the link carries: a ReadinessConfirm that names no
    vehicle in its body (protobuf drops an empty string) falls back to the command's own
    vehicle_id, then to this. `report_seq_of` maps the report_seq in a ReadinessConfirm --
    the number THIS side stamped on the report -- to the cmd_seq the vehicle used in its
    RXL READINESS (None = no such report); left out, report_seq goes through unchanged."""
    found = command_body(cmd)
    if found is None:
        raise Refusal("RxCommand has no body")
    name, body = found
    seq = seq_of(cmd)
    try:
        if name == "run_start":
            run_id = integer(field(body, "run_id"), "run_start.run_id", default=0)
            return Outgoing(rx.RUN_START, rx.pack_run_start(run_id=run_id), ("run_start",), seq,
                            {"run_id": run_id})
        if name == "assistance_request":
            lat, lon = latlng(field(body, "position"), "assistance_request.position")
            vt = enum_value(rx.VEHICLE_TYPE, field(body, "vehicle_type"), "assistance_request.vehicle_type")
            return Outgoing(rx.ASSISTANCE_REQUEST,
                            rx.pack_assistance_request(cmd_seq=seq, lat=lat, lon=lon, vehicle_type=vt),
                            ("assist",), seq, {"affected": [vt]})
        if name == "keep_out_zone":
            lat, lon = latlng(field(body, "center"), "keep_out_zone.center")
            radius = number(field(body, "radius_m"), "keep_out_zone.radius_m")
            vt = enum_value(rx.VEHICLE_TYPE, field(body, "vehicle_type"), "keep_out_zone.vehicle_type")
            return Outgoing(rx.KEEP_OUT_ZONE,
                            rx.pack_keep_out_zone(cmd_seq=seq, lat=lat, lon=lon, radius_m=radius,
                                                  affected=[vt]),
                            ("zone", seq), seq, {"affected": [vt]})
        if name == "all_clear":
            vt = enum_value(rx.VEHICLE_TYPE, field(body, "vehicle_type"), "all_clear.vehicle_type")
            return Outgoing(rx.ALL_CLEAR, rx.pack_all_clear(cmd_seq=seq, vehicle_type=vt),
                            ("clear", seq), seq, {"affected": [vt]})
        if name == "moving_object_alert":
            lat, lon = latlng(field(body, "position"), "moving_object_alert.position")
            hdg = number(field(body, "heading_deg"), "moving_object_alert.heading_deg", default=0.0)
            spd = number(field(body, "speed_mps"), "moving_object_alert.speed_mps", default=0.0)
            types = _types(field(body, "affected_vehicle_types"))
            return Outgoing(rx.MOVING_OBJECT_ALERT,
                            rx.pack_moving_object_alert(lat=lat, lon=lon, heading_deg=hdg, speed_mps=spd,
                                                        affected=types),
                            ("moving",), seq, {"affected": types})
        ours = integer(field(body, "report_seq"), "readiness_confirm.report_seq", default=0)
        node = node_of_vehicle_id(field(body, "vehicle_id") or field(cmd, "vehicle_id")
                                  or default_vehicle_id or "")
        report_seq = ours
        if report_seq_of is not None:
            report_seq = report_seq_of(ours)
            if report_seq is None:
                raise Refusal("readiness_confirm.report_seq %d is not a READINESS this link relayed "
                              "(never sent, forgotten, or the link restarted): no confirm is sent, because "
                              "a guessed cmd_seq could confirm a different report" % ours,
                              key="readiness_confirm.report_seq matches no relayed READINESS")
        return Outgoing(rx.READINESS_CONFIRM, rx.pack_readiness_confirm(report_seq=report_seq, vehicle=node),
                        ("confirm", report_seq, node), seq, {"report_seq": ours})
    except rx.RxlError as e:
        raise from_rxl(e)


# ------------------------------------------------------------- ocs_info: course, fence

def course_payload(course: dict) -> bytes:
    """COURSE payload from an RxCourse (MessageToDict or MessageToJson shape)."""
    if not isinstance(course, dict):
        raise Refusal("course: %r is not an object" % (course,))
    course_id = field(course, "course_id") or ""
    pinger = integer(field(course, "pinger_freq_hz"), "course.pinger_freq_hz", default=0)
    corners = [latlng(c, "course.corners[%d]" % i) for i, c in enumerate(field(course, "corners") or [])]
    try:
        return rx.pack_course(course_id=course_id, pinger_freq_hz=pinger, corners=corners)
    except rx.RxlError as e:
        raise from_rxl(e)


def fence_points(fence) -> List[Tuple[float, float]]:
    """The polygon from `ocs_info.uav_geofence`: a list of LatLng JSON objects (or
    [lat, lon] pairs), or an object wrapping such a list."""
    if isinstance(fence, dict):
        fence = pick(fence, "uav_geofence", "uavGeofence", "points", "vertices", "corners")
    if not isinstance(fence, (list, tuple)):
        raise Refusal("uav_geofence: %r is not a list of points" % (fence,))
    return [latlng(p, "uav_geofence[%d]" % i) for i, p in enumerate(fence)]


def geofence_payloads(fence) -> List[bytes]:
    try:
        return rx.pack_geofence_parts(fence_points(fence))
    except rx.RxlError as e:
        raise from_rxl(e)
