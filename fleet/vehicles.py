"""What the fleet is doing right now, per MAVLink sysid. Pure -- no I/O, no threads,
no clock of its own (the caller passes `now`; callers hold the hub's lock).

BLANKS OVER GUESSES, as in the umbrella CLAUDE.md. Every measured value carries
the age of the message it came from. Once that age passes `stale_s` the value is
reported as null and the age is kept, so a consumer can tell "not heard for 40 s"
from "heard just now and the value is zero". Nothing here back-fills a last-known
value, and a position of exactly 0,0 (what an autopilot reports with no GPS fix)
is a blank, not a point in the Gulf of Guinea.

Only HEARTBEAT, GLOBAL_POSITION_INT and GPS_RAW_INT are decoded; every other frame
just keeps the sysid's "last heard" time, link and rate up to date.

POSITION comes from GLOBAL_POSITION_INT. GPS_RAW_INT stands in for it only when
GLOBAL_POSITION_INT has no usable position right now (never sent, stale, or 0,0 because
the EKF has no origin yet) AND the GPS reports at least a 2D fix. The view says which
message it used (`source`), so a consumer is never shown a raw-GPS point as if it were
the EKF's.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Dict, Iterable, Mapping, Optional

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as _ap

STALE_S = 3.0
MSG_HEARTBEAT = 0
MSG_GPS_RAW_INT = 24
MSG_GLOBAL_POSITION_INT = 33
_DECODED_IDS = frozenset((MSG_HEARTBEAT, MSG_GPS_RAW_INT, MSG_GLOBAL_POSITION_INT))
_MIN_GPS_FIX = _ap.GPS_FIX_TYPE_2D_FIX    # below this a GPS_RAW_INT lat/lon is not a position
_MODE_FLAG_ARMED = 128           # MAV_MODE_FLAG_SAFETY_ARMED
_AUTOPILOT_INVALID = _ap.MAV_AUTOPILOT_INVALID     # a companion / peripheral, not a flight controller
_UNKNOWN_U16 = 65535             # "unknown" in GLOBAL_POSITION_INT.hdg, GPS_RAW_INT.vel and .yaw

# One decoder for every frame: decode() keeps no per-source state we depend on, and
# callers serialise on the hub's lock.
_decoder = _ap.MAVLink(None)


class VehicleNames:
    """sysid -> vehicle, from the [vehicles] table of fleet.toml, and the vehicle's
    display name from the optional [names] table.

    `name_of` is the role key ("boat", "drone", "radio", or "unknown"); `display_of`
    is what a map should print ("Crusader", "Ekko", ...). A sysid in no group has no
    name to show, so it is labelled by its number rather than given someone else's."""

    def __init__(self, groups: Mapping[str, Iterable[int]],
                 display: Optional[Mapping[str, str]] = None):
        self._display = dict(display or {})
        self._by_sysid: Dict[int, str] = {}
        for name, ids in groups.items():
            for sysid in ids:
                if sysid in self._by_sysid:
                    raise ValueError("sysid %d is listed under both %r and %r"
                                     % (sysid, self._by_sysid[sysid], name))
                self._by_sysid[sysid] = name

    def name_of(self, sysid: int) -> str:
        return self._by_sysid.get(sysid, "unknown")

    def display_of(self, sysid: int) -> str:
        role = self._by_sysid.get(sysid)
        return "sysid %d" % sysid if role is None else self._display.get(role, role)


class RateMeter:
    """Events (or bytes) per second over a sliding window, in constant memory:
    counts are kept in quarter-second buckets."""
    _BUCKETS_PER_S = 4

    def __init__(self, window_s: float = 5.0):
        self.window_s = window_s
        self._buckets: deque = deque()       # [bucket_index, total]
        self._sum = 0
        self._t0: Optional[float] = None

    def _trim(self, now: float) -> None:
        oldest = int((now - self.window_s) * self._BUCKETS_PER_S)
        while self._buckets and self._buckets[0][0] <= oldest:
            self._sum -= self._buckets.popleft()[1]

    def add(self, n: float, now: float) -> None:
        if self._t0 is None:
            self._t0 = now
        idx = int(now * self._BUCKETS_PER_S)
        if self._buckets and self._buckets[-1][0] == idx:
            self._buckets[-1][1] += n
        else:
            self._buckets.append([idx, n])
        self._sum += n
        self._trim(now)

    def rate(self, now: float) -> float:
        if self._t0 is None:
            return 0.0
        self._trim(now)
        # Until a full window has elapsed, divide by the time actually observed
        # (at least a second) rather than under-reporting at start-up.
        return self._sum / min(self.window_s, max(now - self._t0, 1.0))


def _enum_name(enum: str, value: int, prefix: str) -> str:
    entry = _ap.enums[enum].get(value)
    if entry is None:
        return "#%d" % value
    name = entry.name
    return name[len(prefix):] if name.startswith(prefix) else name


class _Heartbeat:
    __slots__ = ("t", "type", "autopilot", "mode", "armed", "is_autopilot")

    def __init__(self, t, msg):
        self.t = t
        self.type = _enum_name("MAV_TYPE", msg.type, "MAV_TYPE_")
        self.autopilot = _enum_name("MAV_AUTOPILOT", msg.autopilot, "MAV_AUTOPILOT_")
        self.is_autopilot = msg.autopilot != _AUTOPILOT_INVALID
        # A companion computer's heartbeat has no flight mode; pymavlink would still
        # format one ("Mode(0x...)"), which would be a made-up value.
        self.mode = mavutil.mode_string_v10(msg) if self.is_autopilot else None
        self.armed = bool(msg.base_mode & _MODE_FLAG_ARMED) if self.is_autopilot else None


class _Position:
    """A position-bearing message reduced to what it can honestly say. Anything the
    message does not carry, or marks unknown, is None."""
    __slots__ = ("t", "source", "lat", "lon", "alt_m", "rel_alt_m", "heading_deg",
                 "groundspeed_mps")

    def __init__(self, t, source, lat_e7, lon_e7, alt_mm, rel_alt_mm, heading_deg, speed_mps):
        self.t = t
        self.source = source
        has_fix = not (lat_e7 == 0 and lon_e7 == 0)
        self.lat = lat_e7 / 1e7 if has_fix else None
        self.lon = lon_e7 / 1e7 if has_fix else None
        # MSL altitude and ground speed with no fix are a barometer against an assumed
        # datum and an EKF with no velocity source: blank. relative_alt is height above
        # home/boot and a real measurement either way.
        self.alt_m = alt_mm / 1000.0 if has_fix else None
        self.rel_alt_m = None if rel_alt_mm is None else rel_alt_mm / 1000.0
        self.heading_deg = heading_deg
        self.groundspeed_mps = speed_mps if has_fix else None

    @property
    def has_fix(self) -> bool:
        return self.lat is not None


def _position_from_global(t, msg) -> _Position:
    return _Position(t, "GLOBAL_POSITION_INT", msg.lat, msg.lon, msg.alt, msg.relative_alt,
                     None if msg.hdg == _UNKNOWN_U16 else msg.hdg / 100.0,
                     math.hypot(msg.vx, msg.vy) / 100.0)


def _position_from_gps_raw(t, msg) -> _Position:
    fixed = msg.fix_type >= _MIN_GPS_FIX
    # `yaw` (a GPS with two antennas): 0 = this GPS gives none, 65535 = it should but
    # cannot right now, 36000 = north. COG is a course, not a heading, so it is not used.
    yaw_ok = 0 < msg.yaw < _UNKNOWN_U16
    return _Position(t, "GPS_RAW_INT", msg.lat if fixed else 0, msg.lon if fixed else 0,
                     msg.alt, None, (msg.yaw / 100.0) % 360.0 if yaw_ok else None,
                     None if msg.vel == _UNKNOWN_U16 else msg.vel / 100.0)


class _Record:
    __slots__ = ("last_t", "last_link", "rate", "heartbeats", "position", "gps_position")

    def __init__(self):
        self.last_t = 0.0
        self.last_link = None
        self.rate = RateMeter()
        self.heartbeats: Dict[int, _Heartbeat] = {}      # by compid
        self.position: Optional[_Position] = None        # from GLOBAL_POSITION_INT
        self.gps_position: Optional[_Position] = None    # from GPS_RAW_INT: the fallback


def _r(value, ndigits):
    return None if value is None else round(value, ndigits)


_BLANK_HEARTBEAT = {"age_s": None, "type": None, "autopilot": None, "mode": None, "armed": None}
_BLANK_POSITION = {"age_s": None, "source": None, "lat": None, "lon": None, "alt_m": None,
                   "rel_alt_m": None, "heading_deg": None, "groundspeed_mps": None}


class VehicleTable:
    def __init__(self, names: VehicleNames, stale_s: float = STALE_S):
        self.names = names
        self.stale_s = stale_s
        self._records: Dict[int, _Record] = {}
        self.decode_errors = 0       # frames pymavlink refused; they were still forwarded

    def note(self, frame, link: str, now: float) -> None:
        """Record a CRC-validated frame. Callers must not pass unvalidated frames:
        a junk frame would otherwise invent a vehicle."""
        rec = self._records.get(frame.sysid)
        if rec is None:
            rec = self._records[frame.sysid] = _Record()
        rec.last_t = now
        rec.last_link = link
        rec.rate.add(1, now)
        if frame.msgid in _DECODED_IDS:
            # Whatever pymavlink makes of this frame, it must never stop the hub
            # going on to forward it: a view we cannot build is a blank, not an error.
            try:
                msg = _decoder.decode(bytearray(frame.raw))
                if frame.msgid == MSG_HEARTBEAT:
                    rec.heartbeats[frame.compid] = _Heartbeat(now, msg)
                elif frame.msgid == MSG_GPS_RAW_INT:
                    rec.gps_position = _position_from_gps_raw(now, msg)
                else:
                    rec.position = _position_from_global(now, msg)
            except Exception:                                    # noqa: BLE001
                self.decode_errors += 1

    # ---------------------------------------------------------------- snapshot

    def _aged_view(self, blank: dict, t: Optional[float], now: float, values) -> dict:
        """`blank` (every key None) with the age filled in once something has been
        heard, and the values filled in only while that is no older than stale_s."""
        view = dict(blank)
        if t is None:
            return view
        age = now - t
        view["age_s"] = round(age, 2)
        if age <= self.stale_s:
            view.update(values())
        return view

    def _heartbeat_view(self, rec: _Record, now: float) -> dict:
        """The flight controller's heartbeat if there is one, else the newest of
        whatever else the sysid sends. (Ekko's Cube and its companion may share a
        sysid; mixing them would flip the mode between two components.)"""
        beats = list(rec.heartbeats.values())
        pool = [h for h in beats if h.is_autopilot] or beats
        hb = max(pool, key=lambda h: h.t) if pool else None
        return self._aged_view(
            _BLANK_HEARTBEAT, None if hb is None else hb.t, now,
            lambda: {"type": hb.type, "autopilot": hb.autopilot, "mode": hb.mode,
                     "armed": hb.armed})

    def _chosen_position(self, rec: _Record, now: float) -> Optional[_Position]:
        """GLOBAL_POSITION_INT if it holds a fresh fix, else GPS_RAW_INT if THAT does,
        else whichever arrived last (so the view can still say how old it is)."""
        have = [p for p in (rec.position, rec.gps_position) if p is not None]
        for p in have:
            if p.has_fix and now - p.t <= self.stale_s:
                return p
        return max(have, key=lambda p: p.t, default=None)

    def _position_view(self, rec: _Record, now: float) -> dict:
        p = self._chosen_position(rec, now)
        return self._aged_view(
            _BLANK_POSITION, None if p is None else p.t, now,
            lambda: {"source": p.source, "lat": _r(p.lat, 7), "lon": _r(p.lon, 7),
                     "alt_m": _r(p.alt_m, 2), "rel_alt_m": _r(p.rel_alt_m, 2),
                     "heading_deg": _r(p.heading_deg, 2),
                     "groundspeed_mps": _r(p.groundspeed_mps, 2)})

    def snapshot(self, now: float) -> dict:
        out = {}
        for sysid in sorted(self._records):
            rec = self._records[sysid]
            out[str(sysid)] = {
                "sysid": sysid,
                "vehicle": self.names.name_of(sysid),
                "name": self.names.display_of(sysid),
                "link": rec.last_link,
                "age_s": round(now - rec.last_t, 2),
                "msgs_per_s": round(rec.rate.rate(now), 1),
                "heartbeat": self._heartbeat_view(rec, now),
                "position": self._position_view(rec, now),
            }
        return out
