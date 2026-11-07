"""ocs_heartbeat -- Ekko's RoboCommand heartbeat, built on the laptop from the autopilot's
own MAVLink as it arrives on the radio. Stop-gap: it ends when Ekko's companion sends
its own RXL HEARTBEAT (0x10), which always takes precedence (see ocs_adapter).

A PORT, NOT A REDESIGN. The rules are those of the aircraft's own reporter,
Drone/rx26_uav/uav_groundstation/uav_groundstation/heartbeat_core.py (cited per
function below), fed from the same MAVLink messages telemetry_bridge.py reads
(HEARTBEAT :498, GLOBAL_POSITION_INT :480, ATTITUDE :488, EXTENDED_SYS_STATE :506).
Where this file differs from the original it says so, and every difference is toward
sending LESS:

  * a position of exactly 0,0 (the EKF has no origin) is not a position. heartbeat_core
    passes it through; here it suppresses the heartbeat.
  * a stale ATTITUDE suppresses the heartbeat. heartbeat_core omits roll and pitch, but
    in a protobuf float an omitted field is 0.0 on the wire -- a claim of "level".
  * the armed+altitude flight-phase fallback (heartbeat_core.flight_phase) is OFF unless
    the config turns it on. The aircraft's own node uses it, loudly, when EXTENDED_SYS_STATE
    is missing; on the radio EXTENDED_SYS_STATE is not in any stream group (ArduCopter 4.7
    sends it only when asked per message), so the adapter asks for it with
    MAV_CMD_SET_MESSAGE_INTERVAL (REQUEST_INTERVAL_US below) -- the one thing it ever sends
    the autopilot.
    Network Remote ID is fed from this field, so a heuristic is opt-in, not the default.

THE ONE RULE (OCS/CLAUDE.md "Never invent a value"): every path that cannot produce a
truthful field returns (None, reason) -- "send NO heartbeat". A gap shows on the page as
rising silence; a fabricated value is indistinguishable from a real one. The reason is
kept and shown in /fleet, so the silence is explained, not just observed.

What it cannot know, and says so: current_task. It is the literal TASK_NONE, on purpose and
exactly as Ekko's own heartbeat_core.py:138 sends it ("no task-state source exists on the
aircraft yet"): this mirrors the aircraft's own reporter, it is not our guess about what the
aircraft is doing. Only Ekko's own heartbeat can ever mark a task attempt begun.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as _ap

from .mavframe import Frame

MSG_HEARTBEAT = 0
MSG_ATTITUDE = 30
MSG_GLOBAL_POSITION_INT = 33
MSG_EXTENDED_SYS_STATE = 245
MESSAGE_IDS = frozenset((MSG_HEARTBEAT, MSG_ATTITUDE, MSG_GLOBAL_POSITION_INT, MSG_EXTENDED_SYS_STATE))

AUTOPILOT_COMPID = 1                       # telemetry_bridge.py:498  msg.get_srcComponent() == 1
_MODE_FLAG_ARMED = _ap.MAV_MODE_FLAG_SAFETY_ARMED
_HDG_UNKNOWN = 65535                       # telemetry_bridge.py:481

# heartbeat_core.py:28-34 -- MAV_LANDED_STATE values
LANDED_UNDEFINED = 0
LANDED_ON_GROUND = 1
LANDED_IN_AIR = 2
LANDED_TAKEOFF = 3
LANDED_LANDING = 4

GROUNDED = "FLIGHT_PHASE_GROUNDED"
AIRBORNE = "FLIGHT_PHASE_AIRBORNE"

# heartbeat_core.py:42-47. TAKEOFF and LANDING are AIRBORNE: off the ground in both, and
# the proto offers two values. Reporting a mid-takeoff aircraft as grounded is the more
# dangerous rounding.
_PHASE_BY_LANDED = {
    LANDED_ON_GROUND: GROUNDED,
    LANDED_IN_AIR: AIRBORNE,
    LANDED_TAKEOFF: AIRBORNE,
    LANDED_LANDING: AIRBORNE,
}

#: heartbeat_core.py:52-55 -- modes in which an armed ArduCopter is flying itself. Anything
#: else, and anything at all while disarmed, is STATE_MANUAL. STATE_UNKNOWN is never sent.
#: LOITER is NOT here (Chris, 6 Nov 2026): it is flown on the sticks, and the handbook's
#: STATE_AUTO means "autonomous and ready" -- a pilot taking over into Loiter must show as
#: MANUAL on MyDroneFleets. Ekko's own heartbeat_core.py still lists it.
AUTONOMOUS_MODES = frozenset({
    "AUTO", "GUIDED", "GUIDED_NOGPS", "RTL", "SMART_RTL", "LAND",
    "AUTO_RTL", "CIRCLE", "FOLLOW", "ZIGZAG",
})

#: heartbeat_core.py:138 -- the same literal Ekko's own reporter sends (no task-state source
#: exists on the aircraft); see the module docstring.
CURRENT_TASK = "TASK_NONE"

#: How the adapter asks for EXTENDED_SYS_STATE, taken from the one tool that already does:
#: Drone/rx26_uav/tools/scripts/gcs_radio.py WANTED has (245, "EXTENDED_SYS_STATE", 1.0), i.e.
#: SET_MESSAGE_INTERVAL with param2 = 1e6/1.0 us, and WANTED_STALE_S = 10.0 -- a message
#: unheard for that long is asked for again, and never otherwise (re-sending a request that is
#: already honoured stalled the aircraft's telemetry for 5-10 s). The SAME interval means the
#: two requesters can never fight over the rate: each one's request is the other's no-op.
REQUEST_INTERVAL_US = 1_000_000
REQUEST_STALE_S = 10.0

_decoder = _ap.MAVLink(None)               # decode() keeps no per-source state; callers serialise


# ------------------------------------------------------------- the ported core

def flight_phase(landed, *, armed=None, altitude_rel=None, airborne_alt_m=1.0,
                 allow_fallback=False):
    """(phase, source). phase is None when it cannot be determined truthfully.

    heartbeat_core.py:58-83, with `allow_fallback` added (default False, see the module
    docstring). An UNDEFINED landed_state is the autopilot saying it does not know; with
    the fallback off that is a blank, not a guess."""
    if landed is not None and landed != LANDED_UNDEFINED:
        phase = _PHASE_BY_LANDED.get(landed)
        if phase is not None:
            return phase, "autopilot"
        # A landed_state outside the mapping means a newer MAVLink than this table.
        # Guessing is exactly what the UNKNOWN ban forbids.
        return None, None
    if not allow_fallback or armed is None or altitude_rel is None:
        return None, None
    airborne = bool(armed) and altitude_rel > airborne_alt_m
    return (AIRBORNE if airborne else GROUNDED), "fallback"


#: DISARMED in one of these = "autonomous and ready" (handbook 3.4: STATE_AUTO "when that vehicle is
#: autonomous and ready"; vehicles wait for RunStart). The pilot has switched into the autonomous mode
#: (SC -> GUIDED) and the aircraft sits on the ground, not armed. Chris, 7 Nov 2026. Ekko's own
#: heartbeat_core.py still says MANUAL for anything disarmed.
WAITING_AUTONOMOUS_MODES = frozenset({"GUIDED"})


def robot_state(mode, armed):
    """heartbeat_core.py:86-91, plus WAITING_AUTONOMOUS_MODES. Never STATE_UNKNOWN (the zero value)."""
    if not armed:
        return "STATE_AUTO" if (mode or "").upper() in WAITING_AUTONOMOUS_MODES else "STATE_MANUAL"
    return "STATE_AUTO" if (mode or "").upper() in AUTONOMOUS_MODES else "STATE_MANUAL"


# ---------------------------------------------------------------- the inputs

@dataclass(frozen=True)
class Timeouts:
    """How old an input may be before it counts as absent. The aircraft's own node uses
    1.0 s for all four (uav_params.yaml:ocs_client). The autopilot's HEARTBEAT is 1 Hz on
    the radio and EXTENDED_SYS_STATE is asked for at 1 Hz, so those two get headroom for
    one lost frame; position and attitude keep the aircraft's 1.0 s."""
    pose_s: float = 1.0
    attitude_s: float = 1.0
    status_s: float = 2.5
    flight_s: float = 2.5


class _Stamped:
    __slots__ = ("t", "value")

    def __init__(self, t, value):
        self.t, self.value = t, value


def _age(rec: Optional[_Stamped], now: float) -> Optional[float]:
    return None if rec is None else round(now - rec.t, 2)


def _fresh(rec: Optional[_Stamped], now: float, timeout: float):
    return rec.value if rec is not None and now - rec.t <= timeout else None


class HeartbeatSynth:
    """Collects the autopilot's four messages and says what heartbeat they support."""

    def __init__(self, *, autopilot_sysid: int, geoid_separation_m: Optional[float],
                 airborne_alt_m: float = 1.0, phase_fallback: bool = False,
                 vehicle_type: str = "TYPE_UAV", timeouts: Timeouts = Timeouts()):
        self.autopilot_sysid = autopilot_sysid
        self.geoid_separation_m = geoid_separation_m
        self.airborne_alt_m = airborne_alt_m
        self.phase_fallback = phase_fallback
        self.vehicle_type = vehicle_type
        self.timeouts = timeouts
        self._pose: Optional[_Stamped] = None        # (lat, lon, hdg_deg|None, spd, amsl, rel)
        self._attitude: Optional[_Stamped] = None    # (roll_rad, pitch_rad)
        self._status: Optional[_Stamped] = None      # (mode, armed)
        self._flight: Optional[_Stamped] = None      # landed_state
        self.decode_errors = 0
        #: When the data behind the last heartbeat that build() produced was in hand: the
        #: newest of the inputs it was built from. None after a build that produced nothing.
        self.data_t: Optional[float] = None

    def wants(self, frame: Frame) -> bool:
        return (frame.sysid == self.autopilot_sysid and frame.compid == AUTOPILOT_COMPID
                and frame.msgid in MESSAGE_IDS)

    def note(self, frame: Frame, now: float) -> None:
        """Record one CRC-validated frame from the autopilot. Anything else is ignored.
        A frame pymavlink refuses is counted and skipped: it cannot become a value."""
        if not self.wants(frame) or not frame.validated:
            return
        try:
            msg = _decoder.decode(bytearray(frame.raw))
            if frame.msgid == MSG_GLOBAL_POSITION_INT:
                # telemetry_bridge.py:481-487
                hdg = None if msg.hdg == _HDG_UNKNOWN else msg.hdg / 100.0
                self._pose = _Stamped(now, (msg.lat / 1e7, msg.lon / 1e7, hdg,
                                            math.hypot(msg.vx, msg.vy) / 100.0,
                                            msg.alt / 1000.0, msg.relative_alt / 1000.0))
            elif frame.msgid == MSG_ATTITUDE:
                self._attitude = _Stamped(now, (msg.roll, msg.pitch))
            elif frame.msgid == MSG_HEARTBEAT:
                if msg.autopilot != _ap.MAV_AUTOPILOT_INVALID:
                    self._status = _Stamped(now, (mavutil.mode_string_v10(msg),
                                                  bool(msg.base_mode & _MODE_FLAG_ARMED)))
            else:
                self._flight = _Stamped(now, int(msg.landed_state))
        except Exception:                                            # noqa: BLE001
            self.decode_errors += 1

    # --------------------------------------------------------------- the decision

    def build(self, now: float) -> Tuple[Optional[dict], str]:
        """(heartbeat body, source) -- source "autopilot" or "fallback" -- or
        (None, reason). heartbeat_core.py:94-147. On success `data_t` is the time of the
        newest autopilot message the body was built from (the data's time, not `now`); on
        (None, reason) it is None.

        current_task is TASK_NONE, exactly as heartbeat_core.py:138 sends it: this is a mirror
        of the aircraft's own reporter, not a statement about what the aircraft is doing."""
        self.data_t = None
        t = self.timeouts
        pose = _fresh(self._pose, now, t.pose_s)
        if pose is None:
            return None, ("no fresh GLOBAL_POSITION_INT from sysid %d (never heard, or older than %.1f s)"
                          % (self.autopilot_sysid, t.pose_s))
        status = _fresh(self._status, now, t.status_s)
        if status is None:
            return None, ("no fresh autopilot HEARTBEAT from sysid %d (never heard, or older than %.1f s): "
                          "armed state and mode cannot be known" % (self.autopilot_sysid, t.status_s))
        lat, lon, hdg, spd, amsl, rel = pose
        if lat == 0.0 and lon == 0.0:
            return None, "GLOBAL_POSITION_INT is at 0,0: the EKF has no position origin yet"
        if hdg is None:
            return None, ("heading unresolved (GLOBAL_POSITION_INT.hdg = 65535); the bridge refuses a "
                          "NaN heading, and no plausible value is substituted")
        attitude = _fresh(self._attitude, now, t.attitude_s)
        if attitude is None:
            return None, ("no fresh ATTITUDE from sysid %d (roll and pitch would otherwise go out "
                          "as 0.0)" % self.autopilot_sysid)
        if self.geoid_separation_m is None:
            return None, "geoid_separation_m is not configured: HAE = AMSL + it, and it is venue-specific"
        mode, armed = status
        landed = _fresh(self._flight, now, t.flight_s)
        phase, source = flight_phase(landed, armed=armed, altitude_rel=rel,
                                     airborne_alt_m=self.airborne_alt_m,
                                     allow_fallback=self.phase_fallback)
        if phase is None:
            if landed is None:
                why = ("no fresh EXTENDED_SYS_STATE (it is in no stream group; it is on the radio only once "
                       "requested -- see heartbeat.flight_state_request) and the armed+altitude fallback is %s"
                       % ("on but armed state or altitude is missing" if self.phase_fallback else "off"))
            elif landed == LANDED_UNDEFINED:
                why = "the autopilot reports landed_state UNDEFINED and the fallback is off"
            else:
                why = "landed_state %d is not in the mapping" % landed
            return None, ("flight phase unknown -- the OCS refuses FLIGHT_PHASE_UNKNOWN from a UAV, and a "
                          "guess is relayed for Network Remote ID: " + why)
        used = [self._pose.t, self._status.t, self._attitude.t]
        if source == "autopilot":
            used.append(self._flight.t)
        self.data_t = max(used)
        hb = {
            "state": robot_state(mode, armed),
            "position": {"latitude": lat, "longitude": lon},
            "spd_mps": spd,
            "heading_deg": hdg,
            "altitude_hae_m": amsl + self.geoid_separation_m,        # heartbeat_core.py:132
            "depth_m": 0.0,
            "vehicle_type": self.vehicle_type,
            "current_task": CURRENT_TASK,
            "flight_phase": phase,
            "roll_deg": math.degrees(attitude[0]),                   # heartbeat_core.py:145
            "pitch_deg": math.degrees(attitude[1]),
        }
        return hb, source

    def armed(self, now: float) -> Optional[bool]:
        """The autopilot's armed flag from a HEARTBEAT within the status timeout; None when there
        is none -- unknown, which is not the same as disarmed."""
        status = _fresh(self._status, now, self.timeouts.status_s)
        return None if status is None else status[1]

    def position(self, now: float) -> Optional[Tuple[float, float]]:
        """(lat, lon) from a GLOBAL_POSITION_INT within the pose timeout; None when there is none
        or it is the 0,0 of an EKF without an origin."""
        pose = _fresh(self._pose, now, self.timeouts.pose_s)
        if pose is None or (pose[0] == 0.0 and pose[1] == 0.0):
            return None
        return pose[0], pose[1]

    def flight_age(self, now: float) -> Optional[float]:
        """Seconds since EXTENDED_SYS_STATE was last heard (any age), None if never."""
        return None if self._flight is None else now - self._flight.t

    def status_age(self, now: float) -> Optional[float]:
        """Seconds since the autopilot's own HEARTBEAT was last heard, None if never."""
        return None if self._status is None else now - self._status.t

    def inputs_status(self, now: float) -> Dict[str, Optional[float]]:
        return {"pose_age_s": _age(self._pose, now), "attitude_age_s": _age(self._attitude, now),
                "status_age_s": _age(self._status, now), "landed_age_s": _age(self._flight, now)}
