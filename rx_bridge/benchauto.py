"""benchauto -- the bench dry-run switch: publish STATE_AUTO for a vehicle that says MANUAL.

WHY IT EXISTS. RoboCommand publishes RunStart only once EVERY declared vehicle
reports STATE_AUTO, and both of ours report it only when armed and in an
autonomous mode. On the bench they are disarmed, so a declared run never starts
and nothing downstream of RunStart (the relay to the vehicles, the run timer,
the end of the run) can be rehearsed. This lets the OCS tell RoboCommand what
the vehicle is not saying, so the whole run sequence can be walked on a desk.

THIS IS THE ONE PLACE THE OCS PUBLISHES A VALUE NO VEHICLE SENT, and the rest of
the design rests on never doing that. So it is the explicit, loud, operator-chosen
exception -- the same posture as ocs_web's --inject:

  * command line only, never a bridge.toml key (a config file is what a vehicle
    laptop carries to the course; a command line is typed by somebody who knows)
  * named per vehicle, and every id is checked against the fleet
  * it changes exactly one thing: heartbeat.state, STATE_MANUAL -> STATE_AUTO.
    STATE_KILLED, STATE_UNKNOWN, STATE_AUTO, every other field and every other
    report kind pass through untouched -- a killed vehicle must never be
    published as running, not even on a bench
  * the bridge counts every substitution, wire-logs it, and the pages carry a
    banner while it is on

Pure functions, no I/O, so the rules are testable without a bridge.
"""
from __future__ import annotations

from typing import Collection, Iterable

from .proto import common_pb2

_STATE = common_pb2.RobotState
MANUAL = _STATE.Value("STATE_MANUAL")
AUTO = _STATE.Value("STATE_AUTO")


def resolve(spec: str | Iterable[str], known: Iterable[str]) -> tuple[str, ...]:
    """The vehicle ids a `--bench-auto` names, in the order given, or ValueError.

    `spec` is the command-line text ("USV1,UAV1") or an already-split sequence.
    Unknown ids are an error rather than ignored: a typo that silently left a
    vehicle out would be a bench run that hangs at "waiting for AUTO", the very
    symptom this switch exists to remove.
    """
    names = spec.split(",") if isinstance(spec, str) else list(spec)
    ids = tuple(dict.fromkeys(n.strip() for n in names if n.strip()))
    if not ids:
        raise ValueError("names no vehicle")
    fleet = tuple(known)
    bad = [i for i in ids if i not in fleet]
    if bad:
        raise ValueError("%s %s not in the fleet (configured: %s)"
                         % (", ".join(bad), "is" if len(bad) == 1 else "are",
                            ", ".join(fleet)))
    return ids


def promote(report, vehicles: Collection[str]) -> str | None:
    """Rewrite `report` IN PLACE if it is a MANUAL heartbeat from a listed vehicle.

    Returns the state it was (always "STATE_MANUAL") when it rewrote it, else None
    and the report is exactly as it came in.
    """
    if (report.vehicle_id not in vehicles
            or report.WhichOneof("body") != "heartbeat"
            or report.heartbeat.state != MANUAL):
        return None
    report.heartbeat.state = AUTO
    return _STATE.Name(MANUAL)


def wire_note(original: str) -> str:
    """The suffix a substituted frame carries in the wire log's note."""
    return " BENCH-AUTO %s -> %s" % (original, _STATE.Name(AUTO))
