"""validate — refuse to publish a message that will score as garbage.

Two failure modes, both of which serialize perfectly happily and are therefore
invisible until someone reads RoboCommand's logs after the run.

1. PROTO3 ZERO VALUES. Every enum here has *_UNKNOWN at 0, and 0 is what an
   unset field serializes as. So a field the reporter simply forgot to fill
   arrives as UNKNOWN rather than as an error -- and the handbook says outright
   that TASK_UNKNOWN must never be sent. "Forgot a field" and "sent an illegal
   value" are the same bug on the wire, and only this check separates them.

2. NaN. crusader_fcu/telemetry_bridge.py sets heading to NaN whenever GPS yaw is
   unresolved (msg.hdg == 65535), which is documented and correct ROS-side --
   Attitude.msg spells it out. Protobuf floats take NaN without complaint. It
   reaches scoring as a heading of "not a number". The reporter is supposed to
   substitute EKF yaw before it gets here; this is the net under that.

Descriptor API note: this uses FieldDescriptor.is_repeated, not the older
`label == LABEL_REPEATED`. protobuf 7 removed `.label` outright, and these
schemas are Edition 2024, which needs a 6.x+ runtime anyway -- so the old
spelling is not a compatibility fallback, it is just broken.

Findings are returned, never raised. A bridge that dies on a bad frame stops
reporting the other, good ones -- the right move is drop the frame, publish the
rest, and make the operator's screen say so.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Finding:
    path: str      # dotted field path, e.g. "heartbeat.current_task"
    problem: str   # "UNKNOWN" | "NaN" | "Inf"
    detail: str

    def __str__(self) -> str:
        return f"{self.path}: {self.problem} ({self.detail})"


def check(msg, *, allow_unknown: tuple[str, ...] = ()) -> list[Finding]:
    """Walk a protobuf message and report anything unfit for the wire.

    `allow_unknown` names leaf fields that may legitimately hold their UNKNOWN
    value -- flight_phase on a surface vehicle has no other honest encoding.
    Matching is on the leaf name, so one entry covers the field wherever it
    appears in the tree.
    """
    found: list[Finding] = []
    _walk(msg, "", found, frozenset(allow_unknown))
    return found


def _walk(msg, prefix: str, out: list[Finding], allowed: frozenset[str]) -> None:
    for field, value in msg.ListFields():
        path = f"{prefix}{field.name}"

        if field.is_repeated:
            for i, item in enumerate(value):
                if field.type == field.TYPE_MESSAGE:
                    _walk(item, f"{path}[{i}].", out, allowed)
                else:
                    _scalar(field, item, f"{path}[{i}]", out, allowed)
            continue

        if field.type == field.TYPE_MESSAGE:
            _walk(value, f"{path}.", out, allowed)
        else:
            _scalar(field, value, path, out, allowed)

    # ListFields() omits unset fields, so an enum left at its zero value never
    # appears above. That is precisely the case we care most about, so ask the
    # descriptor directly for every enum this message declares.
    for field in msg.DESCRIPTOR.fields:
        if field.type != field.TYPE_ENUM or field.is_repeated:
            continue
        if field.name in allowed:
            continue
        value = getattr(msg, field.name)
        if value != 0:
            continue
        zero = field.enum_type.values_by_number.get(0)
        if zero is None or not zero.name.endswith("UNKNOWN"):
            continue
        out.append(
            Finding(
                f"{prefix}{field.name}",
                "UNKNOWN",
                f"{zero.name} -- unset, or set to the zero value",
            )
        )


def _scalar(field, value, path: str, out: list[Finding], allowed: frozenset[str]) -> None:
    if field.type in (field.TYPE_FLOAT, field.TYPE_DOUBLE):
        if math.isnan(value):
            out.append(Finding(path, "NaN", "float is not a number"))
        elif math.isinf(value):
            out.append(Finding(path, "Inf", f"float is {value}"))
