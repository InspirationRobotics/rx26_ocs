"""active — which vehicles belong in the RunDeclaration.

Pure: no clock, no sockets, no protobuf. bridge.py holds the state and the
sockets; this file holds the rule, so it can be exercised on a laptop with
nothing installed -- the same split governor.py and runstate.py already use.

WHY THIS IS NOT COSMETIC. RoboCommand publishes RunStart only once EVERY vehicle
named in the declaration has reported STATE_AUTO, and it does so with no timeout
and no error (see docs/TESTING.md). Declare a vehicle that is not on the water
and the run never starts -- and the symptom is indistinguishable from a broken
link to their server, which is the wrong thing to go and debug with a timeslot
running down.

Handbook 3.1 also requires at least two vehicles in the declaration for the
Communications Proof of Readiness. That floor is enforced in bridge.declare()
against cfg.min_declared rather than here, because it is a policy about the
submission rather than about which vehicles are alive.
"""
from __future__ import annotations

#: How recently a vehicle must have reported to count as active. Generous
#: against a 2 Hz heartbeat -- ten missed frames -- and short enough that a
#: vehicle powered down a minute ago is not still being declared.
ACTIVE_WINDOW_S = 5.0


def choose_active(vehicle_ids, overrides, last_seen, now,
                  window: float = ACTIVE_WINDOW_S) -> tuple[str, ...]:
    """The active set, in config order.

    vehicle_ids: every configured vehicle, in declaration order.
    overrides:   vehicle_id -> True (force in) / False (force out). An entry
                 wins over the telemetry either way; absent means "decide from
                 whether it is reporting".
    last_seen:   vehicle_id -> wall-clock seconds of its last report.

    Order is preserved because RunDeclaration.vehicle_ids is a repeated field,
    and a stable order is what makes one run's wire log diffable against the
    last one's.
    """
    out = []
    for vid in vehicle_ids:
        forced = overrides.get(vid)
        if forced is not None:
            if forced:
                out.append(vid)
            continue
        seen = last_seen.get(vid)
        if seen is not None and now - seen <= window:
            out.append(vid)
    return tuple(out)
