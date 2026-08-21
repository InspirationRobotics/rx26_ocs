"""governor — the 5 msg/s cap, with heartbeats given right of way.

The handbook caps a system at 5 messages per second and separately REQUIRES a
2 Hz heartbeat per vehicle. Those two numbers are in tension the moment a task
starts producing reports: a burst of Task 4 traffic that spends the whole budget
does not merely delay a report, it drops the vehicle below its mandated
heartbeat rate, which is the one obligation that runs for the entire run.

So the bucket is not first-come-first-served. Reports may only spend down to a
floor; heartbeats may spend the floor. A vehicle can therefore always afford its
2 Hz no matter how much task traffic is queued behind it.

WHICH "SYSTEM" THE CAP APPLIES TO is genuinely ambiguous in the handbook -- read
as per-team, three vehicles at the mandated 2 Hz would breach it on heartbeats
alone (6 > 5), which cannot be the intent. We therefore meter PER VEHICLE and
have asked RoboNation to confirm. If the answer is per-team, construct one
Governor and share it across vehicles; nothing else changes.

Pure arithmetic, no clock of its own -- the caller passes `now`, so the tests
run in zero seconds instead of sleeping.
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Governor:
    """Token bucket with a reserved floor for heartbeats.

    rate:     tokens added per second, and the bucket's capacity.
    reserve:  tokens only heartbeats may spend.
    """
    rate: float = 5.0
    reserve: float = 2.0
    _tokens: float = field(default=None, repr=False)  # type: ignore[assignment]
    _last: float | None = field(default=None, repr=False)

    #: Counts for the status line, so a starved link is visible not inferred.
    passed: int = 0
    dropped: int = 0

    def __post_init__(self) -> None:
        if self._tokens is None:
            self._tokens = self.rate

    def _refill(self, now: float) -> None:
        if self._last is None:
            self._last = now
            return
        elapsed = max(0.0, now - self._last)
        self._last = now
        self._tokens = min(self.rate, self._tokens + elapsed * self.rate)

    def allow(self, now: float, *, heartbeat: bool) -> bool:
        """Spend a token if the caller's class may afford one."""
        self._refill(now)
        floor = 0.0 if heartbeat else self.reserve
        if self._tokens - 1.0 >= floor - 1e-9:
            self._tokens -= 1.0
            self.passed += 1
            return True
        self.dropped += 1
        return False

    @property
    def tokens(self) -> float:
        return self._tokens
