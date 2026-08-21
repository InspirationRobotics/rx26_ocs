"""seqstore — sequence counters that survive the bridge dying mid-run.

RxReport.seq is scoped per vehicle; RxRequest.seq is one monotonic counter for
the team. Both are trivial to hold in memory and trivial to get wrong: the
bridge crashes at minute nine of a run, systemd restarts it, and it cheerfully
publishes seq=0 again. RoboCommand now sees a vehicle whose sequence went
backwards, which is exactly the signature of a replayed or duplicated stream.

So the counters live on disk, and every handed-out number is durable BEFORE it
goes on the wire. That ordering is the whole design: a number we published but
did not persist is a number we will hand out twice.

The cost is an fsync per message. At 2 Hz per vehicle that is noise on a laptop
SSD, and it buys back the one failure that cannot be repaired after the fact.

RUN EPOCHS. "Do not reuse messages from previous runs" (handbook 3.4), so the
file is keyed by an epoch string. Declaring a new run calls new_epoch(), which
zeroes every counter. A restart WITHIN an epoch resumes; a restart into a new
epoch starts clean.
"""
from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path


class SeqStore:
    """Durable monotonic counters. One file, rewritten atomically."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._epoch: str = ""
        self._request: int = 0
        self._reports: dict[str, int] = {}
        self._run: dict[str, object] = {}
        self._load()

    # ---- persistence ------------------------------------------------------

    def _load(self) -> None:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (FileNotFoundError, json.JSONDecodeError):
            return
        self._epoch = raw.get("epoch", "")
        self._request = int(raw.get("request", 0))
        self._reports = {str(k): int(v) for k, v in raw.get("reports", {}).items()}
        self._run = raw.get("run", {})

    def _flush(self) -> None:
        """Atomic replace + fsync. Both halves matter.

        os.replace alone leaves a window where the rename is in the directory
        cache but the bytes are not on the platter; a power cut there resurrects
        the OLD counters, which is the bug this file exists to prevent.
        """
        payload = json.dumps(
            {"epoch": self._epoch, "request": self._request,
             "reports": self._reports, "run": self._run},
            indent=2,
        )
        fd, tmp = tempfile.mkstemp(dir=str(self.path.parent), suffix=".tmp")
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as fh:
                fh.write(payload)
                fh.flush()
                os.fsync(fh.fileno())
            os.replace(tmp, self.path)
        except BaseException:
            Path(tmp).unlink(missing_ok=True)
            raise

    # ---- the counters -----------------------------------------------------

    @property
    def epoch(self) -> str:
        return self._epoch

    def new_epoch(self, epoch: str) -> None:
        """Start a run. Zeroes every counter -- see the module docstring."""
        self._epoch = epoch
        self._request = 0
        self._reports = {}
        self._run = {}
        self._flush()

    def next_request(self) -> int:
        """Next RxRequest.seq for the team."""
        self._request += 1
        self._flush()
        return self._request

    def next_report(self, vehicle_id: str) -> int:
        """Next RxReport.seq for one vehicle. Counters are independent."""
        nxt = self._reports.get(vehicle_id, 0) + 1
        self._reports[vehicle_id] = nxt
        self._flush()
        return nxt

    def save_run(self, state: str, declaration_seq: int | None,
                 run_id: int | None) -> None:
        """Persist the run itself, not just its counters.

        Durable sequence numbers are worthless on their own. A bridge that
        crashes mid-run and comes back with correct counters but no memory of
        having declared cannot legally publish a single report -- it is in
        COURSE_RX, and the only way out is to declare again, which mints a new
        RxRequest.seq and orphans the RunStart it is waiting for. So the run's
        identity is written the same way and at the same time.
        """
        self._run = {"state": state, "declaration_seq": declaration_seq,
                     "run_id": run_id}
        self._flush()

    @property
    def run(self) -> dict[str, object]:
        return dict(self._run)

    def snapshot(self) -> dict[str, object]:
        """For the status line. Never mutates."""
        return {
            "epoch": self._epoch,
            "request": self._request,
            "reports": dict(self._reports),
            "run": dict(self._run),
        }
