"""wirelog — every frame, both directions, raw bytes kept.

Three jobs it is the only thing that can do:

  * EVIDENCE. "We published that report at 14:03:11" is a claim; a logged frame
    with its bytes is the thing that settles a scoring protest.
  * DEBUGGING. The run is the only time the real system exists. Whatever went
    wrong, this file is where it happened.
  * REPLAY. Raw payloads mean a recorded run feeds straight back into the
    RoboNation stub, so a bug seen once on the water is reproducible on a desk.

JSONL, one frame per line, flushed on write. Not a database and not compressed:
the file has to be readable by someone who is stressed, on a laptop, with no
tooling but a text editor.
"""
from __future__ import annotations

import base64
import json
import time
from pathlib import Path


class WireLog:
    def __init__(self, directory: str | Path, *, run_tag: str) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
        self.path = directory / f"{stamp}-{run_tag}.jsonl"
        self._fh = self.path.open("a", encoding="utf-8")

    def frame(
        self,
        direction: str,      # "tx" to RoboCommand, "rx" from it, "veh" from a vehicle
        topic: str,
        payload: bytes,
        *,
        note: str = "",
        decoded: str = "",
    ) -> None:
        self._write({
            "t": time.time(),
            "mono": time.monotonic(),
            "dir": direction,
            "topic": topic,
            "len": len(payload),
            "b64": base64.b64encode(payload).decode("ascii"),
            "decoded": decoded,
            "note": note,
        })

    def event(self, kind: str, detail: str) -> None:
        """State changes, refusals, dropped frames -- the why between the frames."""
        self._write({
            "t": time.time(),
            "mono": time.monotonic(),
            "dir": "evt",
            "kind": kind,
            "detail": detail,
        })

    def _write(self, obj: dict) -> None:
        self._fh.write(json.dumps(obj) + "\n")
        self._fh.flush()

    def close(self) -> None:
        self._fh.close()
