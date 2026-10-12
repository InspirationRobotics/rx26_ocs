"""wirelog — every frame, every direction, raw bytes kept and decoded beside them.

Three jobs it is the only thing that can do:

  * EVIDENCE. "We published that report at 14:03:11" is a claim; a logged frame
    with its bytes is the thing that settles a scoring protest.
  * DEBUGGING. The run is the only time the real system exists. Whatever went
    wrong, this file is where it happened.
  * REPLAY. Raw payloads mean a recorded run feeds straight back into the
    RoboNation stub, so a bug seen once on the water is reproducible on a desk.

JSONL, one line per frame or event, flushed on write. Not a database and not
compressed: the file has to be readable by someone who is stressed, on a laptop,
with no tooling but a text editor. One file per process start.

DIRECTIONS (the "dir" field):

    tx    OCS -> RoboCommand            (published to its broker)
    rx    RoboCommand -> OCS            (course, commands)
    veh   a vehicle -> OCS              (TCP vehicle link, or the team broker)
    vtx   OCS -> a vehicle              (RxCommand, ocs_directive, ocs_info)
    evt   not a frame: something the OCS decided -- state changes, refusals,
          dropped frames, hellos. Run-boundary events (declare, run_start, end)
          carry run_id / declaration_seq, so a run can be cut out of the file.

Each frame line keeps `b64`, the exact bytes, and `decoded`, the same frame as
readable JSON (the handbook's own field names; what a vehicle sent verbatim).
`decoded` is a convenience and `b64` is the evidence: if decoding fails the line
still carries the bytes.

THIS MUST NEVER BE ABLE TO STOP THE HOT PATH. A full disk, a closed handle, a
frame that will not decode -- none of them may raise into the caller that is in
the middle of publishing a heartbeat. Every failure is caught, counted in
`errors`, and the last one kept in `last_error` for the page to show.
"""
from __future__ import annotations

import base64
import json
import threading
import time
from pathlib import Path


class WireLog:
    def __init__(self, directory: str | Path, *, run_tag: str) -> None:
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
        self.path = directory / f"{stamp}-{run_tag}.jsonl"
        self._fh = self.path.open("a", encoding="utf-8")
        self._lock = threading.Lock()
        #: How many lines (or decodes) failed. Non-zero means the log is
        #: incomplete, and the page says so.
        self.errors = 0
        self.last_error = ""

    def frame(
        self,
        direction: str,      # "tx" | "rx" | "veh" | "vtx" -- see the docstring
        topic: str,
        payload: bytes,
        *,
        note: str = "",
        decoded: object = None,
    ) -> None:
        """One frame. `decoded` is a dict, a string, or a zero-argument callable
        returning either -- the callable is for decodes that can raise, so the
        failure is counted here instead of reaching the hot path."""
        try:
            line = {
                "t": time.time(),
                "mono": time.monotonic(),
                "dir": direction,
                "topic": topic,
                "len": len(payload),
                "b64": base64.b64encode(payload).decode("ascii"),
                "decoded": self._decode(decoded),
                "note": note,
            }
        except Exception as exc:  # noqa: BLE001
            self._fail(exc)
            return
        self._write(line)

    def event(self, kind: str, detail: str, **extra: object) -> None:
        """State changes, refusals, dropped frames -- the why between the frames.

        `extra` rides on the line as further keys: run boundaries use it for
        run_id and declaration_seq.
        """
        self._write({
            "t": time.time(),
            "mono": time.monotonic(),
            "dir": "evt",
            "kind": kind,
            "detail": detail,
            **extra,
        })

    # ---- internals ---------------------------------------------------------

    def _decode(self, decoded: object) -> object:
        if decoded is None:
            return ""
        if callable(decoded):
            try:
                decoded = decoded()
            except Exception as exc:  # noqa: BLE001
                self._fail(exc)
                return ""
        return decoded

    def _write(self, obj: dict) -> None:
        try:
            line = json.dumps(obj, default=str) + "\n"
            with self._lock:
                self._fh.write(line)
                self._fh.flush()
        except Exception as exc:  # noqa: BLE001
            self._fail(exc)

    def _fail(self, exc: BaseException) -> None:
        self.errors += 1
        self.last_error = "%s: %s" % (type(exc).__name__, exc)

    def close(self) -> None:
        with self._lock:
            try:
                self._fh.close()
            except Exception as exc:  # noqa: BLE001
                self._fail(exc)
