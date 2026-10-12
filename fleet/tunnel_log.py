"""tunnel_log -- every custom inter-vehicle message fleet_link sees, to a JSONL file.

THE REQUIREMENT. "Log all information ... custom communication with vehicles through
tunnel; don't log other MAVLink messages, since they are in the drone's flight log."
So the filter is the whole point of this file, and it is deliberately narrow:

  * MAVLink TUNNEL (385) -- the carrier for the RXL catalogue messages and for every
    other thing the vehicles say to each other over the radio.
  * The robotx dialect's own messages (RXL_SAFE_PASSAGE 42011, 42018, 42019 ...): the
    boat speaks those natively. The set is read from the dialect fleet_link loaded,
    not listed here. (If that dialect failed to load, the 42xxx block it would have
    defined is logged instead, so a missing file does not silently empty the log.)

Nothing else. GLOBAL_POSITION_INT, HEARTBEAT, RADIO_STATUS, QGC's traffic -- not here.

WHAT IS LOGGED, one JSON object per line:

  rx       a frame accepted from a link (per-link de-duplicated, CRC-checked)
  tx       a frame written toward a link: from a consumer's output (origin
           "output:qgc") or built by fleet_link itself (origin "adapter:UAV1")
  refused  a frame a consumer's output offered that went nowhere: it was listen-only,
           or no link has heard its target (reason says which)

Every line: t (wall, unix), mono, dir, link (rx/tx) or output (refused), origin, seq
(the MAVLink header sequence), msgid, msg, src_sysid, src_compid, validated, then for
TUNNEL: target_sysid, target_compid, payload_type, payload_type_hex, payload_type_name,
len, b64 (the raw tunnel bytes, exactly `len` of them) and `decoded` -- the fields, when
rxl_codec knows the payload (or `decode_error`, when it does and the bytes are bad: a
corrupt frame is evidence, not something to hide).

The file is run/tunnel/<UTC>.jsonl, one per fleet_link start, under the OCS folder
(`run/` is gitignored). A failure to write is counted and shown in /fleet; it never
raises into the hub.

FILE I/O IS NOT ON THE ROUTING PATH. on_event runs under the hub lock, and `run/` sits in a
OneDrive-synced folder, where a write can stall for as long as the sync client likes. So
on_event only builds the line's dict and appends it to a bounded in-memory buffer (a lock
held for one append); ONE writer thread serialises, writes and flushes. When the buffer is
full -- the disk has been slower than the radio for a long while -- the line is dropped and
counted (`queue_dropped`), never waited for. The writer flushes whenever it has caught up
and at least every `flush_every_s` under load, and close() writes out everything queued
before it returns: a clean shutdown loses nothing, a hard crash loses at most the last
second or so.
"""
from __future__ import annotations

import base64
import collections
import json
import logging
import threading
import time
from pathlib import Path
from typing import Callable, Deque, Dict, FrozenSet, Optional

from . import rxl_codec
from .mavframe import CUSTOM_ID_BLOCK, MSG_TUNNEL, Dialect, Frame, parse_tunnel

log = logging.getLogger("fleet")

_REOPEN_AFTER_S = 5.0
QUEUE_MAX = 4096                    # lines buffered between the hub and the writer thread
FLUSH_EVERY_S = 1.0                 # under sustained load; when idle it flushes at once
_POLL_S = 0.2                       # how long an idle writer sleeps (bounds close() latency)


def logged_ids(dialect: Dialect) -> FrozenSet[int]:
    """The msgids the log keeps: TUNNEL plus the dialect's custom ids."""
    ids = {MSG_TUNNEL} | set(dialect.custom_ids())
    if not dialect.custom_ids():
        ids |= set(CUSTOM_ID_BLOCK)
    return frozenset(ids)


def _json_line(entry: dict) -> str:
    try:
        return json.dumps(entry, separators=(",", ":"), allow_nan=False, default=repr)
    except ValueError:                                  # a NaN in a decoded float
        entry = dict(entry, decoded=repr(entry.get("decoded")))
        return json.dumps(entry, separators=(",", ":"), default=repr)


class TunnelLog:
    def __init__(self, directory, dialect: Dialect, *, wall: Callable[[], float] = time.time,
                 mono: Callable[[], float] = time.monotonic, queue_max: int = QUEUE_MAX,
                 flush_every_s: float = FLUSH_EVERY_S):
        self.directory = Path(directory)
        self.dialect = dialect
        self.wall = wall
        self.mono = mono
        self.msgids = logged_ids(dialect)
        self.queue_max = queue_max
        self.flush_every_s = flush_every_s
        self.path: Optional[Path] = None
        self._fh = None                                   # the writer thread's, once open() has returned
        self.lines = 0
        self.errors = 0
        self.dropped = 0                                  # lines that reached the writer while the file was down
        self.queue_dropped = 0                            # lines refused because the buffer was full
        self.last_error: Optional[str] = None
        self._failed_at: Optional[float] = None
        self._buf: Deque[dict] = collections.deque()
        self._cv = threading.Condition()                  # guards _buf, _busy, queue_dropped; never held over I/O
        self._busy = False                                # the writer holds a line it has not finished with
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._dirty = False                               # written, not yet flushed
        self._flushed_at = 0.0
        self._drops_reported = 0

    # ------------------------------------------------------------------ life cycle

    def open(self) -> Path:
        """Create this start's file and start the writer. Raises OSError if the folder cannot
        be made: the caller decides whether that is fatal (it is not -- see fleet_link)."""
        self.directory.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(self.wall()))
        path, n = self.directory / (stamp + ".jsonl"), 1
        while path.exists():
            n += 1
            path = self.directory / ("%s-%d.jsonl" % (stamp, n))
        self._fh = path.open("a", encoding="utf-8", newline="\n")
        self.path = path
        self._flushed_at = self.mono()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="tunnel-log", daemon=True)
        self._thread.start()
        return path

    def close(self) -> None:
        """Write out everything queued, flush and close. Idempotent."""
        self._stop.set()
        with self._cv:
            self._cv.notify_all()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(5.0)
            if thread.is_alive():
                log.warning("tunnel log: the writer is stuck in a write (%d line(s) queued); closing anyway",
                            len(self._buf))
        self._drop_file()

    def _drop_file(self) -> None:
        fh, self._fh = self._fh, None
        if fh is not None:
            try:
                fh.close()
            except (OSError, ValueError):
                pass

    # ------------------------------------------------------------------- the observer

    def on_event(self, event: str, where: str, frame: Frame, now: float, info: dict) -> None:
        """hub.observe() callback, run under the hub lock. Builds the line's dict and queues it
        for the writer thread: no file I/O, no waiting, never raises."""
        if self._thread is None or self._stop.is_set():
            return                                    # never opened (logging was not asked for), or closing
        try:
            entry = self.entry(event, where, frame, info)
        except Exception as e:                                       # noqa: BLE001
            self._count_error(e)
            return
        with self._cv:
            if len(self._buf) >= self.queue_max:
                self.queue_dropped += 1               # the writer reports it; this thread does not log
                return
            self._buf.append(entry)
            self._cv.notify()

    def drain(self, timeout: float = 5.0) -> bool:
        """Wait until every queued line has been written and flushed. True if it got there."""
        with self._cv:
            return self._cv.wait_for(lambda: not self._buf and not self._busy, timeout)

    def entry(self, event: str, where: str, frame: Frame, info: Optional[dict] = None) -> dict:
        info = info or {}
        e = {"t": round(self.wall(), 3), "mono": round(self.mono(), 3), "dir": event}
        if event == "refused":
            e["output"] = where
            e["reason"] = info.get("reason")
        else:
            e["link"] = where
            e["origin"] = info.get("origin", "link")
            if event == "tx":
                e["sent"] = bool(info.get("sent", True))
        info_msg = self.dialect.get(frame.msgid)
        e.update(seq=frame.seq, msgid=frame.msgid, msg=info_msg.name if info_msg else "UNKNOWN_%d" % frame.msgid,
                 src_sysid=frame.sysid, src_compid=frame.compid, validated=frame.validated)
        if frame.msgid == MSG_TUNNEL:
            self._tunnel_fields(e, frame)
        else:
            e["len"] = frame.payload_len
            e["b64"] = base64.b64encode(frame.payload).decode("ascii")
            decoded = self.dialect.decode_custom(frame.raw) if frame.validated else None
            if decoded is not None:
                e["decoded"] = decoded
        return e

    @staticmethod
    def _tunnel_fields(e: dict, frame: Frame) -> None:
        t = parse_tunnel(frame)
        e.update(target_sysid=t.target_system, target_compid=t.target_component,
                 payload_type=t.payload_type, payload_type_hex="0x%04X" % t.payload_type,
                 payload_type_name=rxl_codec.payload_type_name(t.payload_type),
                 len=t.payload_length, b64=base64.b64encode(t.data).decode("ascii"))
        if not t.length_ok:
            e["length_error"] = "payload_length %d exceeds the 128-byte field" % t.payload_length
        cid = rxl_codec.catalogue_id(t.payload_type)
        if cid in rxl_codec.KNOWN_CLASHES:
            e["note"] = rxl_codec.KNOWN_CLASHES[cid]
        if not t.length_ok:
            return
        try:
            got = rxl_codec.decode(t.payload_type, t.data)
        except rxl_codec.RxlError as err:
            e["decode_error"] = str(err)
        else:
            if got is not None:
                e["decoded"] = got[1]

    # ---------------------------------------------------------------------- output
    # Everything from here to status() runs on the writer thread (after close() has joined it,
    # nothing runs at all). It is the only code that touches the file.

    def _run(self) -> None:
        while True:
            with self._cv:
                if not self._buf and not self._stop.is_set():
                    self._cv.wait(_POLL_S)
                entry = self._buf.popleft() if self._buf else None
                if entry is None and self._stop.is_set():
                    break
                self._busy = entry is not None
            try:
                if entry is not None:
                    self._write_guarded(entry)
                self._flush_if_due(idle=not self._buf)
                self._report_drops()
            except Exception as e:                                   # noqa: BLE001
                self._count_error(e)                  # the writer must outlive any one bad line
            with self._cv:
                self._busy = False
                self._cv.notify_all()
        self._flush_if_due(idle=True)

    def _write_guarded(self, entry: dict) -> None:
        try:
            self._write(entry)
        except Exception as e:                                       # noqa: BLE001
            self._fail(e)

    def _write(self, entry: dict) -> None:
        if self._fh is None:
            if self._failed_at is None or self.mono() - self._failed_at < _REOPEN_AFTER_S \
                    or self.path is None:
                if self._failed_at is not None:
                    self.dropped += 1                     # the file is down: this line is lost, and counted
                return                                    # (never opened: logging was not asked for)
            self._fh = self.path.open("a", encoding="utf-8", newline="\n")
            log.info("tunnel log reopened: %s", self.path)
            self._failed_at = None
        self._fh.write(_json_line(entry) + "\n")
        self._dirty = True
        self.lines += 1

    def _flush_if_due(self, idle: bool) -> None:
        """Flush once the writer has caught up (`idle`), or at least every flush_every_s."""
        if not self._dirty or self._fh is None:
            return
        if not idle and self.mono() - self._flushed_at < self.flush_every_s:
            return
        try:
            self._fh.flush()
        except Exception as e:                                       # noqa: BLE001
            self._fail(e)
        self._dirty = False
        self._flushed_at = self.mono()

    def _report_drops(self) -> None:
        dropped = self.queue_dropped
        if dropped != self._drops_reported:
            log.warning("tunnel log: the write queue (%d lines) is full - %d line(s) dropped so far "
                        "(routing is unaffected; is the disk or OneDrive stalled?)", self.queue_max, dropped)
            self._drops_reported = dropped

    def _count_error(self, exc: BaseException) -> None:
        self.errors += 1
        self.last_error = "%s: %s" % (type(exc).__name__, exc)
        if self.errors == 1:
            log.warning("tunnel log write failed (routing is unaffected): %s", self.last_error)

    def _fail(self, exc: BaseException) -> None:
        self._count_error(exc)
        self._failed_at = self.mono()
        self._dirty = False
        self._drop_file()

    def status(self) -> Dict[str, object]:
        return {"path": None if self.path is None else str(self.path), "lines": self.lines,
                "errors": self.errors, "dropped": self.dropped, "last_error": self.last_error,
                "open": self._fh is not None, "queued": len(self._buf),
                "queue_max": self.queue_max, "queue_dropped": self.queue_dropped}
