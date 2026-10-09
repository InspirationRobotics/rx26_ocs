"""The routing core of fleet_link: who hears what, and who may answer whom.

No sockets, no serial ports and no threads in this file -- the transports live in
endpoints.py and call into the Hub, and the tests drive it with fake ones. All
state is behind ONE lock, taken by every public method, so a frame cannot be
half-counted when the WiFi thread, the radio thread and a consumer's thread land
at once.

THE RULES THE DESIGN RESTS ON

  link -> outputs, never link -> link. Bridging WiFi and the radio would flood the
  radio with the boat's 100+ frames/s and make loops possible. Frames flow from a
  link to the outputs that list it, and from an output to the links it lists.

  Every frame a link yields is de-duplicated against that same link's last 1.0 s.
  The boat's MAVProxy unicasts AND broadcasts the stream, so a frame can arrive
  twice from two source ports.

  Only CRC-validated frames teach the hub anything (the sysid table, the learned
  route, the reply address). A frame with an unknown msgid is forwarded but never
  taught from: junk must not be able to invent a vehicle or redirect a reply.

  An output that is not `transmit` is LISTEN-ONLY: whatever it sends is dropped
  and counted. A `transmit` output's frames go to the link that last heard the
  frame's target_system (within `route_age_s`), or to all of the output's links
  when there is no target.
"""
from __future__ import annotations

import logging
import threading
import time
from collections import OrderedDict
from dataclasses import dataclass
from typing import Callable, Dict, FrozenSet, List, Optional, Sequence, Tuple

from .mavframe import (Dialect, Frame, FrameSplitter, ParseStats, pack_datagrams,
                       split_datagram)
from .vehicles import STALE_S, RateMeter, VehicleNames, VehicleTable

log = logging.getLogger("fleet")

SCHEMA_VERSION = 1
DEDUP_WINDOW_S = 1.0
DEDUP_CAPACITY = 2048          # frames remembered per link, whatever the rate
ROUTE_AGE_S = 30.0             # a sysid not heard this recently has no route
_WRITE_ERROR_LOG_EVERY = 50    # log the 1st write error, then every Nth


@dataclass(frozen=True)
class OutputCfg:
    name: str
    port: int
    links: Tuple[str, ...]
    sysids: Optional[FrozenSet[int]]     # None = all sysids
    transmit: bool                       # False = listen-only


class Endpoint:
    """The life cycle both kinds of transport share: bound to the hub, opened (claim
    the resource, fail loudly), started (threads), stopped."""

    def __init__(self, name: str):
        self.name = name
        self.hub: Optional["Hub"] = None

    def bind(self, hub: "Hub") -> None:
        self.hub = hub

    def open(self) -> None:
        """Claim the resource now so a failure is loud and before anything starts."""

    def start(self) -> None:
        """Start the transport's thread(s)."""

    def stop(self) -> None:
        """Stop and release."""


class LinkIO(Endpoint):
    """What the hub needs from a link's transport. Subclassed by the real ones in
    endpoints.py and by the fakes in the tests."""
    kind = "udp"             # reported in /fleet
    needs_addr = False       # True: a send needs a destination address (UDP)
    stream = False           # True: bytes arrive as a stream (serial), not datagrams
    enabled = True

    def send(self, data: bytes, nframes: int, addr) -> bool:
        """Hand `data` (whole frames) to the transport. Return False if it was
        refused (closed, queue full). On success the transport reports the bytes
        actually written through hub.link_tx(), now or later."""
        raise NotImplementedError


class OutputIO(Endpoint):
    local_port: Optional[int] = None     # our source port, for /fleet

    def send(self, data: bytes) -> bool:
        raise NotImplementedError


class Dedup:
    """Remembers frames for `window_s`, in bounded memory (insertion order == time
    order, so only the front ever needs expiring)."""

    def __init__(self, window_s: float = DEDUP_WINDOW_S, capacity: int = DEDUP_CAPACITY):
        self.window_s = window_s
        self.capacity = capacity
        self._seen: "OrderedDict[bytes, float]" = OrderedDict()

    def clear(self) -> None:
        self._seen.clear()

    def seen(self, raw: bytes, now: float) -> bool:
        """True if byte-identical `raw` was accepted within the window. Otherwise
        remember it and return False. A duplicate does not extend the window."""
        seen = self._seen
        while seen:
            oldest = next(iter(seen.values()))
            if now - oldest > self.window_s:
                seen.popitem(last=False)
            else:
                break
        if raw in seen:
            return True
        if len(seen) >= self.capacity:
            seen.popitem(last=False)
        seen[raw] = now
        return False

    def __len__(self):
        return len(self._seen)


class LinkPort:
    """The hub's side of one link: parsing state, counters, and what it has heard."""

    def __init__(self, io: LinkIO, dialect: Dialect, dedup_window_s: float):
        self.io = io
        self.name = io.name
        self.parse = ParseStats()
        self.splitter = FrameSplitter(dialect, self.parse) if io.stream else None
        self.dedup = Dedup(dedup_window_s)
        self.rate_in = RateMeter()
        self.open = False
        self.error: Optional[str] = None if io.enabled else "disabled"
        self.bytes_in = 0
        self.frames_in = 0           # frames accepted: valid-or-unknown, not duplicates
        self.dups = 0
        self.bytes_out = 0
        self.frames_out = 0
        self.tx_dropped = 0          # offered to the link and refused (closed / queue full)
        self.write_errors = 0
        self.last_frame_t: Optional[float] = None
        #: Address of the last accepted, validated datagram -- where a broadcast
        #: reply goes on a UDP link. None on serial, and until something is heard.
        self.last_addr = None
        #: sysid -> (time, address) of its last accepted, validated frame here.
        self.heard: Dict[int, Tuple[float, object]] = {}

    def reset_stream(self) -> None:
        if self.splitter is not None:
            self.splitter.reset()
        self.dedup.clear()

    def ingest(self, data: bytes, addr, now: float, dialect: Dialect) -> List[Frame]:
        """Split `data`, drop duplicates, return the accepted frames."""
        self.bytes_in += len(data)
        self.rate_in.add(len(data), now)
        frames = (self.splitter.feed(data) if self.splitter is not None
                  else split_datagram(dialect, data, self.parse))
        accepted = []
        for f in frames:
            if self.dedup.seen(f.raw, now):
                self.dups += 1
                continue
            self.frames_in += 1
            self.last_frame_t = now
            accepted.append(f)
            if f.validated:
                self.heard[f.sysid] = (now, addr)
                if addr is not None:
                    self.last_addr = addr
        return accepted

    def status(self, now: float) -> dict:
        p = self.parse
        return {
            "type": self.io.kind,
            "enabled": self.io.enabled,
            "open": self.open,
            "error": self.error,
            "bytes_in": self.bytes_in,
            "frames_in": self.frames_in,
            "bad_crc": p.bad_crc,
            "unknown_id": p.unknown_id,
            "dups": self.dups,
            "garbage_bytes": p.garbage_bytes,
            "bytes_out": self.bytes_out,
            "frames_out": self.frames_out,
            "tx_dropped": self.tx_dropped,
            "write_errors": self.write_errors,
            "rate_in_Bps": round(self.rate_in.rate(now), 1),
            "last_frame_age_s": (None if self.last_frame_t is None
                                 else round(now - self.last_frame_t, 2)),
        }


class OutputPort:
    def __init__(self, cfg: OutputCfg, io: OutputIO):
        self.cfg = cfg
        self.io = io
        self.name = cfg.name
        self.parse = ParseStats()
        self.frames_out = 0
        self.bytes_out = 0
        self.send_errors = 0
        self.frames_in_from_consumer = 0
        self.dropped_listen_only = 0
        self.no_route = 0

    def wants(self, link: str, sysid: int) -> bool:
        c = self.cfg
        return link in c.links and (c.sysids is None or sysid in c.sysids)

    def status(self) -> dict:
        c = self.cfg
        return {
            "port": c.port,
            "reply_port": self.io.local_port,
            "links": list(c.links),
            "sysids": None if c.sysids is None else sorted(c.sysids),
            "transmit": c.transmit,
            "frames_out": self.frames_out,
            "bytes_out": self.bytes_out,
            "send_errors": self.send_errors,
            "frames_in_from_consumer": self.frames_in_from_consumer,
            "dropped_listen_only": self.dropped_listen_only,
            "no_route": self.no_route,
            "bad_crc_in": self.parse.bad_crc,
            "garbage_bytes_in": self.parse.garbage_bytes,
        }


class Hub:
    def __init__(self, dialect: Dialect, link_ios: Sequence[LinkIO],
                 outputs: Sequence[Tuple[OutputCfg, OutputIO]], names: VehicleNames, *,
                 clock: Callable[[], float] = time.monotonic,
                 wall: Callable[[], float] = time.time,
                 stale_s: float = STALE_S, dedup_window_s: float = DEDUP_WINDOW_S,
                 route_age_s: float = ROUTE_AGE_S):
        self.dialect = dialect
        self.clock = clock
        self.wall = wall
        self.stale_s = stale_s
        self.route_age_s = route_age_s
        self.lock = threading.RLock()
        self.links: Dict[str, LinkPort] = {}
        for io in link_ios:
            if io.name in self.links:
                raise ValueError("duplicate link name %r" % io.name)
            self.links[io.name] = LinkPort(io, dialect, dedup_window_s)
        self.outputs: Dict[str, OutputPort] = {}
        for cfg, io in outputs:
            if cfg.name in self.outputs:
                raise ValueError("duplicate output name %r" % cfg.name)
            for ln in cfg.links:
                if ln not in self.links:
                    raise ValueError("output %r lists unknown link %r" % (cfg.name, ln))
            self.outputs[cfg.name] = OutputPort(cfg, io)
        self.vehicles = VehicleTable(names, stale_s)
        self.started_at = clock()
        for port in self.links.values():
            port.io.bind(self)
        for out in self.outputs.values():
            out.io.bind(self)

    # ------------------------------------------------------------ life cycle

    def open(self) -> None:
        for p in self.links.values():
            p.io.open()
        for o in self.outputs.values():
            o.io.open()

    def start(self) -> None:
        for p in self.links.values():
            p.io.start()
        for o in self.outputs.values():
            o.io.start()

    def stop(self) -> None:
        for p in self.links.values():
            p.io.stop()
        for o in self.outputs.values():
            o.io.stop()

    # ------------------------------------------------- called by link transports

    def link_state(self, name: str, is_open: bool, error: Optional[str] = None) -> None:
        """A transport reports it opened, or failed. Each CHANGE is logged once."""
        with self.lock:
            port = self.links[name]
            changed = (port.open, port.error) != (is_open, error)
            port.open, port.error = is_open, error
            if is_open:
                port.reset_stream()
        if changed:
            if is_open:
                log.info("link %s: open", name)
            else:
                log.warning("link %s: DOWN - %s", name, error)

    def link_tx(self, name: str, nframes: int, nbytes: int, error: Optional[str] = None) -> None:
        """A transport reports a write: `error` None = written, else it failed."""
        with self.lock:
            port = self.links[name]
            if error is None:
                port.frames_out += nframes
                port.bytes_out += nbytes
                return
            port.write_errors += 1
            count = port.write_errors
        if count == 1 or count % _WRITE_ERROR_LOG_EVERY == 0:
            log.warning("link %s: write failed (%d so far): %s", name, count, error)

    def link_rx(self, name: str, data: bytes, addr=None) -> None:
        """Bytes (a stream chunk, or one datagram) arrived on a link."""
        with self.lock:
            now = self.clock()
            port = self.links[name]
            frames = port.ingest(data, addr, now, self.dialect)
            for f in frames:
                if f.validated:
                    self.vehicles.note(f, name, now)
            if frames:
                self._fan_out(name, frames)

    def _fan_out(self, link: str, frames: List[Frame]) -> None:
        for out in self.outputs.values():
            raws = [f.raw for f in frames if out.wants(link, f.sysid)]
            for group in pack_datagrams(raws):
                data = b"".join(group)
                if out.io.send(data):
                    out.frames_out += len(group)
                    out.bytes_out += len(data)
                else:
                    out.send_errors += 1

    # ---------------------------------------------- called by output transports

    def output_rx(self, name: str, data: bytes) -> None:
        """A consumer sent a datagram to the output's socket."""
        with self.lock:
            now = self.clock()
            out = self.outputs[name]
            for f in split_datagram(self.dialect, data, out.parse):
                out.frames_in_from_consumer += 1
                if not out.cfg.transmit:
                    out.dropped_listen_only += 1
                    continue
                self._route_reverse(out, f, now)

    def _route_reverse(self, out: OutputPort, frame: Frame, now: float) -> None:
        target = self.dialect.target_system(frame)
        if target:                                    # a specific vehicle
            route = self._learned_route(out, target, now)
            sends = [route] if route else []
        else:                                         # no target, target 0, or unknown id
            sends = [(self.links[n], self.links[n].last_addr) for n in out.cfg.links]
        sent = False
        for port, addr in sends:
            if not port.io.enabled:
                continue                              # switched off with --serial NONE etc.
            if port.io.needs_addr and addr is None:
                continue                              # a UDP link nobody has spoken on yet
            sent = True
            if not port.io.send(frame.raw, 1, addr):
                port.tx_dropped += 1
        if not sent:
            out.no_route += 1

    def _learned_route(self, out: OutputPort, sysid: int, now: float):
        """(LinkPort, address) of the freshest link in this output's list that
        heard `sysid` within route_age_s, else None."""
        best = None
        for name in out.cfg.links:
            port = self.links[name]
            heard = port.heard.get(sysid)
            if heard is None or now - heard[0] > self.route_age_s:
                continue
            if best is None or heard[0] > best[0]:
                best = (heard[0], port, heard[1])
        return None if best is None else (best[1], best[2])

    # ------------------------------------------------------------------ status

    def snapshot(self) -> dict:
        with self.lock:
            now = self.clock()
            return {
                "schema": SCHEMA_VERSION,
                "time": round(self.wall(), 3),
                "uptime_s": round(now - self.started_at, 1),
                "stale_after_s": self.stale_s,
                "dialect": self.dialect.describe(),
                "decode_errors": self.vehicles.decode_errors,
                "links": {n: p.status(now) for n, p in self.links.items()},
                "outputs": {n: o.status() for n, o in self.outputs.items()},
                "sysids": self.vehicles.snapshot(now),
            }
