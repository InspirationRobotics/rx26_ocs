"""MAVLink v1/v2 framing for fleet_link: split, validate, pack. Pure -- no sockets,
no threads, no clocks, no I/O except reading the dialect files once at load.

WHAT IT DOES AND DOES NOT DO. It finds frame boundaries in a byte stream (the
RFD900ux serial port) or in a datagram (UDP), checks the CRC of every frame whose
message id it knows, and hands back whole frames as raw bytes. It never rewrites,
re-encodes or originates a frame: the bytes that went in are the bytes that come
out, so a forwarded frame is bit-identical to what the vehicle sent.

THE POLICY, same as mavlink-router's:
  * known msgid + good CRC   -> a validated Frame
  * known msgid + bad CRC    -> counted, ONE byte dropped, scan resumes from the
                                next byte (the bytes inside the bogus frame may
                                hold the start of the real next one)
  * unknown msgid            -> passed through UNVALIDATED (`validated=False`) and
                                counted; nothing downstream may learn from it
  * stray bytes between      -> counted as garbage and skipped

An unvalidated frame is the one place junk can get through: after a corrupted
frame, a stray 0xFE/0xFD inside its payload can look like the start of a frame
with an id we do not know. The `unknown_id` counter is the tell. With both
dialects loaded it should sit at zero on a healthy link, so a rising count means
either line noise or a vehicle speaking a message nobody loaded.

THE CRC TABLE is pymavlink's ardupilotmega message map MERGED with the robotx
dialect (RobotX_2026/Mesh/dialect/gen/robotx.py, byte-identical to the boat's
crusader_link/dialect/robotx.py). That file holds the 42xxx RXL messages
(42011 RXL_SAFE_PASSAGE, 42018, 42019 ...). Without it those ids are "unknown"
and pass unvalidated; `Dialect.notes` says so and /fleet repeats it.
"""
from __future__ import annotations

import importlib.util
import os
import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence

MAGIC_V1 = 0xFE
MAGIC_V2 = 0xFD
HEADER_V1 = 6           # magic len seq sysid compid msgid
HEADER_V2 = 10          # magic len incompat compat seq sysid compid msgid[3]
CRC_LEN = 2
SIGNATURE_LEN = 13
IFLAG_SIGNED = 0x01
#: Largest possible frame: signed v2 with a 255-byte payload.
MAX_FRAME_LEN = HEADER_V2 + 255 + CRC_LEN + SIGNATURE_LEN
#: Frames are packed into datagrams no larger than this (whole frames only).
MAX_DATAGRAM = 1400


# --------------------------------------------------------------------------- CRC

def _make_crc_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
        table.append(c)
    return tuple(table)


_CRC_TABLE = _make_crc_table()


def x25_crc(data, crc: int = 0xFFFF) -> int:
    """CRC-16/MCRF4XX, the checksum MAVLink calls X.25."""
    table = _CRC_TABLE
    for b in data:
        crc = (crc >> 8) ^ table[(crc ^ b) & 0xFF]
    return crc


def frame_crc(data, crc_extra: int) -> int:
    """The MAVLink CRC: X.25 over `data`, then the message's crc_extra byte.

    `data` is everything after the magic byte up to the end of the payload
    (the signature, when present, is not covered).
    """
    return x25_crc(bytes((crc_extra,)), x25_crc(data))


# ----------------------------------------------------------------------- dialect

@dataclass(frozen=True)
class MsgInfo:
    msgid: int
    name: str
    crc_extra: int
    source: str                          # "ardupilotmega" | "robotx"
    target_system_offset: Optional[int]  # byte offset in the wire payload, or None


def _target_system_offset(cls) -> Optional[int]:
    """Where `target_system` sits in the wire payload, or None if the message has none.

    MAVLink sorts fields by size on the wire, and pymavlink records that order in
    `ordered_fieldnames` and the struct format. The offset is the size of
    everything ahead of the field. tests/test_mavframe.py cross-checks this against
    pymavlink's own decoder for every message that has the field.
    """
    names = list(cls.ordered_fieldnames)
    if "target_system" not in names:
        return None
    tokens = re.findall(r"(\d*[cbB?hHiIlLqQfdsp])", cls.unpacker.format.lstrip("<>=!@"))
    if len(tokens) != len(names):
        return None
    return struct.calcsize("<" + "".join(tokens[:names.index("target_system")]))


def _infos_from_map(mavlink_map, source: str) -> Dict[int, MsgInfo]:
    return {mid: MsgInfo(mid, cls.msgname, cls.crc_extra, source, _target_system_offset(cls))
            for mid, cls in mavlink_map.items()}


class Dialect:
    """msgid -> MsgInfo, built from the merged ardupilotmega + robotx maps."""

    def __init__(self, info: Dict[int, MsgInfo], notes: Sequence[str] = (),
                 robotx_path: Optional[str] = None):
        self._info = info
        self.notes = list(notes)
        self.robotx_path = robotx_path

    def get(self, msgid: int) -> Optional[MsgInfo]:
        return self._info.get(msgid)

    def __contains__(self, msgid: int) -> bool:
        return msgid in self._info

    def __len__(self) -> int:
        return len(self._info)

    def ids(self):
        return self._info.keys()

    def describe(self) -> dict:
        """The /fleet `dialect` block."""
        count = {}
        for i in self._info.values():
            count[i.source] = count.get(i.source, 0) + 1
        return {"messages": len(self._info),
                "ardupilotmega": count.get("ardupilotmega", 0),
                "robotx": count.get("robotx", 0),
                "robotx_path": self.robotx_path,
                "notes": list(self.notes)}

    def target_system(self, frame: "Frame") -> Optional[int]:
        """The frame's target_system, or None when the message has no such field
        (or is not in the dialect, so we cannot know).

        A v2 sender trims trailing zero bytes, so a payload that stops short of the
        field means the field is 0 -- "all systems" -- and is returned as 0.
        """
        info = self._info.get(frame.msgid)
        if info is None or info.target_system_offset is None:
            return None
        payload = frame.payload
        off = info.target_system_offset
        return payload[off] if off < len(payload) else 0


class DialectError(RuntimeError):
    pass


def locate_robotx_dialect() -> Optional[str]:
    """Find robotx.py: $FLEET_ROBOTX_DIALECT, the Mesh copy, then the boat's copy."""
    cands = []
    env = os.environ.get("FLEET_ROBOTX_DIALECT")
    if env:
        cands.append(Path(env))
    root = Path(__file__).resolve().parents[2]            # .../RobotX_2026
    cands.append(root / "Mesh" / "dialect" / "gen" / "robotx.py")
    cands.append(root / "Boat" / "rx26_asv" / "crusader_link" / "crusader_link"
                 / "dialect" / "robotx.py")
    for c in cands:
        if c.is_file():
            return str(c)
    return None


def _load_module_from_path(path: str):
    spec = importlib.util.spec_from_file_location("fleet_robotx_dialect", path)
    if spec is None or spec.loader is None:
        raise DialectError("cannot load %s as a python module" % path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)          # deliberately not put in sys.modules
    return module


def load_dialect(robotx_path: Optional[str] = "auto") -> Dialect:
    """Build the merged table.

    `robotx_path`: "auto" searches (and carries on with a note if nothing is found
    or it fails to load); None skips the robotx dialect; an explicit path must
    load, or DialectError is raised -- a configured file that silently does not
    apply would leave the 42xxx ids unvalidated without anyone choosing that.
    """
    from pymavlink.dialects.v20 import ardupilotmega
    info = _infos_from_map(ardupilotmega.mavlink_map, "ardupilotmega")
    notes: List[str] = []
    used_path: Optional[str] = None

    explicit = robotx_path not in (None, "auto")
    path = locate_robotx_dialect() if robotx_path == "auto" else robotx_path
    if robotx_path is not None:
        if path is None:
            notes.append("robotx dialect not found: 42xxx messages pass through "
                         "UNVALIDATED (set [dialect] robotx in fleet.toml)")
        else:
            try:
                module = _load_module_from_path(path)
                extra = _infos_from_map(module.mavlink_map, "robotx")
            except Exception as e:                        # noqa: BLE001
                if explicit:
                    raise DialectError("robotx dialect %s failed to load: %s" % (path, e))
                notes.append("robotx dialect %s failed to load (%s): 42xxx messages "
                             "pass through UNVALIDATED" % (path, e))
            else:
                used_path = path
                for mid, new in extra.items():
                    old = info.get(mid)
                    if old is None:
                        info[mid] = new
                    elif old.crc_extra != new.crc_extra:
                        notes.append("msgid %d (%s): crc_extra %d in ardupilotmega but %d in "
                                     "robotx; keeping ardupilotmega's" %
                                     (mid, old.name, old.crc_extra, new.crc_extra))
    return Dialect(info, notes, used_path)


_default: Dict[object, Dialect] = {}


def default_dialect(robotx_path: Optional[str] = "auto") -> Dialect:
    """Process-wide cached load_dialect()."""
    if robotx_path not in _default:
        _default[robotx_path] = load_dialect(robotx_path)
    return _default[robotx_path]


# ------------------------------------------------------------------------- frames

class Frame:
    """One whole MAVLink frame. `raw` is the exact bytes on the wire."""
    __slots__ = ("raw", "version", "seq", "sysid", "compid", "msgid",
                 "payload_len", "signed", "validated")

    def __init__(self, raw, version, seq, sysid, compid, msgid, payload_len, signed,
                 validated):
        self.raw = raw
        self.version = version
        self.seq = seq
        self.sysid = sysid
        self.compid = compid
        self.msgid = msgid
        self.payload_len = payload_len
        self.signed = signed
        self.validated = validated      # False = unknown msgid, CRC never checked

    @property
    def payload(self) -> bytes:
        h = HEADER_V2 if self.version == 2 else HEADER_V1
        return self.raw[h:h + self.payload_len]

    def __repr__(self):
        return "Frame(v%d sys=%d comp=%d id=%d len=%d%s%s)" % (
            self.version, self.sysid, self.compid, self.msgid, self.payload_len,
            " signed" if self.signed else "", "" if self.validated else " UNVALIDATED")


class ParseStats:
    """Counters a FrameSplitter increments. Shared across the throw-away splitters
    used for datagrams, so one link keeps one running total."""
    __slots__ = ("frames", "bad_crc", "unknown_id", "garbage_bytes")

    def __init__(self):
        self.frames = 0          # frames returned (validated or not)
        self.bad_crc = 0         # known msgid, CRC failed -> dropped
        self.unknown_id = 0      # unknown msgid -> passed through unvalidated
        self.garbage_bytes = 0   # bytes skipped between frames / left over


_MAGIC_RE = re.compile(b"[\xfd\xfe]")


class FrameSplitter:
    """Incremental splitter for a byte stream. Feed it whatever the port returned.

    Memory is bounded: after every feed() the buffer holds at most one incomplete
    frame (< MAX_FRAME_LEN bytes), because a frame that cannot be completed is only
    ever waited for up to its declared length.
    """

    def __init__(self, dialect: Dialect, stats: Optional[ParseStats] = None):
        self.dialect = dialect
        self.stats = stats if stats is not None else ParseStats()
        self._buf = bytearray()

    def reset(self) -> None:
        """Forget a half-received frame (the port was closed and reopened)."""
        self._buf.clear()

    def finish(self) -> int:
        """Discard a trailing partial frame, counting it as garbage. Returns bytes dropped."""
        n = len(self._buf)
        self.stats.garbage_bytes += n
        self._buf.clear()
        return n

    def feed(self, data) -> List[Frame]:
        buf = self._buf
        buf += data
        stats = self.stats
        info_for = self.dialect.get
        frames: List[Frame] = []
        pos = 0
        end = len(buf)
        while pos < end:
            m = _MAGIC_RE.search(buf, pos)
            if m is None:
                stats.garbage_bytes += end - pos
                pos = end
                break
            if m.start() > pos:
                stats.garbage_bytes += m.start() - pos
                pos = m.start()
            avail = end - pos

            if buf[pos] == MAGIC_V2:
                # MAVLink 2 says to reject a frame with incompat flags it does not
                # understand; for us that also screens out most stray 0xFD bytes.
                if avail >= 3 and buf[pos + 2] & ~IFLAG_SIGNED & 0xFF:
                    stats.garbage_bytes += 1
                    pos += 1
                    continue
                if avail < HEADER_V2:
                    break
                version, header = 2, HEADER_V2
                plen = buf[pos + 1]
                signed = bool(buf[pos + 2] & IFLAG_SIGNED)
                seq, sysid, compid = buf[pos + 4], buf[pos + 5], buf[pos + 6]
                msgid = buf[pos + 7] | (buf[pos + 8] << 8) | (buf[pos + 9] << 16)
                total = header + plen + CRC_LEN + (SIGNATURE_LEN if signed else 0)
            else:
                if avail < HEADER_V1:
                    break
                version, header = 1, HEADER_V1
                plen = buf[pos + 1]
                signed = False
                seq, sysid, compid, msgid = buf[pos + 2], buf[pos + 3], buf[pos + 4], buf[pos + 5]
                total = header + plen + CRC_LEN
            if avail < total:
                break                                   # wait for the rest

            info = info_for(msgid)
            if info is not None:
                body_end = pos + header + plen
                want = buf[body_end] | (buf[body_end + 1] << 8)
                if frame_crc(buf[pos + 1:body_end], info.crc_extra) != want:
                    stats.bad_crc += 1
                    stats.garbage_bytes += 1
                    pos += 1                            # resync from the next byte
                    continue
                validated = True
            else:
                stats.unknown_id += 1
                validated = False
            stats.frames += 1
            frames.append(Frame(bytes(buf[pos:pos + total]), version, seq, sysid, compid,
                                msgid, plen, signed, validated))
            pos += total
        del buf[:pos]
        return frames


def split_datagram(dialect: Dialect, data, stats: Optional[ParseStats] = None) -> List[Frame]:
    """Frames in one UDP datagram.

    Uses the SAME splitter as the serial stream, but a fresh one per datagram: UDP
    preserves boundaries, so a corrupt datagram must not be allowed to poison the
    next one, and a partial frame at the end of a datagram is garbage, not "the
    start of something".
    """
    splitter = FrameSplitter(dialect, stats)
    frames = splitter.feed(data)
    splitter.finish()
    return frames


def pack_datagrams(raws: Sequence[bytes], max_len: int = MAX_DATAGRAM) -> List[List[bytes]]:
    """Group whole frames into datagram-sized lists. A frame is never split; one
    frame bigger than `max_len` (it cannot be: max 280) would get a datagram to itself."""
    groups: List[List[bytes]] = []
    cur: List[bytes] = []
    size = 0
    for r in raws:
        if cur and size + len(r) > max_len:
            groups.append(cur)
            cur, size = [], 0
        cur.append(r)
        size += len(r)
    if cur:
        groups.append(cur)
    return groups
