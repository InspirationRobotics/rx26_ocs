"""framing — one message on a TCP stream, so both ends agree where it ends.

TCP is a byte stream with NO message boundaries. `recv(4096)` returns whatever
happened to arrive: two heartbeats glued together, the first seventeen bytes of
one, or half of a third. Reading a frame as "whatever one recv() returned" works
on a bench at 2 Hz and starts tearing messages the moment traffic is bursty --
which is exactly what a reconnect produces, and exactly when you least want the
telemetry to lie.

So every message on the vehicle link is:

    4 bytes   big-endian unsigned length N
    N bytes   the protobuf payload

THIS FORMAT IS DUPLICATED IN THE VEHICLE REPO. `crusader_groundstation/
ocs_client.py` in rx26_asv carries the same two pieces of logic, deliberately
copied rather than shared -- a ROS package should not import from the OCS repo.
If you change the header here you MUST change it there in the same commit. A
mismatched length prefix is indistinguishable from a corrupt payload, so the
failure is silent garbage rather than an error.
"""
from __future__ import annotations

import struct

_HEADER = struct.Struct(">I")
HEADER_LEN = _HEADER.size

#: Refuse anything larger. A heartbeat is ~81 bytes; a thousandfold headroom is
#: generous. The point is that a garbage length -- which is what a desynced
#: stream looks like -- must not make us buffer forever or allocate a gigabyte.
MAX_FRAME = 1 << 20


def encode(payload: bytes) -> bytes:
    """Length-prefix one payload."""
    if len(payload) > MAX_FRAME:
        raise ValueError("frame of %d bytes exceeds MAX_FRAME (%d)"
                         % (len(payload), MAX_FRAME))
    return _HEADER.pack(len(payload)) + payload


class FrameReader:
    """Accumulates whatever recv() gives you; hands back whole frames only.

    One per connection -- it holds that connection's partial tail between
    reads, which is the entire reason it exists.
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list[bytes]:
        """Add bytes, return every complete frame now available (often none).

        Raises ValueError if the stream has desynced past recovery. The caller
        must drop the connection: once the length prefix is wrong there is no
        way to find where the next frame begins, and guessing produces frames
        that parse as valid protobuf while meaning something else entirely.
        """
        self._buf.extend(chunk)
        out: list[bytes] = []
        while len(self._buf) >= HEADER_LEN:
            (n,) = _HEADER.unpack_from(self._buf, 0)
            if n > MAX_FRAME:
                raise ValueError(
                    "framing desync: length prefix claims %d bytes (max %d)"
                    % (n, MAX_FRAME)
                )
            if len(self._buf) < HEADER_LEN + n:
                break               # tail of a partial frame; wait for more
            out.append(bytes(self._buf[HEADER_LEN:HEADER_LEN + n]))
            del self._buf[:HEADER_LEN + n]
        return out

    @property
    def pending(self) -> int:
        """Bytes held awaiting the rest of their frame. For diagnostics."""
        return len(self._buf)
