"""Tests for the vehicle link's frame boundaries.

TCP is a byte stream. If a vehicle sends heartbeat A and then heartbeat B, this
end may read them as `AB` in one go, as `A` then `B`, or as the first half of A
followed by the rest of A glued to B. All three are normal TCP behaviour, not
faults -- and the naive `recv(4096)` reader treats the first as one message and
the third as two broken ones.

So FrameReader has exactly one job: however the bytes turn up, hand back exactly
A, then exactly B. These tests feed it every awkward arrangement there is.

No sockets. The awkward arrangements are the point, and a real connection cannot
be made to produce them on demand -- it produces them at 3 a.m. on the water
instead. Feeding bytes by hand is both deterministic and faster.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge.framing import (   # noqa: E402
    HEADER_LEN, MAX_FRAME, FrameReader, encode,
)

A = b'{"vehicle_id": "USV1"}'
B = b'{"vehicle_id": "UUV1", "heartbeat": {"depth_m": 4.5}}'


class TestEncode(unittest.TestCase):
    def test_header_is_four_bytes_big_endian(self):
        self.assertEqual(encode(b"xy"), b"\x00\x00\x00\x02xy")

    def test_oversize_payload_is_refused(self):
        with self.assertRaises(ValueError):
            encode(b"x" * (MAX_FRAME + 1))


class TestFrameReader(unittest.TestCase):
    def test_one_whole_frame(self):
        self.assertEqual(FrameReader().feed(encode(A)), [A])

    def test_a_split_frame_waits_for_its_tail(self):
        """The half-message case: nothing until the rest arrives, then one A."""
        wire = encode(A)
        cut = HEADER_LEN + 5
        r = FrameReader()
        self.assertEqual(r.feed(wire[:cut]), [])       # not yet -- and not garbage
        self.assertEqual(r.feed(wire[cut:]), [A])

    def test_a_header_split_across_reads(self):
        """Even the length prefix itself can arrive in pieces."""
        wire = encode(A)
        r = FrameReader()
        self.assertEqual(r.feed(wire[:2]), [])
        self.assertEqual(r.feed(wire[2:]), [A])

    def test_glued_frames_come_back_separately(self):
        """Two messages in one read must be two messages, not one."""
        self.assertEqual(FrameReader().feed(encode(A) + encode(B)), [A, B])

    def test_a_frame_and_a_half(self):
        """The nastiest realistic case: all of A, then part of B."""
        wire = encode(A) + encode(B)
        cut = len(encode(A)) + HEADER_LEN + 3
        r = FrameReader()
        self.assertEqual(r.feed(wire[:cut]), [A])      # A only; B is incomplete
        self.assertEqual(r.feed(wire[cut:]), [B])

    def test_byte_at_a_time(self):
        """The brutal one. Catches essentially every buffering bug there is."""
        wire = encode(A) + encode(B)
        r = FrameReader()
        got: list[bytes] = []
        for i in range(len(wire)):
            got.extend(r.feed(wire[i:i + 1]))
        self.assertEqual(got, [A, B])

    def test_an_empty_payload_is_a_frame_not_a_close(self):
        """A zero-length frame is legal and must not read as end-of-stream."""
        self.assertEqual(FrameReader().feed(encode(b"")), [b""])

    def test_a_desynced_length_raises(self):
        """A garbage prefix cannot be recovered from -- the caller must drop the
        connection. Buffering onwards would eventually emit bytes that parse as
        a valid message meaning something else entirely."""
        r = FrameReader()
        with self.assertRaises(ValueError):
            r.feed(b"\xff\xff\xff\xff" + b"whatever")

    def test_pending_reports_the_parked_tail(self):
        wire = encode(A)
        r = FrameReader()
        r.feed(wire[:HEADER_LEN + 4])
        self.assertEqual(r.pending, HEADER_LEN + 4)
        r.feed(wire[HEADER_LEN + 4:])
        self.assertEqual(r.pending, 0)

    def test_a_reader_survives_many_frames(self):
        """Buffer accounting must not leak across frames."""
        r = FrameReader()
        for _ in range(500):
            self.assertEqual(r.feed(encode(A)), [A])
        self.assertEqual(r.pending, 0)


if __name__ == "__main__":
    unittest.main()
