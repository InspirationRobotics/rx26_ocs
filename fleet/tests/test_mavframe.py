import os
import random
import unittest

from pymavlink.dialects.v20 import ardupilotmega as ap
from pymavlink.generator.mavcrc import x25crc

from fleet import mavframe as mf
from fleet.tests.helpers import (DIALECT, HAVE_ROBOTX, build_frame, command_long, heartbeat,
                                 load_robotx_module)


def unknown_id(version=2):
    limit = 256 if version == 1 else 60000
    return next(i for i in range(limit - 1, 0, -1) if i not in DIALECT)


class Crc(unittest.TestCase):
    def test_matches_pymavlink(self):
        rng = random.Random(7)
        for _ in range(50):
            data = bytes(rng.randrange(256) for _ in range(rng.randrange(1, 300)))
            extra = rng.randrange(256)
            ref = x25crc(data)
            ref.accumulate(bytes([extra]))
            self.assertEqual(mf.frame_crc(data, extra), ref.crc)


class Splitting(unittest.TestCase):
    def split(self, data):
        s = mf.FrameSplitter(DIALECT)
        return s.feed(data), s

    def test_v2(self):
        raw = heartbeat(sysid=7, compid=3, seq=9)
        (f,), s = self.split(raw)
        self.assertEqual((f.version, f.sysid, f.compid, f.seq, f.msgid), (2, 7, 3, 9, 0))
        self.assertTrue(f.validated)
        self.assertFalse(f.signed)
        self.assertEqual(f.raw, raw)
        self.assertEqual(len(f.payload), f.payload_len)

    def test_v1(self):
        raw = heartbeat(sysid=7, version=1)
        self.assertEqual(raw[0], 0xFE)
        (f,), _ = self.split(raw)
        self.assertEqual((f.version, f.sysid, f.msgid, f.validated), (1, 7, 0, True))
        self.assertEqual(f.raw, raw)

    def test_signed_v2_includes_the_13_signature_bytes(self):
        raw = build_frame(0, bytes(9), signed=True, sysid=4)
        self.assertEqual(len(raw), 10 + 9 + 2 + 13)
        trailing = heartbeat(sysid=5)
        frames, s = self.split(raw + trailing)
        self.assertEqual([f.sysid for f in frames], [4, 5])
        self.assertTrue(frames[0].signed)
        self.assertEqual(frames[0].raw, raw)
        self.assertEqual(s.stats.garbage_bytes, 0)

    def test_split_across_reads_one_byte_at_a_time(self):
        raws = [heartbeat(sysid=1, seq=1), heartbeat(sysid=2, seq=2, version=1),
                build_frame(0, bytes(9), signed=True, sysid=3, seq=3)]
        s = mf.FrameSplitter(DIALECT)
        got = []
        for b in b"".join(raws):
            got.extend(s.feed(bytes([b])))
        self.assertEqual([f.raw for f in got], raws)
        self.assertEqual(s.stats.garbage_bytes, 0)

    def test_garbage_between_frames(self):
        a, b = heartbeat(sysid=1, seq=1), heartbeat(sysid=2, seq=2)
        frames, s = self.split(b"\x00\x01\x02" + a + b"\x55\x66" + b + b"\x99")
        self.assertEqual([f.raw for f in frames], [a, b])
        self.assertEqual(s.stats.garbage_bytes, 6)
        self.assertEqual(s.stats.bad_crc, 0)

    def test_bad_crc_drops_one_byte_and_resyncs(self):
        a, b, c = (heartbeat(sysid=i, seq=i) for i in (1, 2, 3))
        bad = bytearray(b)
        bad[12] ^= 0xFF                                  # corrupt the payload
        frames, s = self.split(a + bytes(bad) + c)
        self.assertEqual([f.sysid for f in frames], [1, 3])
        self.assertEqual(s.stats.bad_crc, 1)

    def test_bad_crc_hiding_a_real_frame_inside_its_payload(self):
        # A corrupt frame whose length byte overshoots swallows the next frame if the
        # splitter skips by the declared length; it must advance ONE byte instead.
        a, b = heartbeat(sysid=1, seq=1), heartbeat(sysid=2, seq=2)
        bad = bytearray(a)
        bad[1] = 40                                       # claims a 40-byte payload
        frames, s = self.split(bytes(bad) + b + b)
        self.assertEqual([f.sysid for f in frames], [2, 2])
        self.assertGreaterEqual(s.stats.bad_crc, 1)

    def test_unknown_msgid_passes_through_unvalidated(self):
        mid = unknown_id()
        raw = build_frame(mid, b"hello", crc_extra=33, sysid=9)
        frames, s = self.split(raw)
        self.assertEqual(len(frames), 1)
        self.assertFalse(frames[0].validated)
        self.assertEqual(frames[0].raw, raw)
        self.assertEqual((s.stats.unknown_id, s.stats.bad_crc), (1, 0))

    def test_unknown_msgid_v1_passes_through(self):
        raw = build_frame(unknown_id(1), b"hello", version=1, crc_extra=33)
        (f,), s = self.split(raw)
        self.assertFalse(f.validated)
        self.assertEqual(s.stats.unknown_id, 1)

    def test_unsupported_incompat_flags_are_not_a_frame(self):
        good = heartbeat(sysid=2)
        raw = bytearray(heartbeat())
        raw[2] = 0x02                                     # an incompat flag we do not know
        frames, s = self.split(bytes(raw) + good)
        self.assertEqual([f.sysid for f in frames], [2])
        # The check that matters is for an UNKNOWN msgid, whose CRC is never looked at:
        # without the flag test, this stray 0xFD would be forwarded as a frame.
        junk = bytearray(build_frame(unknown_id(), b"hello", crc_extra=33))
        junk[2] = 0x80
        frames, s = self.split(bytes(junk) + good)
        self.assertEqual([f.sysid for f in frames], [2])
        self.assertEqual(s.stats.unknown_id, 0)

    @unittest.skipUnless(HAVE_ROBOTX, "robotx dialect not found")
    def test_robotx_dialect_ids_are_validated(self):
        rx = load_robotx_module()
        mav = rx.MAVLink(None, 42, 191)
        for msg in (rx.MAVLink_rxl_usv_reached_gate_message(5, 2),
                    rx.MAVLink_rxl_next_buoy_set_message(5, 2, [3, 4])):
            raw = bytes(msg.pack(mav))
            (f,), s = self.split(raw)
            self.assertTrue(f.validated, msg.msgname)
            self.assertEqual(f.msgid, msg.id)
            self.assertEqual((f.sysid, f.compid), (42, 191))
            bad = bytearray(raw)
            bad[10] ^= 0x40
            frames, s = self.split(bytes(bad))
            self.assertEqual((frames, s.stats.bad_crc), ([], 1), "validated, not passed blindly")
        self.assertIn(42011, DIALECT)
        self.assertEqual(DIALECT.get(42011).source, "robotx")

    @unittest.skipUnless(HAVE_ROBOTX, "robotx dialect not found")
    def test_rxl_safe_passage_42011(self):
        rx = load_robotx_module()
        mav = rx.MAVLink(None, 42, 191)
        raw = bytes(mav.rxl_safe_passage_encode(1000, 10, 20, 30, 40, 2, [1] * 10, [2] * 10,
                                                [1] * 10).pack(mav))
        (f,), s = self.split(raw)
        self.assertEqual((f.msgid, f.validated), (42011, True))

    def test_without_the_dialect_42xxx_is_unknown_not_dropped(self):
        d = mf.load_dialect(None)
        self.assertNotIn(42011, d)
        raw = build_frame(42011, bytes(5), crc_extra=154)
        s = mf.FrameSplitter(d)
        (f,) = s.feed(raw)
        self.assertFalse(f.validated)

    def test_buffer_stays_bounded_on_a_stream_of_false_starts(self):
        s = mf.FrameSplitter(DIALECT)
        rng = random.Random(3)
        for _ in range(500):
            s.feed(bytes([0xFD, 255, 0, 0]) + bytes(rng.randrange(256) for _ in range(30)))
            self.assertLess(len(s._buf), mf.MAX_FRAME_LEN)

    def test_many_frames_in_one_read(self):
        raws = [heartbeat(sysid=1, seq=i) for i in range(50)]
        frames, _ = self.split(b"".join(raws))
        self.assertEqual([f.raw for f in frames], raws)


class Datagrams(unittest.TestCase):
    def test_partial_trailing_frame_is_garbage_not_carried_over(self):
        a = heartbeat(sysid=1, seq=1)
        stats = mf.ParseStats()
        frames = mf.split_datagram(DIALECT, a + a[:6], stats)
        self.assertEqual(len(frames), 1)
        self.assertEqual(stats.garbage_bytes, 6)
        # ...and the next datagram starts clean: the stray 6 bytes do not pair with it.
        frames = mf.split_datagram(DIALECT, a[6:] + a, stats)
        self.assertEqual(len(frames), 1)

    def test_pack_datagrams_never_splits_a_frame(self):
        raws = [heartbeat(sysid=1, seq=i) for i in range(200)]
        groups = mf.pack_datagrams(raws, 500)
        self.assertGreater(len(groups), 1)
        self.assertEqual([r for g in groups for r in g], raws)
        for g in groups:
            self.assertLessEqual(sum(map(len, g)), 500)
        self.assertEqual(mf.pack_datagrams([], 500), [])

    def test_oversize_single_frame_gets_its_own_datagram(self):
        self.assertEqual(mf.pack_datagrams([b"a" * 600, b"b" * 10], 500), [[b"a" * 600], [b"b" * 10]])


class DialectTable(unittest.TestCase):
    def test_merged_table(self):
        self.assertEqual(DIALECT.get(0).crc_extra, 50)             # HEARTBEAT
        self.assertGreaterEqual(len(DIALECT), len(ap.mavlink_map))
        d = DIALECT.describe()
        self.assertEqual(d["ardupilotmega"], len(ap.mavlink_map))
        self.assertEqual(d["robotx"] > 0, HAVE_ROBOTX)

    def test_explicit_missing_robotx_path_is_an_error(self):
        with self.assertRaises(mf.DialectError):
            mf.load_dialect(os.path.join(os.path.dirname(__file__), "does_not_exist.py"))

    def test_target_system_offset_agrees_with_pymavlink_for_every_message(self):
        decoder = ap.MAVLink(None)
        checked = 0
        for msgid, cls in ap.mavlink_map.items():
            if "target_system" not in cls.fieldnames:
                self.assertIsNone(DIALECT.get(msgid).target_system_offset, cls.msgname)
                continue
            size = cls.unpacker.size
            payload = bytes((i * 37 + 11) % 250 + 1 for i in range(size))
            raw = build_frame(msgid, payload, sysid=1)
            (frame,) = mf.FrameSplitter(DIALECT).feed(raw)
            msg = decoder.decode(bytearray(raw))
            self.assertEqual(DIALECT.target_system(frame), msg.target_system, cls.msgname)
            checked += 1
        self.assertGreater(checked, 50)

    def test_target_system_is_zero_when_a_v2_sender_trimmed_it(self):
        raw = command_long(0)                                       # target 0 -> trailing zeros trimmed
        (f,) = mf.FrameSplitter(DIALECT).feed(raw)
        self.assertLess(f.payload_len, 33)
        self.assertEqual(DIALECT.target_system(f), 0)

    def test_message_without_a_target_field(self):
        (f,) = mf.FrameSplitter(DIALECT).feed(heartbeat())
        self.assertIsNone(DIALECT.target_system(f))

    def test_unknown_message_has_no_target(self):
        raw = build_frame(unknown_id(), b"hello", crc_extra=33)
        (f,) = mf.FrameSplitter(DIALECT).feed(raw)
        self.assertIsNone(DIALECT.target_system(f))


if __name__ == "__main__":
    unittest.main()
