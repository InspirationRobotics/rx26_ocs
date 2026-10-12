"""The TUNNEL helpers in mavframe: what fleet_link builds, pymavlink must read, and the
other way round (so a bug in fleet's own packer cannot agree with itself)."""
from __future__ import annotations

import unittest

from pymavlink.dialects.v20 import ardupilotmega as ap

from fleet import mavframe

from .helpers import DIALECT, tunnel_frame


class TunnelFrames(unittest.TestCase):
    def test_a_frame_built_here_decodes_in_pymavlink(self):
        raw = mavframe.build_tunnel_frame(DIALECT, bytes([0x20, 5, 1]), payload_type=0x8038, sysid=250,
                                          compid=190, seq=7, target_system=200)
        msg = ap.MAVLink(None).decode(bytearray(raw))
        self.assertEqual((msg.get_type(), msg.get_srcSystem(), msg.get_srcComponent(), msg.get_seq()),
                         ("TUNNEL", 250, 190, 7))
        self.assertEqual((msg.target_system, msg.target_component, msg.payload_type, msg.payload_length),
                         (200, 0, 0x8038, 3))
        self.assertEqual(bytes(msg.payload[:3]), bytes([0x20, 5, 1]))

    def test_trailing_zeros_are_trimmed_like_mavlink_2_requires(self):
        raw = mavframe.build_tunnel_frame(DIALECT, b"ab", payload_type=0x8038, sysid=1, compid=1, seq=0)
        self.assertLess(len(raw), 10 + 5 + 2 + 2 + 1)
        frame = mavframe.split_datagram(DIALECT, raw)[0]
        self.assertTrue(frame.validated)

    def test_parse_tunnel_reads_pymavlink_frames_with_trimmed_payloads(self):
        raw = tunnel_frame(0x8010, b"abc\x00\x00", sysid=200, target=250, target_comp=3)
        frame = mavframe.split_datagram(DIALECT, raw)[0]
        t = mavframe.parse_tunnel(frame)
        self.assertEqual((t.payload_type, t.target_system, t.target_component, t.payload_length),
                         (0x8010, 250, 3, 5))
        self.assertEqual(t.data, b"abc\x00\x00")
        self.assertTrue(t.length_ok)

    def test_full_128_byte_payload_and_the_limit(self):
        data = bytes(range(1, 129))
        raw = mavframe.build_tunnel_frame(DIALECT, data, payload_type=0x8000, sysid=1, compid=1, seq=1)
        t = mavframe.parse_tunnel(mavframe.split_datagram(DIALECT, raw)[0])
        self.assertEqual(t.data, data)
        with self.assertRaises(ValueError):
            mavframe.build_tunnel_frame(DIALECT, data + b"x", payload_type=0, sysid=1, compid=1, seq=1)

    def test_an_all_zero_payload_still_keeps_one_byte(self):
        raw = mavframe.pack_frame(DIALECT, mavframe.MSG_TUNNEL, bytes(133), sysid=1, compid=1, seq=0)
        self.assertEqual(mavframe.split_datagram(DIALECT, raw)[0].payload_len, 1)

    def test_unknown_msgid_cannot_be_built(self):
        with self.assertRaises(ValueError):
            mavframe.pack_frame(DIALECT, 59999, b"x", sysid=1, compid=1, seq=0)


class CommandLongFrames(unittest.TestCase):
    def build(self, **kw):
        args = dict(sysid=250, compid=190, seq=9, target_system=1, target_component=1)
        args.update(kw)
        return mavframe.build_command_long_frame(DIALECT, mavframe.MAV_CMD_SET_MESSAGE_INTERVAL, (245, 1e6), **args)

    def test_a_frame_built_here_decodes_in_pymavlink(self):
        msg = ap.MAVLink(None).decode(bytearray(self.build()))
        self.assertEqual((msg.get_type(), msg.get_srcSystem(), msg.get_srcComponent(), msg.get_seq()),
                         ("COMMAND_LONG", 250, 190, 9))
        self.assertEqual((msg.target_system, msg.target_component, msg.command, msg.confirmation), (1, 1, 511, 0))
        self.assertEqual((msg.param1, msg.param2, msg.param3, msg.param4, msg.param5, msg.param6, msg.param7),
                         (245.0, 1e6, 0.0, 0.0, 0.0, 0.0, 0.0))

    def test_the_hub_validates_it(self):
        f = mavframe.split_datagram(DIALECT, self.build())[0]
        self.assertEqual((f.msgid, f.validated, f.sysid, f.compid), (mavframe.MSG_COMMAND_LONG, True, 250, 190))

    def test_it_matches_pymavlinks_own_encoder_field_for_field(self):
        mav = ap.MAVLink(None, srcSystem=250, srcComponent=190)
        mav.seq = 9
        theirs = bytes(mav.command_long_encode(1, 1, 511, 0, 245, 1e6, 0, 0, 0, 0, 0).pack(mav))
        self.assertEqual(self.build(), theirs)

    def test_other_targets_and_the_confirmation_byte(self):
        msg = ap.MAVLink(None).decode(bytearray(self.build(target_system=7, target_component=0, confirmation=2)))
        self.assertEqual((msg.target_system, msg.target_component, msg.confirmation), (7, 0, 2))

    def test_more_than_seven_params_are_refused(self):
        with self.assertRaises(ValueError):
            mavframe.build_command_long_frame(DIALECT, 511, [0] * 8, sysid=1, compid=1, seq=0, target_system=1)


if __name__ == "__main__":
    unittest.main()
