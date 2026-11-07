"""ocs_heartbeat: the stand-in heartbeat is built only from what the autopilot really said, and
every blank means NOTHING is sent -- with the reason -- never a value made up to fill it."""
from __future__ import annotations

import math
import unittest
from unittest import mock

from fleet import mavframe
from fleet import ocs_heartbeat as hbm
from fleet.ocs_heartbeat import HeartbeatSynth

from .helpers import DIALECT, encode, heartbeat, position

MAV_AUTOPILOT_INVALID = 8
ARMED = 129                  # MAV_MODE_FLAG_SAFETY_ARMED | CUSTOM_MODE_ENABLED
DISARMED = 1
GUIDED, STABILIZE, LOITER = 4, 0, 5
GEOID = -24.6


def frame(raw: bytes) -> mavframe.Frame:
    return mavframe.split_datagram(DIALECT, raw)[0]


def attitude(roll=0.1, pitch=-0.05, sysid=1, compid=1) -> bytes:
    return encode("attitude", 0, roll, pitch, 1.5, 0.0, 0.0, 0.0, sysid=sysid, compid=compid)


def ext_state(landed=1, sysid=1, compid=1) -> bytes:
    return encode("extended_sys_state", 0, landed, sysid=sysid, compid=compid)


def synth(**kw) -> HeartbeatSynth:
    args = dict(autopilot_sysid=1, geoid_separation_m=GEOID)
    args.update(kw)
    return HeartbeatSynth(**args)


def feed(s: HeartbeatSynth, t: float = 100.0, *, hb=True, pos=True, att=True, ext=1, mode=GUIDED, base=ARMED,
         **pos_kw) -> None:
    """Hand the synth a complete, healthy picture (or leave one input out)."""
    if hb:
        s.note(frame(heartbeat(1, base_mode=base, custom_mode=mode)), t)
    if pos:
        s.note(frame(position(1, **pos_kw)), t)
    if att:
        s.note(frame(attitude()), t)
    if ext is not None:
        s.note(frame(ext_state(ext)), t)


class WhatItSends(unittest.TestCase):
    def test_a_full_set_builds_a_heartbeat_from_the_autopilots_numbers(self):
        s = synth()
        feed(s)
        hb, source = s.build(100.5)
        self.assertEqual(source, "autopilot")
        self.assertEqual(hb["state"], "STATE_AUTO")
        self.assertEqual(hb["position"], {"latitude": -35.3632621, "longitude": 149.1652374})
        self.assertAlmostEqual(hb["spd_mps"], 5.0)                          # hypot(300, 400) cm/s
        self.assertEqual(hb["heading_deg"], 90.0)
        self.assertAlmostEqual(hb["altitude_hae_m"], 584.09 + GEOID)
        self.assertEqual(hb["depth_m"], 0.0)
        self.assertEqual(hb["vehicle_type"], "TYPE_UAV")
        self.assertEqual(hb["flight_phase"], "FLIGHT_PHASE_GROUNDED")
        self.assertAlmostEqual(hb["roll_deg"], math.degrees(0.1), 4)
        self.assertAlmostEqual(hb["pitch_deg"], math.degrees(-0.05), 4)

    def test_current_task_is_the_literal_TASK_NONE_the_aircraft_itself_sends(self):
        s = synth()
        feed(s)
        hb, _ = s.build(100.0)
        self.assertEqual(hb["current_task"], "TASK_NONE")
        self.assertEqual(hbm.CURRENT_TASK, "TASK_NONE")
        self.assertIn("heartbeat_core", hbm.__doc__)
        self.assertIn("heartbeat_core.py:138", hbm.__doc__)

    def test_flight_phase_follows_the_autopilots_landed_state(self):
        for landed, phase in ((1, "FLIGHT_PHASE_GROUNDED"), (2, "FLIGHT_PHASE_AIRBORNE"),
                              (3, "FLIGHT_PHASE_AIRBORNE"), (4, "FLIGHT_PHASE_AIRBORNE")):
            s = synth()
            feed(s, ext=landed)
            hb, source = s.build(100.0)
            self.assertEqual((hb["flight_phase"], source), (phase, "autopilot"), landed)

    def test_state_is_AUTO_only_when_armed_in_an_autonomous_mode(self):
        for base, mode, want in ((ARMED, GUIDED, "STATE_AUTO"), (ARMED, LOITER, "STATE_MANUAL"),
                                 (ARMED, STABILIZE, "STATE_MANUAL"), (DISARMED, GUIDED, "STATE_AUTO"),
                                 (DISARMED, LOITER, "STATE_MANUAL"), (DISARMED, STABILIZE, "STATE_MANUAL")):
            s = synth()
            feed(s, base=base, mode=mode)
            self.assertEqual(s.build(100.0)[0]["state"], want, (base, mode))

    def test_state_is_never_UNKNOWN(self):
        self.assertEqual(hbm.robot_state(None, None), "STATE_MANUAL")
        self.assertEqual(hbm.robot_state("", True), "STATE_MANUAL")
        self.assertEqual(hbm.robot_state("guided", True), "STATE_AUTO")

    def test_a_configured_vehicle_type_is_used(self):
        s = synth(vehicle_type="TYPE_UAV")
        feed(s)
        self.assertEqual(s.build(100.0)[0]["vehicle_type"], "TYPE_UAV")


class WhatItRefusesToSend(unittest.TestCase):
    """Each blank is (None, reason). No partial dict, no default, no last-known value."""

    def assertBlank(self, s, now, *words):
        hb, reason = s.build(now)
        self.assertIsNone(hb)
        for w in words:
            self.assertIn(w, reason)
        self.assertIsNone(s.data_t)
        return reason

    def test_nothing_heard(self):
        self.assertBlank(synth(), 100.0, "GLOBAL_POSITION_INT", "never heard")

    def test_no_position(self):
        s = synth()
        feed(s, pos=False)
        self.assertBlank(s, 100.0, "GLOBAL_POSITION_INT")

    def test_no_autopilot_heartbeat(self):
        s = synth()
        feed(s, hb=False)
        self.assertBlank(s, 100.0, "HEARTBEAT", "armed state and mode")

    def test_no_attitude_because_roll_and_pitch_would_go_out_as_zero(self):
        s = synth()
        feed(s, att=False)
        self.assertBlank(s, 100.0, "ATTITUDE", "0.0")

    def test_no_geoid_separation_means_no_altitude_means_no_heartbeat(self):
        s = synth(geoid_separation_m=None)
        feed(s)
        self.assertBlank(s, 100.0, "geoid_separation_m")

    def test_no_extended_sys_state_and_no_fallback(self):
        s = synth()
        feed(s, ext=None)
        self.assertBlank(s, 100.0, "EXTENDED_SYS_STATE", "fallback is off")

    def test_landed_state_undefined(self):
        s = synth()
        feed(s, ext=0)
        self.assertBlank(s, 100.0, "UNDEFINED")

    def test_landed_state_outside_the_mapping(self):
        s = synth()
        feed(s, ext=9)
        self.assertBlank(s, 100.0, "not in the mapping")

    def test_the_ekf_has_no_origin_yet(self):
        s = synth()
        feed(s, lat=0, lon=0)
        self.assertBlank(s, 100.0, "0,0")

    def test_heading_unresolved(self):
        s = synth()
        feed(s, hdg=65535)
        self.assertBlank(s, 100.0, "heading unresolved")

    def test_each_input_goes_stale_on_its_own_clock(self):
        heard_long_ago = {
            "pose": (position(1), "GLOBAL_POSITION_INT"),
            "att": (attitude(), "ATTITUDE"),
            "hb": (heartbeat(1, base_mode=ARMED, custom_mode=GUIDED), "HEARTBEAT"),
            "ext": (ext_state(1), "EXTENDED_SYS_STATE"),
        }
        for stale, (raw, word) in heard_long_ago.items():
            s = synth()
            feed(s, t=100.0, pos=stale != "pose", att=stale != "att", hb=stale != "hb",
                 ext=None if stale == "ext" else 1)
            s.note(frame(raw), 90.0)
            self.assertBlank(s, 100.0, word)

    def test_inputs_at_the_edge_of_their_timeouts_still_count(self):
        s = synth()
        feed(s, t=100.0)
        self.assertIsNotNone(s.build(100.9)[0])                          # pose and attitude: 1.0 s
        self.assertIsNone(s.build(101.1)[0])

    def test_a_blank_after_a_good_heartbeat_is_blank_not_the_last_value(self):
        s = synth()
        feed(s, t=100.0)
        self.assertIsNotNone(s.build(100.5)[0])
        hb, reason = s.build(105.0)
        self.assertIsNone(hb)
        self.assertIn("older than", reason)

    def test_every_blank_carries_a_reason_that_says_what_to_check(self):
        s = synth()
        _, reason = s.build(100.0)
        self.assertTrue(reason and len(reason) > 20)


class Fallback(unittest.TestCase):
    def test_off_by_default(self):
        self.assertFalse(synth().phase_fallback)

    def test_armed_and_high_is_airborne_when_the_fallback_is_on(self):
        s = synth(phase_fallback=True)
        feed(s, ext=None, rel=10500)
        hb, source = s.build(100.0)
        self.assertEqual((hb["flight_phase"], source), ("FLIGHT_PHASE_AIRBORNE", "fallback"))

    def test_disarmed_or_low_is_grounded(self):
        for base, rel in ((DISARMED, 10500), (ARMED, 300)):
            s = synth(phase_fallback=True)
            feed(s, ext=None, base=base, rel=rel)
            self.assertEqual(s.build(100.0)[0]["flight_phase"], "FLIGHT_PHASE_GROUNDED", (base, rel))

    def test_the_autopilots_own_answer_wins_over_the_fallback(self):
        s = synth(phase_fallback=True)
        feed(s, ext=1, rel=10500)                                        # on the ground, per the autopilot
        hb, source = s.build(100.0)
        self.assertEqual((hb["flight_phase"], source), ("FLIGHT_PHASE_GROUNDED", "autopilot"))

    def test_flight_phase_is_a_blank_unless_the_fallback_is_allowed(self):
        phase, source = hbm.flight_phase(0, armed=True, altitude_rel=10.0, allow_fallback=False)
        self.assertEqual((phase, source), (None, None))
        phase, source = hbm.flight_phase(0, armed=True, altitude_rel=10.0, allow_fallback=True)
        self.assertEqual((phase, source), ("FLIGHT_PHASE_AIRBORNE", "fallback"))

    def test_the_airborne_threshold_is_configurable(self):
        s = synth(phase_fallback=True, airborne_alt_m=20.0)
        feed(s, ext=None, rel=10500)
        self.assertEqual(s.build(100.0)[0]["flight_phase"], "FLIGHT_PHASE_GROUNDED")


class WhichFramesCount(unittest.TestCase):
    def test_another_sysid_is_ignored(self):
        s = synth()
        s.note(frame(position(7)), 100.0)
        s.note(frame(heartbeat(7)), 100.0)
        self.assertIsNone(s.inputs_status(100.0)["pose_age_s"])
        self.assertIsNone(s.inputs_status(100.0)["status_age_s"])

    def test_the_gimbal_and_other_components_on_the_autopilots_sysid_are_ignored(self):
        s = synth()
        s.note(frame(heartbeat(1, compid=154, base_mode=0)), 100.0)
        s.note(frame(position(1)), 100.0)                                 # compid 1: counts
        s.note(frame(encode("global_position_int", 0, 1, 1, 1, 1, 0, 0, 0, 0, sysid=1, compid=154)), 101.0)
        self.assertIsNone(s.inputs_status(101.0)["status_age_s"])
        self.assertEqual(s.inputs_status(101.0)["pose_age_s"], 1.0)       # the compid-154 one did not refresh it

    def test_a_heartbeat_with_autopilot_invalid_is_not_the_autopilot(self):
        s = synth()
        s.note(frame(heartbeat(1, autopilot=MAV_AUTOPILOT_INVALID, type=18)), 100.0)
        self.assertIsNone(s.inputs_status(100.0)["status_age_s"])

    def test_an_unvalidated_frame_is_ignored(self):
        s = synth()
        good = frame(position(1))
        bad = mavframe.Frame(good.raw, good.version, good.seq, good.sysid, good.compid, good.msgid,
                             good.payload_len, good.signed, False)
        s.note(bad, 100.0)
        self.assertIsNone(s.inputs_status(100.0)["pose_age_s"])

    def test_other_messages_are_ignored(self):
        s = synth()
        s.note(frame(encode("sys_status", 0, 0, 0, 500, 12000, -1, -1, 0, 0, 0, 0, 0, 0, sysid=1)), 100.0)
        self.assertEqual(s.inputs_status(100.0), {"pose_age_s": None, "attitude_age_s": None,
                                                  "status_age_s": None, "landed_age_s": None})

    def test_a_frame_pymavlink_refuses_is_counted_and_never_becomes_a_value(self):
        s = synth()
        with mock.patch.object(hbm._decoder, "decode", side_effect=ValueError("garbled")):
            s.note(frame(position(1)), 100.0)
        self.assertEqual(s.decode_errors, 1)
        self.assertIsNone(s.inputs_status(100.0)["pose_age_s"])

    def test_ages_report_the_time_since_each_input(self):
        s = synth()
        feed(s, t=100.0)
        self.assertEqual(s.inputs_status(100.5), {"pose_age_s": 0.5, "attitude_age_s": 0.5,
                                                  "status_age_s": 0.5, "landed_age_s": 0.5})
        self.assertEqual(s.flight_age(103.0), 3.0)
        self.assertEqual(s.status_age(104.0), 4.0)
        self.assertIsNone(synth().flight_age(0.0))
        self.assertIsNone(synth().status_age(0.0))


class DataTime(unittest.TestCase):
    """sent_at must say when the DATA was in hand, not when the heartbeat was assembled."""

    def test_it_is_the_newest_input_used(self):
        s = synth()
        s.note(frame(heartbeat(1, base_mode=ARMED, custom_mode=GUIDED)), 100.0)
        s.note(frame(ext_state(1)), 100.1)
        s.note(frame(attitude()), 100.25)
        s.note(frame(position(1)), 100.4)
        self.assertIsNotNone(s.build(100.9)[0])
        self.assertEqual(s.data_t, 100.4)

    def test_a_newer_message_moves_it(self):
        s = synth()
        feed(s, t=100.0)
        s.build(100.2)
        self.assertEqual(s.data_t, 100.0)
        s.note(frame(attitude()), 100.3)
        s.build(100.4)
        self.assertEqual(s.data_t, 100.3)

    def test_the_landed_state_counts_when_it_was_used(self):
        s = synth()
        feed(s, t=100.0)
        s.note(frame(ext_state(1)), 100.6)
        s.build(100.7)
        self.assertEqual(s.data_t, 100.6)

    def test_the_landed_state_does_not_count_when_the_fallback_decided(self):
        s = synth(phase_fallback=True)
        feed(s, t=100.0, ext=None)
        s.note(frame(ext_state(0)), 100.6)                                # UNDEFINED: newest, but not what decided
        hb, source = s.build(100.7)
        self.assertEqual(source, "fallback")
        self.assertEqual(s.data_t, 100.0)

    def test_a_failed_build_clears_it(self):
        s = synth()
        feed(s, t=100.0)
        s.build(100.2)
        self.assertEqual(s.data_t, 100.0)
        self.assertIsNone(s.build(200.0)[0])
        self.assertIsNone(s.data_t)


if __name__ == "__main__":
    unittest.main()
