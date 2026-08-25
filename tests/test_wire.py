"""Tests that need the generated protobuf: what actually goes on the wire.

Split from test_core.py because these need `bash ocs/make_protos.sh` and the
venv to have been built. They skip cleanly when it has not, so the core suite
still runs on a bare laptop.

    ocs/.venv/Scripts/python -m unittest discover -s tests -t .
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

try:
    from rx_bridge import validate
    from rx_bridge.proto import common_pb2, rx_common_pb2
    from rx_bridge.proto import rx_commands_pb2, rx_reports_pb2, rx_requests_pb2
    from rx_bridge.proto import rx_course_pb2
except ImportError as exc:  # pragma: no cover
    raise unittest.SkipTest("generated protobuf unavailable: %s" % exc) from None

#: The surface/submerged allowance. Mirrors config._ALLOW_UNKNOWN_BY_TYPE for
#: TYPE_USV and TYPE_UUV; UAV_ALLOW is the aircraft's, which permits nothing.
ALLOW = ("flight_phase",)
UAV_ALLOW: tuple[str, ...] = ()


def usv_heartbeat(*, nan: bool = False, unknown_task: bool = False):
    """What robocommand_reporter will build on the Jetson, field for field."""
    r = rx_reports_pb2.RxReport()
    r.team_id = "ASTA"
    r.vehicle_id = "USV1"
    r.sent_at.FromNanoseconds(time.time_ns())

    hb = r.heartbeat
    hb.state = common_pb2.RobotState.Value("STATE_AUTO")
    hb.position.latitude = 27.3364
    hb.position.longitude = -82.5307
    hb.spd_mps = 1.4
    hb.heading_deg = float("nan") if nan else 91.2
    hb.roll_deg = 2.0
    hb.pitch_deg = 0.5
    hb.altitude_hae_m = -24.6
    hb.depth_m = 0.0
    hb.vehicle_type = rx_common_pb2.VehicleType.TYPE_USV
    if not unknown_task:
        hb.current_task = rx_common_pb2.RxTask.TASK_NONE
    return r


def uav_heartbeat(*, no_phase: bool = False):
    """BabyDragon. Airborne, so altitude is real and depth is not.

    `no_phase` is the failure this exists to catch: a reporter that filled in
    every other field and left flight_phase at its zero value.
    """
    r = rx_reports_pb2.RxReport()
    r.team_id = "ASTA"
    r.vehicle_id = "UAV1"
    r.sent_at.FromNanoseconds(time.time_ns())

    hb = r.heartbeat
    hb.state = common_pb2.RobotState.Value("STATE_AUTO")
    hb.position.latitude = 27.3364
    hb.position.longitude = -82.5307
    hb.spd_mps = 9.0
    hb.heading_deg = 180.0
    hb.roll_deg = 4.0
    hb.pitch_deg = -2.0
    hb.altitude_hae_m = 42.0
    hb.depth_m = 0.0
    hb.vehicle_type = rx_common_pb2.VehicleType.TYPE_UAV
    hb.current_task = rx_common_pb2.RxTask.TASK_NONE
    if not no_phase:
        hb.flight_phase = rx_common_pb2.FlightPhase.FLIGHT_PHASE_AIRBORNE
    return r


def uuv_heartbeat():
    """Graey. Submerged, so depth is real and altitude_hae is not meaningful.

    Attributed to UUV1 even though Crusader's radio is what carries it: the
    relay is our network's business, and RoboCommand must see the vehicle that
    took the measurement.
    """
    r = rx_reports_pb2.RxReport()
    r.team_id = "ASTA"
    r.vehicle_id = "UUV1"
    r.sent_at.FromNanoseconds(time.time_ns())

    hb = r.heartbeat
    hb.state = common_pb2.RobotState.Value("STATE_AUTO")
    hb.position.latitude = 27.3364
    hb.position.longitude = -82.5307
    hb.spd_mps = 0.6
    hb.heading_deg = 275.0
    hb.roll_deg = 1.0
    hb.pitch_deg = -3.0
    hb.altitude_hae_m = 0.0
    hb.depth_m = 4.5
    hb.vehicle_type = rx_common_pb2.VehicleType.TYPE_UUV
    hb.current_task = rx_common_pb2.RxTask.TASK_INFRA_SURVEY_REPAIR
    return r


class TestHeartbeat(unittest.TestCase):
    def test_a_complete_usv_heartbeat_passes(self):
        self.assertEqual(validate.check(usv_heartbeat(), allow_unknown=ALLOW), [])

    def test_it_is_small_enough_to_ignore_at_2hz(self):
        self.assertLess(len(usv_heartbeat().SerializeToString()), 200)

    def test_round_trip_preserves_every_field(self):
        raw = usv_heartbeat().SerializeToString()
        back = rx_reports_pb2.RxReport()
        back.ParseFromString(raw)
        self.assertEqual(back.vehicle_id, "USV1")
        self.assertEqual(back.heartbeat.current_task, rx_common_pb2.RxTask.TASK_NONE)
        self.assertEqual(back.heartbeat.vehicle_type, rx_common_pb2.VehicleType.TYPE_USV)
        self.assertAlmostEqual(back.heartbeat.position.latitude, 27.3364, places=6)


class TestValidator(unittest.TestCase):
    def test_nan_heading_is_caught(self):
        """crusader_fcu/telemetry_bridge.py:210 emits NaN when GPS yaw is unresolved.

        Protobuf serializes it without complaint, so this check is the only
        thing between an unresolved heading and a scored run.
        """
        findings = validate.check(usv_heartbeat(nan=True), allow_unknown=ALLOW)
        self.assertTrue(any(f.problem == "NaN" for f in findings))
        self.assertTrue(any("heading_deg" in f.path for f in findings))

    def test_unset_current_task_is_caught_as_unknown(self):
        """The footgun: a forgotten field and an illegal value are the same bytes."""
        findings = validate.check(usv_heartbeat(unknown_task=True), allow_unknown=ALLOW)
        self.assertTrue(any("current_task" in f.path for f in findings))
        self.assertTrue(any(f.problem == "UNKNOWN" for f in findings))

    def test_flight_phase_unknown_is_permitted_for_a_surface_vehicle(self):
        clean = validate.check(usv_heartbeat(), allow_unknown=ALLOW)
        self.assertFalse(any("flight_phase" in f.path for f in clean))

    def test_the_allowlist_is_actually_load_bearing(self):
        """Guards against the allowlist quietly matching nothing."""
        without = validate.check(usv_heartbeat(), allow_unknown=())
        self.assertTrue(any("flight_phase" in f.path for f in without))

    def test_it_descends_into_nested_messages(self):
        r = usv_heartbeat()
        r.heartbeat.position.latitude = float("inf")
        findings = validate.check(r, allow_unknown=ALLOW)
        self.assertTrue(any(f.path == "heartbeat.position.latitude" for f in findings))


class TestPerVehicleValidationPolicy(unittest.TestCase):
    """The allowance is chosen by the CONFIGURED type, never by the message.

    While the fleet was one boat, `allow_unknown` was a single global tuple
    holding "flight_phase". Adding an aircraft to that arrangement hands the
    aircraft the boats' exemption -- and the handbook has RoboNation relaying
    UAV heartbeat telemetry to Garuda Robotics for Singapore's Network Remote
    ID, so an UNKNOWN flight phase is a regulatory problem rather than a
    scoring one.
    """

    def test_a_uav_that_forgot_its_flight_phase_is_caught(self):
        r = uav_heartbeat(no_phase=True)
        findings = validate.check(r, allow_unknown=UAV_ALLOW)
        self.assertTrue(any("flight_phase" in f.path for f in findings))

    def test_a_complete_uav_heartbeat_passes(self):
        self.assertEqual(validate.check(uav_heartbeat(), allow_unknown=UAV_ALLOW), [])

    def test_the_surface_allowance_would_have_hidden_it(self):
        """The regression this whole change exists to prevent.

        If this ever passes an empty finding list, the UAV has been handed the
        surface-vehicle policy again and the Remote ID field is unguarded.
        """
        hidden = validate.check(uav_heartbeat(no_phase=True), allow_unknown=ALLOW)
        self.assertEqual(hidden, [], "precondition: the old global policy hid this")

        caught = validate.check(uav_heartbeat(no_phase=True), allow_unknown=UAV_ALLOW)
        self.assertTrue(caught, "the UAV policy must catch what the global one hid")

    def test_a_submerged_vehicle_keeps_the_exemption(self):
        r = uuv_heartbeat()
        self.assertEqual(validate.check(r, allow_unknown=ALLOW), [])


class TestRunDeclaration(unittest.TestCase):
    def _decl(self, task1="TIER_CORE"):
        req = rx_requests_pb2.RxRequest()
        req.team_id = "ASTA"
        req.seq = 1
        req.sent_at.FromNanoseconds(time.time_ns())
        d = req.run_declaration
        d.vehicle_ids.append("USV1")
        tier = common_pb2.TaskTier
        d.task1_tier = tier.Value(task1)
        d.task2_tier = tier.Value("TIER_NONE")
        d.task3_tier = tier.Value("TIER_NONE")
        d.task4_tier = tier.Value("TIER_NONE")
        return req

    def test_usv_only_declaration_with_no_geofence_is_valid(self):
        """An empty uav_geofence is correct while no UAV is declared."""
        self.assertEqual(validate.check(self._decl(), allow_unknown=ALLOW), [])

    def test_a_tier_left_unset_is_caught(self):
        req = self._decl()
        req.run_declaration.ClearField("task3_tier")
        findings = validate.check(req, allow_unknown=ALLOW)
        self.assertTrue(any("task3_tier" in f.path for f in findings))


class TestRunStartMatching(unittest.TestCase):
    def test_run_start_carries_the_declaration_seq_we_must_match(self):
        cmd = rx_commands_pb2.RxCommand()
        cmd.team_id = "ASTA"
        cmd.seq = 5
        cmd.run_start.declaration_seq = 1
        cmd.run_start.run_id = 42

        back = rx_commands_pb2.RxCommand()
        back.ParseFromString(cmd.SerializeToString())
        self.assertEqual(back.WhichOneof("body"), "run_start")
        self.assertEqual(back.run_start.declaration_seq, 1)
        self.assertEqual(back.run_start.run_id, 42)


def a_course(*, corners=4, cid="ALPHA", freq=25000):
    """What RoboCommand publishes retained, at their own test course."""
    c = rx_course_pb2.RxCourse()
    c.course_id = cid
    c.pinger_freq_hz = freq
    pts = [(1.28010, 103.85520), (1.28010, 103.85625),
           (1.28110, 103.85625), (1.28110, 103.85520)]
    for lat, lon in pts[:corners]:
        c.corners.add(latitude=lat, longitude=lon)
    return c


class TestCourseValidation(unittest.TestCase):
    """Project.md's sequence says the OCS receives AND VALIDATES RxCourse.

    The course is the one message we are handed rather than build, and
    everything downstream is positioned relative to it.
    """

    def test_a_good_course_is_clean(self):
        self.assertEqual(validate.check_course(a_course()), [])

    def test_a_course_with_too_few_corners_is_caught(self):
        f = validate.check_course(a_course(corners=2))
        self.assertTrue(any(x.path == "corners" for x in f), f)

    def test_a_nameless_course_is_caught(self):
        f = validate.check_course(a_course(cid=""))
        self.assertTrue(any(x.path == "course_id" for x in f), f)

    def test_a_zero_pinger_frequency_is_caught(self):
        f = validate.check_course(a_course(freq=0))
        self.assertTrue(any(x.path == "pinger_freq_hz" for x in f), f)

    def test_a_nan_corner_is_caught(self):
        """The same net that catches a NaN heading, pointed at their frame."""
        c = a_course()
        c.corners[0].latitude = float("nan")
        f = validate.check_course(c)
        self.assertTrue(any(x.problem == "NaN" for x in f), f)

    def test_findings_are_advisory_and_carry_a_readable_reason(self):
        """Nothing here blocks a run -- the operator has to be able to read it
        and decide, so every finding must say what is wrong in words."""
        for f in validate.check_course(a_course(corners=1, cid="", freq=0)):
            self.assertTrue(str(f).strip())
            self.assertIn(":", str(f))


class TestHeartbeatRate(unittest.TestCase):
    """Project.md: the OCS publishes heartbeats for all declared vehicles.

    The handbook puts a number on that -- 2 Hz -- so the bridge measures what it
    actually published rather than assuming the forward loop kept up.
    """

    def setUp(self):
        from rx_bridge.bridge import HeartbeatRate
        self.r = HeartbeatRate()

    def test_two_hz_measures_as_two_hz(self):
        for i in range(20):
            self.r.record("USV1", 100.0 + i * 0.5)
        self.assertAlmostEqual(self.r.hz("USV1", 100.0 + 19 * 0.5), 2.0, places=2)

    def test_a_slow_vehicle_reads_slow(self):
        for i in range(5):
            self.r.record("UUV1", 100.0 + i * 2.0)
        self.assertAlmostEqual(self.r.hz("UUV1", 108.0), 0.5, places=2)

    def test_a_vehicle_never_heard_from_is_zero_not_an_error(self):
        self.assertEqual(self.r.hz("UAV1", 200.0), 0.0)

    def test_one_lone_frame_is_not_a_rate(self):
        self.r.record("USV1", 100.0)
        self.assertEqual(self.r.hz("USV1", 100.5), 0.0)

    def test_the_window_forgets_old_frames(self):
        """A vehicle that stopped an hour ago must not still read 2 Hz."""
        for i in range(20):
            self.r.record("USV1", 100.0 + i * 0.5)
        self.assertEqual(self.r.hz("USV1", 100.0 + 3600), 0.0)

    def test_vehicles_are_metered_independently(self):
        for i in range(20):
            self.r.record("USV1", 100.0 + i * 0.5)
        self.assertEqual(self.r.hz("UUV1", 109.5), 0.0)


if __name__ == "__main__":
    unittest.main()
