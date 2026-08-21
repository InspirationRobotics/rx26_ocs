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
except ImportError as exc:  # pragma: no cover
    raise unittest.SkipTest("generated protobuf unavailable: %s" % exc) from None

ALLOW = ("flight_phase",)


def usv_heartbeat(*, nan: bool = False, unknown_task: bool = False):
    """What robocommand_reporter will build on the Jetson, field for field."""
    r = rx_reports_pb2.RxReport()
    r.team_id = "INSPIRATION"
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


class TestRunDeclaration(unittest.TestCase):
    def _decl(self, task1="TIER_CORE"):
        req = rx_requests_pb2.RxRequest()
        req.team_id = "INSPIRATION"
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
        cmd.team_id = "INSPIRATION"
        cmd.seq = 5
        cmd.run_start.declaration_seq = 1
        cmd.run_start.run_id = 42

        back = rx_commands_pb2.RxCommand()
        back.ParseFromString(cmd.SerializeToString())
        self.assertEqual(back.WhichOneof("body"), "run_start")
        self.assertEqual(back.run_start.declaration_seq, 1)
        self.assertEqual(back.run_start.run_id, 42)


if __name__ == "__main__":
    unittest.main()
