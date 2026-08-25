"""Tests for the parts of the bridge that hold rules rather than sockets.

stdlib unittest, no protobuf, no MQTT: this suite has to run on any laptop that
turns up at a competition, including one with no network and a fresh Python.

    python -m unittest discover -s ocs/tests -v
"""
from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge.active import ACTIVE_WINDOW_S, choose_active  # noqa: E402
from rx_bridge.config import ConfigError, load          # noqa: E402
from rx_bridge.governor import Governor                 # noqa: E402
from rx_bridge.runstate import RunMachine, RunState     # noqa: E402
from rx_bridge.seqstore import SeqStore                 # noqa: E402


class TestRunMachine(unittest.TestCase):
    def _declared(self) -> RunMachine:
        m = RunMachine()
        m.on_connect()
        m.on_course(object())
        m.on_declared(7)
        return m

    def test_happy_sequence(self):
        m = RunMachine()
        self.assertTrue(m.on_connect())
        self.assertEqual(m.state, RunState.CONNECTED)
        self.assertFalse(m.may_declare())

        self.assertTrue(m.on_course(object()))
        self.assertTrue(m.may_declare())

        self.assertTrue(m.on_declared(7))
        self.assertTrue(m.on_run_start(7, 42))
        self.assertEqual(m.state, RunState.RUNNING)
        self.assertEqual(m.run_id, 42)

    def test_reports_flow_after_declaration_not_after_run_start(self):
        """The window the handbook actually specifies, and the easy one to miss."""
        m = RunMachine()
        m.on_connect()
        self.assertFalse(m.may_report())
        m.on_course(object())
        self.assertFalse(m.may_report())
        m.on_declared(1)
        self.assertTrue(m.may_report(), "heartbeats must flow while awaiting RunStart")
        m.on_run_start(1, 9)
        self.assertTrue(m.may_report())

    def test_run_start_with_wrong_declaration_seq_is_refused(self):
        m = self._declared()
        v = m.on_run_start(8, 42)
        self.assertFalse(v.ok)
        self.assertIn("does not match", v.reason)
        self.assertEqual(m.state, RunState.DECLARED, "must not start on a stray RunStart")

    def test_run_start_before_declaring_is_refused(self):
        m = RunMachine()
        m.on_connect()
        m.on_course(object())
        self.assertFalse(m.on_run_start(1, 1).ok)
        self.assertEqual(m.state, RunState.COURSE_RX)

    def test_reconnect_does_not_rewind_a_declared_run(self):
        """Re-declaring on reconnect would orphan the RunStart we are waiting for."""
        m = self._declared()
        m.on_disconnect()
        self.assertEqual(m.state, RunState.DISCONNECTED)
        self.assertFalse(m.may_report())

        m.on_connect()
        self.assertEqual(m.state, RunState.DECLARED)
        self.assertFalse(m.may_declare(), "must NOT re-declare after a reconnect")
        self.assertEqual(m.declaration_seq, 7)
        self.assertTrue(m.on_run_start(7, 42).ok, "the original RunStart still matches")

    def test_reconnect_mid_run_resumes_running(self):
        m = self._declared()
        m.on_run_start(7, 42)
        m.on_disconnect()
        m.on_connect()
        self.assertEqual(m.state, RunState.RUNNING)
        self.assertTrue(m.may_report())

    def test_retained_course_on_reconnect_does_not_disturb_state(self):
        m = self._declared()
        m.on_disconnect()
        m.on_connect()
        m.on_course(object())          # retained, redelivered on every reconnect
        self.assertEqual(m.state, RunState.DECLARED)


class TestRestoreAfterProcessRestart(unittest.TestCase):
    """A dropped socket and a killed process are different failures.

    on_disconnect/on_connect covers the first; only restore() covers the second.
    Found by bench_stage1.py check A12b, which is why it is a test now.
    """

    def test_restore_resumes_a_declared_run_on_the_next_connect(self):
        m = RunMachine()
        v = m.restore("DECLARED", 7, None)
        self.assertTrue(v.ok, v.reason)
        self.assertEqual(m.state, RunState.DISCONNECTED,
                         "must not claim a connection it does not have")
        self.assertEqual(m.declaration_seq, 7)

        m.on_connect()
        self.assertEqual(m.state, RunState.DECLARED)
        self.assertTrue(m.may_report())
        self.assertFalse(m.may_declare(), "restoring must not invite a re-declare")
        self.assertTrue(m.on_run_start(7, 42).ok, "the original RunStart still matches")

    def test_restore_resumes_a_running_run(self):
        m = RunMachine()
        m.restore("RUNNING", 3, 42)
        m.on_connect()
        self.assertEqual(m.state, RunState.RUNNING)
        self.assertEqual(m.run_id, 42)
        self.assertTrue(m.may_report())

    def test_a_state_that_is_not_resumable_starts_clean(self):
        m = RunMachine()
        v = m.restore("COURSE_RX", None, None)
        self.assertFalse(v.ok)
        m.on_connect()
        self.assertEqual(m.state, RunState.CONNECTED)

    def test_a_corrupt_state_name_does_not_raise(self):
        m = RunMachine()
        v = m.restore("BANANA", 1, 1)
        self.assertFalse(v.ok)
        self.assertIn("BANANA", v.reason)
        m.on_connect()
        self.assertEqual(m.state, RunState.CONNECTED)

    def test_the_store_carries_the_run_across_a_restart(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "seq.json"
            s = SeqStore(path)
            s.new_epoch("run-1")
            s.next_report("USV1")
            s.save_run("RUNNING", 1, 42)

            reborn = SeqStore(path)
            self.assertEqual(reborn.run,
                             {"state": "RUNNING", "declaration_seq": 1, "run_id": 42})

            m = RunMachine()
            m.restore(str(reborn.run["state"]),
                      reborn.run["declaration_seq"], reborn.run["run_id"])
            m.on_connect()
            self.assertEqual(m.state, RunState.RUNNING)

    def test_a_new_epoch_forgets_the_previous_run(self):
        with tempfile.TemporaryDirectory() as d:
            s = SeqStore(Path(d) / "seq.json")
            s.new_epoch("run-1")
            s.save_run("RUNNING", 1, 42)
            s.new_epoch("run-2")
            self.assertEqual(s.run, {})


class TestGovernor(unittest.TestCase):
    def test_heartbeats_survive_a_flood_of_reports(self):
        """The property the whole class exists for."""
        g = Governor(rate=5.0, reserve=2.0)
        t = 100.0
        for _ in range(50):
            g.allow(t, heartbeat=False)
        self.assertTrue(g.allow(t, heartbeat=True),
                        "a report flood must never starve the mandated 2 Hz")

    def test_reports_stop_at_the_reserve(self):
        g = Governor(rate=5.0, reserve=2.0)
        t = 100.0
        passed = sum(1 for _ in range(10) if g.allow(t, heartbeat=False))
        self.assertEqual(passed, 3, "5 tokens minus a 2-token floor")
        self.assertAlmostEqual(g.tokens, 2.0, places=6)

    def test_refill_is_rate_limited_and_capped(self):
        g = Governor(rate=5.0, reserve=0.0)
        t = 0.0
        for _ in range(5):
            g.allow(t, heartbeat=True)
        self.assertFalse(g.allow(t, heartbeat=True))
        self.assertTrue(g.allow(t + 0.2, heartbeat=True), "0.2 s buys one token")
        g.allow(t + 100.0, heartbeat=True)
        self.assertLessEqual(g.tokens, 5.0, "the bucket must not overfill")

    def test_two_hz_is_sustainable_indefinitely(self):
        g = Governor(rate=5.0, reserve=2.0)
        for i in range(200):
            self.assertTrue(g.allow(i * 0.5, heartbeat=True), "dropped at t=%.1f" % (i * 0.5))
        self.assertEqual(g.dropped, 0)


class TestSeqStore(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = Path(self.dir.name) / "seq.json"

    def tearDown(self):
        self.dir.cleanup()

    def test_counters_are_independent_per_vehicle(self):
        s = SeqStore(self.path)
        s.new_epoch("run-1")
        self.assertEqual([s.next_report("USV1") for _ in range(3)], [1, 2, 3])
        self.assertEqual(s.next_report("UAV1"), 1)
        self.assertEqual(s.next_report("USV1"), 4)

    def test_counters_survive_a_restart(self):
        """The crash-at-minute-nine case."""
        s = SeqStore(self.path)
        s.new_epoch("run-1")
        for _ in range(9):
            s.next_report("USV1")

        reborn = SeqStore(self.path)          # as if systemd restarted us
        self.assertEqual(reborn.epoch, "run-1")
        self.assertEqual(reborn.next_report("USV1"), 10, "seq must not go backwards")

    def test_new_epoch_zeroes_everything(self):
        s = SeqStore(self.path)
        s.new_epoch("run-1")
        s.next_report("USV1")
        s.next_request()
        s.new_epoch("run-2")
        self.assertEqual(s.next_report("USV1"), 1)
        self.assertEqual(s.next_request(), 1)

    def test_request_counter_is_team_wide_and_monotonic(self):
        s = SeqStore(self.path)
        s.new_epoch("run-1")
        self.assertEqual([s.next_request() for _ in range(3)], [1, 2, 3])

    def test_a_corrupt_store_does_not_stop_the_bridge(self):
        self.path.write_text("{ this is not json", encoding="utf-8")
        s = SeqStore(self.path)               # must not raise
        s.new_epoch("run-1")
        self.assertEqual(s.next_report("USV1"), 1)

    def test_write_is_atomic_leaving_no_temp_files(self):
        s = SeqStore(self.path)
        s.new_epoch("run-1")
        s.next_report("USV1")
        leftovers = [p.name for p in self.path.parent.iterdir() if p.suffix == ".tmp"]
        self.assertEqual(leftovers, [])
        json.loads(self.path.read_text(encoding="utf-8"))


class TestConfig(unittest.TestCase):
    BASE = """
team_id = "ASTA"
[[vehicle]]
id = "USV1"
type = "TYPE_USV"
[team]
host = "127.0.0.1"
[robocommand]
host = "192.168.65.2"
"""

    #: The full three-vehicle fleet, as bridge.toml ships it. Bare keys first:
    #: see _with_fence for why that ordering is load-bearing.
    FLEET = """
team_id = "ASTA"
uav_geofence = [[1.0,2.0],[1.0,3.0],[2.0,3.0],[1.0,2.0]]
[[vehicle]]
id = "USV1"
type = "TYPE_USV"
[[vehicle]]
id = "UUV1"
type = "TYPE_UUV"
relay_via = "USV1"
stale_ms = 10000.0
[[vehicle]]
id = "UAV1"
type = "TYPE_UAV"
[team]
host = "127.0.0.1"
[robocommand]
host = "192.168.65.2"
"""

    def _load(self, text: str):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bridge.toml"
            p.write_text(text, encoding="utf-8")
            return load(p)

    def test_minimal_config_and_topics(self):
        cfg = self._load(self.BASE)
        self.assertEqual(cfg.team_id, "ASTA")
        self.assertEqual(cfg.rc_report_topic("USV1"),
                         "robocommand/robotx/ASTA/USV1/report")
        self.assertEqual(cfg.team_report_sub(), "team/robotx/ASTA/+/report")
        self.assertEqual(cfg.rc_course_sub(), "robocommand/robotx/course")
        self.assertEqual(cfg.tiers, ("TIER_NONE",) * 4)

    def test_the_command_topic_mirrors_the_report_topic(self):
        cfg = self._load(self.BASE)
        self.assertEqual(cfg.team_command_topic("USV1"),
                         "team/robotx/ASTA/USV1/command")

    # ---- the typed fleet ---------------------------------------------------

    def test_the_fleet_loads_in_declaration_order(self):
        cfg = self._load(self.FLEET)
        self.assertEqual(cfg.vehicle_ids, ("USV1", "UUV1", "UAV1"))
        self.assertTrue(cfg.has_uav())

    def test_a_uav_may_not_leave_flight_phase_unknown(self):
        """The whole reason the fleet had to become typed.

        A global allow_unknown containing flight_phase would hand the aircraft
        the surface vehicles' exemption, and that field is relayed onward for
        Singapore's Network Remote ID.
        """
        cfg = self._load(self.FLEET)
        self.assertEqual(cfg.vehicle("UAV1").allow_unknown, ())
        self.assertIn("flight_phase", cfg.vehicle("USV1").allow_unknown)
        self.assertIn("flight_phase", cfg.vehicle("UUV1").allow_unknown)

    def test_an_unknown_vehicle_type_fails_on_the_trailer(self):
        bad = self.BASE.replace('type = "TYPE_USV"', 'type = "TYPE_BOAT"')
        with self.assertRaises(ConfigError) as cm:
            self._load(bad)
        self.assertIn("TYPE_BOAT", str(cm.exception))

    def test_type_unknown_is_not_configurable(self):
        bad = self.BASE.replace('type = "TYPE_USV"', 'type = "TYPE_UNKNOWN"')
        with self.assertRaises(ConfigError):
            self._load(bad)

    def test_staleness_is_per_vehicle_with_a_global_fallback(self):
        cfg = self._load(self.FLEET)
        self.assertEqual(cfg.stale_ms_for("UUV1"), 10000.0)   # acoustic hop
        self.assertEqual(cfg.stale_ms_for("USV1"), cfg.stale_ms)
        self.assertEqual(cfg.stale_ms_for("nobody"), cfg.stale_ms)

    def test_relay_via_must_name_a_configured_vehicle(self):
        bad = self.FLEET.replace('relay_via = "USV1"', 'relay_via = "USV9"')
        with self.assertRaises(ConfigError) as cm:
            self._load(bad)
        self.assertIn("USV9", str(cm.exception))

    def test_relay_via_may_not_point_at_itself(self):
        bad = self.FLEET.replace('relay_via = "USV1"', 'relay_via = "UUV1"')
        with self.assertRaises(ConfigError) as cm:
            self._load(bad)
        self.assertIn("itself", str(cm.exception))

    def test_a_uav_without_a_geofence_will_not_load(self):
        """Catch it here, not in a rejected RunDeclaration."""
        bad = self.FLEET.replace(
            "uav_geofence = [[1.0,2.0],[1.0,3.0],[2.0,3.0],[1.0,2.0]]", ""
        )
        with self.assertRaises(ConfigError) as cm:
            self._load(bad)
        self.assertIn("geofence", str(cm.exception))

    def test_a_fleet_with_no_uav_needs_no_geofence(self):
        cfg = self._load(self.BASE)
        self.assertFalse(cfg.has_uav())
        self.assertEqual(cfg.uav_geofence, ())

    def test_a_copy_pasted_vehicle_block_is_caught(self):
        bad = self.BASE + '\n[[vehicle]]\nid = "USV2"\ntype = "TYPE_USV"\n'
        with self.assertRaises(ConfigError) as cm:
            self._load(bad)
        self.assertIn("share a type", str(cm.exception))

    def test_a_fleet_with_no_vehicles_says_so(self):
        with self.assertRaises(ConfigError) as cm:
            self._load('team_id = "ASTA"\n[team]\nhost="x"\n[robocommand]\nhost="y"\n')
        self.assertIn("vehicle", str(cm.exception))

    def test_a_misspelled_tier_fails_on_the_trailer(self):
        with self.assertRaises(ConfigError) as cm:
            self._load(self.BASE + '\n[tiers]\ntask1 = "advance"\n')
        self.assertIn("task1", str(cm.exception))

    def test_tier_unknown_is_not_declarable(self):
        with self.assertRaises(ConfigError):
            self._load(self.BASE + '\n[tiers]\ntask1 = "TIER_UNKNOWN"\n')

    def _with_fence(self, fence: str) -> str:
        """Bare keys must go ABOVE the section headers.

        Appending to BASE would drop the key into [robocommand] rather than the
        top level -- which is exactly the trap that was live in the shipped
        bridge.toml, where uav_geofence sat under [tiers] and was ignored.
        """
        return self.BASE.replace(
            'team_id = "ASTA"',
            'team_id = "ASTA"\nuav_geofence = %s' % fence,
        )

    def test_an_open_geofence_is_rejected(self):
        with self.assertRaises(ConfigError) as cm:
            self._load(self._with_fence("[[1.0, 2.0], [1.0, 3.0]]"))
        self.assertIn("closed", str(cm.exception))

    def test_a_closed_geofence_is_accepted(self):
        cfg = self._load(self._with_fence("[[1.0,2.0],[1.0,3.0],[2.0,3.0],[1.0,2.0]]"))
        self.assertEqual(len(cfg.uav_geofence), 4)
        self.assertEqual(cfg.uav_geofence[0], cfg.uav_geofence[-1])

    def test_duplicate_vehicle_ids_are_rejected(self):
        bad = self.BASE + '\n[[vehicle]]\nid = "USV1"\ntype = "TYPE_UUV"\n'
        with self.assertRaises(ConfigError) as cm:
            self._load(bad)
        self.assertIn("unique", str(cm.exception))

    def test_missing_broker_section_names_itself(self):
        with self.assertRaises(ConfigError) as cm:
            self._load('team_id = "T"\n[[vehicle]]\nid = "USV1"\n'
                       'type = "TYPE_USV"\n[team]\nhost = "x"\n')
        self.assertIn("robocommand", str(cm.exception))


class TestMultipleRunsPerTimeslot(unittest.TestCase):
    """A timeslot holds more than one run, so ending one must not strand the OCS.

    The subtle half is not the state machine, it is the sequence numbers: two
    declarations that both carry seq=1 make a stale RunStart from the first run
    match the second, which defeats the one check standing between us and
    starting a run against somebody else's declaration.
    """

    def _ended(self) -> RunMachine:
        m = RunMachine()
        m.on_connect()
        m.on_course(object())
        m.on_declared(1)
        m.on_run_start(1, 77)
        m.on_end()
        return m

    def test_a_finished_run_may_declare_again(self):
        m = self._ended()
        self.assertEqual(m.state, RunState.ENDED)
        self.assertTrue(m.may_declare())

    def test_re_declaring_starts_a_second_run(self):
        m = self._ended()
        v = m.on_declared(2)
        self.assertTrue(v)
        self.assertEqual(m.state, RunState.DECLARED)
        self.assertEqual(m.declaration_seq, 2)
        self.assertEqual(m.runs_declared, 2)
        self.assertIn("re-declared", v.reason)

    def test_the_previous_run_id_does_not_leak_into_the_next(self):
        """DECLARED with last run's run_id still showing reads as already running."""
        m = self._ended()
        self.assertEqual(m.run_id, 77)
        m.on_declared(2)
        self.assertIsNone(m.run_id)

    def test_declaring_over_a_live_run_is_refused_with_a_way_out(self):
        m = RunMachine()
        m.on_connect()
        m.on_course(object())
        m.on_declared(1)
        m.on_run_start(1, 5)
        self.assertFalse(m.may_declare())
        v = m.on_declared(2)
        self.assertFalse(v)
        self.assertIn("end the current run first", v.reason)
        self.assertEqual(m.state, RunState.RUNNING)
        self.assertEqual(m.run_id, 5)

    def test_a_stale_run_start_cannot_start_the_second_run(self):
        """The whole reason declarations must not reuse a sequence number."""
        m = self._ended()
        m.on_declared(2)                       # second run, new seq
        stale = m.on_run_start(1, 77)          # RunStart answering run ONE
        self.assertFalse(stale)
        self.assertEqual(m.state, RunState.DECLARED)
        self.assertIsNone(m.run_id)
        good = m.on_run_start(2, 78)
        self.assertTrue(good)
        self.assertEqual(m.run_id, 78)

    def test_the_run_can_be_ended_and_redeclared_repeatedly(self):
        m = self._ended()
        for seq in (2, 3, 4):
            self.assertTrue(m.on_declared(seq))
            self.assertTrue(m.on_run_start(seq, seq * 10))
            self.assertTrue(m.on_end())
        self.assertEqual(m.runs_declared, 4)


class TestSeqStoreAcrossRuns(unittest.TestCase):
    """ensure_epoch is what keeps declaration sequence numbers unique."""

    def setUp(self):
        self.dir = tempfile.mkdtemp()
        self.path = Path(self.dir) / "seq.json"

    def test_the_request_counter_climbs_across_runs(self):
        s = SeqStore(self.path)
        s.ensure_epoch("ASTA-1")
        first = s.next_request()
        s.ensure_epoch("ASTA-2")               # second run, same timeslot
        second = s.next_request()
        self.assertEqual((first, second), (1, 2))
        self.assertNotEqual(first, second)

    def test_ensure_epoch_only_creates_once(self):
        s = SeqStore(self.path)
        self.assertTrue(s.ensure_epoch("ASTA-1"))
        self.assertFalse(s.ensure_epoch("ASTA-2"))
        self.assertEqual(s.epoch, "ASTA-1")

    def test_ensure_epoch_does_not_rewind_report_counters(self):
        s = SeqStore(self.path)
        s.ensure_epoch("ASTA-1")
        for _ in range(5):
            s.next_report("USV1")
        s.ensure_epoch("ASTA-2")
        self.assertEqual(s.next_report("USV1"), 6)

    def test_new_epoch_still_wipes_when_asked_explicitly(self):
        """The deliberate reset must still exist -- a new day, a new store."""
        s = SeqStore(self.path)
        s.ensure_epoch("ASTA-1")
        s.next_request()
        s.new_epoch("ASTA-day2")
        self.assertEqual(s.next_request(), 1)

    def test_the_climbing_counter_survives_a_restart(self):
        s = SeqStore(self.path)
        s.ensure_epoch("ASTA-1")
        s.next_request()
        del s
        again = SeqStore(self.path)
        self.assertFalse(again.ensure_epoch("ASTA-2"))
        self.assertEqual(again.next_request(), 2)


class TestActiveSet(unittest.TestCase):
    """Which vehicles go into the RunDeclaration.

    Not cosmetic: RoboCommand holds RunStart until EVERY declared vehicle
    reports STATE_AUTO, with no timeout and no error. Declaring a vehicle that
    is not on the water means the run never starts, and the symptom looks like
    a broken link to their server rather than a fleet one vehicle short.
    """

    FLEET = ("USV1", "UUV1", "UAV1")

    def test_only_vehicles_reporting_recently_are_active(self):
        seen = {"USV1": 100.0, "UAV1": 100.0}
        self.assertEqual(
            choose_active(self.FLEET, {}, seen, 101.0), ("USV1", "UAV1"))

    def test_a_vehicle_that_stopped_reporting_falls_out(self):
        seen = {"USV1": 100.0}
        self.assertEqual(choose_active(self.FLEET, {}, seen, 100.0), ("USV1",))
        self.assertEqual(
            choose_active(self.FLEET, {}, seen, 100.0 + ACTIVE_WINDOW_S + 0.1), ())

    def test_a_vehicle_never_heard_from_is_never_active(self):
        """UUV1 has no reporter. Declaring it is how a run fails to start."""
        seen = {"USV1": 100.0, "UAV1": 100.0}
        self.assertNotIn("UUV1", choose_active(self.FLEET, {}, seen, 101.0))

    def test_the_operator_can_force_a_vehicle_in(self):
        """A reporter about to come up, or a vehicle relayed by another."""
        self.assertEqual(
            choose_active(self.FLEET, {"UUV1": True}, {}, 0.0), ("UUV1",))

    def test_the_operator_can_force_a_reporting_vehicle_out(self):
        seen = {"USV1": 100.0, "UAV1": 100.0}
        self.assertEqual(
            choose_active(self.FLEET, {"USV1": False}, seen, 101.0), ("UAV1",))

    def test_config_order_is_preserved(self):
        """RunDeclaration.vehicle_ids is repeated; a stable order keeps one
        run's wire log diffable against the last."""
        seen = {v: 100.0 for v in self.FLEET}
        self.assertEqual(choose_active(self.FLEET, {}, seen, 101.0), self.FLEET)
        # Same set, opposite insertion order in the dicts.
        seen2 = {"UAV1": 100.0, "UUV1": 100.0, "USV1": 100.0}
        self.assertEqual(choose_active(self.FLEET, {}, seen2, 101.0), self.FLEET)

    def test_an_override_beats_the_telemetry_in_both_directions(self):
        seen = {"USV1": 100.0}
        both = {"USV1": False, "UUV1": True}
        self.assertEqual(choose_active(self.FLEET, both, seen, 101.0), ("UUV1",))

    def test_the_por_two_vehicle_case(self):
        """The shape the Communications PoR actually needs: USV + UAV, no UUV."""
        seen = {"USV1": 100.0, "UAV1": 100.2}
        active = choose_active(self.FLEET, {}, seen, 100.5)
        self.assertEqual(active, ("USV1", "UAV1"))
        self.assertGreaterEqual(len(active), 2)


if __name__ == "__main__":
    unittest.main()
