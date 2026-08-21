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
team_id = "INSPIRATION"
vehicle_ids = ["USV1"]
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
        self.assertEqual(cfg.team_id, "INSPIRATION")
        self.assertEqual(cfg.rc_report_topic("USV1"),
                         "robocommand/robotx/INSPIRATION/USV1/report")
        self.assertEqual(cfg.team_report_sub(), "team/robotx/INSPIRATION/+/report")
        self.assertEqual(cfg.rc_course_sub(), "robocommand/robotx/course")
        self.assertEqual(cfg.tiers, ("TIER_NONE",) * 4)

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
            'vehicle_ids = ["USV1"]',
            'vehicle_ids = ["USV1"]\nuav_geofence = %s' % fence,
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
        bad = self.BASE.replace('["USV1"]', '["USV1", "USV1"]')
        with self.assertRaises(ConfigError):
            self._load(bad)

    def test_missing_broker_section_names_itself(self):
        with self.assertRaises(ConfigError) as cm:
            self._load('team_id = "T"\nvehicle_ids = ["USV1"]\n[team]\nhost = "x"\n')
        self.assertIn("robocommand", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
