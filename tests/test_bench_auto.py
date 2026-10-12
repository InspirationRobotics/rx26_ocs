"""The bench dry-run switch (--bench-auto): STATE_MANUAL is published as STATE_AUTO.

The one place the OCS publishes a value no vehicle sent, so the tests are mostly
about the edges of the exception: only the listed vehicles, only MANUAL, never
KILLED, nothing but the state field, nothing when the flag is absent -- and that
it is counted, wire-logged and announced.
"""
from __future__ import annotations

import contextlib
import io
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ocs_web import server                                       # noqa: E402
from rx_bridge import benchauto                                  # noqa: E402
from rx_bridge.bridge import Bridge                              # noqa: E402
from rx_bridge.governor import Governor                          # noqa: E402
from rx_bridge.proto import common_pb2, rx_reports_pb2          # noqa: E402

from tests.harness import BridgeCase, FakeMqtt, heartbeat        # noqa: E402

FLEET = ("USV1", "UUV1", "UAV1")
STATE = common_pb2.RobotState


def report(vid: str, state: str, *, body: str = "heartbeat") -> rx_reports_pb2.RxReport:
    r = rx_reports_pb2.RxReport()
    r.vehicle_id = vid
    if body == "heartbeat":
        r.heartbeat.state = STATE.Value(state)
        r.heartbeat.spd_mps = 1.5
    else:
        r.readiness.command_seq = 3
    return r


class TestResolve(unittest.TestCase):

    def test_names_become_ids_in_order_without_duplicates(self):
        self.assertEqual(benchauto.resolve("UAV1, USV1,,UAV1", FLEET), ("UAV1", "USV1"))
        self.assertEqual(benchauto.resolve(["USV1"], FLEET), ("USV1",))

    def test_an_unknown_id_is_an_error_naming_it_and_the_fleet(self):
        with self.assertRaises(ValueError) as cm:
            benchauto.resolve("USV1,NOPE", FLEET)
        self.assertIn("NOPE", str(cm.exception))
        self.assertIn("UUV1", str(cm.exception))

    def test_naming_nobody_is_an_error(self):
        for spec in ("", " , ", []):
            with self.assertRaises(ValueError):
                benchauto.resolve(spec, FLEET)


class TestPromote(unittest.TestCase):

    def test_a_listed_vehicles_manual_heartbeat_becomes_auto_and_nothing_else_changes(self):
        r = report("USV1", "STATE_MANUAL")
        before = rx_reports_pb2.RxReport()
        before.CopyFrom(r)
        self.assertEqual(benchauto.promote(r, ("USV1",)), "STATE_MANUAL")
        self.assertEqual(r.heartbeat.state, STATE.Value("STATE_AUTO"))
        r.heartbeat.state = before.heartbeat.state
        self.assertEqual(r, before, "only heartbeat.state may differ")

    def test_every_other_state_passes_through(self):
        for state in ("STATE_KILLED", "STATE_AUTO", "STATE_UNKNOWN"):
            r = report("USV1", state)
            self.assertIsNone(benchauto.promote(r, ("USV1",)), state)
            self.assertEqual(r.heartbeat.state, STATE.Value(state))

    def test_a_vehicle_not_listed_is_untouched(self):
        r = report("UAV1", "STATE_MANUAL")
        self.assertIsNone(benchauto.promote(r, ("USV1",)))
        self.assertEqual(r.heartbeat.state, STATE.Value("STATE_MANUAL"))

    def test_a_report_that_is_not_a_heartbeat_is_untouched(self):
        r = report("USV1", "", body="readiness")
        self.assertIsNone(benchauto.promote(r, ("USV1",)))
        self.assertEqual(r.WhichOneof("body"), "readiness")


class StateCase(BridgeCase):
    limits = "rate = 1000.0"        # these tests are not about the governor

    def published_state(self, vid: str) -> list[str]:
        """The heartbeat states the bridge published to RoboCommand for `vid`."""
        return [STATE.Name(r.heartbeat.state)
                for r in self.reports_to_rc(vid) if r.WhichOneof("body") == "heartbeat"]


class BenchCase(StateCase):
    """A declared run, USV1 + UAV1, with bench auto on for USV1 only unless set."""

    bench_auto = ("USV1",)

    def setUp(self):
        super().setUp()
        self.declared()


class TestBenchAutoInTheBridge(BenchCase):

    def test_only_the_listed_vehicle_is_rewritten(self):
        self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.feed(heartbeat("UAV1", state="STATE_MANUAL"), "radio")
        self.assertEqual(self.published_state("USV1"), ["STATE_AUTO"])
        self.assertEqual(self.published_state("UAV1"), ["STATE_MANUAL"])

    def test_killed_is_never_touched_and_auto_is_not_counted(self):
        self.feed(heartbeat("USV1", state="STATE_KILLED"))
        self.feed(heartbeat("USV1", state="STATE_AUTO"))
        self.assertEqual(self.published_state("USV1"), ["STATE_KILLED", "STATE_AUTO"])
        self.assertEqual(self.bridge.counters.bench_auto_substituted, {})

    def test_nothing_but_the_state_changes_on_the_published_frame(self):
        sent = heartbeat("USV1", state="STATE_MANUAL")
        self.feed(sent)
        got = self.reports_to_rc("USV1")[-1].heartbeat
        self.assertEqual(got.spd_mps, 1.0)
        self.assertEqual(got.heading_deg, 90.0)
        self.assertEqual(got.position.latitude, sent["heartbeat"]["position"]["latitude"])
        self.assertEqual(STATE.Name(got.state), "STATE_AUTO")

    def test_it_is_silent_before_the_declaration_and_the_page_keeps_the_truth(self):
        b = self.bridge
        b.machine.on_end()                      # DECLARED -> ENDED: no reports flow
        b.rc.published.clear()
        self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.assertEqual(self.reports_to_rc(), [])
        self.assertEqual(b.counters.bench_auto_substituted, {})
        self.assertEqual(b.latest["USV1"]["heartbeat"]["state"], "STATE_MANUAL")

    def test_validation_still_runs_on_the_rewritten_frame(self):
        bad = heartbeat("USV1", state="STATE_MANUAL")
        bad["heartbeat"]["heading_deg"] = "NaN"
        self.feed(bad)
        self.assertEqual(self.reports_to_rc("USV1"), [])
        self.assertEqual(self.bridge.counters.dropped_invalid, 1)
        self.assertEqual(self.bridge.counters.bench_auto_substituted, {},
                         "a refused frame was never published as AUTO")
        self.assertEqual(self.bridge.latest["USV1"]["heartbeat"]["state"], "STATE_MANUAL")

    def test_counters_and_the_snapshot_field(self):
        for _ in range(3):
            self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.feed(heartbeat("UAV1", state="STATE_MANUAL"), "radio")
        self.assertEqual(self.bridge.snapshot()["bench_auto"],
                         {"vehicles": ["USV1"], "substituted": {"USV1": 3}})

    def test_the_snapshot_lists_a_listed_vehicle_at_zero_before_its_first_frame(self):
        self.assertEqual(self.bridge.snapshot()["bench_auto"],
                         {"vehicles": ["USV1"], "substituted": {"USV1": 0}})

    def test_the_page_and_awaiting_auto_see_what_was_published(self):
        self.assertIn("USV1", self.bridge.awaiting_auto())
        self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.assertNotIn("USV1", self.bridge.awaiting_auto())
        self.assertEqual(self.bridge.snapshot()["vehicles"][0]["report"]["heartbeat"]["state"],
                         "STATE_AUTO")

    def test_the_wire_log_keeps_both_states(self):
        self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.feed(heartbeat("UAV1", state="STATE_MANUAL"), "radio")
        lines = self.wire_lines()

        def of(direction, vid):
            return [l for l in lines if l["dir"] == direction
                    and isinstance(l["decoded"], dict) and l["decoded"].get("vehicle_id") == vid]

        self.assertEqual(of("veh", "USV1")[-1]["decoded"]["heartbeat"]["state"], "STATE_MANUAL",
                         "what the vehicle sent is kept verbatim")
        tx = of("tx", "USV1")[-1]
        self.assertEqual(tx["decoded"]["heartbeat"]["state"], "STATE_AUTO")
        self.assertIn("BENCH-AUTO STATE_MANUAL -> STATE_AUTO", tx["note"])
        self.assertNotIn("BENCH-AUTO", of("tx", "UAV1")[-1]["note"])
        events = [l for l in lines if l.get("kind") == "bench_auto"]
        self.assertEqual(len(events), 1, "one event for the first frame, not one per frame")
        self.assertIn("USV1", events[0]["detail"])

    def test_the_first_substitution_is_said_on_the_console_once(self):
        for _ in range(3):
            self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.assertEqual(len([m for m in self.logs if "BENCH AUTO: USV1 now" in m]), 1)

    def test_only_what_was_published_is_counted(self):
        # Back-to-back frames overrun the governor, which runs BEFORE the rewrite.
        self.bridge._gov["USV1"] = Governor(rate=5.0, reserve=2.0)
        for _ in range(12):
            self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        published = len(self.reports_to_rc("USV1"))
        self.assertLess(published, 12, "the governor dropped some")
        self.assertEqual(self.bridge.counters.bench_auto_substituted["USV1"], published)


class TestAnnounced(BenchCase):

    bench_auto = ("USV1", "UAV1")

    def start(self):
        """Bridge.start() with the sockets stubbed -- only the announcement is wanted."""
        class Client(FakeMqtt):
            def connect_async(self, *a, **k):
                pass

            def loop_start(self):
                pass

        self.bridge.rc, self.bridge.team = Client(), Client()
        self.bridge.start_vehicle_link = lambda: None
        self.bridge.start()

    def test_start_warns_loudly_and_marks_the_evidence_log(self):
        self.start()
        warn = [m for m in self.logs if m.startswith("WARNING: BENCH AUTO")]
        self.assertEqual(len(warn), 1)
        self.assertIn("USV1, UAV1", warn[0])
        self.assertIn("never at competition", warn[0])
        head = [l for l in self.wire_lines() if l.get("kind") == "bench_auto"]
        self.assertIn("ENABLED for USV1, UAV1", head[0]["detail"])

    def test_the_console_status_names_it(self):
        self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.assertIn("BENCH AUTO USV1, UAV1 published as AUTO while MANUAL (USV1=1, UAV1=0)",
                      self.bridge.status())


class TestAbsentFlag(StateCase):

    def test_no_flag_means_no_change_and_an_empty_snapshot_field(self):
        self.declared()
        self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.assertEqual(self.published_state("USV1"), ["STATE_MANUAL"])
        self.assertEqual(self.bridge.snapshot()["bench_auto"], {"vehicles": [], "substituted": {}})
        self.bridge._announce_bench_auto()
        self.assertEqual([m for m in self.logs if "BENCH AUTO" in m], [])
        self.assertNotIn("BENCH AUTO", self.bridge.status())


class TestUnknownVehicleRejected(BridgeCase):

    def test_the_bridge_refuses_an_id_not_in_the_fleet(self):
        with self.assertRaises(ValueError):
            Bridge(self.bridge.cfg, log=lambda m: None, bench_auto=("USV1", "NOPE"))

    def test_ocs_web_exits_2_at_startup_before_starting_anything(self):
        cfg = Path(self._tmp.name) / "bridge.toml"
        err = io.StringIO()
        with contextlib.redirect_stderr(err):
            rc = server.main(["--config", str(cfg), "--bench-auto", "USV1,NOPE"])
        self.assertEqual(rc, 2)
        self.assertIn("NOPE", err.getvalue())


if __name__ == "__main__":
    unittest.main()
