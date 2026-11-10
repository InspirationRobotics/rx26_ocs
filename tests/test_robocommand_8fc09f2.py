"""RoboCommand 8fc09f2 through the OCS: the UAV transit area in the course and in the
fence, the declaration's ceiling and pilot call sign, the aircraft's registration on every
heartbeat, a course that changes after declaring, the "all declared vehicles AUTO" clock,
and what became of each task report (held, dropped, published, acknowledged).

RoboCommand's own refusal rule is geofence.rc_declaration_problems, tested here against
their test server's numbers: Course Alpha and Transit A as rc_test/test_server.py publishes
them at 8fc09f2, and the fence their test_client declares by default.
"""
from __future__ import annotations

import json
import re
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import paho.mqtt.client as mqtt                                  # noqa: E402

from rx_bridge import geofence                                   # noqa: E402
from rx_bridge.config import ConfigError, load                   # noqa: E402

from tests.harness import (CEILING_HAE_M, BridgeCase, course_bytes, heartbeat,   # noqa: E402
                           readiness)
from tests.test_fence_gate import ELSEWHERE, FenceCase           # noqa: E402

#: rc_test/test_server.py at 8fc09f2: Course Alpha and Transit A beside it, closed rings.
ALPHA = [(1.2809652, 103.8548424), (1.2808457, 103.8550331), (1.2810752, 103.8551770),
         (1.2811947, 103.8549863), (1.2809652, 103.8548424)]
TRANSIT_A = [(1.281294119, 103.855048646), (1.281174598, 103.855239379),
             (1.2810752, 103.8551770), (1.2811947, 103.8549863), (1.281294119, 103.855048646)]
#: rc_test/test_client.py's default --geofence-lats/--geofence-lngs: course + transit area.
THEIR_FENCE = list(zip(
    [1.2809652, 1.2808457, 1.2810752, 1.281174598, 1.281294119, 1.2811947, 1.2809652],
    [103.8548424, 103.8550331, 103.8551770, 103.855239379, 103.855048646, 103.8549863,
     103.8548424]))
NORTH_10M = 10.0 / 111320.0             # degrees of latitude in ten metres


# ---- the fence for a course with a transit area -----------------------------------------

class TestFenceWithTransit(unittest.TestCase):
    def test_course_and_transit_area_become_one_ring_robocommand_accepts(self):
        f = geofence.for_course("ALPHA", ALPHA, 2.0, TRANSIT_A)
        self.assertIsNotNone(f.polygon, f.reason)
        self.assertIn("transit", f.source)
        # One convex rectangle: the seam vertices sit ~4 mm off the line and must go.
        self.assertEqual(f.vertices, 4)
        # It reaches into the transit area, north of the course's northernmost corner.
        self.assertGreater(max(p[0] for p in f.polygon), max(p[0] for p in ALPHA))
        self.assertEqual(geofence.rc_declaration_problems(f.polygon, ALPHA, TRANSIT_A, 40.0), [])

    def test_a_transit_area_that_shares_no_edge_leaves_a_course_only_fence_and_says_so(self):
        apart = [(lat + 0.001, lon) for lat, lon in TRANSIT_A]
        f = geofence.for_course("ALPHA", ALPHA, 2.0, apart)
        self.assertIsNotNone(f.polygon)
        self.assertIn("NOT included", f.source)
        self.assertLessEqual(max(p[0] for p in f.polygon), max(p[0] for p in ALPHA))

    def test_no_transit_area_is_the_course_alone(self):
        f = geofence.for_course("ALPHA", ALPHA, 2.0)
        self.assertIsNotNone(f.polygon)
        self.assertNotIn("transit", f.source)


class TestRoboCommandsRule(unittest.TestCase):
    """geofence.rc_declaration_problems against their test server's own cases."""

    def test_their_default_fence_and_ceiling_pass(self):
        self.assertEqual(geofence.rc_declaration_problems(THEIR_FENCE, ALPHA, TRANSIT_A, 40.0), [])

    def test_a_fence_that_leaves_the_area_is_refused_with_how_far_and_where(self):
        fence = list(THEIR_FENCE)
        fence[4] = (fence[4][0] + NORTH_10M, fence[4][1])        # the transit area's far corner
        problems = geofence.rc_declaration_problems(fence, ALPHA, TRANSIT_A, 40.0)
        self.assertEqual(len(problems), 1, problems)
        m = re.match(r"uav_geofence leaves the course and transit area by (\d+\.\d) m at point 5$",
                     problems[0])
        self.assertTrue(m, problems[0])
        self.assertAlmostEqual(float(m.group(1)), 10.0, delta=1.5)

    def test_without_the_transit_area_the_same_fence_is_outside(self):
        problems = geofence.rc_declaration_problems(THEIR_FENCE, ALPHA, (), 40.0)
        self.assertTrue(problems and "leaves the course and transit area" in problems[0])

    def test_too_few_points(self):
        problems = geofence.rc_declaration_problems(THEIR_FENCE[:2], ALPHA, TRANSIT_A, 40.0)
        self.assertEqual(problems, ["uav_geofence has 3 point(s), needs at least 4"])

    def test_the_ceiling(self):
        judge = lambda c, **kw: geofence.rc_declaration_problems(THEIR_FENCE, ALPHA, TRANSIT_A, c, **kw)
        self.assertEqual(judge(None), ["uav_geofence_max_altitude_hae_m is not set"])
        self.assertEqual(judge(0.0), ["uav_geofence_max_altitude_hae_m is 0: it must be above 0 "
                                      "and at most 50"])
        self.assertEqual(judge(50.0), [])
        self.assertEqual(len(judge(50.5)), 1)
        self.assertEqual(judge(55.0, max_ceiling_m=60.0), [])

    def test_with_no_fence_to_declare_only_the_ceiling_is_judged(self):
        self.assertEqual(geofence.rc_declaration_problems(None, ALPHA, TRANSIT_A, 40.0), [])
        self.assertEqual(len(geofence.rc_declaration_problems(None, ALPHA, TRANSIT_A, None)), 1)


# ---- bridge.toml ------------------------------------------------------------------------

MINIMAL = """
team_id = "ASTA"
[[vehicle]]
id = "USV1"
type = "TYPE_USV"
{usv}
[[vehicle]]
id = "UAV1"
type = "TYPE_UAV"
{uav}
{uav_section}
[team]
host = "127.0.0.1"
[robocommand]
host = "127.0.0.1"
{geofence}
"""


class TestConfig(unittest.TestCase):
    def load(self, usv="", uav="", uav_section="", geofence=""):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "bridge.toml"
            path.write_text(MINIMAL.format(usv=usv, uav=uav, uav_section=uav_section,
                                           geofence=geofence), encoding="utf-8")
            return load(path)

    def test_the_limit_is_theirs_and_the_call_sign_starts_blank(self):
        cfg = self.load()
        self.assertEqual(cfg.geofence_ceiling_limit_hae_m, 50.0)
        self.assertEqual(cfg.uav_pilot_callsign, "")

    def test_the_ceiling_is_the_aircrafts_not_the_configs(self):
        with self.assertRaises(ConfigError) as cm:
            self.load(geofence="[geofence]\nmax_altitude_hae_m = 40.0")
        self.assertIn("FENCE_ALT_MAX", str(cm.exception))

    def test_the_limit_can_be_raised(self):
        cfg = self.load(geofence="[geofence]\nceiling_limit_hae_m = 60.0")
        self.assertEqual(cfg.geofence_ceiling_limit_hae_m, 60.0)

    def test_registration_is_for_aircraft_only(self):
        self.assertEqual(self.load(uav='registration = " 53551 "').vehicle("UAV1").registration,
                         "53551")
        with self.assertRaises(ConfigError):
            self.load(usv='registration = "53551"')

    def test_the_pilot_call_sign_default(self):
        cfg = self.load(uav_section='[uav]\npilot_callsign = " Chase UA0000 "')
        self.assertEqual(cfg.uav_pilot_callsign, "Chase UA0000")
        with self.assertRaises(ConfigError):
            self.load(uav_section="[uav]\npilot_callsign = 7")


# ---- what a declaration carries -----------------------------------------------------------

class TestDeclarationFields(FenceCase):
    def test_an_aircraft_declaration_carries_the_ceiling_and_the_call_sign(self):
        self.declared()
        d = self.sent_declaration()
        self.assertEqual(d["uav_geofence_max_altitude_hae_m"], CEILING_HAE_M)
        self.assertEqual(d["uav_pilot_callsign"], "Test UA0000")

    def test_the_pages_box_wins_over_the_default(self):
        self.course()
        self.only("USV1", "UAV1")
        self.confirm_fence()
        self.assertIn("declared", self.bridge.declare("  Other UA1 "))
        self.assertEqual(self.sent_declaration()["uav_pilot_callsign"], "Other UA1")

    def test_a_blank_call_sign_is_refused_and_nothing_is_sent(self):
        self.course()
        self.only("USV1", "UAV1")
        self.confirm_fence()
        self.assertIn("call sign is blank", self.bridge.declare("   "))
        self.assertEqual(self.requests_sent(), 0)

    def test_a_surface_only_declaration_carries_neither(self):
        self.declared(active=("USV1",))
        d = self.sent_declaration()
        self.assertNotIn("uav_geofence_max_altitude_hae_m", d)
        self.assertNotIn("uav_pilot_callsign", d)

    def test_the_snapshot_says_what_the_page_needs(self):
        self.course()
        self.only("USV1", "UAV1")
        s = self.snap()
        u = s["uav"]
        self.assertEqual((u["in_declaration"], u["ceiling_limit_hae_m"]), (True, 50.0))
        self.assertIsNone(u["declaration"]["UAV1"]["ceiling_hae_m"], "nothing read yet")
        self.confirm_fence()
        self.assertEqual(self.snap()["uav"]["declaration"]["UAV1"]["ceiling_hae_m"], CEILING_HAE_M)
        self.assertEqual(s["operator"]["callsign"], {"value": "Test UA0000", "from": "config",
                                                     "saved": None, "config": "Test UA0000"})
        self.assertEqual(s["operator"]["registration"],
                         {"UAV1": {"value": "", "from": None, "saved": None, "config": ""}})


class TestNoCallSignConfigured(FenceCase):
    callsign = ""

    def test_the_button_verdict_includes_the_call_sign(self):
        self.course()
        self.only("USV1", "UAV1")
        self.confirm_fence()
        self.assertIn("call sign is blank", self.snap()["run"]["declare_blocker"])
        self.assertIn("call sign is blank", self.bridge.declare())
        self.assertEqual(self.requests_sent(), 0)
        self.assertTrue(self.bridge.set_operator(callsign="Chase UA0001")[0])
        self.assertIsNone(self.snap()["run"]["declare_blocker"])
        self.assertIn("declared", self.bridge.declare())
        self.assertEqual(self.sent_declaration()["uav_pilot_callsign"], "Chase UA0001")


class TestTransitCourseThroughTheBridge(FenceCase):
    def test_the_fence_joins_the_transit_area_and_robocommands_rule_passes(self):
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes("ALPHA", corners=ALPHA, transit=TRANSIT_A))
        s = self.snap()
        self.assertEqual(len(s["course"]["uav_transit_area"]), 5)
        self.assertIn("transit", s["uav_geofence_source"])
        self.only("USV1", "UAV1")
        # The pilot uses the whole course and transit area: the fence their test_client declares.
        self.confirm_fence(held=[self.rc_open(), THEIR_FENCE[:-1]])
        self.assertIn("declared", self.bridge.declare())
        d = self.sent_declaration()
        fence = [(p["latitude"], p["longitude"]) for p in d["uav_geofence"]]
        self.assertEqual(fence, THEIR_FENCE)
        self.assertEqual(geofence.rc_declaration_problems(
            fence, ALPHA, TRANSIT_A, d["uav_geofence_max_altitude_hae_m"]), [])


# ---- the aircraft's registration ------------------------------------------------------

def uav_hb(registration: str | None = None) -> dict:
    hb = heartbeat("UAV1")
    if registration is not None:
        hb["heartbeat"]["uav_registration"] = registration
    return hb


class TestRegistration(BridgeCase):
    uav_extra = 'registration = "53551"'

    def test_every_aircraft_heartbeat_carries_it(self):
        self.declared()
        self.feed(uav_hb())
        self.assertEqual(self.reports_to_rc("UAV1")[-1].heartbeat.uav_registration, "53551")

    def test_the_same_one_from_the_vehicle_passes(self):
        self.declared()
        self.feed(uav_hb("53551"))
        self.assertEqual(len(self.reports_to_rc("UAV1")), 1)

    def test_a_different_one_is_dropped_not_published(self):
        self.declared()
        self.feed(uav_hb("99999"))
        self.assertEqual(self.reports_to_rc("UAV1"), [])
        self.assertEqual(self.bridge.counters.dropped_misregistered, 1)
        self.assertTrue(any("sent uav_registration '99999' but the OCS has '53551'" in line
                            for line in self.logs), self.logs)

    def test_one_typed_on_the_page_wins_over_bridge_toml(self):
        self.assertTrue(self.bridge.set_operator(registration={"UAV1": "ASTA-1"})[0])
        self.declared()
        self.feed(uav_hb())
        self.assertEqual(self.reports_to_rc("UAV1")[-1].heartbeat.uav_registration, "ASTA-1")
        self.feed(uav_hb("53551"), key="again")
        self.assertEqual(self.bridge.counters.dropped_misregistered, 1,
                         "bridge.toml's is no longer the OCS's")

    def test_it_cannot_change_during_a_run(self):
        self.declared()
        self.bridge.machine.on_run_start(self.bridge.machine.declaration_seq, 7)
        self.assertEqual(self.bridge.machine.state.name, "RUNNING")
        ok, why = self.bridge.set_operator(registration={"UAV1": "ASTA-2"})
        self.assertFalse(ok)
        self.assertIn("cannot change during a run", why)
        self.assertEqual(self.bridge.operator.registration("UAV1"), "53551")
        self.assertTrue(self.bridge.set_operator(registration={"UAV1": "53551"})[0],
                        "the same one again is not a change")
        self.assertTrue(self.bridge.set_operator(callsign="Chase UA1")[0],
                        "the call sign is only in the declaration: it may change")


class TestNoRegistration(BridgeCase):
    def test_blank_stays_blank(self):
        self.declared()
        self.feed(uav_hb())
        self.assertEqual(self.reports_to_rc("UAV1")[-1].heartbeat.uav_registration, "")

    def test_one_the_config_does_not_know_is_dropped(self):
        self.declared()
        self.feed(uav_hb("53551"))
        self.assertEqual(self.reports_to_rc("UAV1"), [])
        self.assertEqual(self.bridge.counters.dropped_misregistered, 1)


# ---- what became of each task report --------------------------------------------------

class TestTaskReports(BridgeCase):
    def reports(self) -> list[dict]:
        return json.loads(json.dumps(self.bridge.snapshot()))["task_reports"]

    def test_before_declaring_it_is_held(self):
        self.feed(readiness("USV1", command_seq=3))
        r = self.reports()
        self.assertEqual([(e["vehicle"], e["kind"], e["verdict"]) for e in r],
                         [("USV1", "readiness", "held")])

    def test_published_then_acknowledged_by_the_broker(self):
        self.declared()
        self.feed(readiness("USV1", command_seq=3, seq=7))
        r = self.reports()[0]
        self.assertEqual((r["verdict"], r["vehicle_seq"]), ("published", 7))
        self.bridge._on_rc_publish(None, None, self.bridge.rc.mids[-1])
        r = self.reports()[0]
        self.assertEqual(r["verdict"], "acknowledged")
        self.assertIn("seq=%d" % r["seq"], r["detail"])

    def test_an_ack_that_beats_the_filing_is_not_lost(self):
        self.declared()
        self.bridge._on_rc_publish(None, None, len(self.bridge.rc.mids) + 1)
        self.feed(readiness("USV1", command_seq=3))
        self.assertEqual(self.reports()[0]["verdict"], "acknowledged")

    def test_with_the_link_down_it_says_queued(self):
        self.declared()
        self.bridge.rc.publish = lambda *a, **k: SimpleNamespace(mid=500, rc=mqtt.MQTT_ERR_NO_CONN)
        self.feed(readiness("USV1", command_seq=3))
        r = self.reports()[0]
        self.assertEqual(r["verdict"], "published")
        self.assertIn("queued: no RoboCommand link", r["detail"])

    def test_heartbeats_are_never_listed(self):
        self.declared()
        self.feed(heartbeat("USV1"))
        self.assertEqual(self.reports(), [])


# ---- a course that changes after declaring, and the all-AUTO clock ---------------------

class TestCourseChange(FenceCase):
    def test_a_new_course_after_declaring_raises_the_banner(self):
        self.declared()
        seq = self.bridge.machine.declaration_seq
        self.bridge._handle_course(course_bytes("ALPHA", corners=ELSEWHERE))
        self.assertEqual(self.snap()["run"]["course_changed_since"], seq)
        self.assertTrue(any("COURSE CHANGED" in str(line) for line in self.logs))

    def test_the_same_course_again_is_not_a_change(self):
        self.declared()
        self.bridge._handle_course(course_bytes())
        self.assertIsNone(self.snap()["run"]["course_changed_since"])

    def test_ending_the_run_clears_it(self):
        self.declared()
        self.bridge._handle_course(course_bytes("ALPHA", corners=ELSEWHERE))
        self.bridge.end()
        self.assertIsNone(self.snap()["run"]["course_changed_since"])


class TestAllAuto(BridgeCase):
    def test_an_undeclared_vehicle_does_not_hold_all_auto(self):
        self.declared(active=("USV1", "UAV1"))           # UUV1 configured, not declared
        self.assertIsNone(self.bridge.snapshot()["run"]["all_auto_since"])
        self.feed(heartbeat("USV1"))
        self.feed(heartbeat("UAV1"))
        self.assertEqual(self.bridge.awaiting_auto(), [])
        self.assertIsNotNone(self.bridge.snapshot()["run"]["all_auto_since"])

    def test_one_vehicle_dropping_out_of_auto_stops_the_clock(self):
        self.declared(active=("USV1", "UAV1"))
        self.feed(heartbeat("USV1"))
        self.feed(heartbeat("UAV1"))
        self.feed(heartbeat("USV1", state="STATE_MANUAL"))
        self.assertIsNone(self.bridge.snapshot()["run"]["all_auto_since"])
        self.assertEqual(self.bridge.awaiting_auto(), ["USV1"])

    def test_a_task_report_between_heartbeats_does_not_stop_the_clock(self):
        # Found on the sim box: a DockingReport left USV1's last frame without a
        # heartbeat, so the next UAV1 heartbeat logged "no longer all AUTO".
        self.declared(active=("USV1", "UAV1"))
        self.feed(heartbeat("USV1"))
        self.feed(heartbeat("UAV1"))
        since = self.bridge.snapshot()["run"]["all_auto_since"]
        self.feed(readiness("USV1", command_seq=3))
        self.feed(heartbeat("UAV1"))
        self.assertEqual(self.bridge.awaiting_auto(), [])
        self.assertEqual(self.bridge.snapshot()["run"]["all_auto_since"], since)


if __name__ == "__main__":
    unittest.main()
