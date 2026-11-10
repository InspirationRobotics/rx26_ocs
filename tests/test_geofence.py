"""geofence.py and its `[geofence]` config: RoboCommand's polygon for one course, the
`ocs_fence` frame, and what a declaration with an aircraft carries -- read off its
autopilot -- or why it cannot be declared. Pure -- no bridge, no socket.

The geometry is checked against an INDEPENDENT calculation (a spherical local frame,
cross products) rather than against the module's own, so a mistake in the WGS84 frame
or the inset cannot agree with itself. The Bridge-level behaviour -- the page snapshot,
the ocs_info frames, the declare gate -- is tests/test_fence_gate.py.
"""
from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge import geofence                                   # noqa: E402
from rx_bridge.config import ConfigError, load                   # noqa: E402

from tests.harness import (CEILING_HAE_M, PARAMS, PILOT, ROBONATION_TEST_COURSE,   # noqa: E402
                           fence_frame, poly_shape)

R = 6371000.0           # an independent frame: spherical, not the module's WGS84 one


def metres(p, origin):
    return (math.radians(p[1] - origin[1]) * R * math.cos(math.radians(origin[0])),
            math.radians(p[0] - origin[0]) * R)


def signed_area(ring):
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(ring, ring[1:] + ring[:1]))


def left_distance(p, a, b):
    """Metres of `p` to the LEFT of the directed line a->b (negative = right)."""
    (px, py), (ax, ay), (bx, by) = p, a, b
    return ((bx - ax) * (py - ay) - (by - ay) * (px - ax)) / math.hypot(bx - ax, by - ay)


def local_course(xy, lat0=1.28, lon0=103.85):
    """A boundary drawn in metres east/north of a point, as lat/lon -- so a test can say
    "a 100 m by 3 m course" and mean it."""
    frame = geofence._Frame(lat0, lon0)
    return [frame.ll(x, y) for x, y in xy]


class TestInset(unittest.TestCase):

    def test_a_rotated_rectangle_is_pulled_in_two_metres_on_every_edge(self):
        """RoboNation's own test course. Every inset vertex inside, every inset edge
        parallel to its original and 2 m inside it -- measured in a frame this
        module did not build."""
        fence = geofence.for_course("ALPHA", ROBONATION_TEST_COURSE, 2.0)
        self.assertIsNone(fence.reason)
        self.assertEqual(fence.source, "course boundary inset 2.0 m")
        self.assertEqual(fence.vertices, 4)
        self.assertEqual(fence.polygon[0], fence.polygon[-1], "closed")

        origin = ROBONATION_TEST_COURSE[0]
        orig = [metres(p, origin) for p in ROBONATION_TEST_COURSE[:-1]]
        ins = [metres(p, origin) for p in fence.open_polygon]
        side = 1.0 if signed_area(orig) > 0 else -1.0          # which side is inside
        for i in range(4):
            a, b = orig[i], orig[(i + 1) % 4]
            for p in (ins[i], ins[(i + 1) % 4]):               # both ends of inset edge i
                self.assertAlmostEqual(side * left_distance(p, a, b), 2.0, delta=0.05,
                                       msg="edge %d" % i)
        for p in ins:                                          # strictly inside, all four
            for i in range(4):
                self.assertGreater(side * left_distance(p, orig[i], orig[(i + 1) % 4]), 1.9)
        self.assertGreater(side * signed_area(ins), 0, "same winding as the boundary")

    def test_clockwise_and_counterclockwise_input_give_the_same_polygon(self):
        ring = ROBONATION_TEST_COURSE[:-1]
        ccw, cw = (ring, ring[::-1]) if signed_area(
            [metres(p, ring[0]) for p in ring]) > 0 else (ring[::-1], ring)
        a = geofence.for_course("X", ccw, 2.0)
        b = geofence.for_course("X", cw, 2.0)
        self.assertIsNone(a.reason)
        self.assertIsNone(b.reason)
        self.assertEqual(set(a.open_polygon), set(b.open_polygon))
        # Each keeps the winding it was given: vertex i is the inset of input vertex i.
        wa = signed_area([metres(p, ring[0]) for p in a.open_polygon])
        wb = signed_area([metres(p, ring[0]) for p in b.open_polygon])
        self.assertGreater(wa, 0)
        self.assertLess(wb, 0)

    def test_a_closed_ring_and_an_open_one_are_the_same_boundary(self):
        closed = geofence.for_course("X", ROBONATION_TEST_COURSE, 2.0)
        opened = geofence.for_course("X", ROBONATION_TEST_COURSE[:-1], 2.0)
        self.assertEqual(closed.polygon, opened.polygon)

    def test_a_vertex_in_the_middle_of_an_edge_is_not_a_corner(self):
        fence = geofence.for_course(
            "X", local_course([(0, 0), (50, 0), (100, 0), (100, 60), (0, 60)]), 2.0)
        self.assertIsNone(fence.reason)
        self.assertEqual(fence.vertices, 4)

    def test_a_triangle_works(self):
        fence = geofence.for_course("X", local_course([(0, 0), (100, 0), (50, 80)]), 2.0)
        self.assertIsNone(fence.reason)
        self.assertEqual(fence.vertices, 3)

    def test_a_boundary_that_is_not_convex_is_refused_and_says_what_to_do(self):
        ell = local_course([(0, 0), (100, 0), (100, 40), (40, 40), (40, 100), (0, 100)])
        fence = geofence.for_course("ALPHA", ell, 2.0)
        self.assertIsNone(fence.polygon)
        self.assertIn("not convex", fence.reason)
        self.assertIn("draw Ekko's fence in QGC against the course on /map", fence.reason)

    def test_a_bow_tie_and_a_pentagram_are_refused(self):
        bow = geofence.for_course("X", local_course([(0, 0), (100, 100), (100, 0), (0, 100)]), 2.0)
        self.assertIsNone(bow.polygon)
        self.assertIn("not convex", bow.reason)
        star = geofence.for_course("X", local_course(
            [(50 * math.cos(math.radians(90 + 144 * k)), 50 * math.sin(math.radians(90 + 144 * k)))
             for k in range(5)]), 2.0)
        self.assertIsNone(star.polygon)
        self.assertIn("not a simple polygon", star.reason)

    def test_a_margin_that_collapses_the_course_is_refused(self):
        thin = geofence.for_course("X", local_course([(0, 0), (100, 0), (100, 3), (0, 3)]), 2.0)
        self.assertIsNone(thin.polygon)
        self.assertIn("margin of 2.0 m collapses a course 3.0 m wide", thin.reason)
        self.assertIn("boundary_margin_m", thin.reason)

    def test_a_margin_that_makes_a_short_edge_vanish_is_refused_too(self):
        """A trapezoid 50 m wide passes the width test at 22 m, but its 20 m top edge
        has already inverted -- the polygon it would give is not the inset."""
        trap = local_course([(0, 0), (100, 0), (60, 50), (40, 50)])
        self.assertIsNone(geofence.for_course("X", trap, 20.0).reason)
        gone = geofence.for_course("X", trap, 22.0)
        self.assertIsNone(gone.polygon)
        self.assertIn("collapses", gone.reason)

    def test_fewer_than_three_distinct_corners_is_refused(self):
        for corners in ([], [(1.0, 103.0)], [(1.0, 103.0), (1.0, 103.0), (1.0, 103.0)],
                        [(1.0, 103.0), (1.0, 103.001)]):
            with self.subTest(corners=corners):
                fence = geofence.for_course("X", corners, 2.0)
                self.assertIsNone(fence.polygon)
                self.assertTrue(fence.reason)

    def test_a_corner_that_is_not_a_position_is_refused_not_computed(self):
        for bad in ((1.0, float("nan")), (float("inf"), 103.0), (95.0, 103.0), (1.0, 200.0),
                    (1e308, 103.0)):
            with self.subTest(corner=bad):
                fence = geofence.for_course("X", [bad, (1.0, 103.0), (2.0, 103.0)], 2.0)
                self.assertIsNone(fence.polygon)
                self.assertIn("not a valid latitude/longitude", fence.reason)

    def test_no_course_means_no_fence(self):
        self.assertIsNone(geofence.NO_COURSE.polygon)
        self.assertIsNone(geofence.NO_COURSE.source)
        self.assertEqual(geofence.NO_COURSE.reason, "no course received yet")

    def test_the_inset_is_the_same_size_a_degree_away_from_the_equator(self):
        """The metre frame follows the latitude: a margin is 2 m at Singapore and at 45 N."""
        for lat0 in (1.28, 45.0):
            with self.subTest(lat0=lat0):
                course = local_course([(0, 0), (100, 0), (100, 100), (0, 100)], lat0=lat0)
                fence = geofence.for_course("X", course, 2.0)
                origin = course[0]
                ins = [metres(p, origin) for p in fence.open_polygon]
                xs, ys = [p[0] for p in ins], [p[1] for p in ins]
                self.assertAlmostEqual(max(xs) - min(xs), 96.0, delta=0.3)
                self.assertAlmostEqual(max(ys) - min(ys), 96.0, delta=0.6)


class TestConfirmationParsing(unittest.TestCase):
    def body(self, **over):
        d = fence_frame("UAV1", [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]])["ocs_fence"]
        d.update(over)
        return d

    def test_a_good_body_is_kept_as_received(self):
        got, why = geofence.parse_confirmation(self.body())
        self.assertIsNone(why)
        self.assertEqual(got["vehicle_id"], "UAV1")
        self.assertEqual(got["polygon"], [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]])
        self.assertIs(got["verified"], True)
        self.assertEqual(got["shapes"], [poly_shape([[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]]),
                                         poly_shape(PILOT)])
        self.assertEqual(got["params"], PARAMS)
        self.assertEqual(got["home_hae_m"], 9.0)
        self.assertTrue(got["read_at"])

    def test_never_read_is_none_and_read_empty_is_an_empty_list(self):
        self.assertIsNone(geofence.parse_confirmation(self.body(shapes=None))[0]["shapes"])
        self.assertEqual(geofence.parse_confirmation(self.body(shapes=[]))[0]["shapes"], [])
        self.assertIsNone(geofence.parse_confirmation(self.body(polygon=None))[0]["polygon"])

    def test_an_older_adapter_that_sends_none_of_the_read_has_nothing_to_go_on(self):
        old = {k: v for k, v in self.body().items()
               if k not in ("read_at", "shapes", "params", "home_hae_m")}
        got, why = geofence.parse_confirmation(old)
        self.assertIsNone(why)
        self.assertEqual((got["read_at"], got["shapes"], got["params"], got["home_hae_m"]),
                         (None, None, dict(), None))

    def test_a_circle_is_kept(self):
        circle = {"type": "circle", "inclusion": False, "center": [1.0, 2.0], "radius_m": 30}
        got, _ = geofence.parse_confirmation(self.body(shapes=[circle]))
        self.assertEqual(got["shapes"], [dict(circle, radius_m=30.0)])

    def test_a_parameter_that_is_not_a_fence_parameter_is_not_ours_to_keep(self):
        got, _ = geofence.parse_confirmation(self.body(params={"FENCE_ALT_MAX": 30, "ARMING_CHECK": 1,
                                                               "FENCE_" + "X" * 20: 1,
                                                               "FENCE_ENABLE": None}))
        self.assertEqual(got["params"], {"FENCE_ALT_MAX": 30.0, "FENCE_ENABLE": None})

    def test_the_things_that_are_refused(self):
        for label, over in (("verified not a bool", {"verified": "yes"}),
                            ("verified missing", {"verified": None}),
                            ("polygon not a list", {"polygon": "x"}),
                            ("polygon vertex", {"polygon": [[1.0, "a"]]}),
                            ("polygon NaN", {"polygon": [[1.0, float("nan")]]}),
                            ("polygon bool", {"polygon": [[True, 1.0]]}),
                            ("no vehicle", {"vehicle_id": ""}),
                            ("verified_at", {"verified_at": 5}),
                            ("read_at", {"read_at": 5}),
                            ("reason", {"reason": 5}),
                            ("shapes not a list", {"shapes": {"type": "polygon"}}),
                            ("shape type", {"shapes": [{"type": "line", "inclusion": True}]}),
                            ("shape inclusion", {"shapes": [dict(poly_shape(PILOT), inclusion=1)]}),
                            ("shape of two points", {"shapes": [poly_shape(PILOT[:2])]}),
                            ("shape vertex", {"shapes": [poly_shape([[1, 2], [1, "x"], [2, 2]])]}),
                            ("circle radius", {"shapes": [{"type": "circle", "inclusion": True,
                                                            "center": [1, 2], "radius_m": "5"}]}),
                            ("params not an object", {"params": [30]}),
                            ("param text", {"params": {"FENCE_ALT_MAX": "30"}}),
                            ("param bool", {"params": {"FENCE_ENABLE": True}}),
                            ("home text", {"home_hae_m": "9"}),
                            ("home NaN", {"home_hae_m": float("nan")})):
            with self.subTest(label):
                got, why = geofence.parse_confirmation(self.body(**over))
                self.assertIsNone(got)
                self.assertTrue(why)
        self.assertIsNone(geofence.parse_confirmation([1, 2])[0])

    def test_free_text_is_cut_to_a_length_the_page_can_show(self):
        got, _ = geofence.parse_confirmation(self.body(reason="x" * 5000, lockout="y" * 5000))
        self.assertLessEqual(len(got["reason"]), 300)
        self.assertLessEqual(len(got["lockout"]), 300)


class TestSameRing(unittest.TestCase):
    SQ = [(1.0, 2.0), (1.0, 3.0), (2.0, 3.0), (2.0, 2.0)]

    def test_any_start_vertex_and_either_direction(self):
        self.assertTrue(geofence.same_ring(self.SQ, self.SQ[2:] + self.SQ[:2]))
        self.assertTrue(geofence.same_ring(self.SQ, list(reversed(self.SQ))))

    def test_within_a_micro_degree_is_the_same(self):
        self.assertTrue(geofence.same_ring(self.SQ, [(a + 5e-7, b - 5e-7) for a, b in self.SQ]))
        self.assertFalse(geofence.same_ring(self.SQ, [(a + 2e-6, b) for a, b in self.SQ]))

    def test_another_vertex_count_is_another_ring(self):
        self.assertFalse(geofence.same_ring(self.SQ, self.SQ[:3]))
        self.assertFalse(geofence.same_ring(self.SQ, None))
        self.assertFalse(geofence.same_ring(self.SQ, []))


class TestAircraftFence(unittest.TestCase):
    """aircraft_fence and declaration_problems: what a declaration with the aircraft would
    carry, read off its autopilot, and every reason it cannot -- each naming the vehicle."""

    NOW = 1_800_000_000.0

    def setUp(self):
        self.rc = geofence.for_course("ALPHA", ROBONATION_TEST_COURSE, 2.0)
        # PILOT is at BOX, not at RoboNation's test course: inside the course is checked
        # by declaration_problems, so these use the course the polygons are drawn in.
        self.corners = [(1.28010, 103.85520), (1.28010, 103.85625),
                        (1.28110, 103.85625), (1.28110, 103.85520)]

    def conf(self, polygon="rc", **kw):
        kw.setdefault("at", self.NOW - 10)
        kw.setdefault("read_at", self.NOW - 10)
        if polygon == "rc":
            polygon = [list(p) for p in self.rc.open_polygon]
        return geofence.parse_confirmation(fence_frame("UAV1", polygon, **kw)["ocs_fence"])[0]

    def af(self, confirm, max_age=180.0, ours=None):
        if ours is None:
            ours = (self.rc.open_polygon, confirm and confirm.get("polygon"))
        return geofence.aircraft_fence("UAV1", confirm, ours, self.NOW, max_age)

    def problems(self, af, ceiling_limit=50.0):
        return geofence.declaration_problems(af, self.corners, (), ceiling_limit)

    # -- what it carries

    def test_the_pilots_polygon_closed_and_the_ceiling(self):
        af = self.af(self.conf())
        self.assertEqual(af.problems, ())
        self.assertEqual(af.polygon, tuple(PILOT) + (PILOT[0],))
        self.assertEqual(af.ceiling_hae_m, CEILING_HAE_M)
        self.assertEqual(af.ceiling_from, "home 9.0 m HAE + FENCE_ALT_MAX 30.0 m")
        self.assertEqual(af.read_age_s, 10.0)
        self.assertEqual(af.notes, ())
        self.assertEqual(self.problems(af), [])

    def test_robocommands_polygon_is_recognised_from_any_start_and_either_direction(self):
        rc = [list(p) for p in self.rc.open_polygon]
        turned = list(reversed(rc[1:] + rc[:1]))
        af = self.af(self.conf(held=[turned, PILOT]))
        self.assertEqual(af.problems, ())
        self.assertEqual(af.polygon[:-1], tuple(PILOT))

    def test_the_polygon_the_adapter_says_it_was_told_is_never_the_pilots(self):
        """After a course change: the adapter still names the old course's polygon."""
        old = [[1.2803, 103.8555], [1.2803, 103.8560], [1.2808, 103.8560], [1.2808, 103.8555]]
        af = self.af(self.conf(old, held=[old, PILOT]))
        self.assertEqual(af.problems, ())
        self.assertEqual(af.polygon[:-1], tuple(PILOT))

    # -- the read

    def test_nothing_from_the_adapter(self):
        self.assertEqual(self.af(None).problems, (
            "UAV1: nothing read from its autopilot yet (no ocs_fence: is fleet_link running, "
            "and the radio up?)",))

    def test_never_read_names_the_adapters_reason(self):
        af = self.af(self.conf(held=None, state="waiting", reason="aircraft is armed"))
        self.assertEqual(af.problems, ("UAV1: its autopilot's fence has not been read yet "
                                       "(aircraft is armed)",))
        self.assertIsNone(af.polygon)

    def test_too_old(self):
        af = self.af(self.conf(read_at=self.NOW - 412))
        self.assertEqual(af.problems, ("UAV1: its autopilot's fence was last read 412 s ago "
                                       "(limit 180 s) -- it is read only while disarmed and "
                                       "heard on the radio",))
        self.assertIsNone(af.polygon, "nothing from a stale read is declared")

    def test_exactly_at_the_limit_is_still_fresh(self):
        self.assertEqual(self.af(self.conf(read_at=self.NOW - 180)).problems, ())
        self.assertTrue(self.af(self.conf(read_at=self.NOW - 181)).problems)

    def test_the_limit_is_the_callers(self):
        self.assertTrue(self.af(self.conf(read_at=self.NOW - 100), max_age=60).problems)

    def test_from_the_future_is_not_believed(self):
        self.assertIn("in the future", self.af(self.conf(read_at=self.NOW + 600)).problems[0])
        self.assertEqual(self.af(self.conf(read_at=self.NOW + 2)).problems, (),
                         "a second of skew is fine")

    def test_no_readable_time_is_an_unknown_age_not_a_fresh_one(self):
        bad = self.conf()
        bad["read_at"] = "yesterday-ish"
        self.assertIn("age is unknown", self.af(bad).problems[0])

    def test_the_verified_time_is_not_the_read_time(self):
        """RoboCommand's polygon last seen long ago says nothing about how fresh the read is."""
        self.assertEqual(self.af(self.conf(at=self.NOW - 5000)).problems, ())

    # -- the polygon

    def test_only_robocommands(self):
        af = self.af(self.conf(held=[self.rc.open_polygon]))
        self.assertEqual(af.problems[0], "UAV1: its autopilot holds only RoboCommand's polygon -- "
                                         "draw Ekko's own fence inside RoboCommand's in QGC and "
                                         "upload it")
        self.assertIsNone(af.polygon)

    def test_nothing_at_all(self):
        self.assertEqual(self.af(self.conf(held=[])).problems[0],
                         "UAV1: its autopilot holds no inclusion polygon -- draw Ekko's own fence "
                         "inside the course in QGC and upload it")

    def test_two_of_the_pilots(self):
        small = [[1.2804, 103.8555], [1.2804, 103.8557], [1.2806, 103.8557], [1.2806, 103.8555]]
        af = self.af(self.conf(held=[self.rc.open_polygon, PILOT, small]))
        self.assertIn("holds 2 inclusion polygons besides RoboCommand's", af.problems[0])
        self.assertIsNone(af.polygon, "the OCS never chooses between two of the pilot's")

    def test_exclusions_and_circles_are_noted_never_declared(self):
        circle = {"type": "circle", "inclusion": True, "center": [1.2806, 103.8557], "radius_m": 20}
        af = self.af(self.conf(held=[PILOT, poly_shape(PILOT, inclusion=False), circle, circle]))
        self.assertEqual(af.problems, ())
        self.assertEqual(af.polygon[:-1], tuple(PILOT))
        self.assertEqual(af.notes, ("its autopilot also holds 2 circles and 1 exclusion polygon: "
                                    "enforced by it, never declared",))

    def test_robocommands_rule_on_the_pilots_polygon(self):
        out = [list(p) for p in PILOT]
        out[1][1] = out[2][1] = 103.85640                       # ~16.7 m east of BOX's edge
        af = self.af(self.conf(held=[out]))
        self.assertEqual(af.problems, ())
        got = self.problems(af)
        self.assertEqual(len(got), 1)
        self.assertRegex(got[0], r"^UAV1: uav_geofence leaves the course and transit area by "
                                 r"1[67]\.\d m at .* \(the polygon drawn in QGC\)$")

    # -- the ceiling

    def test_no_fence_alt_max(self):
        params = {k: v for k, v in PARAMS.items() if k != "FENCE_ALT_MAX"}
        af = self.af(self.conf(params=params))
        self.assertIn("UAV1: FENCE_ALT_MAX has not been read from its autopilot", af.problems[0])
        self.assertIsNone(af.ceiling_hae_m)
        self.assertEqual(self.problems(af), list(af.problems), "no second 'not set' line")

    def test_no_home_altitude(self):
        af = self.af(self.conf(home_hae_m=None))
        self.assertIn("UAV1: its home altitude is unknown", af.problems[0])
        self.assertIn("fleet.toml geoid_separation_m", af.problems[0])

    def test_above_the_limit_says_how_it_was_made_and_what_to_change(self):
        af = self.af(self.conf(params=dict(PARAMS, FENCE_ALT_MAX=45.0)))
        self.assertEqual(af.ceiling_hae_m, 54.0)
        self.assertEqual(self.problems(af), [
            "UAV1: uav_geofence_max_altitude_hae_m is 54: it must be above 0 and at most 50 "
            "(home 9.0 m HAE + FENCE_ALT_MAX 45.0 m; lower FENCE_ALT_MAX in QGC)"])

    def test_the_limit_is_the_callers_too(self):
        af = self.af(self.conf())
        self.assertIn("at most 35", self.problems(af, ceiling_limit=35.0)[0])

    def test_the_ceiling_is_rounded_to_the_centimetre(self):
        af = self.af(self.conf(home_hae_m=9.123456))
        self.assertEqual(af.ceiling_hae_m, 39.12)

    # -- notes, never problems

    def test_a_fence_type_without_the_polygon_or_altitude_bit(self):
        af = self.af(self.conf(params=dict(PARAMS, FENCE_TYPE=2.0)))
        self.assertEqual(af.problems, ())
        self.assertEqual(len(af.notes), 2)
        self.assertIn("no polygon bit (4)", af.notes[0])
        self.assertIn("no altitude bit (1)", af.notes[1])

    def test_a_fence_that_is_off(self):
        af = self.af(self.conf(params=dict(PARAMS, FENCE_ENABLE=0.0, FENCE_AUTOENABLE=0.0)))
        self.assertIn("the fence is off", af.notes[0])

    def test_union_matters_only_with_more_than_one_polygon(self):
        self.assertEqual(self.af(self.conf(held=[PILOT],
                                           params=dict(PARAMS, FENCE_OPTIONS=2.0))).notes, ())
        af = self.af(self.conf(params=dict(PARAMS, FENCE_OPTIONS=2.0)))
        self.assertIn("a union", af.notes[0])


class TestUtc(unittest.TestCase):
    def test_the_utc_clock_reading(self):
        self.assertEqual(geofence.parse_utc("2026-10-11T00:31:05.123Z"),
                         geofence.parse_utc("2026-10-11T00:31:05.123+00:00"))
        self.assertEqual(geofence.parse_utc("2026-10-11T08:31:05+08:00"),
                         geofence.parse_utc("2026-10-11T00:31:05Z"))
        self.assertIsNone(geofence.parse_utc("not a time"))
        self.assertIsNone(geofence.parse_utc(None))


class TestGeofenceConfig(unittest.TestCase):
    """`[geofence]` in bridge.toml."""

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

    def _load(self, extra: str = ""):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bridge.toml"
            p.write_text(self.BASE + extra, encoding="utf-8")
            return load(p)

    def test_defaults_when_there_is_no_section(self):
        cfg = self._load()
        self.assertEqual(cfg.geofence_margin_m, 2.0)
        self.assertEqual(cfg.geofence_confirm_max_age_s, 180.0)
        self.assertEqual(cfg.geofence_ceiling_limit_hae_m, 50.0)

    def test_the_section_is_read(self):
        cfg = self._load("\n[geofence]\nboundary_margin_m = 3.5\nconfirm_max_age_s = 90\n"
                         "ceiling_limit_hae_m = 45\n")
        self.assertEqual(cfg.geofence_margin_m, 3.5)
        self.assertEqual(cfg.geofence_confirm_max_age_s, 90.0)
        self.assertEqual(cfg.geofence_ceiling_limit_hae_m, 45.0)

    def test_the_shipped_bridge_toml_loads_with_the_defaults(self):
        cfg = load(Path(__file__).resolve().parents[1] / "bridge.toml")
        self.assertEqual(cfg.geofence_margin_m, 2.0)
        self.assertEqual(cfg.geofence_confirm_max_age_s, 180.0)
        self.assertEqual(cfg.geofence_ceiling_limit_hae_m, 50.0)
        self.assertGreaterEqual(len(cfg.uav_geofence), 4,
                                "the retired key stays: Drone's check_config.py reads it")

    def test_what_is_refused(self):
        cases = {
            "margin zero": "\n[geofence]\nboundary_margin_m = 0\n",
            "margin negative": "\n[geofence]\nboundary_margin_m = -1\n",
            "margin text": "\n[geofence]\nboundary_margin_m = \"2\"\n",
            "margin bool": "\n[geofence]\nboundary_margin_m = true\n",
            "age zero": "\n[geofence]\nconfirm_max_age_s = 0\n",
            "limit zero": "\n[geofence]\nceiling_limit_hae_m = 0\n",
        }
        for label, extra in cases.items():
            with self.subTest(label):
                with self.assertRaises(ConfigError):
                    self._load(extra)

    def test_a_removed_key_is_an_error_that_says_what_to_do_instead(self):
        """Whoever wrote one expects it to be declared, and it would not be."""
        with self.assertRaises(ConfigError) as cm:
            self._load("\n[geofence]\n[[geofence.course]]\ncourse_id = \"SV1\"\n"
                       "uav_geofence = [[1,2],[1,3],[2,3],[1,2]]\n")
        self.assertIn("[[geofence.course]] was removed", str(cm.exception))
        self.assertIn("pilot draws in QGC", str(cm.exception))
        with self.assertRaises(ConfigError) as cm:
            self._load("\n[geofence]\nmax_altitude_hae_m = 40.0\n")
        self.assertIn("max_altitude_hae_m was removed", str(cm.exception))
        self.assertIn("Set FENCE_ALT_MAX in QGC", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
