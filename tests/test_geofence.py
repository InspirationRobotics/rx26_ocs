"""geofence.py and its `[geofence]` config: the fence for one course, and the rule that
decides whether an aircraft's confirmation can be believed. Pure -- no bridge, no socket.

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

from tests.harness import ROBONATION_TEST_COURSE, fence_frame    # noqa: E402

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
        fence = geofence.for_course("ALPHA", ROBONATION_TEST_COURSE, {}, 2.0)
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
        a = geofence.for_course("X", ccw, {}, 2.0)
        b = geofence.for_course("X", cw, {}, 2.0)
        self.assertIsNone(a.reason)
        self.assertIsNone(b.reason)
        self.assertEqual(set(a.open_polygon), set(b.open_polygon))
        # Each keeps the winding it was given: vertex i is the inset of input vertex i.
        wa = signed_area([metres(p, ring[0]) for p in a.open_polygon])
        wb = signed_area([metres(p, ring[0]) for p in b.open_polygon])
        self.assertGreater(wa, 0)
        self.assertLess(wb, 0)

    def test_a_closed_ring_and_an_open_one_are_the_same_boundary(self):
        closed = geofence.for_course("X", ROBONATION_TEST_COURSE, {}, 2.0)
        opened = geofence.for_course("X", ROBONATION_TEST_COURSE[:-1], {}, 2.0)
        self.assertEqual(closed.polygon, opened.polygon)

    def test_a_vertex_in_the_middle_of_an_edge_is_not_a_corner(self):
        fence = geofence.for_course(
            "X", local_course([(0, 0), (50, 0), (100, 0), (100, 60), (0, 60)]), {}, 2.0)
        self.assertIsNone(fence.reason)
        self.assertEqual(fence.vertices, 4)

    def test_a_triangle_works(self):
        fence = geofence.for_course("X", local_course([(0, 0), (100, 0), (50, 80)]), {}, 2.0)
        self.assertIsNone(fence.reason)
        self.assertEqual(fence.vertices, 3)

    def test_a_boundary_that_is_not_convex_is_refused_and_says_what_to_do(self):
        ell = local_course([(0, 0), (100, 0), (100, 40), (40, 40), (40, 100), (0, 100)])
        fence = geofence.for_course("ALPHA", ell, {}, 2.0)
        self.assertIsNone(fence.polygon)
        self.assertIn("not convex", fence.reason)
        self.assertIn("[[geofence.course]] entry for ALPHA", fence.reason)

    def test_a_bow_tie_and_a_pentagram_are_refused(self):
        bow = geofence.for_course("X", local_course([(0, 0), (100, 100), (100, 0), (0, 100)]), {}, 2.0)
        self.assertIsNone(bow.polygon)
        self.assertIn("not convex", bow.reason)
        star = geofence.for_course("X", local_course(
            [(50 * math.cos(math.radians(90 + 144 * k)), 50 * math.sin(math.radians(90 + 144 * k)))
             for k in range(5)]), {}, 2.0)
        self.assertIsNone(star.polygon)
        self.assertIn("not a simple polygon", star.reason)

    def test_a_margin_that_collapses_the_course_is_refused(self):
        thin = geofence.for_course("X", local_course([(0, 0), (100, 0), (100, 3), (0, 3)]), {}, 2.0)
        self.assertIsNone(thin.polygon)
        self.assertIn("margin of 2.0 m collapses a course 3.0 m wide", thin.reason)
        self.assertIn("boundary_margin_m", thin.reason)

    def test_a_margin_that_makes_a_short_edge_vanish_is_refused_too(self):
        """A trapezoid 50 m wide passes the width test at 22 m, but its 20 m top edge
        has already inverted -- the polygon it would give is not the inset."""
        trap = local_course([(0, 0), (100, 0), (60, 50), (40, 50)])
        self.assertIsNone(geofence.for_course("X", trap, {}, 20.0).reason)
        gone = geofence.for_course("X", trap, {}, 22.0)
        self.assertIsNone(gone.polygon)
        self.assertIn("collapses", gone.reason)

    def test_fewer_than_three_distinct_corners_is_refused(self):
        for corners in ([], [(1.0, 103.0)], [(1.0, 103.0), (1.0, 103.0), (1.0, 103.0)],
                        [(1.0, 103.0), (1.0, 103.001)]):
            with self.subTest(corners=corners):
                fence = geofence.for_course("X", corners, {}, 2.0)
                self.assertIsNone(fence.polygon)
                self.assertTrue(fence.reason)

    def test_a_corner_that_is_not_a_position_is_refused_not_computed(self):
        for bad in ((1.0, float("nan")), (float("inf"), 103.0), (95.0, 103.0), (1.0, 200.0),
                    (1e308, 103.0)):
            with self.subTest(corner=bad):
                fence = geofence.for_course("X", [bad, (1.0, 103.0), (2.0, 103.0)], {}, 2.0)
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
                fence = geofence.for_course("X", course, {}, 2.0)
                origin = course[0]
                ins = [metres(p, origin) for p in fence.open_polygon]
                xs, ys = [p[0] for p in ins], [p[1] for p in ins]
                self.assertAlmostEqual(max(xs) - min(xs), 96.0, delta=0.3)
                self.assertAlmostEqual(max(ys) - min(ys), 96.0, delta=0.6)


class TestTable(unittest.TestCase):
    BOUNDARY = local_course([(0, 0), (200, 0), (200, 100), (0, 100)])

    def entry(self, *xy):
        pts = local_course(list(xy))
        return tuple(pts) + (pts[0],)

    def test_an_entry_inside_the_boundary_is_used_as_written(self):
        entry = self.entry((20, 20), (180, 20), (180, 80), (20, 80))
        fence = geofence.for_course("BRAVO", self.BOUNDARY, {"BRAVO": entry}, 2.0)
        self.assertEqual(fence.polygon, entry)
        self.assertEqual(fence.source, "table: BRAVO")
        self.assertIsNone(fence.reason)

    def test_an_entry_for_another_course_is_not_used(self):
        entry = self.entry((20, 20), (180, 20), (180, 80), (20, 80))
        fence = geofence.for_course("ALPHA", self.BOUNDARY, {"BRAVO": entry}, 2.0)
        self.assertEqual(fence.source, "course boundary inset 2.0 m")

    def test_an_entry_outside_is_refused_naming_the_vertex_and_never_falls_back(self):
        entry = self.entry((20, 20), (180, 20), (260, 80), (20, 80))      # vertex 3 is 60 m out
        fence = geofence.for_course("BRAVO", self.BOUNDARY, {"BRAVO": entry}, 2.0)
        self.assertIsNone(fence.polygon, "a perfectly good inset exists, and must NOT be used")
        self.assertIsNone(fence.source)
        self.assertIn("vertex 3", fence.reason)
        self.assertIn("BRAVO", fence.reason)
        self.assertIn("60.0 m outside", fence.reason)

    def test_every_outside_vertex_is_counted_but_the_first_is_named(self):
        entry = self.entry((-30, 20), (180, 20), (260, 80), (20, 80))
        fence = geofence.for_course("BRAVO", self.BOUNDARY, {"BRAVO": entry}, 2.0)
        self.assertIn("vertex 1", fence.reason)
        self.assertIn("1 more", fence.reason)

    def test_half_a_metre_outside_is_the_width_of_a_typed_coordinate_not_an_error(self):
        edge = self.entry((20, 20), (180, 20), (180, 80), (20, 80))
        near = self.entry((0, -0.4), (180, 20), (180, 80), (20, 80))     # 0.4 m outside
        far = self.entry((0, -0.7), (180, 20), (180, 80), (20, 80))      # 0.7 m outside
        self.assertIsNotNone(geofence.for_course("B", self.BOUNDARY, {"B": near}, 2.0).polygon)
        self.assertIsNone(geofence.for_course("B", self.BOUNDARY, {"B": far}, 2.0).polygon)
        self.assertIsNotNone(geofence.for_course("B", self.BOUNDARY, {"B": edge}, 2.0).polygon)

    def test_a_table_entry_is_checked_even_against_a_boundary_that_is_not_convex(self):
        ell = local_course([(0, 0), (100, 0), (100, 40), (40, 40), (40, 100), (0, 100)])
        inside = self.entry((10, 10), (90, 10), (90, 30), (10, 30))
        notch = self.entry((50, 50), (90, 50), (90, 90), (50, 90))       # in the missing corner
        self.assertIsNotNone(geofence.for_course("B", ell, {"B": inside}, 2.0).polygon)
        self.assertIsNone(geofence.for_course("B", ell, {"B": notch}, 2.0).polygon)

    def test_a_table_entry_with_a_degenerate_boundary_is_refused_not_trusted(self):
        entry = self.entry((20, 20), (180, 20), (180, 80), (20, 80))
        fence = geofence.for_course("B", [(1.0, 103.0), (1.0, 103.001)], {"B": entry}, 2.0)
        self.assertIsNone(fence.polygon)
        self.assertIn("cannot be checked", fence.reason)


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

    def test_cleared_is_an_empty_list_and_absent_is_none(self):
        self.assertEqual(geofence.parse_confirmation(self.body(polygon=[]))[0]["polygon"], [])
        self.assertIsNone(geofence.parse_confirmation(self.body(polygon=None))[0]["polygon"])

    def test_the_things_that_are_refused(self):
        for label, over in (("verified not a bool", {"verified": "yes"}),
                            ("verified missing", {"verified": None}),
                            ("polygon not a list", {"polygon": "x"}),
                            ("polygon vertex", {"polygon": [[1.0, "a"]]}),
                            ("polygon NaN", {"polygon": [[1.0, float("nan")]]}),
                            ("polygon bool", {"polygon": [[True, 1.0]]}),
                            ("no vehicle", {"vehicle_id": ""}),
                            ("verified_at", {"verified_at": 5}),
                            ("reason", {"reason": 5})):
            with self.subTest(label):
                got, why = geofence.parse_confirmation(self.body(**over))
                self.assertIsNone(got)
                self.assertTrue(why)
        self.assertIsNone(geofence.parse_confirmation([1, 2])[0])

    def test_free_text_is_cut_to_a_length_the_page_can_show(self):
        got, _ = geofence.parse_confirmation(self.body(reason="x" * 5000, lockout="y" * 5000))
        self.assertLessEqual(len(got["reason"]), 300)
        self.assertLessEqual(len(got["lockout"]), 300)


class TestGate(unittest.TestCase):
    """confirmation_problem: verified, the same polygon, recent -- and the message says
    which one is missing, naming the vehicle."""

    NOW = 1_800_000_000.0

    def setUp(self):
        self.fence = geofence.for_course("ALPHA", ROBONATION_TEST_COURSE, {}, 2.0)

    def conf(self, polygon="fence", **kw):
        kw.setdefault("at", self.NOW - 10)
        if polygon == "fence":
            polygon = self.fence.open_polygon
        return geofence.parse_confirmation(fence_frame("UAV1", polygon, **kw)["ocs_fence"])[0]

    def problem(self, confirm, fence=None, max_age=180.0):
        return geofence.confirmation_problem("UAV1", fence or self.fence, confirm,
                                             self.NOW, max_age)

    def test_a_verified_matching_recent_confirmation_passes(self):
        self.assertIsNone(self.problem(self.conf()))

    def test_no_fence_for_the_course_means_no_aircraft_can_be_declared(self):
        why = self.problem(self.conf(), fence=geofence.NO_COURSE)
        self.assertIn("UAV1", why)
        self.assertIn("no UAV geofence", why)
        self.assertIn("no course received yet", why)

    def test_a_confirmation_that_never_came(self):
        why = self.problem(None)
        self.assertIn("UAV1: autopilot fence not confirmed", why)

    def test_unverified_names_the_state_and_the_reason(self):
        why = self.problem(self.conf(verified=False, state="held", reason="armed: not writing"))
        self.assertEqual(why, "UAV1: autopilot fence not confirmed (state held: armed: not writing)")

    def test_a_different_polygon(self):
        shifted = [[lat + 2e-6, lon] for lat, lon in self.fence.open_polygon]
        why = self.problem(self.conf(shifted))
        self.assertIn("UAV1: autopilot holds a different polygon than the ALPHA fence", why)
        self.assertIn("4 vertices", why)

    def test_a_polygon_within_a_micro_degree_is_the_same_polygon(self):
        near = [[lat + 5e-7, lon - 5e-7] for lat, lon in self.fence.open_polygon]
        self.assertIsNone(self.problem(self.conf(near)))

    def test_the_wrong_number_of_vertices(self):
        self.assertIn("different polygon", self.problem(self.conf(self.fence.open_polygon[:3])))

    def test_cleared_is_a_polygon_that_does_not_match(self):
        why = self.problem(self.conf([]))
        self.assertIn("different polygon", why)
        self.assertIn("cleared", why)
        self.assertIn("different polygon", self.problem(self.conf(None)))

    def test_a_confirmation_that_is_too_old(self):
        why = self.problem(self.conf(at=self.NOW - 412))
        self.assertEqual(why, "UAV1: fence confirmation 412 s old (limit 180 s)")

    def test_the_limit_is_the_configured_one(self):
        self.assertIsNone(self.problem(self.conf(at=self.NOW - 100), max_age=180))
        self.assertIn("100 s old (limit 60 s)", self.problem(self.conf(at=self.NOW - 100), max_age=60))

    def test_exactly_at_the_limit_is_still_fresh(self):
        self.assertIsNone(self.problem(self.conf(at=self.NOW - 180)))
        self.assertIsNotNone(self.problem(self.conf(at=self.NOW - 181)))

    def test_a_confirmation_from_the_future_is_not_believed(self):
        self.assertIn("future", self.problem(self.conf(at=self.NOW + 600)))
        self.assertIsNone(self.problem(self.conf(at=self.NOW + 2)), "a second of skew is fine")

    def test_no_readable_timestamp_is_an_unknown_age_not_a_fresh_one(self):
        self.assertIn("age is unknown", self.problem(self.conf(at=None)))
        bad = self.conf()
        bad["verified_at"] = "yesterday-ish"
        self.assertIn("age is unknown", self.problem(bad))

    def test_the_utc_clock_reading(self):
        self.assertEqual(geofence.parse_utc("2026-10-11T00:31:05.123Z"),
                         geofence.parse_utc("2026-10-11T00:31:05.123+00:00"))
        self.assertEqual(geofence.parse_utc("2026-10-11T08:31:05+08:00"),
                         geofence.parse_utc("2026-10-11T00:31:05Z"))
        self.assertIsNone(geofence.parse_utc("not a time"))
        self.assertIsNone(geofence.parse_utc(None))


class TestGeofenceConfig(unittest.TestCase):
    """`[geofence]` and `[[geofence.course]]` in bridge.toml."""

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

    ENTRY = """
[[geofence.course]]
course_id = "BRAVO"
uav_geofence = [[1.0, 103.0], [1.0, 103.001], [1.001, 103.001], [1.0, 103.0]]
"""

    def test_defaults_when_there_is_no_section(self):
        cfg = self._load()
        self.assertEqual(cfg.geofence_margin_m, 2.0)
        self.assertEqual(cfg.geofence_confirm_max_age_s, 180.0)
        self.assertEqual(cfg.geofence_table, {})

    def test_the_section_is_read(self):
        cfg = self._load("\n[geofence]\nboundary_margin_m = 3.5\nconfirm_max_age_s = 90\n" + self.ENTRY)
        self.assertEqual(cfg.geofence_margin_m, 3.5)
        self.assertEqual(cfg.geofence_confirm_max_age_s, 90.0)
        self.assertEqual(cfg.geofence_table["BRAVO"][0], (1.0, 103.0))
        self.assertEqual(len(cfg.geofence_table["BRAVO"]), 4)

    def test_the_shipped_bridge_toml_loads_with_the_defaults_and_the_sv_fences(self):
        cfg = load(Path(__file__).resolve().parents[1] / "bridge.toml")
        self.assertEqual(cfg.geofence_margin_m, 2.0)
        self.assertEqual(cfg.geofence_confirm_max_age_s, 180.0)
        self.assertEqual(sorted(cfg.geofence_table), ["SV1", "SV3"],
                         "Ekko's own fences for the 7-8 Nov screening areas; ALPHA uses the inset")
        self.assertGreaterEqual(len(cfg.uav_geofence), 4,
                                "the retired key stays: Drone's check_config.py reads it")

    def test_what_is_refused_on_the_trailer(self):
        cases = {
            "margin zero": "\n[geofence]\nboundary_margin_m = 0\n",
            "margin negative": "\n[geofence]\nboundary_margin_m = -1\n",
            "margin text": "\n[geofence]\nboundary_margin_m = \"2\"\n",
            "margin bool": "\n[geofence]\nboundary_margin_m = true\n",
            "age zero": "\n[geofence]\nconfirm_max_age_s = 0\n",
            "no course id": "\n[geofence]\n[[geofence.course]]\nuav_geofence = [[1,2],[1,3],[2,3],[1,2]]\n",
            "no polygon": "\n[geofence]\n[[geofence.course]]\ncourse_id = \"A\"\n",
            "open polygon": "\n[geofence]\n[[geofence.course]]\ncourse_id = \"A\"\nuav_geofence = [[1,2],[1,3],[2,3],[2,2]]\n",
            "three points": "\n[geofence]\n[[geofence.course]]\ncourse_id = \"A\"\nuav_geofence = [[1,2],[1,3],[1,2]]\n",
            "repeated vertex": "\n[geofence]\n[[geofence.course]]\ncourse_id = \"A\"\nuav_geofence = [[1,2],[1,2],[1,2],[1,2]]\n",
            "bad latitude": "\n[geofence]\n[[geofence.course]]\ncourse_id = \"A\"\nuav_geofence = [[91,2],[1,3],[2,3],[91,2]]\n",
            "a vertex that is not a pair": "\n[geofence]\n[[geofence.course]]\ncourse_id = \"A\"\nuav_geofence = [[1,2],[1,3,4],[2,3],[1,2]]\n",
            "duplicate course": "\n[geofence]\n" + self.ENTRY + self.ENTRY,
        }
        for label, extra in cases.items():
            with self.subTest(label):
                with self.assertRaises(ConfigError):
                    self._load(extra)

    def test_the_error_names_the_course_and_what_is_wrong(self):
        with self.assertRaises(ConfigError) as cm:
            self._load("\n[geofence]\n[[geofence.course]]\ncourse_id = \"BRAVO\"\n"
                       "uav_geofence = [[1,2],[1,3],[2,3],[2,2]]\n")
        self.assertIn("BRAVO", str(cm.exception))
        self.assertIn("closed", str(cm.exception))


if __name__ == "__main__":
    unittest.main()
