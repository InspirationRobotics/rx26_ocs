"""fence_upload: the autopilot's fence over the radio, against a fake ArduPilot (helpers.FakeAutopilot)
that speaks the fence side of the mission protocol through pymavlink -- no radio, no sleeping.

What is pinned: the wire layouts (both directions, against pymavlink); Ekko's own rules carried over
(closing vertex stripped, param1 = vertex count, never armed, never a parameter written, read back
within 1 m); the READ that always runs -- every fence item, the FENCE_* params and home HAE, which is
what the bridge declares from; and the one WRITE -- RoboCommand's polygon, once per course, skipped
when it is there already, never over the pilot's polygon inside it, never put back once the pilot has
taken it off, the old fence saved before it is replaced (and not replaced unsaved), never locking the
aircraft out, surviving a lossy radio, giving up loudly."""
from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as ap

from fleet import fence_upload as fu
from fleet import mavframe

from .helpers import DIALECT, FakeAutopilot, encode, fence_items

T0 = 1000.0
#: bridge.toml's placeholder box at the Singapore test course (closed, as declared): RoboCommand's polygon.
DECLARED = [(1.28010, 103.85520), (1.28010, 103.85625), (1.28110, 103.85625), (1.28110, 103.85520),
            (1.28010, 103.85520)]
OPEN = DECLARED[:-1]
INSIDE = (1.28060, 103.85570)
SARASOTA = (27.3364, -82.5307)
#: The pilot's own fence for the course, drawn in QGC inside RoboCommand's -- five vertices, to tell them apart.
PILOT = [(1.28030, 103.85540), (1.28030, 103.85600), (1.28090, 103.85600), (1.28100, 103.85570),
         (1.28090, 103.85540)]
CIRCLE = [{"command": 5003, "param1": 35.0, "lat": 32.92357, "lon": -117.03857}]
ENABLED = {"FENCE_TYPE": 7.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}
ALLOWED_TO_SEND = {"PARAM_REQUEST_READ", "MISSION_REQUEST_LIST", "MISSION_REQUEST_INT", "MISSION_ACK",
                   "MISSION_COUNT", "MISSION_ITEM_INT"}
#: Only an upload sends these (a download ends with our MISSION_ACK, so that one is not a write).
WRITES = {"MISSION_COUNT", "MISSION_ITEM_INT"}


def frame_of(raw: bytes) -> mavframe.Frame:
    (f,) = mavframe.split_datagram(DIALECT, raw)
    return f


def ours(msgid: int, payload: bytes) -> object:
    """What pymavlink makes of a frame built from one of fence_upload's payloads."""
    raw = mavframe.pack_frame(DIALECT, msgid, payload, sysid=250, compid=190, seq=7)
    return ap.MAVLink(None).decode(bytearray(raw))


def east(point, metres):
    """`point` moved `metres` east, in fence_upload's own flat metres."""
    return (point[0], point[1] + metres / (111195.08 * math.cos(math.radians(point[0]))))


def reach(metres):
    """PILOT with its east side moved out to `metres` beyond RoboCommand's east edge (lon 103.85625)."""
    return [east((lat, 103.85625), metres) if lon == 103.85600 else (lat, lon) for lat, lon in PILOT]


def held(*polygons):
    """Downloaded fence items holding these (open) polygons, in this order."""
    return [dict(i, seq=n) for n, i in enumerate(i for p in polygons for i in fence_items(p))]


class Rig:
    """One FenceUploader wired to one FakeAutopilot on a fake clock."""

    def __init__(self, autopilot=None, *, timing=fu.Timing(), backup_dir=None, position=SARASOTA,
                 may_block_arming=False, write=True, home_hae=None):
        self.t = T0
        self.ap = autopilot or FakeAutopilot()
        self._tmp = tempfile.TemporaryDirectory()
        self.backup_dir = backup_dir if backup_dir is not None else str(Path(self._tmp.name) / "fence")
        self.up = fu.FenceUploader(label="UAV1", target_sysid=1, target_compid=1, our_sysid=250,
                                   send=self._send, armed=lambda now: self.armed,
                                   position=lambda now: self.position, route_ok=lambda: self.route_ok,
                                   backup_dir=self.backup_dir, may_block_arming=may_block_arming,
                                   write=write, home_hae=home_hae,
                                   wall=lambda: 1_790_000_000.0 + (self.t - T0), timing=timing)
        self.armed, self.position, self.route_ok = False, position, True
        self.accept = True
        self._seq = 0

    def close(self):
        self._tmp.cleanup()

    def _send(self, msgid, payload, now):
        if not self.accept:
            return False
        self._seq = (self._seq + 1) & 0xFF
        self.ap.hear(mavframe.pack_frame(DIALECT, msgid, payload, sysid=250, compid=190, seq=self._seq), now)
        return True

    def deliver(self):
        for raw in self.ap.take():
            self.up.note(frame_of(raw), self.t)

    def declare(self, polygon=DECLARED):
        self.up.set_polygon(polygon, self.t)

    def run(self, seconds, step=0.05):
        end = self.t + seconds
        while self.t < end - 1e-9:
            self.t += step
            self.deliver()
            self.ap.tick(self.t)
            self.deliver()
            self.up.tick(self.t)
            self.deliver()

    def sent(self, name):
        return [m for m in self.ap.heard if m.get_type() == name]

    def status(self):
        st = self.up.status(self.t)
        json.dumps(st, allow_nan=False)
        return st


class RigCase(unittest.TestCase):
    def rig(self, *a, **kw):
        r = Rig(*a, **kw)
        self.addCleanup(r.close)
        return r

    def assertHolds(self, r, polygon=OPEN):
        self.assertIsNone(fu.matches([dict(i, seq=n) for n, i in enumerate(r.ap.fence)], polygon), r.ap.fence)

    def assertNothingWritten(self, r, since=0):
        later = r.ap.names()[since:]
        self.assertFalse(WRITES & set(later), later)


# ============================================================================ wire

class WireLayouts(RigCase):
    """Every hand-packed layout, read by pymavlink; every pymavlink frame, read by ours."""

    def test_what_we_send_is_what_pymavlink_reads(self):
        m = ours(fu.MSG_PARAM_REQUEST_READ, fu.pack_param_request_read("FENCE_AUTOENABLE", 1, 1))
        self.assertEqual((m.get_type(), m.param_id, m.param_index, m.target_system, m.target_component),
                         ("PARAM_REQUEST_READ", "FENCE_AUTOENABLE", -1, 1, 1))
        m = ours(fu.MSG_MISSION_REQUEST_LIST, fu.pack_mission_request_list(1, 1))
        self.assertEqual((m.get_type(), m.target_system, m.target_component, m.mission_type),
                         ("MISSION_REQUEST_LIST", 1, 1, 1))
        m = ours(fu.MSG_MISSION_COUNT, fu.pack_mission_count(4, 1, 1))
        self.assertEqual((m.count, m.target_system, m.target_component, m.mission_type), (4, 1, 1, 1))
        m = ours(fu.MSG_MISSION_REQUEST_INT, fu.pack_mission_request_int(3, 1, 1))
        self.assertEqual((m.get_type(), m.seq, m.target_system, m.mission_type), ("MISSION_REQUEST_INT", 3, 1, 1))
        m = ours(fu.MSG_MISSION_ACK, fu.pack_mission_ack(0, 1, 1))
        self.assertEqual((m.get_type(), m.type, m.target_system, m.mission_type), ("MISSION_ACK", 0, 1, 1))

    def test_a_vertex_is_what_fence_core_sends(self):
        m = ours(fu.MSG_MISSION_ITEM_INT, fu.pack_fence_vertex(2, 1.2811, 103.85625, 4, 1, 1))
        self.assertEqual((m.seq, m.command, m.frame, m.current, m.autocontinue, m.param1, m.mission_type,
                          m.target_system, m.target_component),
                         (2, 5001, 0, 0, 0, 4.0, 1, 1, 1))
        self.assertEqual((m.x, m.y), (12811000, 1038562500))

    def test_a_southern_western_vertex_keeps_its_sign(self):
        m = ours(fu.MSG_MISSION_ITEM_INT, fu.pack_fence_vertex(0, -35.3632621, -82.5307, 3, 1, 1))
        self.assertEqual((m.x, m.y), (-353632621, -825307000))

    def test_what_pymavlink_sends_is_what_we_read(self):
        f = frame_of(encode("param_value", b"FENCE_TYPE", 7.0, 9, 900, 12))
        self.assertEqual(fu.param_value(f), ("FENCE_TYPE", 7.0))
        f = frame_of(encode("mission_item_int", 250, 190, 3, 0, 5003, 0, 0, 35.0, 0, 0, 0, -353632621, 1491652374,
                            0, 1))
        item = fu.mission_item(f)
        self.assertEqual((item["seq"], item["command"], item["param1"], item["target_system"], item["mission_type"]),
                         (3, 5003, 35.0, 250, 1))
        self.assertAlmostEqual(item["lat"], -35.3632621, places=7)
        self.assertAlmostEqual(item["lon"], 149.1652374, places=7)

    def test_trailing_zeros_trimmed_by_a_v2_sender_read_as_zeros(self):
        # seq 0, mission_type 0 (a plain mission): v2 trims the payload's zero tail.
        f = frame_of(encode("mission_item_int", 250, 0, 0, 0, 16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
        self.assertLess(len(f.payload), 38)
        item = fu.mission_item(f)
        self.assertEqual((item["seq"], item["mission_type"], item["command"]), (0, 0, 16))

    def test_only_the_autopilots_fence_answers_addressed_to_us_are_taken(self):
        a = FakeAutopilot()
        a.silent_missions = True
        r = self.rig(a)
        r.declare()
        r.run(0.2)
        self.assertEqual((r.up._phase, r.up._count), ("read", None))
        n = len(r.ap.heard)
        # QGC (255) downloading its own copy over the same radio, another vehicle, a mission count
        r.up.note(frame_of(encode("mission_count", 255, 0, 9, 1)), r.t)
        r.up.note(frame_of(encode("mission_count", 250, 190, 9, 1, sysid=2)), r.t)
        r.up.note(frame_of(encode("mission_count", 250, 190, 9, 0)), r.t)
        self.assertIsNone(r.up._count)
        self.assertEqual(len(r.ap.heard), n)
        r.up.note(frame_of(encode("mission_count", 250, 190, 9, 1)), r.t)       # ours
        self.assertEqual(r.up._count, 9)


# ======================================================================== geometry

class Geometry(unittest.TestCase):
    def test_the_closing_vertex_is_stripped_and_an_open_polygon_kept(self):
        self.assertEqual(fu.open_polygon(DECLARED), OPEN)
        self.assertEqual(fu.open_polygon(OPEN), OPEN)

    def test_fewer_than_three_distinct_vertices_is_refused(self):
        with self.assertRaises(fu.FenceError):
            fu.open_polygon([(1.0, 103.0), (1.1, 103.0), (1.0, 103.0)])

    def test_a_vertex_that_is_not_a_coordinate_is_refused(self):
        for bad in ((math.nan, 103.0), (91.0, 103.0), (1.0, 181.0)):
            with self.assertRaises(fu.FenceError):
                fu.open_polygon([bad, (1.1, 103.0), (1.1, 103.1)])

    def test_inside(self):
        self.assertTrue(fu.inside(INSIDE, OPEN))
        self.assertFalse(fu.inside(SARASOTA, OPEN))
        self.assertFalse(fu.inside((1.2806, 103.8565), OPEN))      # just east

    def test_outside_by_m(self):
        self.assertEqual(fu.outside_by_m(INSIDE, OPEN), 0.0)
        self.assertAlmostEqual(fu.outside_by_m(east((1.28060, 103.85625), 10.0), OPEN), 10.0, delta=0.05)
        self.assertGreater(fu.outside_by_m(SARASOTA, OPEN), 1e6)

    def test_distance_is_right_across_an_ocean_and_the_dateline(self):
        san_diego = (32.7157, -117.1611)
        self.assertAlmostEqual(fu.offset_m(san_diego, OPEN[0]) / 1000.0, 14140, delta=150)
        self.assertAlmostEqual(fu.offset_m((0.0, 179.9), (0.0, -179.9)), 22239, delta=5)
        self.assertAlmostEqual(fu.offset_m(OPEN[0], (OPEN[0][0] + 1.0 / 111195.08, OPEN[0][1])), 1.0, places=3)

    def test_matches(self):
        items = held(OPEN)
        self.assertIsNone(fu.matches(items, OPEN))
        moved = [dict(i) for i in items]
        moved[2]["lat"] += 2.0 / 111195.0
        self.assertIn("vertex 2 is 2.0 m", fu.matches(moved, OPEN))
        self.assertIn("holds 3 fence item(s)", fu.matches(items[:3], OPEN))
        wrong_n = [dict(i, param1=5.0) for i in items]
        self.assertIn("5 vertices", fu.matches(wrong_n, OPEN))
        circle = [dict(i) for i in items]
        circle[0]["command"] = fu.CMD_FENCE_CIRCLE_INCLUSION
        self.assertIn("inclusion circle", fu.matches(circle, OPEN))

    def test_the_same_ring_from_any_vertex_in_either_direction(self):
        self.assertTrue(fu.same_ring(OPEN, OPEN[2:] + OPEN[:2]))
        self.assertTrue(fu.same_ring(OPEN, list(reversed(OPEN))))
        self.assertTrue(fu.same_ring(OPEN, [east(p, 0.5) for p in OPEN]))         # within 1 m
        moved = list(OPEN)
        moved[1] = east(OPEN[1], 2.0)
        self.assertFalse(fu.same_ring(OPEN, moved))
        self.assertFalse(fu.same_ring(OPEN, OPEN[:3]))

    def test_contains_looks_among_every_inclusion_polygon(self):
        self.assertTrue(fu.contains(held(PILOT, OPEN), OPEN))
        self.assertTrue(fu.contains(held(list(reversed(OPEN))), OPEN))
        self.assertFalse(fu.contains(held(PILOT), OPEN))
        excluded = [dict(i, command=fu.CMD_FENCE_POLYGON_VERTEX_EXCLUSION) for i in held(OPEN)]
        self.assertFalse(fu.contains(excluded, OPEN))

    def test_drawn_inside_is_the_pilots_polygon(self):
        self.assertEqual(fu.drawn_inside(held(PILOT), OPEN), 5)
        self.assertEqual(fu.drawn_inside(held(OPEN, PILOT), OPEN), 5)     # RoboCommand's own is never the pilot's
        self.assertIsNone(fu.drawn_inside(held(OPEN), OPEN))
        self.assertEqual(fu.drawn_inside(held(reach(3.0)), OPEN), 5)      # placed by hand on RoboCommand's line
        self.assertIsNone(fu.drawn_inside(held(reach(10.0)), OPEN))       # sticks out: another course's, or a test
        self.assertIsNone(fu.drawn_inside(held(PILOT), OPEN, ignore=[PILOT]))   # this OCS's own
        excluded = [dict(i, command=fu.CMD_FENCE_POLYGON_VERTEX_EXCLUSION) for i in held(PILOT)]
        self.assertIsNone(fu.drawn_inside(excluded, OPEN))


# ======================================================================== the write

class Writing(RigCase):
    def test_fence_off_far_away_it_is_written_and_read_back(self):
        r = self.rig()
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done", st)
        self.assertIn("written and read back: 4 vertices", st["reason"])
        self.assertHolds(r)
        self.assertEqual([i["param1"] for i in r.ap.fence], [4.0] * 4)          # the closing vertex is gone
        self.assertEqual((r.ap.uploads, st["writes"], st["attempts"], st["failures"]), (1, 1, 1, 0))
        self.assertEqual([m.param_id for m in r.sent("PARAM_REQUEST_READ")], list(fu.REPORTED_PARAMS))
        names = r.ap.names()
        self.assertEqual(names[:5], ["PARAM_REQUEST_READ"] * 5)
        self.assertEqual(names.count("MISSION_COUNT"), 1)
        self.assertEqual(names.count("MISSION_REQUEST_LIST"), 2)                 # read first, then read back
        self.assertIsNone(st["backup"])                                          # nothing was there to save
        self.assertTrue(st["verified"])

    def test_nothing_but_the_fence_dialog_and_parameter_READS_is_ever_sent(self):
        r = self.rig()
        r.declare()
        r.run(5)
        self.assertLessEqual(set(r.ap.names()), ALLOWED_TO_SEND)
        self.assertNotIn("PARAM_SET", r.ap.names())
        self.assertEqual(r.ap.params, {"FENCE_TYPE": 7.0, "FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 0.0})
        self.assertTrue(all(m.target_system == 1 for m in r.ap.heard))

    def test_a_fence_already_there_is_not_written_again(self):
        for there in (OPEN, list(reversed(OPEN[1:] + OPEN[:1]))):               # as written, or redrawn in QGC
            r = self.rig(FakeAutopilot(fence=fence_items(there)))
            r.declare()
            r.run(5)
            st = r.status()
            self.assertEqual(st["state"], "done")
            self.assertIn("already holds RoboCommand's polygon; nothing written", st["reason"])
            self.assertNothingWritten(r)
            self.assertEqual(r.ap.uploads, 0)
            self.assertTrue(st["verified"])

    def test_the_old_fence_is_saved_before_it_is_replaced(self):
        circle = [{"command": 5003, "param1": 35.0, "lat": 27.3364, "lon": -82.5307}]
        r = self.rig(FakeAutopilot(fence=circle))
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done")
        self.assertHolds(r)
        saved = json.loads(Path(st["backup"]).read_text(encoding="utf-8"))
        self.assertEqual(Path(st["backup"]).parent, Path(r.backup_dir))
        self.assertEqual([(i["command"], i["param1"]) for i in saved["items"]], [(5003, 35.0)])
        self.assertAlmostEqual(saved["items"][0]["lat"], 27.3364, places=6)
        self.assertEqual(saved["summary"], "1 inclusion circle")
        self.assertEqual(saved["replaced_by"], [list(p) for p in OPEN])

    def test_an_old_fence_that_cannot_be_saved_is_not_replaced(self):
        circle = [{"command": 5003, "param1": 35.0, "lat": 27.3364, "lon": -82.5307}]
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        blocker = Path(tmp.name) / "fence"
        blocker.write_text("a file where the directory should be")
        r = self.rig(FakeAutopilot(fence=circle), backup_dir=str(blocker))
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "waiting")
        self.assertIn("so it was not replaced", st["last_error"])
        self.assertEqual(r.ap.fence, circle)
        self.assertNothingWritten(r)

    def test_one_write_per_polygon_and_a_new_polygon_starts_over(self):
        r = self.rig()
        r.declare()
        r.run(5)
        n = len(r.ap.heard)
        r.declare()                                       # the bridge re-sends ocs_info on every reconnect
        r.run(60)
        later = r.ap.names()[n:]
        self.assertIn("MISSION_REQUEST_LIST", later)      # read every 30 s ...
        self.assertLessEqual(set(later), ALLOWED_TO_SEND - WRITES)    # ... and never written again
        self.assertEqual(r.ap.uploads, 1)
        # A new course whose polygon encloses the old one: the old one is this OCS's own doing, not
        # the pilot's fence drawn inside the new one, so it is saved and replaced.
        bigger = [(1.2800, 103.8551), (1.2800, 103.8564), (1.2812, 103.8564), (1.2812, 103.8551)]
        r.declare(bigger)
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done", st)
        self.assertHolds(r, bigger)
        self.assertEqual(r.ap.uploads, 2)
        self.assertEqual(len(json.loads(Path(st["backup"]).read_text(encoding="utf-8"))["items"]), 4)

    def test_a_withdrawn_polygon_is_never_written_and_the_fence_is_left_alone(self):
        r = self.rig(FakeAutopilot(fence=CIRCLE))
        r.armed = True
        r.declare()
        r.run(2)
        r.up.set_polygon(None, r.t)
        r.armed = False
        r.run(10)
        st = r.status()
        self.assertEqual(st["state"], "off")
        self.assertIn("withdrew", st["reason"])
        self.assertNothingWritten(r)
        self.assertEqual(r.ap.fence, CIRCLE)
        self.assertEqual(st["reads"], 1)                  # still read: the bridge declares from it

    def test_before_a_polygon_arrives_the_autopilot_is_only_read(self):
        r = self.rig(FakeAutopilot(fence=CIRCLE))
        r.run(10)
        st = r.status()
        self.assertEqual(st["state"], "off")
        self.assertNothingWritten(r)
        self.assertEqual((st["reads"], st["autopilot_fence"]), (1, "1 inclusion circle"))
        self.assertEqual([m.param_id for m in r.sent("PARAM_REQUEST_READ")], list(fu.REPORTED_PARAMS))


class WriteOff(RigCase):
    """fence_upload = false: RoboCommand's polygon is only looked for on the autopilot."""

    def test_nothing_is_written_even_to_an_empty_fence(self):
        r = self.rig(write=False)
        r.declare()
        r.run(50)
        st = r.status()
        self.assertEqual(st["state"], "off")
        self.assertIn("fence_upload is off", st["reason"])
        self.assertNothingWritten(r)
        self.assertEqual((st["verified"], st["reads"]), (False, 2))

    def test_robocommands_polygon_put_there_in_qgc_is_seen(self):
        r = self.rig(FakeAutopilot(fence=fence_items(OPEN) + fence_items(PILOT)), write=False)
        r.declare()
        r.run(5)
        self.assertTrue(r.status()["verified"])
        self.assertNothingWritten(r)


class NeverOverThePilot(RigCase):
    """The pilot's fence -- drawn in QGC inside RoboCommand's polygon -- is the polygon the bridge
    declares, so the write never replaces it (Chase, 7 Nov 2026)."""

    def test_the_pilots_polygon_alone_is_kept_and_robocommands_not_written(self):
        r = self.rig(FakeAutopilot(fence=fence_items(PILOT)))
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done", st)
        self.assertIn("NOT written: the autopilot holds a 5-vertex polygon inside RoboCommand's", st["reason"])
        self.assertNothingWritten(r)
        self.assertEqual((r.ap.uploads, st["verified"], st["backup"]), (0, False, None))
        self.assertHolds(r, PILOT)

    def test_drawn_by_hand_on_robocommands_line_it_is_still_the_pilots(self):
        r = self.rig(FakeAutopilot(fence=fence_items(reach(3.0))))
        r.declare()
        r.run(5)
        self.assertIn("NOT written", r.status()["reason"])
        self.assertNothingWritten(r)

    def test_robocommands_and_the_pilots_side_by_side_nothing_is_written(self):
        r = self.rig(FakeAutopilot(fence=fence_items(OPEN) + fence_items(PILOT)))
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done")
        self.assertIn("already holds RoboCommand's polygon and 1 polygon(s) beside it", st["reason"])
        self.assertTrue(st["verified"])
        self.assertNothingWritten(r)

    def test_the_pilots_polygon_added_after_the_write_is_never_touched(self):
        r = self.rig()
        r.declare()
        r.run(5)
        n = len(r.ap.heard)
        r.ap.fence = fence_items(OPEN) + fence_items(PILOT)        # drawn in QGC, both uploaded
        r.run(65)
        st = r.status()
        self.assertEqual((st["state"], st["verified"], r.ap.uploads), ("done", True, 1))
        self.assertEqual([len(s["points"]) for s in st["autopilot_shapes"]], [4, 5])
        self.assertLess(st["verified_age_s"], 31)
        self.assertNothingWritten(r, since=n)

    def test_one_that_sticks_out_is_not_the_pilots_for_this_course_and_is_replaced(self):
        r = self.rig(FakeAutopilot(fence=fence_items(reach(10.0))))
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done", st)
        self.assertHolds(r)
        self.assertEqual(len(json.loads(Path(st["backup"]).read_text(encoding="utf-8"))["items"]), 5)


# ======================================================================== the read

class Watch(RigCase):
    """Read every watch_s while disarmed: what the bridge declares from. Never a reason to write."""

    def test_read_every_30_s_and_the_confirmation_refreshed(self):
        r = self.rig()
        r.declare()
        r.run(5)
        self.assertEqual(r.status()["reads"], 2)                        # before the write, and the read-back
        n = len(r.ap.heard)
        r.run(60)
        st = r.status()
        self.assertEqual((st["state"], st["reads"]), ("done", 4))
        self.assertLess(st["verified_age_s"], 30)
        self.assertNothingWritten(r, since=n)

    def test_robocommands_polygon_taken_off_in_qgc_is_never_put_back(self):
        r = self.rig()
        r.declare()
        r.run(5)
        n = len(r.ap.heard)
        r.ap.fence = [dict(i) for i in CIRCLE]                          # the pilot uploads something else
        with self.assertLogs("fleet", "WARNING") as logs:
            r.run(70)
        self.assertTrue(any("no longer on the autopilot" in line for line in logs.output))
        st = r.status()
        self.assertEqual((st["state"], st["verified"], r.ap.uploads), ("done", False, 1))
        self.assertIn("no longer on the autopilot (it holds 1 inclusion circle)", st["reason"])
        self.assertEqual(r.ap.fence, CIRCLE)
        self.assertNothingWritten(r, since=n)

    def test_no_read_while_armed_and_an_unanswered_one_only_ages(self):
        r = self.rig()
        r.declare()
        r.run(5)
        n = len(r.ap.heard)
        r.armed = True
        r.run(120)
        self.assertEqual(len(r.ap.heard), n)                            # nothing said to an armed autopilot
        self.assertEqual(r.status()["state"], "done")
        r.armed = False
        r.ap.silent_missions = True
        r.run(80)
        st = r.status()
        self.assertEqual((st["state"], st["failures"], st["verified"]), ("done", 0, True))
        self.assertGreater(st["verified_age_s"], 190)                   # it just ages: the bridge judges it

    def test_a_read_refused_mid_qgc_upload_is_tried_again_in_seconds(self):
        a = FakeAutopilot()
        a.deny_downloads = 1
        r = self.rig(a)
        r.run(1)
        st = r.status()
        self.assertEqual((st["reads"], st["read_refusals"], st["failures"]), (0, 1, 0))
        r.run(5)
        self.assertEqual(r.status()["reads"], 1)


class WhatTheBridgeIsTold(RigCase):
    """confirmation(): the body of `ocs_fence` -- what the bridge declares the polygon and ceiling from."""

    PARAMS = {"FENCE_TYPE": 7.0, "FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 0.0, "FENCE_ALT_MAX": 30.0,
              "FENCE_OPTIONS": 0.0}

    def test_before_a_read_nothing_is_claimed(self):
        c = self.rig().up.confirmation()
        self.assertEqual((c["read_at"], c["shapes"], c["home_hae_m"], c["verified"]), (None, None, None, False))
        self.assertEqual(c["params"], dict.fromkeys(fu.REPORTED_PARAMS))

    def test_every_shape_the_params_and_home_hae_at_the_last_read(self):
        r = self.rig(FakeAutopilot(params=self.PARAMS, fence=fence_items(OPEN) + fence_items(PILOT)),
                     home_hae=lambda now: 31.2468)
        r.declare()
        r.run(5)
        c = r.up.confirmation()
        json.dumps(c, allow_nan=False)
        self.assertEqual((c["vehicle_id"], c["state"], c["verified"]), ("UAV1", "done", True))
        self.assertEqual([(s["type"], s["inclusion"], len(s["points"])) for s in c["shapes"]],
                         [("polygon", True, 4), ("polygon", True, 5)])
        self.assertEqual(c["params"], self.PARAMS)
        self.assertEqual(c["home_hae_m"], 31.25)
        self.assertTrue(1_790_000_000.0 <= c["read_at"] <= 1_790_000_005.0)

    def test_an_unknown_home_altitude_is_a_blank_not_a_guess(self):
        r = self.rig(FakeAutopilot(params=self.PARAMS), home_hae=lambda now: None)
        r.run(1)
        c = r.up.confirmation()
        self.assertIsNotNone(c["read_at"])
        self.assertEqual((c["shapes"], c["home_hae_m"], c["polygon"]), ([], None, None))


# ======================================================================== the rules

class NeverArmed(RigCase):
    def test_armed_nothing_is_sent_and_disarming_lets_it_through(self):
        r = self.rig()
        r.armed = True
        r.declare()
        r.run(10)
        self.assertEqual(r.ap.heard, [])
        self.assertIn("armed", r.status()["reason"])
        r.armed = False
        r.run(5)
        self.assertEqual(r.status()["state"], "done")

    def test_an_unknown_armed_state_is_not_disarmed(self):
        r = self.rig()
        r.armed = None
        r.declare()
        r.run(10)
        self.assertEqual(r.ap.heard, [])
        self.assertIn("armed state is unknown", r.status()["reason"])

    def test_no_route_on_the_radio_nothing_is_sent(self):
        r = self.rig()
        r.route_ok = False
        r.declare()
        r.run(10)
        self.assertEqual(r.ap.heard, [])

    def test_arming_mid_upload_stops_it_and_the_old_fence_stays(self):
        old = fence_items([(1.0, 103.0), (1.0, 103.1), (1.1, 103.1)])
        r = self.rig(FakeAutopilot(fence=old))

        def arm():
            r.armed = True
            r.ap.on_count = None
        r.ap.on_count = arm
        r.declare()
        r.run(5)
        self.assertEqual(r.sent("MISSION_ITEM_INT"), [])                 # not one vertex after the arm
        self.assertEqual(r.ap.fence, old)
        self.assertEqual(r.status()["failures"], 0)                       # not a failure, an interruption
        r.armed = False
        r.run(5)
        self.assertEqual(r.status()["state"], "done")
        self.assertHolds(r)


class NeverLockedOut(RigCase):
    def test_enabled_and_outside_it_is_held_and_nothing_is_written(self):
        r = self.rig(FakeAutopilot(params=ENABLED))
        r.declare()
        r.run(10)
        st = r.status()
        self.assertEqual(st["state"], "held")
        self.assertIn("FENCE_ENABLE 1", st["reason"])
        self.assertIn("outside it", st["reason"])
        self.assertNothingWritten(r)

    def test_held_until_the_operator_switches_it_off_in_qgc(self):
        r = self.rig(FakeAutopilot(params=ENABLED))
        r.declare()
        r.run(5)
        r.ap.set_param("FENCE_ENABLE", 0.0)            # broadcast, as ArduPilot does on a set
        r.run(5)
        self.assertEqual(r.status()["state"], "done")
        self.assertHolds(r)

    def test_held_asks_the_params_again_in_case_the_broadcast_was_lost(self):
        r = self.rig(FakeAutopilot(params=ENABLED))
        r.declare()
        r.run(5)
        r.ap.params["FENCE_ENABLE"] = 0.0              # changed, no broadcast heard
        r.run(20)
        self.assertEqual(r.status()["state"], "held")
        r.run(15)
        self.assertEqual(r.status()["state"], "done")

    def test_enabled_and_inside_it_is_written(self):
        r = self.rig(FakeAutopilot(params=ENABLED), position=INSIDE)
        r.declare()
        r.run(5)
        self.assertEqual(r.status()["state"], "done")

    def test_walking_inside_releases_a_hold(self):
        r = self.rig(FakeAutopilot(params=ENABLED))
        r.declare()
        r.run(5)
        r.position = INSIDE
        r.run(5)
        self.assertEqual(r.status()["state"], "done")

    def test_an_unknown_position_is_not_inside(self):
        r = self.rig(FakeAutopilot(params=ENABLED), position=None)
        r.declare()
        r.run(5)
        self.assertIn("position is unknown", r.status()["reason"])

    def test_auto_enable_counts_as_enabled(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 4.0, "FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 1.0}))
        r.declare()
        r.run(5)
        self.assertEqual(r.status()["state"], "held")
        self.assertIn("FENCE_AUTOENABLE 1", r.status()["reason"])

    def test_a_parameter_that_never_answers_counts_as_can_engage(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_ENABLE": 0.0}))
        r.declare()
        r.run(10)
        st = r.status()
        self.assertEqual(st["state"], "held")
        self.assertIn("FENCE_AUTOENABLE unanswered", st["reason"])
        asked = [m.param_id for m in r.sent("PARAM_REQUEST_READ")]
        self.assertEqual(asked.count("FENCE_AUTOENABLE"), fu.Timing().tries + 1)   # the gate's tries, then a read
        self.assertEqual(asked.count("FENCE_TYPE"), 1 + 1)                          # answered at once; the read
        r.position = INSIDE
        r.run(5)
        self.assertEqual(r.status()["state"], "done")

    def test_no_polygon_bit_the_polygon_cannot_engage_but_the_autopilot_nacks_it(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 1.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}),
                     timing=fu.Timing(retry_s=2.0))
        r.declare()
        r.run(1)
        st = r.status()
        self.assertEqual(st["failures"], 1)
        self.assertIn("rejected the fence: ERROR (1)", st["last_error"])
        self.assertIn("FENCE_TYPE 1 has no polygon bit", st["last_error"])
        r.run(30)
        st = r.status()
        self.assertEqual((st["state"], st["failures"]), ("gave up", 5))
        n = len(r.ap.heard)
        r.run(60)
        self.assertEqual(r.status()["state"], "gave up")
        self.assertIn("MISSION_REQUEST_LIST", r.ap.names()[n:])          # gave up: still read ...
        self.assertNothingWritten(r, since=n)                             # ... never written
        r.ap.set_param("FENCE_TYPE", 7.0)                                 # fixed in QGC: tried again --
        r.run(5)
        self.assertEqual(r.status()["state"], "held")                     # and now it CAN lock it out
        r.ap.set_param("FENCE_ENABLE", 0.0)
        r.run(5)
        self.assertEqual(r.status()["state"], "done")
        self.assertHolds(r)

    def test_a_fix_whose_broadcast_was_lost_is_caught_by_the_next_read(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 1.0, "FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 0.0}),
                     timing=fu.Timing(retry_s=2.0))
        r.declare()
        r.run(31)
        self.assertEqual(r.status()["state"], "gave up")
        r.ap.params["FENCE_TYPE"] = 7.0                                   # set in QGC, broadcast lost
        r.run(40)
        self.assertEqual(r.status()["state"], "done")
        self.assertHolds(r)


class MayBlockArming(RigCase):
    """fence_may_block_arming: the operator accepts "it will not arm here" -- and nothing more."""

    def test_a_hold_says_how_to_write_it_anyway(self):
        r = self.rig(FakeAutopilot(params=ENABLED))
        r.declare()
        r.run(5)
        self.assertEqual(r.status()["state"], "held")
        self.assertIn("fence_may_block_arming = true", r.status()["reason"])

    def test_over_an_enabled_fence_it_is_written_and_says_the_aircraft_will_not_arm(self):
        r = self.rig(FakeAutopilot(params=ENABLED), may_block_arming=True)
        r.declare()
        with self.assertLogs("fleet", "WARNING") as logs:
            r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done", st)
        self.assertHolds(r)
        self.assertIn("UAV1 will NOT ARM", st["lockout"])
        self.assertIn("FENCE_ENABLE 1", st["lockout"])
        self.assertTrue(any("will NOT ARM" in line for line in logs.output))
        self.assertEqual(r.ap.params, ENABLED)                           # still never a parameter written

    def test_it_does_not_cover_an_aircraft_that_could_still_arm_and_meet_the_fence_in_the_air(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 1.0}),
                     may_block_arming=True)
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "held")
        self.assertIn("does not cover this", st["reason"])
        self.assertIsNone(st["lockout"])
        self.assertNothingWritten(r)

    def test_it_does_not_cover_an_unanswered_fence_enable(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_AUTOENABLE": 0.0}), may_block_arming=True)
        r.declare()
        r.run(10)
        self.assertEqual(r.status()["state"], "held")
        self.assertIn("FENCE_ENABLE unanswered", r.status()["reason"])

    def test_inside_the_polygon_there_is_no_lockout_to_report(self):
        r = self.rig(FakeAutopilot(params=ENABLED), may_block_arming=True, position=INSIDE)
        r.declare()
        r.run(5)
        self.assertEqual((r.status()["state"], r.status()["lockout"]), ("done", None))


class ForTheMap(RigCase):
    def test_shapes_group_polygons_by_their_count_and_keep_circles(self):
        items = fence_items(OPEN) + [
            {"command": 5004, "param1": 20.0, "lat": 1.2805, "lon": 103.8556},
            {"command": 5000, "param1": 0.0, "lat": 1.2806, "lon": 103.8557},          # return point
            {"command": 5002, "param1": 3.0, "lat": 1.0, "lon": 103.0},                 # a group that is short
        ]
        got = fu.shapes([dict(i, seq=n) for n, i in enumerate(items)])
        self.assertEqual(got, [{"type": "polygon", "inclusion": True, "points": [list(p) for p in OPEN]},
                               {"type": "circle", "inclusion": False, "center": [1.2805, 103.8556],
                                "radius_m": 20.0}])

    def test_the_status_carries_the_declared_polygon_and_what_the_autopilot_holds(self):
        r = self.rig(FakeAutopilot(fence=CIRCLE))
        r.declare()
        self.assertEqual(r.status()["polygon"], [list(p) for p in OPEN])
        self.assertIsNone(r.status()["autopilot_shapes"])                  # never read: unknown, not empty
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done")
        (shape,) = st["autopilot_shapes"]                                  # the read-back, not the circle
        self.assertEqual((shape["type"], shape["inclusion"], len(shape["points"])), ("polygon", True, 4))
        for got, want in zip(shape["points"], OPEN):
            self.assertLess(fu.offset_m(tuple(got), want), 0.05)
        self.assertIsNotNone(st["autopilot_read_age_s"])


class LossyRadio(RigCase):
    def test_lost_frames_both_ways_are_re_sent_and_it_completes(self):
        a = FakeAutopilot()
        a.drop_replies.update({"PARAM_VALUE": 1, "MISSION_COUNT": 1, "MISSION_ITEM_INT": 2,
                               "MISSION_REQUEST_INT": 1})
        a.drop_ours.update({"MISSION_REQUEST_LIST": 1, "MISSION_ITEM_INT": 1, "MISSION_COUNT": 1})
        r = self.rig(a)
        r.declare()
        r.run(30)
        st = r.status()
        self.assertEqual((st["state"], st["failures"]), ("done", 0), st)
        self.assertHolds(r)
        self.assertFalse(+a.drop_replies, "every planned loss happened")
        self.assertFalse(+a.drop_ours, "every planned loss happened")

    def test_a_lost_upload_ack_writes_once_more_and_still_verifies(self):
        a = FakeAutopilot()
        a.drop_replies["MISSION_ACK"] = 1
        r = self.rig(a)
        r.declare()
        r.run(30)
        st = r.status()
        self.assertEqual((st["state"], st["failures"]), ("done", 0), st)
        self.assertEqual(a.uploads, 2)
        self.assertHolds(r)

    def test_a_silent_autopilot_fails_retries_and_gives_up_loudly(self):
        a = FakeAutopilot()
        a.silent_missions = True
        r = self.rig(a)
        r.declare()
        with self.assertLogs("fleet", "WARNING") as logs:
            r.run(5 * 30 + 30)
        st = r.status()
        self.assertEqual((st["state"], st["failures"], st["attempts"], st["reads"]), ("gave up", 5, 5, 0))
        self.assertIn("no answer during the fence download after 4 request(s)", st["last_error"])
        self.assertTrue(any("GAVE UP" in line for line in logs.output))
        self.assertGreaterEqual(len(r.sent("MISSION_REQUEST_LIST")), 5 * 4)    # and the reads' own between them

    def test_a_link_that_refuses_the_frame_is_counted_and_retried(self):
        r = self.rig()
        r.accept = False
        r.declare()
        r.run(2)
        self.assertGreater(r.status()["send_refused"], 0)
        r.accept = True
        r.run(40)
        self.assertEqual(r.status()["state"], "done")

    def test_a_readback_that_disagrees_is_a_failure_and_never_taken_for_the_pilots(self):
        a = FakeAutopilot()
        a.shift_m = 5.0
        r = self.rig(a)
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["failures"], 1)
        self.assertIn("read back wrong: the polygon just written is not on the autopilot (it holds 4 "
                      "inclusion-polygon vertices)", st["last_error"])
        r.run(35)               # the retry: the mangled polygon lies within 5 m of RoboCommand's, but it is ours
        st = r.status()
        self.assertEqual((st["failures"], a.uploads), (2, 2))


if __name__ == "__main__":
    unittest.main()
