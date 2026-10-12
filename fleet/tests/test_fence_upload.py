"""fence_upload: the declared geofence onto the autopilot, against a fake ArduPilot (helpers.FakeAutopilot)
that speaks the fence side of the mission protocol through pymavlink -- no radio, no sleeping.

What is pinned: the wire layouts (both directions, against pymavlink), Ekko's own rules carried
over (closing vertex stripped, param1 = vertex count, never armed, never a parameter written,
read back within 1 m), and what this adds -- read first and skip a fence already there, save the
old fence before replacing it (and refuse to replace it unsaved), never lock the aircraft out,
survive a lossy radio, give up loudly."""
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
#: bridge.toml's placeholder box at the Singapore test course (closed, as declared).
DECLARED = [(1.28010, 103.85520), (1.28010, 103.85625), (1.28110, 103.85625), (1.28110, 103.85520),
            (1.28010, 103.85520)]
OPEN = DECLARED[:-1]
INSIDE = (1.28060, 103.85570)
SARASOTA = (27.3364, -82.5307)
ALLOWED_TO_SEND = {"PARAM_REQUEST_READ", "MISSION_REQUEST_LIST", "MISSION_REQUEST_INT", "MISSION_ACK",
                   "MISSION_COUNT", "MISSION_ITEM_INT"}


def frame_of(raw: bytes) -> mavframe.Frame:
    (f,) = mavframe.split_datagram(DIALECT, raw)
    return f


def ours(msgid: int, payload: bytes) -> object:
    """What pymavlink makes of a frame built from one of fence_upload's payloads."""
    raw = mavframe.pack_frame(DIALECT, msgid, payload, sysid=250, compid=190, seq=7)
    return ap.MAVLink(None).decode(bytearray(raw))


class Rig:
    """One FenceUploader wired to one FakeAutopilot on a fake clock."""

    def __init__(self, autopilot=None, *, timing=fu.Timing(), backup_dir=None, position=SARASOTA,
                 may_block_arming=False):
        self.t = T0
        self.ap = autopilot or FakeAutopilot()
        self._tmp = tempfile.TemporaryDirectory()
        self.backup_dir = backup_dir if backup_dir is not None else str(Path(self._tmp.name) / "fence")
        self.up = fu.FenceUploader(label="UAV1", target_sysid=1, target_compid=1, our_sysid=250,
                                   send=self._send, armed=lambda now: self.armed,
                                   position=lambda now: self.position, route_ok=lambda: self.route_ok,
                                   backup_dir=self.backup_dir, may_block_arming=may_block_arming,
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

    def test_distance_is_right_across_an_ocean_and_the_dateline(self):
        san_diego = (32.7157, -117.1611)
        self.assertAlmostEqual(fu.offset_m(san_diego, OPEN[0]) / 1000.0, 14140, delta=150)
        self.assertAlmostEqual(fu.offset_m((0.0, 179.9), (0.0, -179.9)), 22239, delta=5)
        self.assertAlmostEqual(fu.offset_m(OPEN[0], (OPEN[0][0] + 1.0 / 111195.08, OPEN[0][1])), 1.0, places=3)

    def test_matches(self):
        items = [dict(i, seq=n) for n, i in enumerate(fence_items(OPEN))]
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


# ======================================================================== the dialog

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
        names = r.ap.names()
        self.assertEqual(names[:3], ["PARAM_REQUEST_READ"] * 3)
        self.assertEqual(names.count("MISSION_COUNT"), 1)
        self.assertEqual(names.count("MISSION_REQUEST_LIST"), 2)                 # read first, then read back
        self.assertIsNone(st["backup"])                                          # nothing was there to save

    def test_nothing_but_the_fence_dialog_and_parameter_READS_is_ever_sent(self):
        r = self.rig()
        r.declare()
        r.run(5)
        self.assertLessEqual(set(r.ap.names()), ALLOWED_TO_SEND)
        self.assertNotIn("PARAM_SET", r.ap.names())
        self.assertEqual(r.ap.params, {"FENCE_TYPE": 7.0, "FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 0.0})
        self.assertTrue(all(m.target_system == 1 for m in r.ap.heard))

    def test_a_fence_already_there_is_not_written_again(self):
        r = self.rig(FakeAutopilot(fence=fence_items(OPEN)))
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["state"], "done")
        self.assertIn("already holds this polygon", st["reason"])
        self.assertNotIn("MISSION_COUNT", r.ap.names())
        self.assertEqual(r.ap.uploads, 0)

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
        self.assertNotIn("MISSION_COUNT", r.ap.names())

    def test_one_write_per_polygon_and_a_new_polygon_starts_over(self):
        r = self.rig()
        r.declare()
        r.run(5)
        n = len(r.ap.heard)
        r.declare()                                       # the bridge re-sends ocs_info on every reconnect
        r.run(60)
        later = r.ap.names()[n:]
        self.assertNotIn("MISSION_COUNT", later)          # re-read (the 60 s re-check), never re-written
        self.assertLessEqual(set(later), {"MISSION_REQUEST_LIST", "MISSION_REQUEST_INT", "MISSION_ACK"})
        self.assertEqual(r.ap.uploads, 1)
        bigger =[(1.2800, 103.8551), (1.2800, 103.8564), (1.2812, 103.8564), (1.2812, 103.8551)]
        r.declare(bigger)
        r.run(5)
        self.assertEqual(r.status()["state"], "done")
        self.assertHolds(r, bigger)
        self.assertEqual(r.ap.uploads, 2)

    def test_a_withdrawn_polygon_stops_and_leaves_the_autopilots_fence_alone(self):
        r = self.rig(FakeAutopilot(fence=fence_items(OPEN)))
        r.armed = True
        r.declare()
        r.run(2)
        r.up.set_polygon(None, r.t)
        r.armed = False
        r.run(10)
        self.assertEqual(r.status()["state"], "off")
        self.assertEqual(r.ap.heard, [])
        self.assertEqual(len(r.ap.fence), 4)

    def test_nothing_happens_before_a_polygon_arrives(self):
        r = self.rig()
        r.run(10)
        self.assertEqual((r.status()["state"], r.ap.heard), ("off", []))


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
    def test_enabled_and_outside_it_is_held_before_anything_is_read_or_written(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}))
        r.declare()
        r.run(10)
        st = r.status()
        self.assertEqual(st["state"], "held")
        self.assertIn("FENCE_ENABLE 1", st["reason"])
        self.assertIn("outside it", st["reason"])
        self.assertEqual(set(r.ap.names()), {"PARAM_REQUEST_READ"})

    def test_held_until_the_operator_switches_it_off_in_qgc(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}))
        r.declare()
        r.run(5)
        r.ap.set_param("FENCE_ENABLE", 0.0)            # broadcast, as ArduPilot does on a set
        r.run(5)
        self.assertEqual(r.status()["state"], "done")
        self.assertHolds(r)

    def test_held_asks_the_params_again_in_case_the_broadcast_was_lost(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}))
        r.declare()
        r.run(5)
        r.ap.params["FENCE_ENABLE"] = 0.0              # changed, no broadcast heard
        r.run(20)
        self.assertEqual(r.status()["state"], "held")
        r.run(15)
        self.assertEqual(r.status()["state"], "done")

    def test_enabled_and_inside_it_is_written(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}),
                     position=INSIDE)
        r.declare()
        r.run(5)
        self.assertEqual(r.status()["state"], "done")

    def test_walking_inside_releases_a_hold(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}))
        r.declare()
        r.run(5)
        r.position = INSIDE
        r.run(5)
        self.assertEqual(r.status()["state"], "done")

    def test_an_unknown_position_is_not_inside(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 7.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}),
                     position=None)
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
        self.assertEqual(len(r.sent("PARAM_REQUEST_READ")), 3 + 3)       # 3 once, then the silent one 3 more times
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
        self.assertEqual(len(r.ap.heard), n)                              # gave up means quiet
        r.ap.set_param("FENCE_TYPE", 7.0)                                 # fixed in QGC: tried again --
        r.run(5)
        self.assertEqual(r.status()["state"], "held")                     # and now it CAN lock it out
        r.ap.set_param("FENCE_ENABLE", 0.0)
        r.run(5)
        self.assertEqual(r.status()["state"], "done")
        self.assertHolds(r)


ENABLED = {"FENCE_TYPE": 7.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0}


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
        self.assertNotIn("MISSION_COUNT", r.ap.names())

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
        circle = [{"command": 5003, "param1": 35.0, "lat": 32.92357, "lon": -117.03857}]
        r = self.rig(FakeAutopilot(fence=circle))
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


CIRCLE = [{"command": 5003, "param1": 35.0, "lat": 32.92357, "lon": -117.03857}]


class NoCourseClears(RigCase):
    """set_clear: the bridge has no course, so the autopilot is left with NO fence items --
    the bench case, flying without a RoboCommand server."""

    def test_after_the_grace_the_fence_is_saved_cleared_and_read_back_empty(self):
        r = self.rig(FakeAutopilot(fence=fence_items(OPEN)))
        r.up.set_clear(r.t)
        r.run(19)
        self.assertNotIn("MISSION_COUNT", r.ap.names())                 # the grace: a course may still come
        r.run(6)
        st = r.status()
        self.assertEqual(st["state"], "done", st)
        self.assertEqual(st["reason"].split(".")[0], "cleared and read back empty")
        self.assertEqual(r.ap.fence, [])
        self.assertEqual([m.count for m in r.sent("MISSION_COUNT")], [0])
        saved = json.loads(Path(st["backup"]).read_text(encoding="utf-8"))
        self.assertEqual(len(saved["items"]), 4)
        c = r.up.confirmation()
        self.assertEqual((c["polygon"], c["verified"], c["state"]), ([], True, "done"))

    def test_an_already_empty_fence_is_left_alone(self):
        r = self.rig()
        r.up.set_clear(r.t)
        r.run(30)
        self.assertIn("nothing to clear", r.status()["reason"])
        self.assertNotIn("MISSION_COUNT", r.ap.names())

    def test_a_course_inside_the_grace_cancels_the_clear(self):
        r = self.rig(FakeAutopilot(fence=fence_items(OPEN)))
        r.up.set_clear(r.t)
        r.run(5)
        r.declare()
        r.run(30)
        self.assertEqual(r.status()["state"], "done")
        self.assertNotIn("MISSION_COUNT", r.ap.names())                 # it already held this course's fence
        self.assertHolds(r)

    def test_clearing_is_never_held_by_the_lock_out_rule(self):
        r = self.rig(FakeAutopilot(params=ENABLED, fence=fence_items(OPEN)))
        r.up.set_clear(r.t)
        r.run(30)
        self.assertEqual((r.status()["state"], r.ap.fence), ("done", []))
        self.assertIsNone(r.status()["lockout"])

    def test_a_polygons_only_fence_type_says_it_will_not_arm_cleared(self):
        r = self.rig(FakeAutopilot(params={"FENCE_TYPE": 4.0, "FENCE_ENABLE": 1.0, "FENCE_AUTOENABLE": 0.0},
                                   fence=fence_items(OPEN)))
        r.up.set_clear(r.t)
        r.run(30)
        self.assertIn("Fences enabled, but none selected", r.status()["lockout"])

    def test_still_never_while_armed(self):
        r = self.rig(FakeAutopilot(fence=fence_items(OPEN)))
        r.armed = True
        r.up.set_clear(r.t)
        r.run(60)
        self.assertEqual(r.ap.heard, [])
        self.assertEqual(len(r.ap.fence), 4)


class Recheck(RigCase):
    """Done is re-read every 60 s while disarmed: the confirmation never ages past that, and a
    fence changed behind our back is put back."""

    def test_the_confirmation_is_refreshed_by_a_read_every_minute(self):
        r = self.rig()
        r.declare()
        r.run(5)
        self.assertLess(r.status()["verified_age_s"], 5)
        r.run(50)
        self.assertGreater(r.status()["verified_age_s"], 49)
        r.run(10)
        st = r.status()
        self.assertLess(st["verified_age_s"], 10)
        self.assertEqual((st["state"], st["rechecks"], r.ap.uploads), ("done", 1, 1))

    def test_a_fence_changed_in_qgc_is_saved_and_the_declared_one_put_back(self):
        r = self.rig()
        r.declare()
        r.run(5)
        r.ap.fence = [dict(i) for i in CIRCLE]                          # someone uploads a circle
        with self.assertLogs("fleet", "WARNING") as logs:
            r.run(70)
        self.assertTrue(any("changed since it was verified" in line for line in logs.output))
        st = r.status()
        self.assertEqual(st["state"], "done")
        self.assertHolds(r)
        self.assertEqual(r.ap.uploads, 2)
        saved = json.loads(Path(st["backup"]).read_text(encoding="utf-8"))
        self.assertEqual(saved["items"][0]["command"], 5003)

    def test_no_recheck_while_armed_and_a_silent_one_only_ages(self):
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
        self.assertEqual((st["state"], st["failures"]), ("done", 0))    # an unanswered read is not a failure
        self.assertGreater(st["verified_age_s"], 190)                   # it just ages: the bridge judges it


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
        self.assertEqual((st["state"], st["failures"], st["attempts"]), ("gave up", 5, 5))
        self.assertIn("no answer during the fence download after 4 request(s)", st["last_error"])
        self.assertTrue(any("GAVE UP" in line for line in logs.output))
        self.assertEqual(len(r.sent("MISSION_REQUEST_LIST")), 5 * 4)

    def test_a_link_that_refuses_the_frame_is_counted_and_retried(self):
        r = self.rig()
        r.accept = False
        r.declare()
        r.run(2)
        self.assertGreater(r.status()["send_refused"], 0)
        r.accept = True
        r.run(40)
        self.assertEqual(r.status()["state"], "done")

    def test_a_readback_that_disagrees_is_a_failure(self):
        a = FakeAutopilot()
        a.shift_m = 5.0
        r = self.rig(a)
        r.declare()
        r.run(5)
        st = r.status()
        self.assertEqual(st["failures"], 1)
        self.assertIn("read back wrong: vertex 0 is 5.0 m", st["last_error"])


if __name__ == "__main__":
    unittest.main()
