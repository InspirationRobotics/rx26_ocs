"""The UAV geofence through the Bridge, as Chase set the workflow on 7 Nov 2026:

  * RoboCommand's course arrives and the OCS makes RoboCommand's polygon from it (the course
    and transit area, inset), tells the aircraft in `ocs_info`, and fleet_link puts it on the
    autopilot for the pilot to draw inside;
  * the pilot draws Ekko's own fence inside it in QGC;
  * fleet_link reads the autopilot back and sends `ocs_fence`: every shape it holds, the
    FENCE_* parameters, home HAE;
  * a declaration carries the polygon the pilot drew and the ceiling the aircraft enforces
    (home HAE + FENCE_ALT_MAX), both as read -- or is refused in words, before RoboCommand
    could refuse it silently.

The pure geometry and rules are tests/test_geofence.py. Everything here is the wiring: what
the page snapshot says, which frames go to which vehicle, what a declaration carries, and
that the declare endpoint refuses on its own account and not just the button.

Vehicles are real sockets speaking the real framing (tests/harness.py); the radio adapter
is played by `VehicleClient.adapter_for`, which answers ocs_info the way fleet_link does.
"""
from __future__ import annotations

import http.client
import json
import sys
import threading
import time
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from google.protobuf import json_format                          # noqa: E402

from rx_bridge.proto import rx_requests_pb2                      # noqa: E402

from tests.harness import (CEILING_HAE_M, PARAMS, PILOT, ROBONATION_TEST_COURSE,   # noqa: E402
                           BridgeCase, course_bytes, poly_shape, wait_for)

#: A course nowhere near BOX, for the "a different course" cases.
ELSEWHERE = [(1.2900, 103.8600), (1.2900, 103.8610), (1.2910, 103.8610), (1.2910, 103.8600)]

#: PILOT closed, as a declaration carries it.
PILOT_CLOSED = [*PILOT, PILOT[0]]

#: A second polygon of the pilot's, also inside BOX.
SMALL = [(1.28040, 103.85550), (1.28040, 103.85570), (1.28060, 103.85570), (1.28060, 103.85550)]

#: PILOT with its east side pushed out past BOX's east edge (103.85625) by ~11 m.
OUTSIDE = [(1.28030, 103.85540), (1.28030, 103.85635), (1.28060, 103.85635),
           (1.28090, 103.85580), (1.28080, 103.85545)]

#: An L-shaped course: no RoboCommand polygon can be made for it (not convex).
ELL = [(1.2801, 103.8552), (1.2801, 103.8562), (1.2805, 103.8562),
       (1.2805, 103.8556), (1.2811, 103.8556), (1.2811, 103.8552)]

#: A fence drawn inside ELL's lower arm.
IN_ELL = [(1.28015, 103.8553), (1.28015, 103.8560), (1.28045, 103.8560), (1.28045, 103.8553)]


def infos(client):
    return [f["ocs_info"] for f in client.of_kind("ocs_info")]


def latlons(points):
    return [(p["latitude"], p["longitude"]) for p in points]


class FenceCase(BridgeCase):
    """BridgeCase plus the few moves every test here makes."""

    def course(self, cid="ALPHA", corners=None):
        """RoboCommand's course arrives."""
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes(cid, corners=corners))

    def snap(self) -> dict:
        return json.loads(json.dumps(self.bridge.snapshot()))     # the page gets JSON

    def rc_pts(self):
        """RoboCommand's polygon, as the snapshot gives it: [(lat, lon), ...] closed."""
        return latlons(self.snap()["uav_geofence"])

    def rc_open(self):
        return [list(p) for p in self.bridge._fence.open_polygon]

    def only(self, *ids):
        for vid in self.bridge.cfg.vehicle_ids:
            self.bridge.set_active(vid, vid in ids)

    def adapter(self, vid="UAV1", link="radio"):
        """A radio-adapter stand-in on a real socket, answering every ocs_info from
        its very first (it is armed BEFORE the hello, so the link-up one is answered)."""
        c = self.client()
        c.adapter_for(vid)
        c.hello(vid, link, "fleet_link")
        self.assertTrue(wait_for(lambda: link in self.bridge.tcp.links_of(vid)))
        return c

    def sent_declaration(self):
        req = rx_requests_pb2.RxRequest()
        req.ParseFromString(next(p for t, p, _ in self.bridge.rc.published
                                 if t.endswith("/request")))
        return json_format.MessageToDict(req.run_declaration, preserving_proto_field_name=True,
                                         always_print_fields_with_no_presence=True)

    def requests_sent(self) -> int:
        return sum(1 for t, _, _ in self.bridge.rc.published if t.endswith("/request"))


# ---- RoboCommand's polygon: made from the course, put on the aircraft, never declared ----

class TestRoboCommandsPolygon(FenceCase):

    def test_the_boundary_inset_is_robocommands_polygon(self):
        self.course("ALPHA", ROBONATION_TEST_COURSE)
        s = self.snap()
        self.assertEqual(len(s["uav_geofence"]), 5, "4 vertices, closed")
        self.assertEqual(self.rc_pts()[0], self.rc_pts()[-1])
        self.assertEqual(s["uav_geofence_source"], "course boundary inset 2.0 m")
        self.assertEqual(s["course_findings"], [])

    def test_it_is_what_the_aircraft_is_told(self):
        uav = self.connect("UAV1", "radio")
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 2))
        self.assertEqual(latlons(infos(uav)[-1]["uav_geofence"]), self.rc_pts())

    def test_it_is_never_what_is_declared(self):
        self.declared()
        got = latlons(self.sent_declaration()["uav_geofence"])
        self.assertEqual(got, PILOT_CLOSED, "the pilot's polygon, read off the autopilot")
        self.assertNotEqual(got, self.rc_pts())


class TestNoRoboCommandPolygon(FenceCase):
    """A course no polygon can be made for: a finding, nothing put on the aircraft -- and the
    pilot can still draw one in QGC against the course, which is what is declared."""

    def test_a_boundary_that_is_not_convex_is_a_finding_and_no_polygon(self):
        self.course("ALPHA", ELL)
        s = self.snap()
        self.assertIsNone(s["uav_geofence"])
        self.assertIn("not convex", s["uav_geofence_reason"])
        self.assertTrue(any("not convex" in f and "draw Ekko's fence in QGC" in f
                            for f in s["course_findings"]), s["course_findings"])

    def test_a_course_with_no_corners_is_a_finding_and_no_polygon(self):
        self.course("ALPHA", [])
        s = self.snap()
        self.assertIsNone(s["uav_geofence"])
        self.assertTrue(any(f.startswith("UAV geofence:") for f in s["course_findings"]))

    def test_the_aircraft_is_told_there_is_a_course_and_no_polygon(self):
        """`course: {...}, uav_geofence: null`: the adapter must not touch the autopilot."""
        uav = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(uav)))
        self.course("ALPHA", ELL)
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 2))
        last = infos(uav)[-1]
        self.assertEqual(last["course"]["course_id"], "ALPHA")
        self.assertIn("uav_geofence", last)
        self.assertIsNone(last["uav_geofence"])

    def test_what_the_pilot_drew_against_the_course_is_still_declared(self):
        self.course("ALPHA", ELL)
        self.only("USV1", "UAV1")
        self.confirm_fence("UAV1", None, held=[IN_ELL])
        self.assertIn("declared", self.bridge.declare())
        self.assertEqual(latlons(self.sent_declaration()["uav_geofence"]), [*IN_ELL, IN_ELL[0]])

    def test_and_judged_against_the_course_by_robocommands_rule(self):
        self.course("ALPHA", ELL)
        self.only("USV1", "UAV1")
        # The top-right corner of the L's missing square: inside the bounding box, not the course.
        notch = [(1.2807, 103.8558), (1.2807, 103.8561), (1.2810, 103.8561), (1.2810, 103.8558)]
        self.confirm_fence("UAV1", None, held=[notch])
        msg = self.bridge.declare()
        self.assertIn("UAV1: uav_geofence leaves the course and transit area by", msg)
        self.assertIn("(the polygon drawn in QGC)", msg)


class TestMarginTooBig(FenceCase):
    geofence = "\n[geofence]\nboundary_margin_m = 100\n"

    def test_a_margin_that_collapses_the_course_is_refused_with_the_figures(self):
        self.course("ALPHA")
        s = self.snap()
        self.assertIsNone(s["uav_geofence"])
        self.assertRegex(s["uav_geofence_reason"],
                         r"margin of 100\.0 m collapses a course \d+\.\d m wide")
        self.assertTrue(any("boundary_margin_m" in f for f in s["course_findings"]))


class TestMarginFromConfig(FenceCase):
    geofence = "\n[geofence]\nboundary_margin_m = 5\n"

    def test_the_margin_is_the_configured_one(self):
        self.course("ALPHA")
        self.assertEqual(self.snap()["uav_geofence_source"], "course boundary inset 5.0 m")


# ---- no course, a new course, a dropped link ----------------------------------------

class TestNoCourse(FenceCase):

    def test_no_course_no_polygon_and_the_page_says_why(self):
        s = self.snap()
        self.assertIsNone(s["uav_geofence"])
        self.assertIsNone(s["uav_geofence_source"])
        self.assertEqual(s["uav_geofence_reason"], "no course received yet")
        self.assertFalse(s["run"]["may_declare"])

    def test_nothing_can_be_declared_without_a_course(self):
        self.only("UAV1")
        self.bridge.machine.on_connect()
        self.confirm_fence("UAV1", None)
        self.assertTrue(self.bridge.declare().startswith("cannot declare in CONNECTED"))

    def test_a_connecting_aircraft_is_told_both_keys_null(self):
        uav = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(uav)))
        self.assertEqual(infos(uav)[0], {"course": None, "uav_geofence": None})


class TestNewCourse(FenceCase):

    def test_a_new_course_recomputes_the_polygon_and_resends_ocs_info_with_it(self):
        usv, uav = self.connect("USV1", None), self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(usv) and infos(uav)))
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 2 and len(infos(usv)) == 2))
        first = infos(uav)[-1]["uav_geofence"]
        self.assertEqual(first[0], first[-1])

        self.bridge._handle_course(course_bytes("BRAVO", corners=ELSEWHERE))
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 3 and len(infos(usv)) == 3))
        new = infos(uav)[-1]
        self.assertEqual(new["course"]["course_id"], "BRAVO")
        self.assertNotEqual(new["uav_geofence"], first)
        self.assertEqual(latlons(new["uav_geofence"]), self.rc_pts())
        self.assertTrue(1.2900 < new["uav_geofence"][0]["latitude"] < 1.2910, "BRAVO's, not ALPHA's")
        self.assertNotIn("uav_geofence", infos(usv)[-1])

    def test_the_pilots_polygon_for_the_old_course_is_judged_against_the_new_one(self):
        self.course("ALPHA")
        self.only("USV1", "UAV1")
        self.confirm_fence("UAV1")
        self.assertIsNone(self.bridge.declare_blocker())
        self.bridge._handle_course(course_bytes("BRAVO", corners=ELSEWHERE))
        c = self.snap()["fence_confirm"]["UAV1"]
        self.assertFalse(c["matches_course_fence"], "the adapter still has ALPHA's polygon")
        blocker = self.bridge.declare_blocker()
        self.assertIn("UAV1: uav_geofence leaves the course and transit area by", blocker)
        self.assertNotIn("RoboCommand's polygon", blocker,
                         "ALPHA's polygon, which the OCS put there, is never taken for the pilot's")

    def test_the_same_course_again_changes_nothing_and_resends_nothing(self):
        uav = self.connect("UAV1", "radio")
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 2))
        before = self.snap()["uav_geofence"]
        self.bridge._handle_course(course_bytes("ALPHA"))
        time.sleep(0.15)
        self.assertEqual(len(infos(uav)), 2)
        self.assertEqual(self.snap()["uav_geofence"], before)


class _Sub:
    """Just enough of a paho client for the connect callback."""
    def subscribe(self, *_a, **_kw):
        pass


class TestCoursePersists(FenceCase):
    """The course is what RoboCommand LAST told us. A dropped link does not un-tell us."""

    def test_a_broker_drop_leaves_the_course_and_the_polygon_in_place(self):
        uav = self.connect("UAV1", "radio")
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 2))
        before = self.snap()
        n = len(infos(uav))

        self.bridge._on_rc_disconnect(None, None, None, 7)
        during = self.snap()
        self.assertEqual(during["course"], before["course"])
        self.assertEqual(during["uav_geofence"], before["uav_geofence"])
        self.assertEqual(during["uav_geofence_source"], before["uav_geofence_source"])
        self.assertFalse(during["links"]["robocommand"])

        self.bridge._on_rc_connect(_Sub(), None, None, 0)
        self.bridge._handle_course(course_bytes("ALPHA"))          # the retained one, redelivered
        after = self.snap()
        self.assertEqual(after["course"]["course_id"], "ALPHA")
        self.assertEqual(after["uav_geofence"], before["uav_geofence"])
        time.sleep(0.15)
        self.assertEqual(len(infos(uav)), n, "nothing was sent, least of all a null polygon")

    def test_an_aircraft_that_connects_during_the_drop_is_told_the_course_and_the_polygon(self):
        self.course("ALPHA")
        want = self.snap()["uav_geofence"]
        self.bridge._on_rc_disconnect(None, None, None, 7)
        late = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(late)))
        self.assertEqual(infos(late)[0]["course"]["course_id"], "ALPHA")
        self.assertEqual(infos(late)[0]["uav_geofence"], want)

    def test_only_a_newer_course_replaces_it(self):
        self.course("ALPHA")
        self.bridge._on_rc_disconnect(None, None, None, 7)
        self.bridge._on_rc_connect(_Sub(), None, None, 0)
        self.bridge._handle_course(course_bytes("DELTA", corners=ELSEWHERE))
        s = self.snap()
        self.assertEqual(s["course"]["course_id"], "DELTA")
        self.assertTrue(1.2900 < s["uav_geofence"][0]["latitude"] < 1.2910)


# ---- the ocs_fence frame ------------------------------------------------------------

class TestOcsFenceFrame(FenceCase):

    def test_it_is_stored_per_vehicle_and_shown_with_both_ages_and_the_matches_flag(self):
        self.course("ALPHA")
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", self.rc_open(), at=time.time() - 12, read_at=time.time() - 5)
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
        c = self.snap()["fence_confirm"]["UAV1"]
        self.assertTrue(c["verified"])
        self.assertEqual(c["state"], "done")
        self.assertEqual(c["polygon"], self.rc_open())
        self.assertTrue(c["matches_course_fence"])
        self.assertAlmostEqual(c["age_s"], 5, delta=2)
        self.assertAlmostEqual(c["verified_age_s"], 12, delta=2)
        self.assertEqual(c["vehicle_id"], "UAV1")
        self.assertIsNone(c["lockout"])
        self.assertEqual([s["points"] for s in c["shapes"]], [self.rc_open(), [list(p) for p in PILOT]])
        self.assertEqual(c["params"], PARAMS)
        self.assertEqual(c["home_hae_m"], 9.0)
        self.assertEqual(self.snap()["fence_confirm_max_age_s"], 180.0)

    def test_the_newest_one_wins(self):
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", None, held=None, state="waiting", reason="aircraft is armed")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
        self.assertIsNone(self.snap()["fence_confirm"]["UAV1"]["shapes"])
        uav.fence("UAV1", [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]], state="done")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]["UAV1"]["verified"]))
        self.assertEqual(self.snap()["fence_confirm"]["UAV1"]["state"], "done")

    def test_a_lockout_is_carried_through(self):
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]],
                  lockout="the aircraft will not arm where it is")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
        self.assertEqual(self.snap()["fence_confirm"]["UAV1"]["lockout"],
                         "the aircraft will not arm where it is")

    def test_it_is_never_forwarded_to_robocommand_and_burns_no_sequence_number(self):
        self.declared()
        published = list(self.bridge.rc.published)
        forwarded = self.bridge.counters.forwarded
        seq = self.bridge.seqs.snapshot()
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", self.rc_open())
        uav.fence("UAV1", self.rc_open())
        self.assertTrue(wait_for(lambda: sum(1 for l in self.wire_lines()
                                             if l.get("note", "").startswith("ocs_fence")) >= 3))
        self.assertEqual(self.bridge.rc.published, published)
        self.assertEqual(self.bridge.counters.forwarded, forwarded)
        self.assertEqual(self.bridge.seqs.snapshot(), seq)
        self.assertEqual(self.bridge.team.published, [])

    def test_the_wire_log_records_it_like_any_other_vehicle_frame(self):
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]], state="done")
        self.assertTrue(wait_for(lambda: any(l.get("note", "").startswith("ocs_fence")
                                             for l in self.wire_lines())))
        line = next(l for l in self.wire_lines() if l.get("note", "").startswith("ocs_fence"))
        self.assertEqual(line["dir"], "veh")
        self.assertEqual(line["topic"], "tcp://in/radio")
        self.assertEqual(line["decoded"]["ocs_fence"]["vehicle_id"], "UAV1")
        self.assertTrue(line["b64"])
        self.assertIn("verified=True", line["note"])
        self.assertIn("shapes=2", line["note"])

    def test_the_console_hears_of_a_change_once(self):
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", None, read_at=time.time() - 30)
        uav.fence("UAV1", None)                         # a fresh read, the same fence
        self.assertTrue(wait_for(lambda: sum(1 for l in self.wire_lines()
                                             if l.get("note", "").startswith("ocs_fence")) == 2))
        said = [m for m in self.logs if m.startswith("fence read UAV1")]
        self.assertEqual(len(said), 1, said)
        self.assertIn("1 shape(s), FENCE_ALT_MAX 30 m", said[0])

    def test_a_body_naming_another_vehicle_is_ignored_counted_and_logged(self):
        usv = self.connect("USV1", None)
        usv.fence("UAV1", [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]])
        self.assertTrue(wait_for(lambda: self.snap()["counters"]["fence_frames_ignored"] == 1))
        self.assertEqual(self.snap()["fence_confirm"], {})
        self.assertTrue(any("ocs_fence IGNORED" in m and "names UAV1" in m for m in self.logs))
        self.assertTrue(any(l.get("kind") == "fence_ignored" for l in self.wire_lines()))
        self.assertFalse(usv.closed.wait(0.2), "the socket stays up")

    def test_a_socket_that_has_not_said_who_it_is_cannot_vouch_for_an_aircraft(self):
        c = self.client()
        c.fence("UAV1", [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]])
        self.assertTrue(wait_for(lambda: self.snap()["counters"]["fence_frames_ignored"] == 1))
        self.assertEqual(self.snap()["fence_confirm"], {})

    def test_only_an_aircraft_has_a_fence(self):
        usv = self.connect("USV1", None)
        usv.fence("USV1", [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]])
        self.assertTrue(wait_for(lambda: self.snap()["counters"]["fence_frames_ignored"] == 1))
        self.assertEqual(self.snap()["fence_confirm"], {})

    def test_a_malformed_body_is_ignored(self):
        uav = self.connect("UAV1", "radio")
        uav.send({"ocs_fence": {"vehicle_id": "UAV1", "verified": "yes", "polygon": None}})
        uav.send({"ocs_fence": "nonsense"})
        uav.send({"ocs_fence": {"vehicle_id": "UAV1", "verified": False,
                                "shapes": [{"type": "polygon", "inclusion": True,
                                            "points": [[1.0, "x"], [1, 2], [2, 2]]}]}})
        uav.send({"ocs_fence": {"vehicle_id": "UAV1", "verified": False,
                                "params": {"FENCE_ALT_MAX": "30"}}})
        self.assertTrue(wait_for(lambda: self.snap()["counters"]["fence_frames_ignored"] == 4))
        self.assertEqual(self.snap()["fence_confirm"], {})
        self.assertFalse(uav.closed.wait(0.2))

    def test_a_read_that_found_nothing_is_not_a_read_that_never_happened(self):
        self.course("ALPHA")
        self.only("USV1", "UAV1")
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", None, held=[])
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
        c = self.snap()["fence_confirm"]["UAV1"]
        self.assertEqual(c["shapes"], [])
        self.assertIsNotNone(c["read_at"])
        self.assertIn("UAV1: its autopilot holds no inclusion polygon -- draw Ekko's own fence "
                      "inside the course in QGC", self.bridge.declare_blocker())


class TestAdapterEndToEnd(FenceCase):
    """The stand-in answers ocs_info the way fleet_link does, so the OCS and the adapter
    meet through the real socket."""

    def test_robocommands_polygon_is_told_and_read_back_and_the_pilots_is_declared(self):
        self.adapter("UAV1")
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"].get("UAV1", {})
                                 .get("matches_course_fence")))
        self.only("USV1", "UAV1")
        self.assertIn("declared", self.bridge.declare())
        decl = self.sent_declaration()
        self.assertEqual(latlons(decl["uav_geofence"]), PILOT_CLOSED)
        self.assertEqual(decl["uav_geofence_max_altitude_hae_m"], CEILING_HAE_M)
        self.assertEqual(decl["uav_pilot_callsign"], "Test UA0000")

    def test_no_course_is_answered_with_a_read_and_nothing_is_cleared(self):
        self.adapter("UAV1")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
        c = self.snap()["fence_confirm"]["UAV1"]
        self.assertIsNone(c["polygon"])
        self.assertFalse(c["verified"])
        self.assertEqual(c["shapes"], [poly_shape(PILOT)], "the pilot's fence stands")

    def test_a_refused_polygon_is_answered_with_none_and_the_pilots_fence_stands(self):
        self.adapter("UAV1")
        self.course("ALPHA", ELL)                                 # a course, and no polygon for it
        self.assertTrue(wait_for(lambda: len(self.snap()["fence_confirm"]) == 1))
        time.sleep(0.2)
        c = self.snap()["fence_confirm"]["UAV1"]
        self.assertIsNone(c["polygon"])
        self.assertEqual(c["shapes"], [poly_shape(PILOT)])


class TestFakeVehicleAdapter(FenceCase):
    """`fake_vehicle.py --ocs-tcp`: the bench's UAV stand-in for the adapter's half."""

    def test_it_reads_nothing_without_a_course_then_the_pilots_polygon_inside_robocommands(self):
        import fake_vehicle
        fake_vehicle.print = lambda *a, **k: None               # its thread talks; the suite need not hear
        self.addCleanup(lambda: fake_vehicle.__dict__.pop("print", None))
        ad = fake_vehicle.FenceAdapter("UAV1", "127.0.0.1", self.bridge.tcp.bound_port)
        ad.start()
        self.addCleanup(ad.halt.set)

        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"].get("UAV1", {}).get("shapes") == []),
                        "no course: an empty autopilot, read")
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]["UAV1"]["matches_course_fence"]
                                 and self.snap()["fence_confirm"]["UAV1"]["verified"]))
        self.only("USV1", "UAV1")
        self.assertIn("declared", self.bridge.declare())
        decl = self.sent_declaration()
        got = latlons(decl["uav_geofence"])
        self.assertEqual(len(got), 5, "the stand-in pilot's polygon, 4 vertices, closed")
        self.assertNotEqual(got, self.rc_pts())
        self.assertEqual(decl["uav_geofence_max_altitude_hae_m"],
                         fake_vehicle.HOME_HAE_M + fake_vehicle.PARAMS["FENCE_ALT_MAX"])


# ---- the gate -----------------------------------------------------------------------

class TestGate(FenceCase):

    def setUp(self):
        super().setUp()
        self.course("ALPHA")
        self.only("USV1", "UAV1")

    def refused(self, *parts):
        msg = self.bridge.declare()
        self.assertTrue(msg.startswith("cannot declare"), msg)
        for part in parts:
            self.assertIn(part, msg)
        self.assertEqual(self.requests_sent(), 0, "nothing reached RoboCommand")
        self.assertEqual(self.bridge.machine.state.name, "COURSE_RX")
        s = self.snap()["run"]
        self.assertFalse(s["may_declare"])
        self.assertEqual(s["declare_blocker"], msg, "the page shows the same words")
        return msg

    # -- the read

    def test_nothing_read_at_all(self):
        self.refused("UAV1: nothing read from its autopilot yet (no ocs_fence: is fleet_link running")

    def test_never_read(self):
        self.confirm_fence("UAV1", held=None, state="waiting", reason="aircraft is armed")
        self.refused("UAV1: its autopilot's fence has not been read yet (aircraft is armed)")

    def test_stale(self):
        self.confirm_fence("UAV1", read_at=time.time() - 412)
        self.assertRegex(self.refused("UAV1: its autopilot's fence was last read"),
                         r"last read 41[1-4] s ago \(limit 180 s\)")

    def test_fresh_enough_is_accepted(self):
        self.confirm_fence("UAV1", read_at=time.time() - 170)
        self.assertIn("declared", self.bridge.declare())

    def test_dated_in_the_future(self):
        self.confirm_fence("UAV1", read_at=time.time() + 60)
        self.assertRegex(self.refused("UAV1: the fence read is dated"), r"dated (59|60|61) s in the future")

    # -- the polygon

    def test_only_robocommands_polygon(self):
        self.confirm_fence("UAV1", held=[self.rc_open()])
        self.refused("UAV1: its autopilot holds only RoboCommand's polygon -- draw Ekko's own fence "
                     "inside RoboCommand's in QGC and upload it")

    def test_no_inclusion_polygon(self):
        self.confirm_fence("UAV1", held=[])
        self.refused("UAV1: its autopilot holds no inclusion polygon")

    def test_two_of_the_pilots(self):
        self.confirm_fence("UAV1", held=[self.rc_open(), PILOT, SMALL])
        self.refused("UAV1: its autopilot holds 2 inclusion polygons besides RoboCommand's, and a "
                     "declaration carries one -- leave only Ekko's own in QGC")

    def test_an_exclusion_polygon_and_a_circle_are_not_the_pilots_polygon(self):
        circle = {"type": "circle", "inclusion": True, "center": [1.2806, 103.8557], "radius_m": 20.0}
        self.confirm_fence("UAV1", held=[self.rc_open(), PILOT, poly_shape(SMALL, inclusion=False),
                                         circle])
        self.assertIn("declared", self.bridge.declare())
        self.assertEqual(latlons(self.sent_declaration()["uav_geofence"]), PILOT_CLOSED)
        notes = self.snap()["uav"]["declaration"]["UAV1"]["notes"]
        self.assertIn("its autopilot also holds 1 circle and 1 exclusion polygon: enforced by it, "
                      "never declared", notes)

    def test_robocommands_polygon_need_not_stay_on_it(self):
        self.confirm_fence("UAV1", held=[PILOT])
        self.assertIn("declared", self.bridge.declare())
        self.assertEqual(latlons(self.sent_declaration()["uav_geofence"]), PILOT_CLOSED)

    def test_a_polygon_outside_the_course(self):
        self.confirm_fence("UAV1", held=[self.rc_open(), OUTSIDE])
        msg = self.refused("UAV1: uav_geofence leaves the course and transit area by",
                           "(the polygon drawn in QGC)")
        self.assertRegex(msg, r"by 1[01]\.\d m at")

    # -- the ceiling

    def test_fence_alt_max_not_read(self):
        self.confirm_fence("UAV1", params={k: v for k, v in PARAMS.items() if k != "FENCE_ALT_MAX"})
        msg = self.refused("UAV1: FENCE_ALT_MAX has not been read from its autopilot")
        self.assertNotIn("is not set", msg, "one reason, not two")

    def test_home_altitude_unknown(self):
        self.confirm_fence("UAV1", home_hae_m=None)
        self.refused("UAV1: its home altitude is unknown")

    def test_a_ceiling_above_robocommands_limit(self):
        self.confirm_fence("UAV1", params=dict(PARAMS, FENCE_ALT_MAX=45.0))
        self.refused("UAV1: uav_geofence_max_altitude_hae_m is 54: it must be above 0 and at most 50 "
                     "(home 9.0 m HAE + FENCE_ALT_MAX 45.0 m; lower FENCE_ALT_MAX in QGC)")

    def test_a_ceiling_at_or_below_the_ellipsoid(self):
        self.confirm_fence("UAV1", home_hae_m=-40.0)
        msg = self.refused("UAV1: uav_geofence_max_altitude_hae_m is -10: it must be above 0",
                           "(home -40.0 m HAE + FENCE_ALT_MAX 30.0 m)")
        self.assertNotIn("lower FENCE_ALT_MAX", msg)

    # -- the call sign

    def test_a_blank_call_sign(self):
        self.confirm_fence("UAV1")
        self.assertTrue(self.bridge.set_operator(callsign="")[0])
        self.refused("the UAV pilot's call sign is blank: type it next to Declare")

    def test_a_call_sign_given_with_the_declaration_is_the_one_declared(self):
        self.confirm_fence("UAV1")
        self.assertIn("declared", self.bridge.declare("  Chase   UA1234 "))
        self.assertEqual(self.sent_declaration()["uav_pilot_callsign"], "Chase UA1234")
        self.assertEqual(self.snap()["operator"]["callsign"]["from"], "typed")

    def test_a_call_sign_that_cannot_be_sent_refuses_the_declaration(self):
        self.confirm_fence("UAV1")
        msg = self.bridge.declare("Chase\x07UA1234")
        self.assertEqual(msg, "cannot declare -- the pilot's call sign has a character in it "
                              "that cannot be sent")
        self.assertEqual(self.requests_sent(), 0)

    # -- what goes out

    def test_good_is_accepted_and_carries_the_pilots_polygon_the_ceiling_and_the_call_sign(self):
        self.confirm_fence("UAV1")
        self.assertIsNone(self.bridge.declare_blocker())
        self.assertTrue(self.snap()["run"]["may_declare"])
        self.assertIn("declared", self.bridge.declare())
        got = self.sent_declaration()
        self.assertEqual(got["vehicle_ids"], ["USV1", "UAV1"])
        self.assertEqual(latlons(got["uav_geofence"]), PILOT_CLOSED)
        self.assertEqual(got["uav_geofence_max_altitude_hae_m"], CEILING_HAE_M)
        self.assertEqual(got["uav_pilot_callsign"], "Test UA0000")
        said = next(m for m in self.logs if m.startswith("declared with"))
        self.assertIn("UAV1: the 5-vertex polygon and ceiling 39 m HAE (home 9.0 m HAE + "
                      "FENCE_ALT_MAX 30.0 m) read off its autopilot", said)
        self.assertTrue(any(l.get("kind") == "declare_uav" for l in self.wire_lines()))

    def test_the_snapshot_shows_what_would_be_declared(self):
        self.confirm_fence("UAV1", params=dict(PARAMS, FENCE_TYPE=0.0))
        d = self.snap()["uav"]["declaration"]["UAV1"]
        self.assertEqual(latlons(d["polygon"]), PILOT_CLOSED)
        self.assertEqual((d["ceiling_hae_m"], d["ceiling_from"]),
                         (CEILING_HAE_M, "home 9.0 m HAE + FENCE_ALT_MAX 30.0 m"))
        self.assertEqual(d["problems"], [])
        self.assertTrue(any("no polygon bit" in n for n in d["notes"]), d["notes"])
        self.assertTrue(any("no altitude bit" in n for n in d["notes"]), d["notes"])
        self.assertIn("declared", self.bridge.declare(), "a note never blocks")

    def test_a_declaration_without_an_aircraft_needs_no_read(self):
        self.only("USV1")
        self.assertIn("declared", self.bridge.declare())
        got = self.sent_declaration()
        self.assertEqual(got["uav_geofence"], [])
        self.assertNotIn("uav_pilot_callsign", got)

    def test_an_aircraft_forced_out_is_not_gated(self):
        self.only("USV1", "UUV1")
        self.assertIsNone(self.bridge.declare_blocker())

    def test_the_boat_and_the_sub_are_not_gated(self):
        """Nothing read from either, and the gate has nothing to say about them."""
        self.assertEqual(self.bridge._uav_rules(("USV1", "UUV1"), time.time()), ([], None))

    def test_state_and_floor_still_come_first(self):
        self.bridge.machine.on_disconnect()
        self.assertIn("cannot declare in DISCONNECTED", self.bridge.declare())
        self.bridge.machine.on_connect()

    def test_the_declare_endpoint_refuses_on_its_own_account(self):
        """Not just the button: the POST the page makes -- and anything else that makes
        it -- gets the refusal, whatever the page thought it could press."""
        call = self.http()
        run = call("GET", "/api/state")["run"]
        self.assertFalse(run["may_declare"])
        self.assertIn("UAV1: nothing read from its autopilot yet", run["declare_blocker"])
        got = call("POST", "/api/declare", b"{}")
        self.assertIn("UAV1: nothing read from its autopilot yet", got["result"])
        self.assertEqual(self.requests_sent(), 0)

        self.confirm_fence("UAV1")
        self.assertTrue(call("GET", "/api/state")["run"]["may_declare"])
        self.assertIn("declared", call("POST", "/api/declare",
                                       b'{"callsign": "Chris UA5678"}')["result"])
        self.assertEqual(self.requests_sent(), 1)
        self.assertEqual(self.sent_declaration()["uav_pilot_callsign"], "Chris UA5678")

    def http(self):
        """ocs_web on an ephemeral port, in front of this bridge: call(method, path, body)."""
        from ocs_web import server
        srv = ThreadingHTTPServer(("127.0.0.1", 0), server.make_handler(self.bridge, None, []))
        threading.Thread(target=srv.serve_forever, args=(0.01,), daemon=True).start()
        self.addCleanup(srv.server_close)
        self.addCleanup(srv.shutdown)

        def call(method, path, body=b""):
            conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
            conn.request(method, path, body=body, headers={"Content-Length": str(len(body))})
            out = json.loads(conn.getresponse().read())
            conn.close()
            return out
        return call


class TestGateLimit(FenceCase):
    geofence = "\n[geofence]\nconfirm_max_age_s = 30\n"

    def test_the_age_limit_is_the_configured_one(self):
        self.course("ALPHA")
        self.only("USV1", "UAV1")
        self.confirm_fence("UAV1", read_at=time.time() - 40)
        self.assertRegex(self.bridge.declare(), r"last read 4[0-2] s ago \(limit 30 s\)")
        self.confirm_fence("UAV1", read_at=time.time() - 20)
        self.assertIn("declared", self.bridge.declare())


class TestCeilingLimit(FenceCase):
    geofence = "\n[geofence]\nceiling_limit_hae_m = 35\n"

    def test_the_ceiling_limit_is_the_configured_one(self):
        self.course("ALPHA")
        self.only("USV1", "UAV1")
        self.confirm_fence("UAV1")
        self.assertIn("uav_geofence_max_altitude_hae_m is 39: it must be above 0 and at most 35",
                      self.bridge.declare())
        self.assertEqual(self.snap()["uav"]["ceiling_limit_hae_m"], 35.0)


class TestStatusLine(FenceCase):
    def test_the_console_status_names_robocommands_polygon_and_each_aircraft(self):
        self.assertIn("uav fence  RoboCommand's: none (no course received yet)", self.bridge.status())
        self.course("ALPHA")
        self.confirm_fence("UAV1")
        line = next(l for l in self.bridge.status().splitlines() if l.startswith("uav fence"))
        self.assertIn("RoboCommand's course boundary inset 2.0 m, 4 vertices", line)
        self.assertIn("UAV1 read 0 s ago, RoboCommand's on it, declares 5 vertices to 39 m HAE", line)

    def test_and_what_stops_it(self):
        self.course("ALPHA")
        self.confirm_fence("UAV1", held=[self.rc_open()])
        line = next(l for l in self.bridge.status().splitlines() if l.startswith("uav fence"))
        self.assertIn("UAV1 read 0 s ago, RoboCommand's on it, CANNOT declare: UAV1: its autopilot "
                      "holds only RoboCommand's polygon", line)


if __name__ == "__main__":
    unittest.main()
