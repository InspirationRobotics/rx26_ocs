"""The UAV geofence through the Bridge: per course, "table, else the boundary"; told to the
aircraft in `ocs_info`; confirmed back in `ocs_fence`; and the gate that refuses to
declare an aircraft whose autopilot is not KNOWN to hold it.

The pure geometry and rule are tests/test_geofence.py. Everything here is the wiring:
what the page snapshot says, which frames go to which vehicle, what a declaration carries,
and that the declare endpoint refuses on its own account and not just the button.

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

from rx_bridge import geofence                                   # noqa: E402
from rx_bridge.proto import rx_requests_pb2                      # noqa: E402

from tests.harness import (BOX, ROBONATION_TEST_COURSE, BridgeCase, course_bytes,   # noqa: E402
                           fence_frame, wait_for)

#: A course nowhere near BOX, for the "a different course" cases.
ELSEWHERE = [(1.2900, 103.8600), (1.2900, 103.8610), (1.2910, 103.8610), (1.2910, 103.8600)]

#: A fence for ALPHA drawn by hand, comfortably inside BOX.
TABLE_ALPHA = ((1.2802, 103.8554), (1.2802, 103.8560), (1.2808, 103.8560), (1.2802, 103.8554))


def infos(client):
    return [f["ocs_info"] for f in client.of_kind("ocs_info")]


def table(cid: str, pts) -> str:
    return ("\n[geofence]\n[[geofence.course]]\ncourse_id = \"%s\"\nuav_geofence = %s\n"
            % (cid, [list(p) for p in pts]))


class FenceCase(BridgeCase):
    """BridgeCase plus the few moves every test here makes."""

    def course(self, cid="ALPHA", corners=None):
        """RoboCommand's course arrives."""
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes(cid, corners=corners))

    def snap(self) -> dict:
        return json.loads(json.dumps(self.bridge.snapshot()))     # the page gets JSON

    def fence_pts(self):
        """The current fence, as the snapshot gives it: [(lat, lon), ...] closed."""
        return [(p["latitude"], p["longitude"]) for p in self.snap()["uav_geofence"]]

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


# ---- where the fence comes from -----------------------------------------------------

class TestTableEntry(FenceCase):
    geofence = table("ALPHA", TABLE_ALPHA)

    def test_a_table_entry_for_the_course_is_the_fence(self):
        self.course("ALPHA")
        s = self.snap()
        self.assertEqual(self.fence_pts(), list(TABLE_ALPHA))
        self.assertEqual(s["uav_geofence_source"], "table: ALPHA")
        self.assertIsNone(s["uav_geofence_reason"])
        self.assertEqual(s["course_findings"], [])

    def test_another_course_does_not_use_it(self):
        self.course("BRAVO")
        self.assertEqual(self.snap()["uav_geofence_source"], "course boundary inset 2.0 m")

    def test_the_declaration_carries_exactly_the_table_fence_closed(self):
        self.declared()
        got = [(p["latitude"], p["longitude"]) for p in self.sent_declaration()["uav_geofence"]]
        self.assertEqual(got, list(TABLE_ALPHA))
        self.assertEqual(got[0], got[-1])


class TestTableEntryOutside(FenceCase):
    """A hand-written fence for the wrong place: refused loudly, and the inset that would
    have worked is NOT quietly used instead."""

    geofence = table("ALPHA", [(1.30, 103.90), (1.30, 103.91), (1.31, 103.91), (1.30, 103.90)])

    def test_no_fence_and_a_finding_that_names_the_vertex(self):
        self.course("ALPHA")
        s = self.snap()
        self.assertIsNone(s["uav_geofence"], "no fallback to the boundary inset")
        self.assertIsNone(s["uav_geofence_source"])
        self.assertIn("outside the course boundary", s["uav_geofence_reason"])
        self.assertIn("vertex 1", s["uav_geofence_reason"])
        finding = next(f for f in s["course_findings"] if f.startswith("UAV geofence:"))
        self.assertIn("vertex 1", finding)
        self.assertIn("ALPHA", finding)
        self.assertTrue(any("COURSE SUSPECT" in m and "vertex 1" in m for m in self.logs))

    def test_the_aircraft_is_told_there_is_a_course_and_no_fence(self):
        """`course: {...}, uav_geofence: null`: the adapter must NOT clear anything on this."""
        uav = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(uav)))
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 2))
        last = infos(uav)[-1]
        self.assertEqual(last["course"]["course_id"], "ALPHA")
        self.assertIn("uav_geofence", last)
        self.assertIsNone(last["uav_geofence"])

    def test_an_aircraft_cannot_be_declared_without_one(self):
        self.course("ALPHA")
        self.only("USV1", "UAV1")
        self.confirm_fence("UAV1", [[1.2802, 103.8554], [1.2802, 103.8560], [1.2808, 103.8560]])
        msg = self.bridge.declare()
        self.assertTrue(msg.startswith("cannot declare"), msg)
        self.assertIn("UAV1: no UAV geofence for this course", msg)
        self.assertEqual(self.requests_sent(), 0)


class TestInset(FenceCase):

    def test_the_boundary_inset_is_the_default(self):
        self.course("ALPHA", ROBONATION_TEST_COURSE)
        s = self.snap()
        self.assertEqual(len(s["uav_geofence"]), 5, "4 vertices, closed")
        self.assertEqual(self.fence_pts()[0], self.fence_pts()[-1])
        self.assertEqual(s["uav_geofence_source"], "course boundary inset 2.0 m")
        self.assertEqual(s["course_findings"], [])

    def test_the_declaration_carries_the_inset_closed(self):
        self.declared()
        got = self.sent_declaration()["uav_geofence"]
        self.assertEqual(got[0], got[-1])
        self.assertEqual([(p["latitude"], p["longitude"]) for p in got], self.fence_pts())


class TestInsetRefused(FenceCase):

    def test_a_boundary_that_is_not_convex_is_a_finding_and_no_fence(self):
        ell = [(1.2801, 103.8552), (1.2801, 103.8562), (1.2805, 103.8562),
               (1.2805, 103.8556), (1.2811, 103.8556), (1.2811, 103.8552)]
        self.course("ALPHA", ell)
        s = self.snap()
        self.assertIsNone(s["uav_geofence"])
        self.assertIn("not convex", s["uav_geofence_reason"])
        self.assertTrue(any("not convex" in f and "[[geofence.course]] entry for ALPHA" in f
                            for f in s["course_findings"]), s["course_findings"])

    def test_a_course_with_no_corners_is_a_finding_and_no_fence(self):
        self.course("ALPHA", [])
        s = self.snap()
        self.assertIsNone(s["uav_geofence"])
        self.assertTrue(any(f.startswith("UAV geofence:") for f in s["course_findings"]))


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

    def test_no_course_no_fence_and_the_page_says_why(self):
        s = self.snap()
        self.assertIsNone(s["uav_geofence"])
        self.assertIsNone(s["uav_geofence_source"])
        self.assertEqual(s["uav_geofence_reason"], "no course received yet")
        self.assertFalse(s["run"]["may_declare"])

    def test_an_aircraft_cannot_be_declared_without_a_course(self):
        self.only("UAV1")
        self.bridge.machine.on_connect()
        why = self.bridge._uav_gate(("UAV1",), time.time())
        self.assertEqual(len(why), 1)
        self.assertIn("UAV1: no UAV geofence", why[0])
        self.assertIn("no course received yet", why[0])
        self.assertTrue(self.bridge.declare().startswith("cannot declare"))

    def test_a_connecting_aircraft_is_told_both_keys_null(self):
        uav = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(uav)))
        self.assertEqual(infos(uav)[0], {"course": None, "uav_geofence": None})


class TestNewCourse(FenceCase):

    def test_a_new_course_recomputes_the_fence_and_resends_ocs_info_with_it(self):
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
        self.assertEqual([(p["latitude"], p["longitude"]) for p in new["uav_geofence"]],
                         self.fence_pts())
        self.assertTrue(1.2900 < new["uav_geofence"][0]["latitude"] < 1.2910, "BRAVO's, not ALPHA's")
        self.assertNotIn("uav_geofence", infos(usv)[-1])

    def test_a_confirmation_for_the_old_course_no_longer_counts(self):
        self.course("ALPHA")
        self.only("USV1", "UAV1")
        self.confirm_fence("UAV1")
        self.assertIsNone(self.bridge.declare_blocker())
        self.bridge._handle_course(course_bytes("BRAVO", corners=ELSEWHERE))
        self.assertFalse(self.snap()["fence_confirm"]["UAV1"]["matches_course_fence"])
        self.assertIn("different polygon than the BRAVO fence", self.bridge.declare_blocker())

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

    def test_a_broker_drop_leaves_the_course_and_the_fence_in_place(self):
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
        self.assertEqual(len(infos(uav)), n, "nothing was sent, least of all a null fence")

    def test_an_aircraft_that_connects_during_the_drop_is_told_the_course_and_the_fence(self):
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

    def test_it_is_stored_per_vehicle_shown_with_its_age_and_matches_flag(self):
        self.course("ALPHA")
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", self.bridge._fence.open_polygon, state="done", at=time.time() - 12)
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
        c = self.snap()["fence_confirm"]["UAV1"]
        self.assertTrue(c["verified"])
        self.assertEqual(c["state"], "done")
        self.assertEqual(c["polygon"], [list(p) for p in self.bridge._fence.open_polygon])
        self.assertTrue(c["matches_course_fence"])
        self.assertAlmostEqual(c["age_s"], 12, delta=2)
        self.assertEqual(c["vehicle_id"], "UAV1")
        self.assertIsNone(c["lockout"])
        self.assertEqual(self.snap()["fence_confirm_max_age_s"], 180.0)

    def test_the_newest_one_wins(self):
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", None, verified=False, state="waiting", reason="aircraft is armed")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
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
        uav.fence("UAV1", self.bridge._fence.open_polygon)
        uav.fence("UAV1", self.bridge._fence.open_polygon)
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
        self.assertTrue(wait_for(lambda: self.snap()["counters"]["fence_frames_ignored"] == 2))
        self.assertEqual(self.snap()["fence_confirm"], {})
        self.assertFalse(uav.closed.wait(0.2))

    def test_a_cleared_confirmation_is_shown_as_an_empty_polygon(self):
        uav = self.connect("UAV1", "radio")
        uav.fence("UAV1", [], reason="cleared and read back empty")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
        c = self.snap()["fence_confirm"]["UAV1"]
        self.assertEqual(c["polygon"], [])
        self.assertTrue(c["verified"])
        self.assertFalse(c["matches_course_fence"])


class TestAdapterEndToEnd(FenceCase):
    """The stand-in answers ocs_info the way fleet_link does, so the OCS and the adapter
    meet through the real socket."""

    def test_a_fence_is_told_confirmed_and_the_declaration_goes_through(self):
        self.adapter("UAV1")
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"].get("UAV1", {})
                                 .get("matches_course_fence")))
        self.only("USV1", "UAV1")
        self.assertIn("declared", self.bridge.declare())
        self.assertEqual([(p["latitude"], p["longitude"]) for p in self.sent_declaration()["uav_geofence"]],
                         self.fence_pts())

    def test_no_course_is_answered_with_a_cleared_confirmation(self):
        self.adapter("UAV1")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]))
        c = self.snap()["fence_confirm"]["UAV1"]
        self.assertEqual((c["polygon"], c["verified"]), ([], True))
        self.assertFalse(c["matches_course_fence"])

    def test_a_refused_fence_gets_no_answer_so_the_old_one_stands(self):
        self.adapter("UAV1")
        self.course("ALPHA", [])                                  # a course, and no fence for it
        time.sleep(0.3)
        self.assertEqual(self.snap()["fence_confirm"].get("UAV1", {}).get("polygon"), [],
                         "still the cleared answer from before the course")


class TestFakeVehicleAdapter(FenceCase):
    """`fake_vehicle.py --ocs-tcp`: the bench's UAV stand-in for the adapter's half."""

    def test_it_clears_with_no_course_then_confirms_the_fence_and_the_gate_opens(self):
        import fake_vehicle
        fake_vehicle.print = lambda *a, **k: None               # its thread talks; the suite need not hear
        self.addCleanup(lambda: fake_vehicle.__dict__.pop("print", None))
        ad = fake_vehicle.FenceAdapter("UAV1", "127.0.0.1", self.bridge.tcp.bound_port)
        ad.start()
        self.addCleanup(ad.halt.set)

        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"].get("UAV1", {}).get("polygon") == []),
                        "no course: it answers 'cleared'")
        self.course("ALPHA")
        self.assertTrue(wait_for(lambda: self.snap()["fence_confirm"]["UAV1"]["matches_course_fence"]))
        self.only("USV1", "UAV1")
        self.assertIn("declared", self.bridge.declare())


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

    def test_no_confirmation_at_all(self):
        self.refused("UAV1: autopilot fence not confirmed")

    def test_unverified(self):
        self.confirm_fence("UAV1", verified=False, state="held", reason="armed: not writing")
        self.refused("UAV1: autopilot fence not confirmed (state held: armed: not writing)")

    def test_a_different_polygon(self):
        shifted = [[lat + 0.0001, lon] for lat, lon in self.bridge._fence.open_polygon]
        self.confirm_fence("UAV1", shifted)
        self.refused("UAV1: autopilot holds a different polygon than the ALPHA fence")

    def test_cleared_is_not_the_fence(self):
        self.confirm_fence("UAV1", [], reason="cleared and read back empty")
        self.refused("different polygon", "cleared")

    def test_stale(self):
        self.confirm_fence("UAV1", at=time.time() - 412)
        self.assertRegex(self.refused("UAV1: fence confirmation"), r"41[1-4] s old \(limit 180 s\)")

    def test_fresh_enough_is_accepted(self):
        self.confirm_fence("UAV1", at=time.time() - 170)
        self.assertIn("declared", self.bridge.declare())

    def test_good_is_accepted_and_the_declaration_carries_the_fence_closed(self):
        self.confirm_fence("UAV1")
        self.assertIsNone(self.bridge.declare_blocker())
        self.assertTrue(self.snap()["run"]["may_declare"])
        self.assertIn("declared", self.bridge.declare())
        got = self.sent_declaration()
        self.assertEqual(got["vehicle_ids"], ["USV1", "UAV1"])
        self.assertEqual([(p["latitude"], p["longitude"]) for p in got["uav_geofence"]],
                         self.fence_pts())
        self.assertEqual(got["uav_geofence"][0], got["uav_geofence"][-1])

    def test_a_declaration_without_an_aircraft_needs_no_confirmation(self):
        self.only("USV1")
        self.assertIn("declared", self.bridge.declare())

    def test_an_aircraft_forced_out_is_not_gated(self):
        self.only("USV1", "UUV1")
        self.assertIsNone(self.bridge.declare_blocker())

    def test_the_boat_and_the_sub_are_not_gated(self):
        """No confirmation from either, and the gate has nothing to say about them."""
        self.assertEqual(self.bridge._uav_gate(("USV1", "UUV1"), time.time()), [])

    def test_state_and_floor_still_come_first(self):
        self.bridge.machine.on_disconnect()
        self.assertIn("cannot declare in DISCONNECTED", self.bridge.declare())
        self.bridge.machine.on_connect()

    def test_the_declare_endpoint_refuses_on_its_own_account(self):
        """Not just the button: the POST the page makes -- and anything else that makes
        it -- gets the refusal, whatever the page thought it could press."""
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

        run = call("GET", "/api/state")["run"]
        self.assertFalse(run["may_declare"])
        self.assertIn("UAV1: autopilot fence not confirmed", run["declare_blocker"])
        got = call("POST", "/api/declare", b"{}")
        self.assertIn("UAV1: autopilot fence not confirmed", got["result"])
        self.assertEqual(self.requests_sent(), 0)

        self.confirm_fence("UAV1")
        self.assertTrue(call("GET", "/api/state")["run"]["may_declare"])
        self.assertIn("declared", call("POST", "/api/declare", b"{}")["result"])
        self.assertEqual(self.requests_sent(), 1)


class TestGateLimit(FenceCase):
    geofence = "\n[geofence]\nconfirm_max_age_s = 30\n"

    def test_the_age_limit_is_the_configured_one(self):
        self.course("ALPHA")
        self.only("USV1", "UAV1")
        self.confirm_fence("UAV1", at=time.time() - 40)
        self.assertRegex(self.bridge.declare(), r"4[0-2] s old \(limit 30 s\)")
        self.confirm_fence("UAV1", at=time.time() - 20)
        self.assertIn("declared", self.bridge.declare())


class TestStatusLine(FenceCase):
    def test_the_console_status_names_the_fence_and_each_confirmation(self):
        self.assertIn("uav fence  none: no course received yet", self.bridge.status())
        self.course("ALPHA")
        self.confirm_fence("UAV1")
        line = next(l for l in self.bridge.status().splitlines() if l.startswith("uav fence"))
        self.assertIn("course boundary inset 2.0 m, 4 vertices", line)
        self.assertIn("UAV1 verified, holds the fence", line)


if __name__ == "__main__":
    unittest.main()
