"""`ocs_info`: the course (every vehicle) and RoboCommand's UAV polygon for it (aircraft
only, for fleet_link to put on the autopilot for the pilot to draw inside), sent as
information to each connected vehicle -- on connect, when RoboCommand's course arrives or
changes, and (the polygon) after a declaration.

Nothing in the frame is a command; the OCS does not control a vehicle in autonomous
mode (handbook 5.3) and these tests include the check that it does not even try.
"""
from __future__ import annotations

import json
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from google.protobuf import json_format                          # noqa: E402

from rx_bridge.proto import rx_requests_pb2                      # noqa: E402

from tests.harness import BridgeCase, course_bytes, wait_for     # noqa: E402


def infos(client):
    return [f["ocs_info"] for f in client.of_kind("ocs_info")]


class TestOcsInfo(BridgeCase):

    def informed(self, vid, link):
        """Connect, and wait for the snapshot a connecting vehicle is always sent, so
        what a test counts afterwards is only what it caused."""
        c = self.connect(vid, link)
        self.assertTrue(wait_for(lambda: infos(c)), "no ocs_info on connect")
        return c

    def test_a_vehicle_that_connects_is_told_what_the_ocs_knows(self):
        """Before any course: course is null, said so rather than omitted."""
        usv = self.connect("USV1", None)
        self.assertTrue(wait_for(lambda: infos(usv)))
        self.assertEqual(infos(usv)[0], {"course": None})

    def test_a_uav_also_gets_the_geofence_and_nobody_else_does(self):
        usv = self.connect("USV1", None)
        uav = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(usv) and infos(uav)))
        # No course yet: an aircraft is told so with BOTH keys explicit, never by
        # omission. {course: null, uav_geofence: null} is what tells the radio
        # adapter there is nothing to hold (it clears the autopilot's polygon).
        self.assertEqual(infos(uav)[0], {"course": None, "uav_geofence": None})
        self.assertEqual(infos(usv)[0], {"course": None})
        n_uav, n_usv = len(infos(uav)), len(infos(usv))
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        self.assertTrue(wait_for(lambda: len(infos(uav)) > n_uav and len(infos(usv)) > n_usv))
        fence = infos(uav)[-1]["uav_geofence"]
        self.assertEqual(len(fence), len(self.bridge._fence.polygon))
        self.assertEqual(fence[0], fence[-1], "closed, as RunDeclaration needs it")
        self.assertNotIn("uav_geofence", infos(usv)[-1])

    def test_robocommands_polygon_has_runDeclarations_shape_but_is_not_what_is_declared(self):
        """The shape of RunDeclaration.uav_geofence -- compared against a declaration
        RoboCommand is actually sent -- but never its content: that is the pilot's polygon,
        read off the aircraft."""
        uav = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(uav)))
        self.declared(("UAV1",))
        self.assertTrue(wait_for(lambda: len(infos(uav)) >= 3))       # connect, course, declaration
        req = rx_requests_pb2.RxRequest()
        req.ParseFromString(next(p for t, p, _ in self.bridge.rc.published if t.endswith("/request")))
        decl = json_format.MessageToDict(req.run_declaration, preserving_proto_field_name=True,
                                         always_print_fields_with_no_presence=True)
        told = infos(uav)[-1]["uav_geofence"]
        self.assertEqual(told, infos(uav)[1]["uav_geofence"], "the declaration changed nothing in it")
        self.assertEqual(set(told[0]), set(decl["uav_geofence"][0]))
        self.assertEqual(set(told[0]), {"latitude", "longitude"})
        self.assertEqual([(p["latitude"], p["longitude"]) for p in told],
                         list(self.bridge._fence.polygon))
        self.assertNotEqual(told, decl["uav_geofence"])

    def test_a_received_course_goes_to_every_connected_vehicle(self):
        usv = self.informed("USV1", None)
        uav = self.informed("UAV1", "radio")
        n_usv, n_uav = len(infos(usv)), len(infos(uav))
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes("ALPHA", 25000))
        self.assertTrue(wait_for(lambda: len(infos(usv)) > n_usv and len(infos(uav)) > n_uav))
        for got in (infos(usv)[-1], infos(uav)[-1]):
            self.assertEqual(got["course"]["course_id"], "ALPHA")
            self.assertEqual(got["course"]["pinger_freq_hz"], 25000)
            self.assertEqual(len(got["course"]["corners"]), 4)
            self.assertEqual(set(got["course"]["corners"][0]), {"latitude", "longitude"})
        self.assertNotIn("uav_geofence", infos(usv)[-1])
        self.assertIn("uav_geofence", infos(uav)[-1])

    def test_a_vehicle_that_connects_after_the_course_is_told_it_at_once(self):
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes("BRAVO", 30000))
        usv = self.connect("USV1", None)
        self.assertTrue(wait_for(lambda: infos(usv)))
        self.assertEqual(infos(usv)[0]["course"]["course_id"], "BRAVO")

    def test_the_same_course_delivered_again_is_not_resent(self):
        """RoboCommand re-delivers its retained course on every reconnect of OUR link."""
        usv = self.informed("USV1", None)
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        self.assertTrue(wait_for(lambda: len(infos(usv)) == 2))
        self.bridge._handle_course(course_bytes())
        time.sleep(0.15)
        self.assertEqual(len(infos(usv)), 2)

    def test_a_changed_course_is_resent(self):
        usv = self.informed("USV1", None)
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes("ALPHA"))
        self.bridge._handle_course(course_bytes("BRAVO"))
        self.assertTrue(wait_for(lambda: len(infos(usv)) == 3))
        self.assertEqual(infos(usv)[-1]["course"]["course_id"], "BRAVO")

    def test_a_declaration_sends_the_fence_to_the_declared_aircraft_only(self):
        usv = self.informed("USV1", None)
        uav = self.informed("UAV1", "radio")
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        self.assertTrue(wait_for(lambda: len(infos(usv)) == 2 and len(infos(uav)) == 2))
        for vid in self.bridge.cfg.vehicle_ids:
            self.bridge.set_active(vid, vid in ("USV1", "UAV1"))
        self.confirm_fence("UAV1")                      # the gate: the adapter has read the fence back
        self.assertIn("declared", self.bridge.declare())
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 3))
        time.sleep(0.15)
        self.assertEqual(len(infos(usv)), 2, "the boat is not bound by the aircraft's fence")
        self.assertEqual(infos(uav)[-1]["course"]["course_id"], "ALPHA")
        self.assertEqual(len(infos(uav)[-1]["uav_geofence"]), len(self.bridge._fence.polygon))

    def test_an_aircraft_that_is_not_declared_is_not_sent_a_declaration_info(self):
        uav = self.informed("UAV1", "radio")
        self.informed("USV1", None)
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 2))
        for vid in self.bridge.cfg.vehicle_ids:
            self.bridge.set_active(vid, vid == "USV1")
        self.assertIn("declared", self.bridge.declare())
        time.sleep(0.15)
        self.assertEqual(len(infos(uav)), 2)

    def test_a_vehicle_not_connected_is_skipped_without_error(self):
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())                # nobody connected
        self.assertEqual(self.bridge.team.published, [], "info has no MQTT fallback")

    def test_info_goes_down_one_link(self):
        radio = self.informed("UAV1", "radio")
        wifi = self.informed("UAV1", "wifi")                         # told on ITS link, see below
        n_radio, n_wifi = len(infos(radio)), len(infos(wifi))
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        self.assertTrue(wait_for(lambda: len(infos(radio)) > n_radio))
        time.sleep(0.15)
        self.assertEqual(len(infos(wifi)), n_wifi, "the non-preferred link is not sent a second copy")

    def test_a_link_that_comes_up_is_told_on_that_link_not_the_best_one(self):
        """The radio is preferred and already up. A WiFi client that connects knows nothing,
        and a snapshot sent down the radio would never reach it."""
        radio = self.informed("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        self.assertTrue(wait_for(lambda: infos(wifi)), "the wifi client was never told")
        time.sleep(0.15)
        self.assertEqual(len(infos(wifi)), 1)
        self.assertEqual(len(infos(radio)), 1, "the radio is not sent the wifi client's snapshot")

    def test_and_a_radio_that_comes_up_on_a_vehicle_with_no_preference(self):
        """No `prefer` and a tie: wifi (the wider pipe) is the best link, and not the one that came up."""
        wifi = self.informed("USV1", None)
        radio = self.connect("USV1", "radio")
        self.assertTrue(wait_for(lambda: infos(radio)))
        time.sleep(0.15)
        self.assertEqual(len(infos(radio)), 1)
        self.assertEqual(len(infos(wifi)), 1)

    def test_the_link_up_snapshot_carries_the_course_and_the_fence_for_an_aircraft(self):
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes("CHARLIE", 27000))
        self.informed("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        self.assertTrue(wait_for(lambda: infos(wifi)))
        self.assertEqual(infos(wifi)[0]["course"]["course_id"], "CHARLIE")
        self.assertEqual(len(infos(wifi)[0]["uav_geofence"]), len(self.bridge._fence.polygon))

    def test_the_page_snapshot_carries_the_course_fence_for_the_map(self):
        snap = json.loads(json.dumps(self.bridge.snapshot()))
        self.assertIsNone(snap["uav_geofence"], "no course yet: no fence")
        self.assertIsNone(snap["uav_geofence_source"])
        self.assertIn("no course", snap["uav_geofence_reason"])
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        snap = json.loads(json.dumps(self.bridge.snapshot()))
        self.assertEqual([(p["latitude"], p["longitude"]) for p in snap["uav_geofence"]],
                         list(self.bridge._fence.polygon))
        self.assertEqual(snap["uav_geofence_source"], "course boundary inset 2.0 m")
        self.assertIsNone(snap["uav_geofence_reason"])

    def test_the_link_up_snapshot_is_logged_against_the_link_that_came_up(self):
        self.informed("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        self.assertTrue(wait_for(lambda: infos(wifi)))
        topics = [l["topic"] for l in self.wire_lines()
                  if l.get("dir") == "vtx" and l.get("note", "").startswith("ocs_info")]
        self.assertEqual(sorted(topics), ["tcp://UAV1/radio", "tcp://UAV1/wifi"])
        self.assertTrue(any("sent=UAV1/wifi" in l.get("detail", "") for l in self.wire_lines()
                            if l.get("kind") == "ocs_info"), self.wire_lines())

    def test_a_reconnect_is_told_again_on_the_new_socket(self):
        old = self.informed("UAV1", "radio")
        new = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: infos(new)))
        self.assertTrue(wait_for(old.closed.is_set))

    def test_info_is_a_snapshot_so_a_vehicle_can_replace_not_merge(self):
        usv = self.informed("USV1", None)
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes("ALPHA"))
        self.assertTrue(wait_for(lambda: len(infos(usv)) == 2))
        for got in infos(usv):
            self.assertIn("course", got)

    def test_info_carries_nothing_that_could_be_taken_for_a_command(self):
        usv = self.informed("USV1", None)
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        self.assertTrue(wait_for(lambda: len(infos(usv)) == 2))
        for frame in usv.frames:
            if "ocs_info" in frame:
                self.assertEqual(set(frame), {"ocs_info"})
                self.assertEqual(set(frame["ocs_info"]) - {"course", "uav_geofence"}, set())

    def test_info_sends_are_logged_with_decoded_content_and_an_event(self):
        self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: any(l.get("note", "").startswith("ocs_info")
                                             for l in self.wire_lines())))
        line = next(l for l in self.wire_lines() if l.get("note", "").startswith("ocs_info"))
        self.assertEqual(line["dir"], "vtx")
        self.assertEqual(line["topic"], "tcp://UAV1/radio")
        self.assertIn("uav_geofence", line["decoded"]["ocs_info"])
        self.assertTrue(line["b64"])
        self.assertTrue(any(l.get("kind") == "ocs_info" for l in self.wire_lines()))


class TestSendInfoOff(BridgeCase):
    """Per-vehicle `send_info = false`: the escape hatch for a client that chokes on it."""

    usv_extra = "send_info = false"

    def test_a_vehicle_with_send_info_false_is_never_sent_info(self):
        usv = self.connect("USV1", None)
        uav = self.connect("UAV1", "radio")
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        self.assertTrue(wait_for(lambda: len(infos(uav)) == 2))
        for vid in self.bridge.cfg.vehicle_ids:
            self.bridge.set_active(vid, True)
        self.confirm_fence("UAV1")
        self.assertIn("declared", self.bridge.declare())
        time.sleep(0.2)
        self.assertEqual(infos(usv), [])
        self.assertTrue(usv.of_kind("ocs_directive"), "directives are unaffected")

    def test_commands_still_reach_it(self):
        from rx_bridge.proto import rx_commands_pb2
        usv = self.connect("USV1", None)
        cmd = rx_commands_pb2.RxCommand()
        cmd.seq = 3
        cmd.vehicle_id = "USV1"
        cmd.keep_out_zone.SetInParent()
        self.bridge._route_command(cmd, "keep_out_zone")
        self.assertTrue(wait_for(lambda: usv.of_kind("keep_out_zone")))


if __name__ == "__main__":
    unittest.main()
