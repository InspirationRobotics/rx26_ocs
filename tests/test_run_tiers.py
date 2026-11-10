"""Tiers as a runtime value, `ocs_tiers` (vehicle -> OCS), `ocs_run` (OCS -> vehicles) and the
`run_end` directive.

The tiers a declaration carries start as bridge.toml's, are set from the page
(`POST /api/tiers`) or by a vehicle's `ocs_tiers`, and are fixed from declaring until the run
is ended. Every non-aircraft vehicle is sent the run state and the tiers as `ocs_run`; an
aircraft is sent nothing new (Ekko's radio adapter must not see these frames).
"""
from __future__ import annotations

import http.client
import json
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge.proto import rx_commands_pb2, rx_requests_pb2     # noqa: E402

from tests.harness import BridgeCase, TEAM, course_bytes, rfc3339, wait_for   # noqa: E402

ALL_NONE = {"task1": "TIER_NONE", "task2": "TIER_NONE", "task3": "TIER_NONE", "task4": "TIER_NONE"}
MIXED = {"task1": "TIER_ADVANCED", "task2": "TIER_NONE", "task3": "TIER_CORE", "task4": "TIER_DISRUPTIVE"}
#: What the harness's bridge.toml says: [tiers] task1 = "TIER_CORE", the rest default.
TOML = dict(ALL_NONE, task1="TIER_CORE")


def runs(client):
    return [f["ocs_run"] for f in client.of_kind("ocs_run")]


def directives(client, action):
    return [f["ocs_directive"] for f in client.of_kind("ocs_directive")
            if f["ocs_directive"]["action"] == action]


def tiers_frame(vid="USV1", tiers=MIXED, mission="survey_all_core", **over):
    body = {"vehicle_id": vid, "tiers": tiers, "mission": mission, "sent_at": rfc3339()}
    body.update(over)
    return {"ocs_tiers": body}


class TiersCase(BridgeCase):

    def boat(self):
        """USV1 connected, with the snapshot it is always sent on connecting received."""
        usv = self.connect("USV1", None)
        self.assertTrue(wait_for(lambda: runs(usv)), "no ocs_run on connect")
        return usv

    def declared_request(self):
        req = rx_requests_pb2.RxRequest()
        req.ParseFromString(next(p for t, p, _ in reversed(self.bridge.rc.published)
                                 if t.endswith("/request")))
        return req.run_declaration

    def tiers_now(self):
        return self.bridge.snapshot()["tiers"]

    def run_start(self, run_id=3):
        cmd = rx_commands_pb2.RxCommand()
        cmd.team_id, cmd.seq = TEAM, 1
        cmd.run_start.declaration_seq = self.bridge.machine.declaration_seq
        cmd.run_start.run_id = run_id
        self.bridge._handle_command(cmd.SerializeToString())


class TestTiersAreARuntimeValue(TiersCase):

    def test_they_start_as_bridge_toml_says_and_the_page_is_told_where_from(self):
        t = self.tiers_now()
        self.assertEqual(t["current"], TOML)
        self.assertEqual(t["source"], "bridge.toml")
        self.assertIsNone(t["declared"])
        self.assertIsNone(t["blocker"])
        self.assertIsInstance(t["set_at"], float)

    def test_the_page_sets_them_and_a_declaration_carries_them(self):
        ok, why = self.bridge.set_tiers(MIXED, "page")
        self.assertTrue(ok, why)
        t = self.tiers_now()
        self.assertEqual((t["current"], t["source"]), (MIXED, "page"))
        self.declared(("USV1",))
        decl = self.declared_request()
        self.assertEqual([decl.task1_tier, decl.task2_tier, decl.task3_tier, decl.task4_tier],
                         [3, 1, 2, 4], "ADVANCED, NONE, CORE, DISRUPTIVE")
        self.assertEqual(self.tiers_now()["declared"], MIXED)

    def test_an_untouched_declaration_still_carries_bridge_tomls(self):
        self.declared(("USV1",))
        self.assertEqual(self.tiers_now()["declared"], TOML)
        self.assertEqual(self.declared_request().task1_tier, 2)

    def test_refused_while_declared_or_running_with_the_reason_in_words(self):
        self.declared(("USV1",))
        ok, why = self.bridge.set_tiers(MIXED, "page")
        self.assertFalse(ok)
        self.assertEqual(why, "a run is DECLARED: end it before changing tiers")
        self.assertEqual(self.tiers_now()["current"], TOML)
        self.assertEqual(self.tiers_now()["blocker"], why)
        self.run_start()
        ok, why = self.bridge.set_tiers(MIXED, "page")
        self.assertEqual((ok, why), (False, "a run is RUNNING: end it before changing tiers"))
        self.bridge.end()
        self.assertTrue(self.bridge.set_tiers(MIXED, "page")[0], "allowed again once ended")
        self.assertEqual(self.tiers_now()["declared"], TOML, "the last declaration is kept")

    def test_values_that_are_not_four_real_tiers_are_refused(self):
        for bad in (dict(MIXED, task2="TIER_UNKNOWN"), dict(MIXED, task2="core"),
                    {k: v for k, v in MIXED.items() if k != "task4"},
                    dict(MIXED, task5="TIER_NONE"), "TIER_CORE", None):
            ok, why = self.bridge.set_tiers(bad, "page")
            self.assertFalse(ok, bad)
            self.assertTrue(why)
        self.assertEqual(self.tiers_now()["current"], TOML)
        self.assertEqual(self.tiers_now()["source"], "bridge.toml")

    def test_no_combination_is_refused_by_a_vehicle_rule(self):
        """geofence and validate have no tier-dependent rule: all NONE and all DISRUPTIVE
        both set and declare (RoboCommand's own checks are not guessed here)."""
        for tiers in (ALL_NONE, dict.fromkeys(ALL_NONE, "TIER_DISRUPTIVE")):
            self.assertTrue(self.bridge.set_tiers(tiers, "page")[0])
        self.declared(("USV1",))
        self.assertEqual(self.tiers_now()["declared"], dict.fromkeys(ALL_NONE, "TIER_DISRUPTIVE"))


class TestOcsTiersFromAVehicle(TiersCase):

    def test_it_sets_the_tiers_and_the_vehicle_is_answered(self):
        usv = self.boat()
        n = len(runs(usv))
        usv.send(tiers_frame())
        self.assertTrue(wait_for(lambda: len(runs(usv)) > n))
        got = runs(usv)[-1]
        self.assertEqual(got["reply"], {"ok": True, "reason": "tiers set for the next declaration "
                                                              "by USV1 (mission survey_all_core)"})
        self.assertEqual(got["tiers"], MIXED)
        self.assertEqual(got["tiers_source"], "USV1 (mission survey_all_core)")
        self.assertEqual(self.tiers_now()["source"], "USV1 (mission survey_all_core)")
        self.assertEqual(self.tiers_now()["current"], MIXED)

    def test_a_blank_mission_is_just_the_vehicle(self):
        usv = self.boat()
        usv.send(tiers_frame(mission=""))
        self.assertTrue(wait_for(lambda: self.tiers_now()["source"] == "USV1"))

    def test_the_other_vehicles_hear_the_change_without_a_reply(self):
        usv, uuv = self.boat(), self.connect("UUV1", None)
        self.assertTrue(wait_for(lambda: runs(uuv)))
        n_usv, n_uuv = len(runs(usv)), len(runs(uuv))
        usv.send(tiers_frame())
        self.assertTrue(wait_for(lambda: len(runs(usv)) > n_usv and len(runs(uuv)) > n_uuv))
        self.assertEqual(runs(uuv)[-1]["tiers"], MIXED)
        self.assertNotIn("reply", runs(uuv)[-1])
        self.assertIn("reply", runs(usv)[-1])

    def test_refused_while_declared_the_vehicle_gets_ok_false(self):
        usv = self.boat()
        self.declared(("USV1",))
        n = len(runs(usv))
        usv.send(tiers_frame())
        self.assertTrue(wait_for(lambda: len(runs(usv)) > n))
        got = runs(usv)[-1]
        self.assertEqual(got["reply"], {"ok": False, "reason":
                                        "a run is DECLARED: end it before changing tiers"})
        self.assertEqual(got["state"], "DECLARED")
        self.assertEqual(got["tiers"], TOML)
        self.assertEqual(self.tiers_now()["current"], TOML)

    def test_malformed_frames_are_refused_and_answered_and_the_socket_stays_up(self):
        usv = self.boat()
        bad = [("not an object", {"ocs_tiers": "nonsense"}),
               ("names another vehicle", tiers_frame("UUV1")),
               ("TIER_UNKNOWN", tiers_frame(tiers=dict(MIXED, task3="TIER_UNKNOWN"))),
               ("missing key", tiers_frame(tiers={"task1": "TIER_CORE"})),
               ("extra key", tiers_frame(tiers=dict(MIXED, task9="TIER_CORE"))),
               ("mission", tiers_frame(mission=7))]
        for label, frame in bad:
            n = len(runs(usv))
            usv.send(frame)
            self.assertTrue(wait_for(lambda: len(runs(usv)) > n), label)
            self.assertFalse(runs(usv)[-1]["reply"]["ok"], label)
            self.assertTrue(runs(usv)[-1]["reply"]["reason"], label)
        self.assertEqual(self.tiers_now()["current"], TOML)
        self.assertEqual(self.tiers_now()["source"], "bridge.toml")
        self.assertFalse(usv.closed.is_set())

    def test_an_unidentified_socket_is_ignored(self):
        self.feed(tiers_frame("USV1"), key="anon")
        self.assertEqual(self.tiers_now()["current"], TOML)
        self.assertTrue(any("ocs_tiers REFUSED" in m and "no hello" in m for m in self.logs))

    def test_an_aircraft_cannot_set_tiers_and_is_sent_nothing(self):
        uav = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(lambda: uav.of_kind("ocs_info")))
        n = len(uav.frames)
        uav.send(tiers_frame("UAV1"))
        self.assertTrue(wait_for(lambda: any("aircraft" in m for m in self.logs)))
        self.assertEqual(self.tiers_now()["current"], TOML)
        self.assertEqual(len(uav.frames), n, "no reply, nothing")
        self.assertEqual(uav.of_kind("ocs_run"), [])

    def test_never_a_report_never_a_seq_never_roboCommand(self):
        usv = self.boat()
        before = (self.bridge.counters.forwarded, self.bridge.counters.dropped_invalid,
                  list(self.bridge.rc.published))
        usv.send(tiers_frame())
        usv.send(tiers_frame(tiers=dict(MIXED, task2="bad")))
        self.assertTrue(wait_for(lambda: len(runs(usv)) >= 3))
        self.assertEqual((self.bridge.counters.forwarded, self.bridge.counters.dropped_invalid,
                          list(self.bridge.rc.published)), before)

    def test_wire_logged_once_each(self):
        usv = self.boat()
        usv.send(tiers_frame())
        usv.send(tiers_frame(tiers=dict(MIXED, task2="bad")))
        self.assertTrue(wait_for(lambda: len(runs(usv)) >= 3))
        notes = [l.get("note", "") for l in self.wire_lines() if l.get("dir", "veh") == "veh"
                 or "ocs_tiers" in l.get("note", "")]
        self.assertEqual(len([n for n in notes if n.startswith("ocs_tiers vehicle=USV1")]), 1)
        self.assertEqual(len([n for n in notes if n.startswith("ocs_tiers REFUSED")]), 1)


class TestOcsRun(TiersCase):

    def test_a_connecting_vehicle_is_told_the_run_and_the_tiers(self):
        usv = self.boat()
        self.assertEqual(runs(usv)[0], {
            "state": "DISCONNECTED", "declaration_seq": None, "run_id": None, "tiers": TOML,
            "tiers_source": "bridge.toml", "tiers_set_at": runs(usv)[0]["tiers_set_at"],
            "declared_tiers": None, "sent_at": runs(usv)[0]["sent_at"]})
        self.assertRegex(runs(usv)[0]["tiers_set_at"], r"^\d{4}-\d\d-\d\dT\d\d:\d\d:\d\dZ$")

    def test_declare_start_and_end_each_send_a_snapshot(self):
        usv = self.boat()
        self.bridge.set_tiers(MIXED, "page")
        self.assertTrue(wait_for(lambda: runs(usv)[-1]["tiers"] == MIXED))
        n = len(runs(usv))
        self.declared(("USV1",))
        self.assertTrue(wait_for(lambda: len(runs(usv)) > n))
        got = runs(usv)[-1]
        seq = self.bridge.machine.declaration_seq
        self.assertEqual((got["state"], got["declaration_seq"], got["run_id"], got["declared_tiers"]),
                         ("DECLARED", seq, None, MIXED))
        n = len(runs(usv))
        self.run_start(run_id=3)
        self.assertTrue(wait_for(lambda: len(runs(usv)) > n))
        self.assertEqual((runs(usv)[-1]["state"], runs(usv)[-1]["run_id"]), ("RUNNING", 3))
        n = len(runs(usv))
        self.bridge.end()
        self.assertTrue(wait_for(lambda: len(runs(usv)) > n))
        self.assertEqual(runs(usv)[-1]["state"], "ENDED")
        self.assertEqual(runs(usv)[-1]["declared_tiers"], MIXED)

    def test_a_refused_run_start_sends_nothing(self):
        usv = self.boat()
        self.declared(("USV1",))
        n = len(runs(usv))
        cmd = rx_commands_pb2.RxCommand()
        cmd.team_id, cmd.seq = TEAM, 1
        cmd.run_start.declaration_seq = self.bridge.machine.declaration_seq + 5
        cmd.run_start.run_id = 9
        self.bridge._handle_command(cmd.SerializeToString())
        self.assertEqual(len(runs(usv)), n)

    def test_an_aircraft_is_never_sent_one_at_any_of_those_moments(self):
        uav = self.connect("UAV1", "radio")
        usv = self.boat()
        self.assertTrue(wait_for(lambda: uav.of_kind("ocs_info")))
        self.bridge.set_tiers(MIXED, "page")
        self.declared(("USV1", "UAV1"))
        self.run_start()
        self.bridge.end()
        self.assertTrue(wait_for(lambda: runs(usv)[-1]["state"] == "ENDED"))
        self.assertTrue(uav.of_kind("ocs_directive"), "it still hears standby_auto")
        self.assertEqual(uav.of_kind("ocs_run"), [])
        self.assertEqual(directives(uav, "run_end"), [])


class TestSendInfoOff(TiersCase):
    usv_extra = "send_info = false"

    def test_a_vehicle_with_send_info_off_is_not_sent_one(self):
        usv = self.connect("USV1", None)
        uuv = self.connect("UUV1", None)
        self.assertTrue(wait_for(lambda: runs(uuv)))
        self.bridge.set_tiers(MIXED, "page")
        self.assertTrue(wait_for(lambda: runs(uuv)[-1]["tiers"] == MIXED))
        self.assertEqual(runs(usv), [])
        self.assertEqual(self.bridge._run_audience(), ["UUV1"])


class TestRunEndDirective(TiersCase):

    def test_an_accepted_end_tells_the_declared_non_uav_vehicles(self):
        usv = self.boat()
        uuv = self.connect("UUV1", None)
        self.declared(("USV1",))
        self.run_start(run_id=3)
        seq = self.bridge.machine.declaration_seq
        self.bridge.end()
        self.assertTrue(wait_for(lambda: directives(usv, "run_end")))
        d = directives(usv, "run_end")[0]
        self.assertEqual({k: d[k] for k in ("action", "declaration_seq", "run_id", "vehicles")},
                         {"action": "run_end", "declaration_seq": seq, "run_id": 3,
                          "vehicles": ["USV1"]})
        self.assertIn("sent_at", d)
        self.assertEqual(directives(uuv, "run_end"), [], "UUV1 was not declared")

    def test_ended_from_declared_has_a_null_run_id(self):
        usv = self.boat()
        self.declared(("USV1",))
        self.bridge.end()
        self.assertTrue(wait_for(lambda: directives(usv, "run_end")))
        self.assertIsNone(directives(usv, "run_end")[0]["run_id"])

    def test_a_refused_end_sends_nothing(self):
        usv = self.boat()
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes())
        n = len(runs(usv))
        self.assertIn("nothing running", self.bridge.end())
        self.assertEqual(directives(usv, "run_end"), [])
        self.assertEqual(len(runs(usv)), n)

    def test_standby_auto_is_unchanged_in_shape(self):
        usv = self.boat()
        self.declared(("USV1",))
        self.assertTrue(wait_for(lambda: directives(usv, "standby_auto")))
        self.assertEqual(set(directives(usv, "standby_auto")[0]),
                         {"action", "declaration_seq", "vehicles", "sent_at"})


class TestTiersEndpoint(TiersCase):

    def request(self, method, path, body=None):
        from ocs_web import server
        srv = ThreadingHTTPServer(("127.0.0.1", 0), server.make_handler(self.bridge, None, []))
        threading.Thread(target=srv.serve_forever, args=(0.01,), daemon=True).start()
        self.addCleanup(srv.server_close)
        self.addCleanup(srv.shutdown)
        conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
        conn.request(method, path, json.dumps(body) if body is not None else None)
        r = conn.getresponse()
        out = (r.status, json.loads(r.read()))
        conn.close()
        return out

    def test_post_sets_and_state_shows_source_and_time(self):
        code, j = self.request("POST", "/api/tiers", MIXED)
        self.assertEqual((code, j["ok"]), (200, True))
        code, state = self.request("GET", "/api/state")
        self.assertEqual(state["tiers"]["current"], MIXED)
        self.assertEqual(state["tiers"]["source"], "page")
        self.assertIsNotNone(state["tiers"]["set_at"])

    def test_post_while_declared_is_409_with_the_reason(self):
        self.declared(("USV1",))
        code, j = self.request("POST", "/api/tiers", MIXED)
        self.assertEqual(code, 409)
        self.assertEqual(j, {"ok": False, "result": "a run is DECLARED: end it before changing tiers"})
        self.assertEqual(self.request("GET", "/api/state")[1]["tiers"]["declared"], TOML)

    def test_post_with_bad_values_is_400(self):
        code, j = self.request("POST", "/api/tiers", dict(MIXED, task1="TIER_UNKNOWN"))
        self.assertEqual((code, j["ok"]), (400, False))
        self.assertIn("TIER_UNKNOWN", j["result"])


if __name__ == "__main__":
    unittest.main()
