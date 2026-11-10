"""operatorstore and the page's call-sign and registration boxes: typed, saved as this
laptop's default (run/operator.json), bridge.toml's as the fallback -- and through the
Bridge and the `/api/operator` endpoint.

Pure tests of the store first (a temporary directory, no bridge), then the wiring.
"""
from __future__ import annotations

import http.client
import json
import sys
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge.operatorstore import MAX_LEN, REMEMBER, OperatorError, OperatorStore, clean  # noqa: E402

from tests.harness import BridgeCase, heartbeat                 # noqa: E402


class StoreCase(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.path = Path(self._tmp.name) / "run" / "operator.json"
        self.logs: list[str] = []

    def store(self, callsign="Config UA0", registrations=None):
        return OperatorStore(self.path, callsign=callsign,
                             registrations={"UAV1": "CFG-1"} if registrations is None
                             else registrations, log=self.logs.append)

    def saved(self) -> dict:
        return json.loads(self.path.read_text(encoding="utf-8"))


class TestPrecedence(StoreCase):

    def test_bridge_toml_when_nothing_else(self):
        s = self.store()
        self.assertEqual((s.callsign, s.registration("UAV1")), ("Config UA0", "CFG-1"))
        v = s.view()
        self.assertEqual(v["callsign"], {"value": "Config UA0", "from": "config", "saved": None,
                                         "config": "Config UA0"})
        self.assertFalse(self.path.exists(), "reading writes nothing")

    def test_nothing_anywhere_is_blank_from_nowhere(self):
        s = self.store(callsign="", registrations={"UAV1": ""})
        self.assertEqual(s.view()["callsign"]["from"], None)
        self.assertEqual(s.view()["registration"]["UAV1"],
                         {"value": "", "from": None, "saved": None, "config": ""})

    def test_typed_wins_and_says_so(self):
        s = self.store()
        lines = s.set(callsign="Chase UA1", registration={"UAV1": "ASTA-1"})
        self.assertEqual((s.callsign, s.registration("UAV1")), ("Chase UA1", "ASTA-1"))
        self.assertEqual(lines, ["the pilot's call sign is 'Chase UA1' (was 'Config UA0')",
                                 "UAV1's registration is 'ASTA-1' (was 'CFG-1')"])
        self.assertEqual(s.view()["callsign"]["from"], "typed")
        self.assertFalse(self.path.exists(), "typing is not saving")

    def test_a_box_emptied_stays_empty(self):
        s = self.store()
        s.set(callsign="")
        self.assertEqual(s.callsign, "", "never back-filled from a default")
        self.assertEqual(s.view()["callsign"]["from"], "typed")

    def test_the_same_value_again_is_no_change(self):
        s = self.store()
        self.assertEqual(s.set(callsign="Config UA0"), [])

    def test_saved_wins_over_bridge_toml_and_survives_a_restart(self):
        s = self.store()
        lines = s.set(callsign="Chase UA1", save=True)
        self.assertIn("the pilot's call sign default saved as 'Chase UA1' on this laptop (was blank)",
                      lines)
        again = self.store()
        self.assertEqual(again.callsign, "Chase UA1")
        self.assertEqual(again.view()["callsign"]["from"], "saved")
        self.assertEqual(again.registration("UAV1"), "CFG-1", "only what was saved is saved")

    def test_saving_a_blank_clears_the_default(self):
        s = self.store()
        s.set(callsign="Chase UA1", save=True)
        lines = s.set(callsign="", save=True)
        self.assertIn("the pilot's call sign default cleared on this laptop (was 'Chase UA1')", lines)
        self.assertEqual(self.store().callsign, "Config UA0")

    def test_an_id_that_is_not_an_aircraft_is_refused_and_nothing_changes(self):
        s = self.store()
        with self.assertRaises(OperatorError) as cm:
            s.set(callsign="Chase UA1", registration={"USV1": "X"}, save=True)
        self.assertIn("USV1 is not an aircraft in bridge.toml (aircraft: UAV1)", str(cm.exception))
        self.assertEqual(s.callsign, "Config UA0")
        self.assertFalse(self.path.exists())

    def test_a_frozen_registration_refuses_a_change_but_not_the_same_value(self):
        s = self.store()
        with self.assertRaises(OperatorError) as cm:
            s.set(registration={"UAV1": "ASTA-2"}, registration_frozen="a run is on")
        self.assertEqual(str(cm.exception), "a run is on")
        self.assertEqual(s.set(registration={"UAV1": " CFG-1 "}, registration_frozen="a run is on"),
                         [])


class TestRemembered(StoreCase):

    def test_saved_values_are_offered_newest_first_then_bridge_tomls(self):
        s = self.store()
        s.set(callsign="A UA1", save=True)
        s.set(callsign="B UA2", save=True)
        s.set(callsign="A UA1", save=True)
        self.assertEqual(s.view()["callsigns"], ["A UA1", "B UA2", "Config UA0"])
        s.set(registration={"UAV1": "R-1"}, save=True)
        self.assertEqual(s.view()["registrations"], ["R-1", "CFG-1"])

    def test_only_so_many(self):
        s = self.store(callsign="")
        for n in range(REMEMBER + 3):
            s.set(callsign="P UA%d" % n, save=True)
        self.assertEqual(len(self.saved()["callsigns"]), REMEMBER)
        self.assertEqual(self.saved()["callsigns"][0], "P UA%d" % (REMEMBER + 2))

    def test_a_saved_registration_for_another_configs_aircraft_is_kept(self):
        self.path.parent.mkdir(parents=True)
        self.path.write_text(json.dumps({"registration": {"UAV9": "OTHER"}}), encoding="utf-8")
        s = self.store()
        self.assertNotIn("UAV9", s.view()["registration"])
        s.set(registration={"UAV1": "R-1"}, save=True)
        self.assertEqual(self.saved()["registration"], {"UAV9": "OTHER", "UAV1": "R-1"})


class TestTheFile(StoreCase):

    def test_a_file_that_is_not_json_is_ignored_said_and_rewritten_by_a_save(self):
        self.path.parent.mkdir(parents=True)
        self.path.write_text("{not json", encoding="utf-8")
        s = self.store()
        self.assertEqual(s.callsign, "Config UA0")
        self.assertIn("could not be read", s.load_error)
        self.assertIn("bridge.toml's values apply", s.load_error)
        self.assertTrue(any("operator:" in m and "could not be read" in m for m in self.logs))
        s.set(callsign="Chase UA1", save=True)
        self.assertIsNone(s.load_error)
        self.assertEqual(self.saved()["callsign"], "Chase UA1")

    def test_a_bad_value_in_it_is_left_out_and_the_rest_used(self):
        self.path.parent.mkdir(parents=True)
        self.path.write_text(json.dumps({"callsign": 7, "registration": {"UAV1": "R-1"}}),
                             encoding="utf-8")
        s = self.store()
        self.assertEqual((s.callsign, s.registration("UAV1")), ("Config UA0", "R-1"))
        self.assertIn("the saved call sign must be text", s.load_error)

    def test_a_save_that_cannot_be_written_changes_nothing(self):
        self.path.parent.mkdir(parents=True)
        self.path.mkdir()                                   # a directory where the file should be
        s = self.store()
        with self.assertRaises(OperatorError) as cm:
            s.set(callsign="Chase UA1", save=True)
        self.assertIn("could not save", str(cm.exception))
        self.assertEqual(s.callsign, "Config UA0", "not even the typed half")


class TestClean(unittest.TestCase):

    def test_whitespace_is_tidied(self):
        self.assertEqual(clean("  Chase \t UA1  ", "x"), "Chase UA1")

    def test_what_cannot_be_sent(self):
        for label, value in (("control", "Chase\x07UA1"), ("not text", 7), ("None", None),
                             ("too long", "x" * (MAX_LEN + 1))):
            with self.subTest(label), self.assertRaises(OperatorError):
                clean(value, "the call sign")
        self.assertEqual(len(clean("x" * MAX_LEN, "x")), MAX_LEN)


# ---- through the Bridge ---------------------------------------------------------------

class TestThroughTheBridge(BridgeCase):
    uav_extra = 'registration = "CFG-1"'

    def test_the_store_lives_beside_the_sequence_counters(self):
        self.assertEqual(self.bridge.operator.path, Path(self._tmp.name) / "operator.json")

    def test_a_saved_default_is_there_after_a_restart(self):
        ok, what = self.bridge.set_operator(callsign="Chase UA1", registration={"UAV1": "ASTA-1"},
                                            save=True)
        self.assertTrue(ok, what)
        self.assertTrue(any(m.startswith("operator: ") for m in self.logs))
        self.assertTrue(any(l.get("kind") == "operator" for l in self.wire_lines()))
        again = self.make_bridge()
        self.assertEqual((again.operator.callsign, again.operator.registration("UAV1")),
                         ("Chase UA1", "ASTA-1"))

    def test_the_heartbeat_carries_the_saved_registration(self):
        self.bridge.set_operator(registration={"UAV1": "ASTA-1"}, save=True)
        self.declared()
        self.feed(heartbeat("UAV1"))
        self.assertEqual(self.reports_to_rc("UAV1")[-1].heartbeat.uav_registration, "ASTA-1")

    def test_a_refusal_is_said_and_nothing_changes(self):
        ok, why = self.bridge.set_operator(callsign="x" * 100)
        self.assertFalse(ok)
        self.assertIn("at most %d" % MAX_LEN, why)
        self.assertEqual(self.bridge.operator.callsign, "Test UA0000")

    def test_the_endpoint(self):
        from ocs_web import server
        srv = ThreadingHTTPServer(("127.0.0.1", 0), server.make_handler(self.bridge, None, []))
        threading.Thread(target=srv.serve_forever, args=(0.01,), daemon=True).start()
        self.addCleanup(srv.server_close)
        self.addCleanup(srv.shutdown)

        def post(body: dict):
            conn = http.client.HTTPConnection("127.0.0.1", srv.server_address[1], timeout=5)
            raw = json.dumps(body).encode()
            conn.request("POST", "/api/operator", body=raw,
                         headers={"Content-Length": str(len(raw))})
            r = conn.getresponse()
            out = (r.status, json.loads(r.read()))
            conn.close()
            return out

        status, got = post({"callsign": "Chris UA5678", "save": True})
        self.assertEqual(status, 200)
        self.assertTrue(got["ok"])
        self.assertIn("default saved as 'Chris UA5678'", got["result"])
        self.assertEqual(self.bridge.snapshot()["operator"]["callsign"]["saved"], "Chris UA5678")
        status, got = post({"registration": {"UUV1": "X"}})
        self.assertEqual(status, 400)
        self.assertFalse(got["ok"])
        self.assertIn("UUV1 is not an aircraft", got["result"])
        status, got = post({})
        self.assertEqual((status, got["result"]), (200, "nothing changed"))


if __name__ == "__main__":
    unittest.main()
