"""The vehicle link's per-link behaviour: hello, two links per vehicle, which one the OCS
listens to and which one it talks on.

    ocs/.venv/Scripts/python -m unittest discover -s tests -t .

The contract under test is docs/VEHICLE_LINK.md, shared with the fleet_link radio
adapter: a client MAY open with {"ocs_hello": {...}}; without one it is "wifi";
at most one socket per vehicle PER LINK. Duplicates are dropped before the governor
PER REPORT KIND: a real report beats a synthesized one, among equals the vehicle's
`prefer` link wins, and a kind only one link carries is never dropped. Downlink
frames go to one link, never both.
"""
from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge import linkpolicy                                  # noqa: E402
from rx_bridge.bridge import REPORT_KINDS                         # noqa: E402
from rx_bridge.config import ConfigError, load                    # noqa: E402
from rx_bridge.proto import rx_commands_pb2, rx_common_pb2       # noqa: E402
from rx_bridge.tcpserver import TcpServer                         # noqa: E402

from tests.harness import (BridgeCase, VehicleClient, heartbeat,  # noqa: E402
                           hello_frame, incident_ack, readiness, rfc3339,
                           wait_for)


class TestLinkPolicy(unittest.TestCase):
    """The pure rules, with the clock passed in."""

    NOW, FRESH = 10.0, 3.0

    def dup(self, prefer, link, arrivals, *, synth=False, kind="heartbeat"):
        """duplicate_reason with the clock fixed at 10.0 and the window at 3.0.

        arrivals: link -> age in seconds of the last copy of `kind` there, or
        (age, synthesized). A bare age is a real copy.
        """
        table = {l: (self.NOW - (a[0] if isinstance(a, tuple) else a),
                     a[1] if isinstance(a, tuple) else False)
                 for l, a in arrivals.items()}
        return linkpolicy.duplicate_reason(prefer, link, kind, synth, table, self.NOW, self.FRESH)

    # -- real against real: the vehicle's preferred link wins -------------------

    def test_a_report_on_the_preferred_link_is_never_a_duplicate(self):
        self.assertIsNone(self.dup("radio", "radio", {"wifi": 0.1}))

    def test_no_preference_means_equals_both_stand(self):
        self.assertIsNone(self.dup(None, "wifi", {"radio": 0.1}))
        self.assertIsNone(self.dup(None, "radio", {"wifi": 0.1}))

    def test_the_other_link_is_dropped_while_the_preferred_one_is_fresh(self):
        why = self.dup("radio", "wifi", {"radio": 1.0})
        self.assertIn("radio", why)
        self.assertIn("not preferred", why)
        self.assertIn("heartbeat", why)

    def test_the_other_link_is_kept_once_the_preferred_one_has_gone_quiet(self):
        self.assertIsNone(self.dup("radio", "wifi", {"radio": 3.1}))

    def test_the_freshness_edge_is_inclusive(self):
        self.assertIsNotNone(self.dup("radio", "wifi", {"radio": 3.0}))

    def test_a_kind_seen_on_only_one_link_is_never_dropped(self):
        """The preferred link has never delivered THIS kind (the caller files arrivals by
        kind), so there is nothing for this copy to be a duplicate of."""
        self.assertIsNone(self.dup("radio", "wifi", {}, kind="incident_ack"))
        self.assertIsNone(self.dup("radio", "wifi", {"wifi": 0.1}, kind="readiness"))

    def test_the_links_own_earlier_copy_is_not_a_competitor(self):
        self.assertIsNone(self.dup("wifi", "wifi", {"wifi": 0.1}))
        self.assertIsNone(self.dup("radio", "radio", {"radio": 0.1}))

    def test_the_other_link_losing_a_tie_does_not_drop_the_preferred_one(self):
        """Both real, prefer wifi: a radio copy is the duplicate, never the wifi one."""
        self.assertIsNotNone(self.dup("wifi", "radio", {"wifi": 0.1}))
        self.assertIsNone(self.dup("wifi", "wifi", {"radio": 0.1}))

    # -- real beats synthesized, whatever the preference ------------------------

    def test_a_synthesized_copy_is_dropped_while_the_other_link_has_a_fresh_real_one(self):
        why = self.dup("radio", "radio", {"wifi": 0.5}, synth=True)      # even on the preferred link
        self.assertIn("not preferred", why)
        self.assertIn("synthesized", why)
        self.assertIn("wifi", why)
        self.assertIn("heartbeat", why)

    def test_real_beats_synthesized_with_no_preference_and_with_the_other_preference(self):
        for prefer in (None, "wifi", "radio"):
            with self.subTest(prefer):
                self.assertIsNotNone(self.dup(prefer, "radio", {"wifi": 0.5}, synth=True))

    def test_a_real_copy_is_kept_though_the_other_link_is_preferred_and_fresh(self):
        """The case the whole rule exists for: Ekko's own WiFi heartbeat must not be
        dropped because the radio adapter's rebuilt one arrived a moment before."""
        self.assertIsNone(self.dup("radio", "wifi", {"radio": (0.1, True)}))

    def test_a_real_copy_is_kept_when_the_other_is_synthesized_whatever_the_preference(self):
        for prefer in (None, "wifi", "radio"):
            with self.subTest(prefer):
                self.assertIsNone(self.dup(prefer, "wifi", {"radio": (0.1, True)}))

    def test_a_synthesized_copy_is_kept_once_the_real_one_has_gone_quiet(self):
        self.assertIsNone(self.dup("radio", "radio", {"wifi": 3.5}, synth=True))

    def test_a_synthesized_copy_is_kept_when_no_real_one_has_been_seen(self):
        self.assertIsNone(self.dup("radio", "radio", {}, synth=True))

    # -- synthesized against synthesized: back to the preference ----------------

    def test_two_synthesized_copies_fall_back_to_the_preferred_link(self):
        self.assertIsNotNone(self.dup("radio", "wifi", {"radio": (0.1, True)}, synth=True))
        self.assertIsNone(self.dup("radio", "radio", {"wifi": (0.1, True)}, synth=True))
        self.assertIsNone(self.dup(None, "wifi", {"radio": (0.1, True)}, synth=True))

    # -- the relation is strict: no combination drops both copies ---------------

    def test_no_combination_drops_both_links_copies(self):
        """If each link's copy could be judged a duplicate of the other's, the vehicle
        would go silent with both links alive."""
        for prefer in (None, "radio", "wifi"):
            for radio_synth in (False, True):
                for wifi_synth in (False, True):
                    with self.subTest(prefer=prefer, radio_synth=radio_synth, wifi_synth=wifi_synth):
                        radio_dropped = self.dup(prefer, "radio", {"wifi": (0.1, wifi_synth)}, synth=radio_synth)
                        wifi_dropped = self.dup(prefer, "wifi", {"radio": (0.1, radio_synth)}, synth=wifi_synth)
                        self.assertFalse(radio_dropped and wifi_dropped)

    # -- sending ----------------------------------------------------------------

    def test_send_order_puts_a_fresh_preferred_link_first(self):
        self.assertEqual(linkpolicy.send_order({"wifi": 9.9, "radio": 9.5}, "radio", 10.0, 3.0),
                         ["radio", "wifi"])

    def test_send_order_puts_the_other_link_first_when_the_preferred_is_stale(self):
        self.assertEqual(linkpolicy.send_order({"wifi": 9.9, "radio": 5.0}, "radio", 10.0, 3.0),
                         ["wifi", "radio"])

    def test_a_stale_link_is_still_tried_after_the_fresh_ones(self):
        self.assertEqual(linkpolicy.send_order({"radio": 1.0}, "radio", 10.0, 3.0), ["radio"])
        self.assertEqual(linkpolicy.send_order({"wifi": 1.0, "radio": 2.0}, "radio", 10.0, 3.0),
                         ["radio", "wifi"])

    def test_nothing_connected_is_empty_which_means_mqtt(self):
        self.assertEqual(linkpolicy.send_order({}, "radio", 10.0, 3.0), [])

    def test_with_no_preference_the_wider_pipe_wins_a_tie(self):
        self.assertEqual(linkpolicy.send_order({"radio": 9.9, "wifi": 9.9}, None, 10.0, 3.0),
                         ["wifi", "radio"])


class TestLinkConfig(unittest.TestCase):
    BASE = """
team_id = "ASTA"
[[vehicle]]
id = "USV1"
type = "TYPE_USV"
{usv}
[team]
host = "127.0.0.1"
[robocommand]
host = "192.168.65.2"
[limits]
{limits}
"""

    def _load(self, usv: str = "", limits: str = ""):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bridge.toml"
            p.write_text(self.BASE.format(usv=usv, limits=limits), encoding="utf-8")
            return load(p)

    def test_defaults(self):
        cfg = self._load()
        self.assertIsNone(cfg.vehicle("USV1").prefer)
        self.assertTrue(cfg.vehicle("USV1").send_info)
        self.assertEqual(cfg.link_prefer_s, 3.0)

    def test_prefer_and_send_info_and_the_window_are_read(self):
        cfg = self._load('prefer = "radio"\nsend_info = false', "link_prefer_s = 1.5")
        v = cfg.vehicle("USV1")
        self.assertEqual((v.prefer, v.send_info), ("radio", False))
        self.assertEqual(cfg.link_prefer_s, 1.5)

    def test_an_unknown_link_is_refused_at_load(self):
        with self.assertRaisesRegex(ConfigError, "prefer"):
            self._load('prefer = "satellite"')

    def test_send_info_must_be_a_real_boolean(self):
        with self.assertRaisesRegex(ConfigError, "send_info"):
            self._load('send_info = "no"')

    def test_the_window_must_be_positive(self):
        for bad in ("0", "-1"):
            with self.subTest(bad), self.assertRaisesRegex(ConfigError, "link_prefer_s"):
                self._load(limits="link_prefer_s = " + bad)

    def test_the_shipped_config_prefers_ekkos_radio_and_the_boat_has_no_preference(self):
        cfg = load(Path(__file__).resolve().parents[1] / "bridge.toml")
        self.assertEqual({v.id: v.prefer for v in cfg.vehicles},
                         {"USV1": None, "UUV1": "radio", "UAV1": "radio"})
        self.assertTrue(all(v.send_info for v in cfg.vehicles))


class TestHelloAndCoexistence(BridgeCase):
    """Who is on which link, over real sockets."""

    def test_a_client_that_sends_no_hello_is_on_wifi(self):
        """Today's boat and Ekko clients open with a report and must keep working unchanged."""
        self.connect("USV1", None)
        info = self.bridge.tcp.links_of("USV1")
        self.assertEqual(list(info), ["wifi"])
        self.assertFalse(info["wifi"]["hello"])
        self.assertEqual(info["wifi"]["agent"], "")

    def test_a_hello_files_the_socket_on_the_link_it_names(self):
        self.connect("UAV1", "radio", agent="fleet_link")
        info = self.bridge.tcp.links_of("UAV1")
        self.assertEqual(list(info), ["radio"])
        self.assertTrue(info["radio"]["hello"])
        self.assertEqual(info["radio"]["agent"], "fleet_link")

    def test_a_hello_without_a_link_means_wifi(self):
        c = self.client()
        c.send({"ocs_hello": {"vehicle_id": "USV1"}})
        self.assertTrue(wait_for(lambda: "wifi" in self.bridge.tcp.links_of("USV1")))

    def test_a_hello_is_not_a_report_and_does_not_make_the_vehicle_active(self):
        self.connect("UAV1", "radio")
        self.assertNotIn("UAV1", self.bridge.counters.last_seen)
        self.assertNotIn("UAV1", self.bridge.active_ids())
        self.assertNotIn("UAV1", self.bridge.latest)

    def test_two_links_for_one_vehicle_coexist(self):
        radio = self.connect("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        self.assertEqual(set(self.bridge.tcp.links_of("UAV1")), {"radio", "wifi"})
        self.assertFalse(radio.closed.is_set())
        self.assertFalse(wifi.closed.is_set())
        self.assertIn("UAV1", self.bridge.tcp.peers())      # connected on either

    def test_a_second_socket_on_the_same_link_replaces_the_first(self):
        old = self.connect("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        who_old = self.bridge.tcp.links_of("UAV1")["radio"]["who"]
        new = self.connect("UAV1", "radio")
        self.assertTrue(wait_for(old.closed.is_set), "the stale radio socket was not closed")
        self.assertTrue(wait_for(lambda: self.bridge.tcp.links_of("UAV1")["radio"]["who"] != who_old))
        self.assertFalse(wifi.closed.is_set(), "a reconnect on one link must not touch the other")
        self.assertFalse(new.closed.is_set())

    def test_the_old_socket_dying_does_not_unfile_its_replacement(self):
        old = self.connect("UAV1", "radio")
        self.connect("UAV1", "radio")
        self.assertTrue(wait_for(old.closed.is_set))
        time.sleep(0.2)                       # the old reader thread unwinds
        self.assertIn("radio", self.bridge.tcp.links_of("UAV1"))

    def test_a_hello_for_a_vehicle_not_in_the_fleet_drops_the_connection(self):
        c = self.client()
        c.hello("GHOST1", "radio")
        self.assertTrue(wait_for(c.closed.is_set))
        self.assertEqual(self.bridge.tcp.all_links(), {})
        self.assertTrue(any(l["kind"] == "hello_refused" for l in self.wire_lines()
                            if l["dir"] == "evt"))

    def test_a_hello_naming_an_unknown_link_drops_the_connection(self):
        """Guessing wifi for a radio adapter would get its telemetry dropped as the
        non-preferred copy, silently."""
        c = self.client()
        c.hello("UAV1", "satellite")
        self.assertTrue(wait_for(c.closed.is_set))
        self.assertEqual(self.bridge.tcp.all_links(), {})

    def test_a_hello_that_is_not_an_object_drops_the_connection(self):
        c = self.client()
        c.send({"ocs_hello": "UAV1"})
        self.assertTrue(wait_for(c.closed.is_set))

    def test_a_late_hello_cannot_move_a_filed_socket(self):
        c = self.connect("USV1", None)
        c.hello("USV1", "radio")
        c.send(heartbeat("USV1"))
        time.sleep(0.2)
        self.assertFalse(c.closed.is_set())
        self.assertEqual(list(self.bridge.tcp.links_of("USV1")), ["wifi"])

    def test_closing_a_socket_files_its_link_away(self):
        radio = self.connect("UAV1", "radio")
        self.connect("UAV1", "wifi")
        radio.close()
        self.assertTrue(wait_for(lambda: list(self.bridge.tcp.links_of("UAV1")) == ["wifi"]))

    def test_one_socket_may_carry_two_vehicles_and_both_are_filed(self):
        """A boat relaying the sub's reports on its own socket."""
        c = self.connect("USV1", None)
        c.send(heartbeat("UUV1"))
        self.assertTrue(wait_for(lambda: "wifi" in self.bridge.tcp.links_of("UUV1")))
        self.assertIn("wifi", self.bridge.tcp.links_of("USV1"))

    def test_a_frame_that_makes_the_handler_raise_does_not_drop_the_link(self):
        original = self.bridge._on_tcp_frame

        def boom(payload, conn):
            if b"poison" in payload:
                raise RuntimeError("deliberate")
            return original(payload, conn)

        self.bridge.tcp.on_frame = boom
        c = self.client()
        c.send_raw(b'{"poison": 1}')
        c.send(heartbeat("USV1"))
        self.assertTrue(wait_for(lambda: "wifi" in self.bridge.tcp.links_of("USV1")))
        self.assertFalse(c.closed.is_set())


class TestHelloSynthesized(BridgeCase):
    """The hello's optional `synthesized`: the report kinds a link rebuilds itself."""

    def synthesized(self, vid="UAV1", link="radio"):
        return self.bridge.tcp.links_of(vid)[link]["synthesized"]

    def refused(self, value):
        """Send a hello carrying `value`; return (client, the 'bad ocs_hello' log lines it added)."""
        # "bad ocs_hello (" is the refusal itself; the disconnect line repeats the reason.
        refusals = lambda: [m for m in self.logs if "bad ocs_hello (" in m]  # noqa: E731
        before = len(refusals())
        c = self.client()
        c.hello("UAV1", "radio", "fleet_link", value)
        self.assertTrue(wait_for(c.closed.is_set), "a bad synthesized list must drop the connection")
        return c, refusals()[before:]

    def test_the_kind_names_are_the_rxreport_body_names(self):
        """The strings a hello may use. fleet_link must send exactly "heartbeat"."""
        self.assertEqual(set(REPORT_KINDS),
                         {"heartbeat", "safe_passage", "pipeline_survey", "resource_delivery",
                          "docking", "firefighting", "incident_ack", "readiness"})

    def test_a_hello_names_the_kinds_the_link_synthesizes(self):
        self.connect("UAV1", "radio", agent="fleet_link", synthesized=["heartbeat"])
        self.assertEqual(self.synthesized(), ["heartbeat"])
        self.assertEqual(self.bridge.tcp.links_of("UAV1")["radio"]["label"], "radio via fleet_link")

    def test_no_synthesized_field_means_nothing_is_synthesized(self):
        self.connect("UAV1", "radio", agent="fleet_link")
        self.assertEqual(self.synthesized(), [])

    def test_null_and_an_empty_list_also_mean_nothing(self):
        self.connect("UAV1", "wifi", synthesized=None)
        self.connect("UAV1", "radio", synthesized=[])
        self.assertEqual((self.synthesized(link="wifi"), self.synthesized()), ([], []))
        self.assertTrue(self.bridge.tcp.links_of("UAV1")["wifi"]["hello"])

    def test_several_kinds_are_filed_once_each_and_sorted(self):
        self.connect("UAV1", "radio", synthesized=["readiness", "heartbeat", "heartbeat"])
        self.assertEqual(self.synthesized(), ["heartbeat", "readiness"])

    def test_a_link_without_a_hello_synthesizes_nothing(self):
        self.connect("USV1", None)
        self.assertEqual(self.synthesized("USV1", "wifi"), [])

    def test_an_unknown_kind_is_a_hello_error_logged_once(self):
        c, lines = self.refused(["heartbeat", "heartbeet"])
        self.assertEqual(len(lines), 1, lines)
        self.assertIn("heartbeet", lines[0])
        self.assertEqual(self.bridge.tcp.all_links(), {})
        self.assertEqual(sum(l.get("kind") == "hello_refused" for l in self.wire_lines()
                             if l["dir"] == "evt"), 1)

    def test_anything_that_is_not_a_list_of_kind_names_is_refused(self):
        for bad in ("heartbeat", {"heartbeat": True}, 1, True, [1], [None], [["heartbeat"]],
                    ["Heartbeat"], [""], ["heartbeat", "bogus"]):
            with self.subTest(bad):
                _, lines = self.refused(bad)
                self.assertEqual(len(lines), 1, lines)
        self.assertEqual(self.bridge.tcp.all_links(), {})

    def test_a_refused_hello_files_nothing_and_tells_the_vehicle_nothing(self):
        c, _ = self.refused(["nope"])
        time.sleep(0.1)
        self.assertEqual(c.frames, [])


class TestLinkPreference(BridgeCase):
    """Reports: which copy survives, and that the duplicate never reaches the hot path."""

    limits = "link_prefer_s = 0.3"

    def setUp(self):
        super().setUp()
        self.declared()

    def test_a_duplicate_on_the_non_preferred_link_is_dropped(self):
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")
        self.assertEqual(len(self.reports_to_rc("UAV1")), 1)
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {"UAV1": 1})

    def test_the_duplicate_never_reaches_the_governor(self):
        """Five tokens in the bucket: five pairs fit if the governor sees one of each
        pair, and would not if it saw both."""
        gov = self.bridge._gov["UAV1"]
        calls = []
        real = gov.allow
        gov.allow = lambda now, heartbeat: calls.append(1) or real(now, heartbeat=heartbeat)
        for _ in range(5):
            self.feed(heartbeat("UAV1"), "radio")
            self.feed(heartbeat("UAV1"), "wifi")
        self.assertEqual(len(calls), 5)
        self.assertEqual(self.bridge.counters.dropped_rate, 0)
        self.assertEqual(self.bridge.counters.forwarded, 5)

    def test_the_duplicate_burns_no_sequence_number(self):
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")
        self.feed(heartbeat("UAV1"), "radio")
        self.assertEqual([r.seq for r in self.reports_to_rc("UAV1")], [1, 2])
        self.assertEqual(self.bridge.seqs.snapshot()["reports"]["UAV1"], 2)

    def test_the_duplicate_is_dropped_before_the_staleness_check(self):
        """A stale copy on the other link must not even be counted as stale."""
        self.feed(heartbeat("UAV1"), "radio")
        old = heartbeat("UAV1")
        old["sent_at"] = rfc3339(time.time() - 30)
        self.feed(old, "wifi")
        self.assertEqual(self.bridge.counters.stale, 0)
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {"UAV1": 1})

    def test_the_duplicate_is_dropped_before_validation(self):
        self.feed(heartbeat("UAV1"), "radio")
        bad = heartbeat("UAV1")
        del bad["heartbeat"]["current_task"]
        self.feed(bad, "wifi")
        self.assertEqual(self.bridge.counters.dropped_invalid, 0)

    def test_it_does_not_touch_what_the_page_shows_or_the_active_clock(self):
        self.feed(heartbeat("UAV1"), "radio")
        seen = self.bridge.counters.last_seen["UAV1"]
        latest = self.bridge.latest["UAV1"]
        time.sleep(0.02)
        self.feed(heartbeat("UAV1"), "wifi")
        self.assertEqual(self.bridge.counters.last_seen["UAV1"], seen)
        self.assertIs(self.bridge.latest["UAV1"], latest)

    def test_wifi_is_used_once_the_preferred_link_has_gone_quiet(self):
        self.feed(heartbeat("UAV1"), "radio")
        time.sleep(0.4)                                    # past link_prefer_s
        self.feed(heartbeat("UAV1"), "wifi")
        self.assertEqual(len(self.reports_to_rc("UAV1")), 2)
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {})

    def test_the_preferred_link_takes_over_again_when_it_resumes(self):
        self.feed(heartbeat("UAV1"), "radio")
        time.sleep(0.4)
        self.feed(heartbeat("UAV1"), "wifi")               # fallback: kept
        self.feed(heartbeat("UAV1"), "radio")              # radio back
        self.feed(heartbeat("UAV1"), "wifi")               # duplicate again: dropped
        self.assertEqual(len(self.reports_to_rc("UAV1")), 3)
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {"UAV1": 1})

    def test_wifi_alone_is_ingested_normally(self):
        for _ in range(3):
            self.feed(heartbeat("UAV1"), "wifi")
        self.assertEqual(len(self.reports_to_rc("UAV1")), 3)
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {})

    def test_a_vehicle_with_no_preference_keeps_both_links_reports(self):
        self.feed(heartbeat("USV1"), "radio")
        self.feed(heartbeat("USV1"), "wifi")
        self.assertEqual(len(self.reports_to_rc("USV1")), 2)
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {})

    def test_the_preference_is_per_vehicle(self):
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("USV1"), "wifi")               # USV1 has no preference
        self.assertEqual(len(self.reports_to_rc("USV1")), 1)

    def test_a_refused_report_on_the_preferred_link_still_counts_as_that_link_being_alive(self):
        bad = heartbeat("UAV1")
        del bad["heartbeat"]["current_task"]
        self.feed(bad, "radio")                            # refused by the validator
        self.feed(heartbeat("UAV1"), "wifi")               # still a duplicate
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {"UAV1": 1})
        self.assertEqual(self.bridge.counters.dropped_invalid, 1)

    def test_the_mqtt_path_has_no_links_and_is_never_gated(self):
        from rx_bridge.proto import rx_reports_pb2
        from google.protobuf import json_format
        self.feed(heartbeat("UAV1"), "radio")
        r = rx_reports_pb2.RxReport()
        json_format.ParseDict(heartbeat("UAV1"), r)
        self.bridge._ingest_report(r, via="mqtt")
        self.assertEqual(len(self.reports_to_rc("UAV1")), 2)

    def test_the_drop_is_in_the_wire_log_with_the_reason_and_the_bytes(self):
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")
        line = [l for l in self.wire_lines() if l.get("dir") == "veh"][-1]
        self.assertEqual(line["topic"], "tcp://in/wifi")
        self.assertIn("DROPPED", line["note"])
        self.assertIn("not preferred", line["note"])
        self.assertTrue(line["b64"])
        self.assertEqual(line["decoded"]["vehicle_id"], "UAV1")

    def test_the_hot_path_order_is_kept_for_a_report_that_is_kept(self):
        """The one frame that survives still goes governor -> validate -> seq -> restamp."""
        h = heartbeat("UAV1")
        h["sent_at"] = rfc3339(time.time() - 60)           # skewed vehicle clock
        self.feed(h, "radio")
        r = self.reports_to_rc("UAV1")[0]
        self.assertEqual(self.bridge.counters.stale, 1)    # staleness measured on its clock...
        self.assertLess(abs(r.sent_at.ToNanoseconds() / 1e9 - time.time()), 5)  # ...sent_at is ours
        self.assertEqual(r.seq, 1)


class TestPerKindPreference(BridgeCase):
    """Preference is per report KIND, and a real report beats a synthesized one.

    Ekko prefers its radio. The radio adapter relays what the radio carries and
    rebuilds a heartbeat from the autopilot stream; WiFi carries Ekko's own
    heartbeats, task reports, IncidentAcks and Readiness.
    """

    limits = "link_prefer_s = 0.3"

    def setUp(self):
        super().setUp()
        self.declared()

    def adapter(self, vid="UAV1", synthesized=("heartbeat",)):
        """File a hello'd radio adapter (no socket: the frame goes straight to the bridge)."""
        self.assertEqual(self.feed(hello_frame(vid, "radio", "fleet_link", list(synthesized)), "radio"), vid)

    def kept(self, kind, vid="UAV1"):
        return [r for r in self.reports_to_rc(vid) if r.WhichOneof("body") == kind]

    def dropped(self, vid="UAV1"):
        return self.bridge.counters.dropped_not_preferred_kinds.get(vid, {})

    # -- a kind only one link carries is never dropped --------------------------

    def test_wifi_task_reports_pass_while_the_radio_heartbeat_is_fresh(self):
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(readiness("UAV1", command_seq=3, seq=9), "wifi")
        self.feed(incident_ack("UAV1", command_seq=4), "wifi")
        self.assertEqual([r.WhichOneof("body") for r in self.reports_to_rc("UAV1")],
                         ["heartbeat", "readiness", "incident_ack"])
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {})

    def test_the_other_links_traffic_in_general_is_not_the_test(self):
        """A fresh preferred link says nothing about a kind it has never carried."""
        for _ in range(2):                    # three would leave the governor's heartbeat reserve
            self.feed(heartbeat("UAV1"), "radio")
        self.feed(readiness("UAV1", command_seq=1), "wifi")
        self.assertEqual(len(self.kept("readiness")), 1)

    # -- the same kind on both links: the preferred link wins -------------------

    def test_a_kind_both_links_carry_is_still_taken_from_the_preferred_link(self):
        self.feed(readiness("UAV1", command_seq=1), "radio")
        self.feed(readiness("UAV1", command_seq=1), "wifi")
        self.assertEqual(len(self.kept("readiness")), 1)
        self.assertEqual(self.dropped(), {"readiness": 1})

    def test_kinds_are_judged_independently(self):
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")                         # dup: dropped
        self.feed(readiness("UAV1", command_seq=1), "wifi")          # only wifi: kept
        self.assertEqual(self.dropped(), {"heartbeat": 1})
        self.assertEqual(len(self.kept("readiness")), 1)

    def test_a_kind_the_preferred_link_stopped_carrying_falls_to_the_other(self):
        self.feed(readiness("UAV1", command_seq=1), "radio")
        time.sleep(0.4)
        self.feed(readiness("UAV1", command_seq=2), "wifi")
        self.assertEqual(len(self.kept("readiness")), 2)
        self.assertEqual(self.dropped(), {})

    # -- real beats synthesized --------------------------------------------------

    def test_a_real_wifi_heartbeat_outranks_the_adapters_synthesized_one(self):
        self.adapter()
        self.feed(heartbeat("UAV1", task="TASK_SAFE_PASSAGE"), "wifi")
        self.feed(heartbeat("UAV1", task="TASK_NONE"), "radio")      # rebuilt: always TASK_NONE
        kept = self.kept("heartbeat")
        self.assertEqual([r.heartbeat.current_task for r in kept],
                         [rx_common_pb2.RxTask.TASK_SAFE_PASSAGE])
        self.assertEqual(self.dropped(), {"heartbeat": 1})
        self.assertEqual(self.bridge.counters.dropped_not_preferred, {"UAV1": 1})

    def test_the_real_heartbeat_is_kept_though_the_synthesized_one_came_first(self):
        """The radio is preferred and fresh, and the WiFi heartbeat is Ekko's own: it stands."""
        self.adapter()
        self.feed(heartbeat("UAV1"), "radio")                        # nothing to compare: kept
        self.feed(heartbeat("UAV1", task="TASK_SAFE_PASSAGE"), "wifi")
        self.feed(heartbeat("UAV1"), "radio")                        # now there is a real one: dropped
        self.assertEqual(len(self.kept("heartbeat")), 2)
        self.assertEqual(self.dropped(), {"heartbeat": 1})

    def test_the_synthesized_heartbeat_carries_the_vehicle_once_the_real_one_goes_quiet(self):
        self.adapter()
        self.feed(heartbeat("UAV1"), "wifi")
        time.sleep(0.4)
        self.feed(heartbeat("UAV1"), "radio")
        self.assertEqual(len(self.kept("heartbeat")), 2)
        self.assertEqual(self.dropped(), {})

    def test_a_synthesized_heartbeat_alone_is_forwarded(self):
        self.adapter()
        for _ in range(3):
            self.feed(heartbeat("UAV1"), "radio")
        self.assertEqual(len(self.kept("heartbeat")), 3)
        self.assertEqual(self.dropped(), {})

    def test_real_beats_synthesized_with_no_preference_configured(self):
        self.adapter("USV1")                                         # the boat has no `prefer`
        self.feed(heartbeat("USV1"), "wifi")
        self.feed(heartbeat("USV1"), "radio")
        self.assertEqual(len(self.kept("heartbeat", "USV1")), 1)
        self.assertEqual(self.dropped("USV1"), {"heartbeat": 1})

    def test_two_real_copies_with_no_preference_both_stand(self):
        self.feed(heartbeat("USV1"), "wifi")
        self.feed(heartbeat("USV1"), "radio")
        self.assertEqual(len(self.kept("heartbeat", "USV1")), 2)

    def test_only_the_declared_kinds_are_demoted(self):
        """An adapter that synthesizes some other kind still has its heartbeats treated as real."""
        self.adapter(synthesized=("docking",))
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")                         # real vs real: radio is preferred
        self.assertEqual(self.dropped(), {"heartbeat": 1})
        self.assertEqual(len(self.kept("heartbeat")), 1)

    def test_a_dropped_synthesized_copy_still_proves_its_link_is_alive(self):
        """So a real WiFi heartbeat that arrives next is weighed against it correctly."""
        self.adapter()
        self.feed(heartbeat("UAV1"), "wifi")
        self.feed(heartbeat("UAV1"), "radio")                        # dropped
        self.feed(heartbeat("UAV1"), "wifi")                         # real vs synthesized: kept
        self.assertEqual(len(self.kept("heartbeat")), 2)

    def test_the_drop_reason_in_the_wire_log_says_synthesized(self):
        self.adapter()
        self.feed(heartbeat("UAV1"), "wifi")
        self.feed(heartbeat("UAV1"), "radio")
        line = [l for l in self.wire_lines() if l.get("dir") == "veh"][-1]
        self.assertIn("DROPPED", line["note"])
        self.assertIn("synthesized", line["note"])
        self.assertEqual(line["topic"], "tcp://in/radio")

    def test_the_hello_in_the_wire_log_names_the_synthesized_kinds(self):
        self.adapter()
        hello = [l for l in self.wire_lines() if "ocs_hello" in l.get("note", "")][-1]
        self.assertIn("synthesized=heartbeat", hello["note"])

    def test_a_dropped_synthesized_copy_burns_no_sequence_number_and_skips_the_governor(self):
        self.adapter()
        self.feed(heartbeat("UAV1"), "wifi")
        before = self.bridge.seqs.snapshot()["reports"]["UAV1"]
        for _ in range(20):
            self.feed(heartbeat("UAV1"), "radio")
        self.assertEqual(self.bridge.seqs.snapshot()["reports"]["UAV1"], before)
        self.assertEqual(self.bridge.counters.dropped_rate, 0)
        self.assertEqual(self.dropped(), {"heartbeat": 20})

    def test_over_real_sockets(self):
        radio = self.connect("UAV1", "radio", agent="fleet_link", synthesized=["heartbeat"])
        self.connect("UAV1", None)                                   # Ekko's own client: a heartbeat
        self.assertEqual(len(self.kept("heartbeat")), 1)
        radio.send(heartbeat("UAV1"))                                # the adapter's rebuilt one
        self.assertTrue(wait_for(lambda: self.dropped() == {"heartbeat": 1}), self.dropped())
        self.assertEqual(len(self.kept("heartbeat")), 1)


class TestSingleLinkSending(BridgeCase):
    """Everything sent to a vehicle goes down ONE link."""

    limits = "link_prefer_s = 0.3"

    def command(self, vid="UAV1", seq=5):
        cmd = rx_commands_pb2.RxCommand()
        cmd.team_id = "ASTA"
        cmd.seq = seq
        cmd.vehicle_id = vid
        cmd.keep_out_zone.SetInParent()
        self.bridge._route_command(cmd, "keep_out_zone")
        return cmd

    def got(self, client, key="keep_out_zone"):
        return wait_for(lambda: client.of_kind(key), 1.0) and client.of_kind(key)

    def test_a_command_goes_to_the_preferred_link_only(self):
        radio = self.connect("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        self.command()
        self.assertTrue(self.got(radio))
        time.sleep(0.15)
        self.assertEqual(wifi.of_kind("keep_out_zone"), [])
        self.assertEqual(self.bridge.recent[0]["routed"], [{"vehicle": "UAV1", "link": "radio"}])
        self.assertEqual(self.bridge.recent[0]["routes"], ["UAV1/radio"])

    def test_it_falls_to_the_other_link_when_the_preferred_has_gone_quiet(self):
        radio = self.connect("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        time.sleep(0.4)                                    # both quiet...
        wifi.send(heartbeat("UAV1"))                       # ...then only wifi speaks
        self.assertTrue(wait_for(lambda: time.monotonic()
                                 - self.bridge.tcp.links_of("UAV1")["wifi"]["last_rx"] < 0.1))
        self.command()
        self.assertTrue(self.got(wifi))
        time.sleep(0.15)
        self.assertEqual(radio.of_kind("keep_out_zone"), [])
        self.assertEqual(self.bridge.recent[0]["routes"], ["UAV1/wifi"])

    def test_the_only_connected_link_is_used_even_if_it_is_not_the_preferred_one(self):
        wifi = self.connect("UAV1", "wifi")
        self.command()
        self.assertTrue(self.got(wifi))

    def test_a_connected_but_quiet_socket_still_beats_mqtt(self):
        """A vehicle that stopped reporting (stale caches) is still listening."""
        radio = self.connect("UAV1", "radio")
        time.sleep(0.4)
        self.command()
        self.assertTrue(self.got(radio))
        self.assertEqual(self.bridge.team.published, [])

    def test_with_nothing_connected_a_command_falls_back_to_mqtt(self):
        self.command()
        self.assertEqual(len(self.bridge.team.published), 1)
        topic, payload, qos = self.bridge.team.published[0]
        self.assertEqual((topic, qos), ("team/robotx/ASTA/UAV1/command", 1))
        self.assertEqual(self.bridge.recent[0]["routed"], [{"vehicle": "UAV1", "link": "mqtt"}])

    def test_a_failed_send_tries_the_other_link_and_stops_at_the_first_success(self):
        self.connect("UAV1", "radio")
        self.connect("UAV1", "wifi")
        attempts = []

        def send(vid, payload, link=None):
            attempts.append(link)
            return link == "wifi"                          # the radio write fails

        self.bridge.tcp.send = send
        self.assertEqual(self.bridge._send_to_vehicle("UAV1", b"{}"), "wifi")
        self.assertEqual(attempts, ["radio", "wifi"])
        attempts.clear()
        self.bridge.tcp.send = lambda vid, payload, link=None: attempts.append(link) or True
        self.assertEqual(self.bridge._send_to_vehicle("UAV1", b"{}"), "radio")
        self.assertEqual(attempts, ["radio"], "a delivered frame must not be sent again on the other link")

    def test_a_send_pinned_to_one_link_does_not_fall_back_to_the_other(self):
        self.connect("UAV1", "radio")
        self.connect("UAV1", "wifi")
        attempts = []
        self.bridge.tcp.send = lambda vid, payload, link=None: attempts.append(link) or False
        self.assertIsNone(self.bridge._send_to_vehicle("UAV1", b"{}", link="wifi"))
        self.assertEqual(attempts, ["wifi"])
        attempts.clear()
        self.bridge.tcp.send = lambda vid, payload, link=None: attempts.append(link) or True
        self.assertEqual(self.bridge._send_to_vehicle("UAV1", b"{}", link="wifi"), "wifi")
        self.assertEqual(attempts, ["wifi"])

    def test_every_send_fails_means_none_and_the_caller_falls_back(self):
        self.connect("UAV1", "radio")
        self.bridge.tcp.send = lambda vid, payload, link=None: False
        self.assertIsNone(self.bridge._send_to_vehicle("UAV1", b"{}"))

    def test_the_standby_directive_goes_down_one_link(self):
        radio = self.connect("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        self.declared(("UAV1",))
        self.assertTrue(self.got(radio, "ocs_directive"))
        time.sleep(0.15)
        self.assertEqual(wifi.of_kind("ocs_directive"), [])

    def test_the_directive_names_the_link_in_the_log(self):
        self.connect("UAV1", "radio")
        self.declared(("UAV1",))
        self.assertTrue(any("standby_auto -> UAV1/radio" in m for m in self.logs), self.logs)

    def test_an_unaddressed_command_reaches_each_vehicle_once_on_its_own_best_link(self):
        usv = self.connect("USV1", None)
        radio = self.connect("UAV1", "radio")
        wifi = self.connect("UAV1", "wifi")
        cmd = rx_commands_pb2.RxCommand()
        cmd.seq = 8
        cmd.all_clear.SetInParent()
        self.bridge._route_command(cmd, "all_clear")
        self.assertTrue(self.got(usv, "all_clear"))
        self.assertTrue(self.got(radio, "all_clear"))
        time.sleep(0.15)
        self.assertEqual(wifi.of_kind("all_clear"), [])
        self.assertEqual(len(usv.of_kind("all_clear")), 1)
        self.assertEqual(len(radio.of_kind("all_clear")), 1)


class TestSnapshotLinks(BridgeCase):
    limits = "link_prefer_s = 0.3"

    def vehicle(self, vid="UAV1"):
        return next(v for v in self.bridge.snapshot()["vehicles"] if v["id"] == vid)

    def test_a_vehicle_with_no_socket_has_null_links_and_no_active_link(self):
        v = self.vehicle()
        self.assertEqual(v["links"], {"radio": None, "wifi": None})
        self.assertIsNone(v["active_link"])
        self.assertEqual(v["dropped_not_preferred"], 0)
        self.assertEqual(v["prefer"], "radio")

    def test_a_connected_link_reports_connected_and_the_age_of_its_last_frame(self):
        self.connect("UAV1", "radio", agent="fleet_link")
        time.sleep(0.1)
        radio = self.vehicle()["links"]["radio"]
        self.assertTrue(radio["connected"])
        self.assertEqual(radio["agent"], "fleet_link")
        self.assertGreaterEqual(radio["last_rx_age_s"], 0.05)
        self.assertLess(radio["last_rx_age_s"], 2.0)
        self.assertIsNone(self.vehicle()["links"]["wifi"])
        self.assertEqual(self.vehicle()["active_link"], "radio")

    def test_the_active_link_is_the_one_a_command_would_use(self):
        self.connect("UAV1", "radio")
        self.connect("UAV1", "wifi")
        self.assertEqual(self.vehicle()["active_link"], "radio")
        time.sleep(0.4)
        c = self.clients[1]                                # wifi speaks, radio stays quiet
        c.send(heartbeat("UAV1"))
        self.assertTrue(wait_for(lambda: self.vehicle()["active_link"] == "wifi"))

    def test_the_boat_without_a_hello_shows_wifi(self):
        self.connect("USV1", None)
        v = self.vehicle("USV1")
        self.assertIsNone(v["links"]["radio"])
        self.assertTrue(v["links"]["wifi"]["connected"])
        self.assertEqual(v["active_link"], "wifi")
        self.assertIsNone(v["prefer"])

    def test_the_drop_count_is_per_vehicle_and_in_the_totals(self):
        self.declared()
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")
        snap = self.bridge.snapshot()
        self.assertEqual(self.vehicle()["dropped_not_preferred"], 1)
        self.assertEqual(snap["counters"]["dropped_not_preferred"], 1)
        self.assertEqual(snap["counters"]["dropped_rate"], 0)
        self.assertEqual(snap["link_prefer_s"], 0.3)

    def test_a_closed_link_leaves_the_snapshot(self):
        radio = self.connect("UAV1", "radio")
        self.connect("UAV1", "wifi")
        radio.close()
        self.assertTrue(wait_for(lambda: self.vehicle()["links"]["radio"] is None))
        self.assertEqual(self.vehicle()["active_link"], "wifi")

    def test_the_snapshot_is_json_serialisable(self):
        self.connect("UAV1", "radio", synthesized=["heartbeat"])
        json.dumps(self.bridge.snapshot())

    def test_the_drop_count_is_split_by_kind(self):
        self.declared()
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")
        self.feed(heartbeat("UAV1"), "wifi")
        self.feed(readiness("UAV1", command_seq=1), "radio")
        self.feed(readiness("UAV1", command_seq=1), "wifi")
        v = self.vehicle()
        self.assertEqual(v["dropped_not_preferred"], 3)
        self.assertEqual(v["dropped_not_preferred_by_kind"], {"heartbeat": 2, "readiness": 1})
        self.assertEqual(self.vehicle("USV1")["dropped_not_preferred_by_kind"], {})

    def test_a_radio_adapter_is_labelled_by_what_it_is_not_by_its_address(self):
        """Its ip:port is the laptop's own loopback, and says nothing about the vehicle."""
        self.connect("UAV1", "radio", agent="fleet_link", synthesized=["heartbeat"])
        radio = self.vehicle()["links"]["radio"]
        self.assertEqual(radio["label"], "radio via fleet_link")
        self.assertTrue(radio["adapter"])
        self.assertTrue(radio["hello"])
        self.assertEqual(radio["synthesized"], ["heartbeat"])
        self.assertTrue(radio["peer"].startswith("127.0.0.1:"))      # the address is still there to read
        self.assertEqual(self.vehicle()["peer"], "radio via fleet_link")
        self.assertEqual(self.bridge.tcp.peers()["UAV1"], "radio via fleet_link")

    def test_an_adapter_with_no_agent_name_is_still_not_a_bare_address(self):
        self.connect("UAV1", "radio", agent="")
        self.assertEqual(self.vehicle()["links"]["radio"]["label"], "radio via an adapter")

    def test_a_vehicles_own_wifi_client_is_labelled_by_its_address(self):
        self.connect("USV1", None)
        wifi = self.vehicle("USV1")["links"]["wifi"]
        self.assertFalse(wifi["adapter"])
        self.assertFalse(wifi["hello"])
        self.assertEqual(wifi["synthesized"], [])
        self.assertEqual(wifi["label"], wifi["peer"])
        self.assertTrue(wifi["label"].startswith("127.0.0.1:"))

    def test_a_wifi_hello_is_the_vehicles_own_client_not_an_adapter(self):
        self.connect("UAV1", "wifi", agent="ocs_client")
        wifi = self.vehicle()["links"]["wifi"]
        self.assertTrue(wifi["hello"])
        self.assertFalse(wifi["adapter"])
        self.assertEqual(wifi["label"], wifi["peer"])

    def test_an_adapter_that_has_relayed_nothing_does_not_make_the_vehicle_present(self):
        """Connected is a fact about the laptop; heard is a fact about the vehicle."""
        self.connect("UAV1", "radio", agent="fleet_link")
        v = self.vehicle()
        radio = v["links"]["radio"]
        self.assertTrue(radio["connected"])
        self.assertIsNone(radio["last_report_age_s"])
        self.assertFalse(radio["fresh"])
        self.assertIsNone(v["silent_s"])
        self.assertEqual(v["report"], {})
        self.assertFalse(v["active"])
        self.assertNotIn("UAV1", self.bridge.snapshot()["active"])

    def test_a_report_on_a_link_makes_it_fresh_until_it_goes_quiet(self):
        radio = self.connect("UAV1", "radio", agent="fleet_link", synthesized=["heartbeat"])
        radio.send(heartbeat("UAV1"))
        self.assertTrue(wait_for(lambda: self.vehicle()["links"]["radio"]["fresh"]))
        link = self.vehicle()["links"]["radio"]
        self.assertLess(link["last_report_age_s"], 0.3)
        self.assertTrue(wait_for(lambda: not self.vehicle()["links"]["radio"]["fresh"]))   # link_prefer_s = 0.3
        link = self.vehicle()["links"]["radio"]
        self.assertGreaterEqual(link["last_report_age_s"], 0.3)      # rounded to 10 ms
        self.assertTrue(link["connected"])

    def test_a_hello_is_not_a_report_but_any_frame_is_activity(self):
        self.connect("UAV1", "radio", agent="fleet_link")
        time.sleep(0.1)
        radio = self.vehicle()["links"]["radio"]
        self.assertGreaterEqual(radio["last_rx_age_s"], 0.05)
        self.assertIsNone(radio["last_report_age_s"])

    def test_a_dropped_duplicate_still_counts_as_that_link_having_reported(self):
        """The page asks whether the link is alive, not whether its copy was kept."""
        self.declared()
        self.connect("UAV1", "radio")
        self.connect("UAV1", "wifi")
        self.feed(heartbeat("UAV1"), "radio", key="r")
        self.feed(heartbeat("UAV1"), "wifi", key="w")                # dropped as not preferred
        # fed frames carry no socket, so only the bridge's own table can answer
        self.assertEqual(set(self.bridge._last_report_rx("UAV1")), {"radio", "wifi"})

    def test_the_snapshot_keeps_one_entry_per_link_in_a_fixed_order(self):
        self.assertEqual(list(self.vehicle()["links"]), ["radio", "wifi"])

    def test_the_status_line_names_an_adapter_by_label_not_address(self):
        self.connect("UAV1", "radio", agent="fleet_link")
        self.assertIn("radio via fleet_link", self.bridge.status())

    def test_the_status_line_names_both_links_and_what_the_duplicates_cost(self):
        self.declared()
        self.connect("UAV1", "radio")
        self.connect("UAV1", "wifi")
        self.feed(heartbeat("UAV1"), "radio", key="r")
        self.feed(heartbeat("UAV1"), "wifi", key="w")
        self.assertIn("radio+wifi (+1 dup dropped)", self.bridge.status())


#: Runs index.html's own script against a stub DOM. argv[2] is the page. With "page",
#: stdin is a full /api/state body: refresh() runs on it and the stub elements' innerHTML
#: is printed. Otherwise stdin is a JSON list of snapshot vehicles and each vehicleCard(...)
#: is printed.
_CARD_JS = r"""
const fs = require('fs'), vm = require('vm');
const html = fs.readFileSync(process.argv[2], 'utf8');
const code = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)].map(m => m[1]).pop();
const input = JSON.parse(fs.readFileSync(0, 'utf8'));
const els = {};
const ctx = vm.createContext({
  document: {getElementById: id => (els[id] = els[id] || {style: {}}), querySelector: () => null,
             documentElement: {getAttribute: () => null, setAttribute() {}}},
  fetch: () => process.argv[3] === 'page'
    ? Promise.resolve({json: () => Promise.resolve(input)})
    : Promise.reject(new Error('no network in this test')),
  setInterval: () => 0, localStorage: {getItem: () => null, setItem() {}}});
vm.runInContext(code, ctx);
if (process.argv[3] === 'page') {
  ctx.refresh().then(() => process.stdout.write(JSON.stringify(
    Object.fromEntries(Object.entries(els).map(([k, v]) => [k, v.innerHTML || v.textContent || ''])))));
} else {
  process.stdout.write(JSON.stringify(input.map(v => ctx.vehicleCard(v, [], false))));
}
"""


@unittest.skipUnless(shutil.which("node"), "needs node to run the page's own script")
class TestFleetCard(BridgeCase):
    """The operator page's vehicle card, executed on what the bridge really snapshots.

    The data comes from Bridge.snapshot(), so a renamed field breaks here and not on
    the dock. Text is what the operator reads; markup is only checked where it matters.
    """

    limits = "link_prefer_s = 0.3"
    PAGE = Path(__file__).resolve().parents[1] / "ocs_web" / "index.html"

    @classmethod
    def setUpClass(cls):
        cls._js = tempfile.NamedTemporaryFile("w", suffix=".js", delete=False, encoding="utf-8")
        cls._js.write(_CARD_JS)
        cls._js.close()

    @classmethod
    def tearDownClass(cls):
        Path(cls._js.name).unlink(missing_ok=True)

    def node(self, stdin, *mode):
        out = subprocess.run(["node", self._js.name, str(self.PAGE), *mode], check=True, timeout=30,
                             input=json.dumps(stdin), capture_output=True, text=True,
                             encoding="utf-8")
        return json.loads(out.stdout)

    def cards(self, *vids):
        """{vehicle id: card html} from a fresh snapshot."""
        by_id = {v["id"]: v for v in self.bridge.snapshot()["vehicles"]}
        return dict(zip(vids, self.node([by_id[v] for v in vids])))

    def page(self):
        """{element id: innerHTML} after the page's refresh() has run on a real /api/state body."""
        state = self.bridge.snapshot()
        state["inject"] = {"enabled": False, "streaming": []}
        state["log"] = []
        return self.node(state, "page")

    def adapter_up(self):
        """Ekko's radio adapter, connected and synthesizing the heartbeat."""
        return self.connect("UAV1", "radio", agent="fleet_link", synthesized=["heartbeat"])

    def card(self, vid="UAV1"):
        return self.cards(vid)[vid]

    def link(self, name, vid="UAV1"):
        return next(v for v in self.bridge.snapshot()["vehicles"] if v["id"] == vid)["links"][name]

    @staticmethod
    def text(html):
        return re.sub(r"\s+", " ", re.sub(r"<[^>]+>", " ", html)).strip()

    def test_an_adapter_that_has_relayed_nothing_reads_as_adapter_up_vehicle_silent(self):
        self.adapter_up()
        text = self.text(self.card())
        self.assertIn("radio via fleet_link", text)
        self.assertIn("adapter up, vehicle silent", text)
        self.assertIn("never heard", text)                            # the vehicle itself: not present
        self.assertNotIn("127.0.0.1", text)                           # not "tcp 127.0.0.1:..."
        self.assertNotIn("tcp", text)
        self.assertNotIn("fresh", text)

    def test_the_card_shows_each_link_preferred_active_and_what_it_synthesizes(self):
        self.adapter_up()
        text = self.text(self.card())
        self.assertRegex(text, r"radio via fleet_link .*preferred active synthesizes heartbeat")
        self.assertRegex(text, r"wifi not connected")

    def test_a_link_that_is_preferred_but_not_connected_still_says_preferred(self):
        self.connect("UAV1", "wifi")
        text = self.text(self.card())
        self.assertRegex(text, r"radio not connected preferred")
        self.assertRegex(text, r"wifi 127\.0\.0\.1:\d+ .*active")

    def test_a_link_that_relays_reports_reads_fresh_then_silent(self):
        radio = self.adapter_up()
        radio.send(heartbeat("UAV1"))
        self.assertTrue(wait_for(lambda: self.link("radio")["fresh"]))
        text = self.text(self.card())
        self.assertRegex(text, r"fresh \d\.\d s")
        self.assertNotIn("vehicle silent", text)
        self.assertTrue(wait_for(lambda: not self.link("radio")["fresh"]))
        self.assertRegex(self.text(self.card()), r"adapter up, vehicle silent \d\.\d+ s")

    def test_a_vehicles_own_client_is_not_called_an_adapter(self):
        self.connect("USV1", None)
        text = self.text(self.card("USV1"))
        self.assertRegex(text, r"wifi 127\.0\.0\.1:\d+ fresh")
        self.assertNotIn("adapter", text)
        self.assertNotIn("preferred", text)                           # the boat has no preference

    def test_a_connected_client_that_has_sent_no_report_says_so(self):
        self.connect("UAV1", "wifi")                                  # a hello is not a report
        self.assertIn("connected, nothing reported", self.text(self.card()))

    def test_the_drop_counts_are_shown_per_kind(self):
        self.declared()
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")
        self.feed(heartbeat("UAV1"), "wifi")
        self.feed(readiness("UAV1", command_seq=1), "radio")
        self.feed(readiness("UAV1", command_seq=1), "wifi")
        self.assertRegex(self.text(self.card()), r"not pref\. 3 heartbeat 2, readiness 1")

    def test_a_vehicle_with_no_preference_and_no_drops_has_no_drops_row(self):
        self.assertNotIn("not pref.", self.text(self.card("USV1")))
        self.assertIn("not pref. 0", self.text(self.card("UAV1")))    # it has a preference: 0 is news

    def test_a_vehicle_with_no_socket_says_how_it_last_reported(self):
        from rx_bridge.proto import rx_reports_pb2
        from google.protobuf import json_format
        r = rx_reports_pb2.RxReport()
        json_format.ParseDict(heartbeat("USV1"), r)
        self.bridge._ingest_report(r, via="mqtt")
        text = self.text(self.card("USV1"))
        self.assertIn("radio not connected", text)
        self.assertIn("wifi not connected", text)
        self.assertIn("last report via mqtt", text)
        self.assertNotIn("last report via", self.text(self.card("UUV1")))   # never reported at all

    def test_an_agent_name_cannot_inject_markup(self):
        self.connect("UAV1", "radio", agent="<img src=x onerror=alert(1)>")
        html = self.card()
        self.assertNotIn("<img", html)
        self.assertIn("&lt;img", html)

    def test_the_whole_page_renders_a_real_snapshot_and_the_counter_row_lines_up(self):
        self.declared()
        self.adapter_up()
        self.feed(heartbeat("UAV1"), "radio")
        self.feed(heartbeat("UAV1"), "wifi")                          # one not-preferred drop
        els = self.page()
        header = re.search(r"<tr><th>forwarded</th>.*?</tr>", self.PAGE.read_text(encoding="utf-8"), re.S)
        self.assertEqual(els["counters"].count("<td>"), header.group(0).count("<th"))
        self.assertRegex(self.text(els["counters"]), r"^\d+ \d+ \d+ \d+ \d+ 1 \d+ \d+ \d+$")
        self.assertIn("UAV1", els["fleet"])
        self.assertIn("radio via fleet_link", els["fleet"])

    def test_it_adds_no_colour_of_its_own(self):
        """Day and night are one set of variables; a literal colour would break one of them."""
        self.adapter_up()
        self.connect("UAV1", "wifi")
        html = self.card()
        self.assertNotRegex(html, r"#[0-9a-fA-F]{3,8}\b|rgb\(")
        self.assertEqual(set(re.findall(r'class="pill (\w+)"', html)) - {"ok", "warn", "bad", "idle"}, set())


class TestTcpServerSending(unittest.TestCase):
    """tcpserver on its own: frames from several threads must not interleave."""

    def test_concurrent_senders_never_tear_a_frame(self):
        srv = TcpServer("127.0.0.1", 0, on_frame=lambda payload, conn: "V1", log=lambda m: None)
        srv.start()
        self.addCleanup(srv.stop)
        c = VehicleClient(srv.bound_port)
        self.addCleanup(c.close)
        c.send({"hi": 1})
        self.assertTrue(wait_for(lambda: "wifi" in srv.links_of("V1")))
        blob = "x" * 100_000                         # big enough to need several send() calls

        def spam(tag):
            for i in range(10):
                srv.send("V1", json.dumps({"tag": tag, "i": i, "blob": blob}).encode(), link="wifi")

        threads = [threading.Thread(target=spam, args=(t,)) for t in range(4)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertTrue(wait_for(lambda: len(c.frames) == 40, 10.0), len(c.frames))
        self.assertTrue(all(f["blob"] == blob for f in c.frames))

    def test_a_send_to_a_vehicle_that_is_not_there_is_false(self):
        srv = TcpServer("127.0.0.1", 0, on_frame=lambda p, c: None, log=lambda m: None)
        self.assertFalse(srv.send("NOBODY", b"{}"))
        self.assertFalse(srv.send("NOBODY", b"{}", link="radio"))


if __name__ == "__main__":
    unittest.main()
