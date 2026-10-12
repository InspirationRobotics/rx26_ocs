"""ReadinessConfirm's sequence reference, and the evidence log.

Handbook 3.4: "the OCS must preserve the command and report sequence references".
We re-assign every report's seq, so RoboCommand's ReadinessConfirm.report_seq is OUR
number; the vehicle is told its own. And the wire log must record every frame in
both directions, decoded, without ever being able to stop the hot path.
"""
from __future__ import annotations

import json
import sys
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge import bridge as bridge_module                    # noqa: E402
from rx_bridge.proto import rx_commands_pb2, rx_reports_pb2      # noqa: E402
from rx_bridge.wirelog import WireLog                            # noqa: E402

from tests.harness import (BridgeCase, confirm_command, course_bytes,  # noqa: E402
                           heartbeat, incident_ack, readiness, wait_for)


def confirms(client):
    return [f for f in client.of_kind("readiness_confirm")]


class TestReadinessConfirm(BridgeCase):

    # These tests are about seq rewriting, not rate limiting. Their frames arrive
    # back to back, so at the default rate (5 tokens, 2 reserved for heartbeats)
    # three heartbeats leave too little for the readiness report and the governor
    # -- tested on its own -- drops it before the seq map ever sees it.
    limits = "rate = 1000.0"

    def setUp(self):
        super().setUp()
        self.declared()

    def readiness_report(self, own_seq, n_heartbeats=3):
        """Heartbeats first so OUR seq for the readiness report differs from the vehicle's."""
        for _ in range(n_heartbeats):
            self.feed(heartbeat("UAV1"), "radio")
        self.feed(readiness("UAV1", command_seq=12, seq=own_seq), "radio")
        return self.reports_to_rc("UAV1")[-1].seq

    def test_the_confirm_carries_the_vehicles_own_seq_and_goes_to_that_vehicle_only(self):
        usv = self.connect("USV1", None)
        uav = self.connect("UAV1", "radio")
        ours = self.readiness_report(own_seq=7)
        self.assertEqual(ours, 4)
        self.bridge._handle_command(confirm_command(vehicle="UAV1", report_seq=ours).SerializeToString())
        self.assertTrue(wait_for(lambda: confirms(uav)))
        self.assertEqual(confirms(uav)[0]["readiness_confirm"], {"report_seq": 7, "vehicle_id": "UAV1"})
        self.assertEqual(confirms(uav)[0]["seq"], 90, "the command's own seq passes through")
        time.sleep(0.15)
        self.assertEqual(confirms(usv), [], "an unaddressed confirm must still go only to the vehicle it names")
        self.assertEqual(self.bridge.recent[0]["rewrite"],
                         {"vehicle": "UAV1", "ocs_seq": 4, "vehicle_seq": 7})

    def test_both_numbers_are_in_the_wire_log(self):
        self.connect("UAV1", "radio")
        ours = self.readiness_report(own_seq=7)
        self.bridge._handle_command(confirm_command(vehicle="UAV1", report_seq=ours).SerializeToString())
        evt = [l for l in self.wire_lines() if l.get("kind") == "readiness_seq"][-1]
        self.assertIn("4", evt["detail"])
        self.assertIn("7", evt["detail"])
        vtx = [l for l in self.wire_lines() if l["dir"] == "vtx" and "readiness_confirm" in l["note"]][-1]
        self.assertIn("ours=4", vtx["note"])
        self.assertIn("vehicle's=7", vtx["note"])
        self.assertEqual(vtx["decoded"]["readiness_confirm"]["report_seq"], 7)

    def test_a_vehicle_that_sent_no_seq_gets_the_confirm_unchanged(self):
        uav = self.connect("UAV1", "radio")
        ours = self.readiness_report(own_seq=0)
        self.bridge._handle_command(confirm_command(vehicle="UAV1", report_seq=ours).SerializeToString())
        self.assertTrue(wait_for(lambda: confirms(uav)))
        self.assertEqual(confirms(uav)[0]["readiness_confirm"]["report_seq"], ours)
        self.assertIsNone(self.bridge.recent[0]["rewrite"])

    def test_an_unknown_seq_is_relayed_as_received_and_said_so(self):
        uav = self.connect("UAV1", "radio")
        self.bridge._handle_command(confirm_command(vehicle="UAV1", report_seq=999).SerializeToString())
        self.assertTrue(wait_for(lambda: confirms(uav)))
        self.assertEqual(confirms(uav)[0]["readiness_confirm"]["report_seq"], 999)
        self.assertTrue(any("no report seq=999" in m for m in self.logs))

    def test_the_vehicle_is_found_by_seq_alone_only_when_that_is_unambiguous(self):
        usv, uav = self.connect("USV1", None), self.connect("UAV1", "radio")
        for _ in range(2):
            self.feed(heartbeat("UAV1"), "radio")
        self.feed(readiness("UAV1", command_seq=1, seq=5), "radio")        # UAV1's seq 3
        self.bridge._handle_command(confirm_command(vehicle="", report_seq=3).SerializeToString())
        self.assertTrue(wait_for(lambda: confirms(uav)))
        self.assertEqual(confirms(uav)[0]["readiness_confirm"]["report_seq"], 5)
        self.assertEqual(confirms(usv), [])

    def test_only_non_heartbeat_reports_are_remembered_and_the_map_is_bounded(self):
        for _ in range(20):
            self.feed(heartbeat("UAV1"), "radio")
        self.assertEqual(len(self.bridge._seq_map), 0)
        old = bridge_module._SEQ_MAP_MAX
        bridge_module._SEQ_MAP_MAX = 4
        self.addCleanup(setattr, bridge_module, "_SEQ_MAP_MAX", old)
        for i in range(10):
            self.feed(readiness("UAV1", command_seq=i, seq=100 + i), "radio")
        self.assertEqual(len(self.bridge._seq_map), 4)

    def test_the_mqtt_fallback_carries_the_rewritten_seq_too(self):
        ours = self.readiness_report(own_seq=7)                  # no vehicle socket connected
        self.bridge._handle_command(confirm_command(vehicle="UAV1", report_seq=ours).SerializeToString())
        topic, payload, _ = self.bridge.team.published[-1]
        self.assertEqual(topic, "team/robotx/ASTA/UAV1/command")
        cmd = rx_commands_pb2.RxCommand()
        cmd.ParseFromString(payload)
        self.assertEqual(cmd.readiness_confirm.report_seq, 7)

    def test_command_seq_passes_through_both_ways(self):
        """IncidentAck.command_seq refers to RxCommand.seq; neither is translated."""
        uav = self.connect("UAV1", "radio")
        cmd = rx_commands_pb2.RxCommand()
        cmd.team_id, cmd.seq, cmd.vehicle_id = "ASTA", 77, "UAV1"
        cmd.assistance_request.SetInParent()
        self.bridge._handle_command(cmd.SerializeToString())
        self.assertTrue(wait_for(lambda: uav.of_kind("assistance_request")))
        self.assertEqual(uav.of_kind("assistance_request")[0]["seq"], 77)
        self.feed(incident_ack("UAV1", command_seq=77), "radio")
        self.assertEqual(self.reports_to_rc("UAV1")[-1].incident_ack.command_seq, 77)


class TestWireLogUnit(unittest.TestCase):
    def setUp(self):
        import tempfile
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.log = WireLog(self._tmp.name, run_tag="T")
        self.addCleanup(self.log.close)

    def lines(self):
        return [json.loads(l) for l in self.log.path.read_text(encoding="utf-8").splitlines()]

    def test_a_frame_keeps_the_bytes_and_the_decoded_form(self):
        self.log.frame("veh", "t", b"abc", decoded={"a": 1}, note="n")
        line = self.lines()[0]
        self.assertEqual((line["b64"], line["decoded"], line["note"]), ("YWJj", {"a": 1}, "n"))

    def test_a_decode_that_raises_costs_the_decode_not_the_line(self):
        def boom():
            raise ValueError("no")
        self.log.frame("tx", "t", b"abc", decoded=boom)
        line = self.lines()[0]
        self.assertEqual((line["b64"], line["decoded"]), ("YWJj", ""))
        self.assertEqual(self.log.errors, 1)
        self.assertIn("ValueError", self.log.last_error)

    def test_a_dead_file_handle_is_counted_never_raised(self):
        self.log._fh.close()
        self.log.frame("tx", "t", b"x")
        self.log.event("k", "d")
        self.assertEqual(self.log.errors, 2)

    def test_an_unserialisable_value_is_still_written(self):
        self.log.frame("tx", "t", b"x", decoded={"s": {1, 2}})
        self.assertEqual(len(self.lines()), 1)

    def test_events_carry_extra_fields(self):
        self.log.event("declare", "d", run_id=None, declaration_seq=3)
        line = self.lines()[0]
        self.assertEqual((line["dir"], line["kind"], line["declaration_seq"]), ("evt", "declare", 3))


class TestWireLogInTheBridge(BridgeCase):

    def of_dir(self, d):
        return [l for l in self.wire_lines() if l["dir"] == d]

    def test_every_roboCommand_frame_in_and_out_is_decoded(self):
        self.bridge.machine.on_connect()
        self.bridge._handle_course(course_bytes("ALPHA"))
        rx = self.of_dir("rx")[0]
        self.assertEqual(rx["decoded"]["course_id"], "ALPHA")
        self.assertTrue(rx["b64"])
        self.declared()
        decl = [l for l in self.of_dir("tx") if "RunDeclaration" in l["note"]][0]
        self.assertEqual(decl["decoded"]["run_declaration"]["vehicle_ids"], ["USV1", "UAV1"])
        self.feed(heartbeat("USV1"), "wifi")
        hb = [l for l in self.of_dir("tx") if l["note"].endswith("heartbeat")][0]
        self.assertEqual(hb["decoded"]["vehicle_id"], "USV1")
        run_start = rx_commands_pb2.RxCommand()
        run_start.seq, run_start.run_start.declaration_seq, run_start.run_start.run_id = 9, 1, 42
        self.bridge._handle_command(run_start.SerializeToString())
        self.assertEqual(self.of_dir("rx")[-1]["decoded"]["run_start"], {"declaration_seq": 1, "run_id": 42})

    def test_vehicle_frames_in_and_out_are_decoded(self):
        self.declared()
        self.feed(heartbeat("USV1"), "wifi")
        veh = self.of_dir("veh")[-1]
        self.assertEqual(veh["decoded"]["heartbeat"]["state"], "STATE_AUTO")
        self.assertEqual(veh["topic"], "tcp://in/wifi")
        self.assertTrue(veh["b64"])
        cmd = rx_commands_pb2.RxCommand()
        cmd.seq, cmd.vehicle_id = 4, "USV1"
        cmd.all_clear.SetInParent()
        self.bridge._route_command(cmd, "all_clear")
        vtx = self.of_dir("vtx")[-1]
        self.assertEqual((vtx["decoded"]["seq"], vtx["topic"]), (4, "team/robotx/ASTA/USV1/command"))

    def test_hello_and_link_changes_are_logged(self):
        c = self.connect("UAV1", "radio", agent="fleet_link")
        hello = [l for l in self.of_dir("veh") if l["note"].startswith("ocs_hello")][0]
        self.assertEqual(hello["decoded"]["ocs_hello"]["agent"], "fleet_link")
        self.assertTrue(any(l["kind"] == "link_up" for l in self.of_dir("evt")))
        c.close()
        self.assertTrue(wait_for(lambda: any(l["kind"] == "link_down" for l in self.of_dir("evt"))))

    def test_inbound_team_mqtt_reports_are_logged_raw_and_decoded(self):
        report = rx_reports_pb2.RxReport()
        report.team_id, report.vehicle_id = "ASTA", "USV1"
        report.heartbeat.state = 1

        class Msg:
            topic = "team/robotx/ASTA/USV1/report"
            payload = report.SerializeToString()

        self.bridge._on_team_message(None, None, Msg)
        line = [l for l in self.of_dir("veh") if l["note"].startswith("team mqtt")][0]
        self.assertEqual(line["topic"], Msg.topic)
        self.assertTrue(line["b64"])
        self.assertEqual(line["decoded"]["vehicle_id"], "USV1")

        class Bad:
            topic = "team/robotx/ASTA/USV1/report"
            payload = b"\xff\xff\xff"

        self.bridge._on_team_message(None, None, Bad)
        self.assertIn("undecodable", self.of_dir("veh")[-1]["note"])

    def test_run_boundaries_write_events_carrying_the_run_id(self):
        self.declared()
        run_start = rx_commands_pb2.RxCommand()
        run_start.seq, run_start.run_start.declaration_seq, run_start.run_start.run_id = 9, 1, 42
        self.bridge._handle_command(run_start.SerializeToString())
        self.bridge.end()
        evts = {l["kind"]: l for l in self.of_dir("evt") if l["kind"] in ("declare", "run_start", "end")}
        self.assertEqual(set(evts), {"declare", "run_start", "end"})
        self.assertEqual(evts["declare"]["declaration_seq"], 1)
        self.assertIsNone(evts["declare"]["run_id"])
        self.assertEqual(evts["run_start"]["run_id"], 42)
        self.assertEqual(evts["end"]["run_id"], 42)
        self.assertEqual(evts["end"]["state"], "ENDED")

    def test_a_failing_log_cannot_stop_the_hot_path_or_the_link(self):
        self.declared()
        c = self.connect("USV1", None)
        self.bridge.wire._fh.close()                       # every write now raises
        before = self.bridge.counters.forwarded
        c.send(heartbeat("USV1"))
        self.assertTrue(wait_for(lambda: self.bridge.counters.forwarded > before))
        self.assertFalse(c.closed.is_set())
        self.assertGreater(self.bridge.wire.errors, 0)
        snap = self.bridge.snapshot()
        self.assertGreater(snap["wire"]["errors"], 0)
        self.assertIn("WRITE ERROR", self.bridge.status())

    def test_a_decode_that_raises_cannot_stop_the_hot_path(self):
        self.declared()
        real = bridge_module._wire_dict

        def boom(msg):
            raise RuntimeError("decode bug")

        bridge_module._wire_dict = boom
        self.addCleanup(setattr, bridge_module, "_wire_dict", real)
        self.feed(heartbeat("USV1"), "wifi")
        self.assertEqual(len(self.reports_to_rc("USV1")), 1)
        self.assertGreater(self.bridge.wire.errors, 0)
        line = [l for l in self.of_dir("tx") if l["note"].endswith("heartbeat")][-1]
        self.assertTrue(line["b64"], "the raw bytes survive a failed decode")


if __name__ == "__main__":
    unittest.main()
