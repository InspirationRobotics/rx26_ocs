"""ocs_receipt (Phase C): what the OCS tells a vehicle about each task report it numbered,
and the resend that a final receipt ends.

The boat's ocs_link numbers its task reports and, after a reconnect, sends again any
report it has no final receipt for. So the OCS must answer a resend from what became of
the first -- never publish it twice -- and must tell the vehicle, on the link the report
came in on, each verdict: held, dropped, published, acknowledged.
"""
from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from tests.harness import BridgeCase, readiness   # noqa: E402

LATER = "2026-10-31T23:00:00.000000Z"


class TestReceipts(BridgeCase):
    def setUp(self) -> None:
        super().setUp()
        self.sent: list[tuple[str, str, dict]] = []
        self.bridge.tcp.send = lambda vid, payload, link=None: self.sent.append(
            (vid, link, json.loads(payload))) or True

    def receipts(self) -> list[tuple[str, str, dict]]:
        return [(vid, link, f["ocs_receipt"]) for vid, link, f in self.sent
                if "ocs_receipt" in f]

    def verdicts(self) -> list[str]:
        return [r["verdict"] for _vid, _link, r in self.receipts()]

    def ack_last(self) -> None:
        self.bridge._on_rc_publish(None, None, self.bridge.rc.mids[-1])

    def test_before_declaring_the_vehicle_hears_held(self):
        self.feed(readiness("USV1", command_seq=3, seq=7))
        [(vid, link, r)] = self.receipts()
        self.assertEqual((vid, link), ("USV1", "wifi"))
        self.assertEqual((r["vehicle_seq"], r["kind"], r["verdict"], r["seq"]),
                         (7, "readiness", "held", None))
        self.assertIn("not declared", r["detail"])

    def test_published_then_acknowledged_by_the_broker(self):
        self.declared()
        self.feed(readiness("USV1", command_seq=3, seq=7))
        self.assertEqual(self.verdicts(), ["published"])
        ocs_seq = self.receipts()[0][2]["seq"]
        self.assertTrue(ocs_seq)
        self.ack_last()
        self.assertEqual(self.verdicts(), ["published", "acknowledged"])
        self.assertEqual(self.receipts()[1][2]["seq"], ocs_seq)

    def test_a_resend_is_answered_and_not_published_twice(self):
        self.declared()
        report = readiness("USV1", command_seq=3, seq=7)
        self.feed(report)
        self.ack_last()
        published = len(self.bridge.rc.published)
        self.feed(report)                                 # the same frame, after a reconnect
        self.assertEqual(len(self.bridge.rc.published), published)
        self.assertEqual(self.verdicts(), ["published", "acknowledged", "acknowledged"])
        self.assertEqual(len([e for e in self.bridge.snapshot()["task_reports"]
                              if e["vehicle_seq"] == 7]), 1)

    def test_a_resend_before_the_puback_hears_published_and_then_the_ack(self):
        self.declared()
        report = readiness("USV1", command_seq=3, seq=7)
        self.feed(report)
        self.feed(report)
        self.ack_last()
        self.assertEqual(self.verdicts(), ["published", "published", "acknowledged"])

    def test_the_same_seq_with_a_new_sent_at_is_a_new_report(self):
        # A restarted node numbering from where an old one did: not a resend.
        self.declared()
        self.feed(readiness("USV1", command_seq=3, seq=7))
        later = readiness("USV1", command_seq=3, seq=7)
        later["sent_at"] = LATER
        published = len(self.bridge.rc.published)
        self.feed(later)
        self.assertEqual(len(self.bridge.rc.published), published + 1)

    def test_a_report_without_its_own_seq_gets_no_receipt(self):
        self.declared()
        self.feed(readiness("USV1", command_seq=3))
        self.ack_last()
        self.assertEqual(self.receipts(), [])

    def test_never_over_the_radio(self):
        self.feed(readiness("USV1", command_seq=3, seq=7), "radio")
        self.assertEqual(self.receipts(), [])

    def test_a_receipt_no_socket_took_goes_when_the_link_comes_up(self):
        # A report that is a connection's first frame is handled before tcpserver
        # files the socket under its vehicle: its receipt has nowhere to go yet.
        self.declared()
        filed = []
        self.bridge.tcp.send = lambda vid, payload, link=None: bool(filed) and (
            self.sent.append((vid, link, json.loads(payload))) or True)
        self.feed(readiness("USV1", command_seq=3, seq=7))
        self.assertEqual(self.receipts(), [])
        filed.append(True)
        self.bridge._on_link_change("USV1", "wifi", True, self._fed["wifi"])
        self.assertEqual(self.verdicts(), ["published"])

    def test_the_receipt_goes_back_on_the_link_the_report_came_in_on(self):
        self.declared()
        self.feed(readiness("UAV1", command_seq=3, seq=9), "wifi", key="uav-wifi")
        self.assertEqual({link for _vid, link, _r in self.receipts()}, {"wifi"})


if __name__ == "__main__":
    unittest.main()
