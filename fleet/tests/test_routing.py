import unittest

from fleet import mavframe as mf
from fleet.hub import Dedup, Hub, OutputCfg
from fleet.tests.helpers import (DIALECT, FakeLink, FakeOutput, Rig, build_frame, command_long,
                                 frames_of, gcs_heartbeat, heartbeat, mission_request_list)
from fleet.vehicles import VehicleNames

BOAT_A = ("192.168.100.5", 51001)         # the boat's MAVProxy unicast socket
BOAT_B = ("192.168.100.5", 51002)         # ...and its broadcast socket
BOAT_COMPANION = ("192.168.100.6", 40000)


class Fanout(unittest.TestCase):
    def test_wifi_frames_follow_the_sysid_filters(self):
        r = Rig()
        r.heard("wifi", heartbeat(2, seq=1), BOAT_A)
        r.heard("wifi", heartbeat(42, seq=1), BOAT_A)
        r.heard("wifi", heartbeat(1, seq=1), BOAT_A)
        self.assertEqual(r.got_sysids("qgc"), [1, 2, 42])          # all sysids
        self.assertEqual(r.got_sysids("boat_gui"), [2])            # sysids = [2]
        self.assertEqual(r.got("drone_gui"), [])                   # rfd only
        self.assertEqual(r.got("drone_diag"), [])

    def test_rfd_frames_go_to_the_drone_outputs_and_qgc_not_the_boat_gui(self):
        r = Rig()
        for sysid in (1, 200, 42, 51):
            r.heard("rfd", heartbeat(sysid, seq=sysid))
        for name in ("qgc", "drone_gui", "drone_diag"):
            self.assertEqual(r.got_sysids(name), [1, 42, 51, 200], name)
        self.assertEqual(r.got("boat_gui"), [])

    def test_forwarded_frames_are_bit_identical(self):
        r = Rig()
        raw = heartbeat(1, seq=77)
        r.heard("rfd", raw)
        self.assertEqual(r.out["qgc"].sent, [raw])

    def test_no_link_to_link_bridging(self):
        r = Rig()
        r.heard("wifi", heartbeat(2), BOAT_A)
        r.heard("rfd", heartbeat(1))
        self.assertEqual(r.wifi.sent, [])
        self.assertEqual(r.rfd.sent, [])

    def test_whole_frames_per_datagram_even_when_a_chunk_holds_many(self):
        r = Rig()
        chunk = b"".join(heartbeat(1, seq=i) for i in range(120))      # ~2.5 kB
        r.heard("rfd", chunk)
        datagrams = r.out["qgc"].sent
        self.assertGreater(len(datagrams), 1)
        for d in datagrams:
            self.assertLessEqual(len(d), mf.MAX_DATAGRAM)
        self.assertEqual(len(frames_of(datagrams)), 120)
        self.assertEqual(r.hub.snapshot()["outputs"]["qgc"]["frames_out"], 120)

    def test_a_serial_chunk_that_ends_mid_frame_waits_for_the_rest(self):
        r = Rig()
        raw = heartbeat(1)
        r.heard("rfd", raw[:7])
        self.assertEqual(r.out["qgc"].sent, [])
        r.heard("rfd", raw[7:])
        self.assertEqual(r.out["qgc"].sent, [raw])

    def test_a_refusing_output_is_counted_not_fatal(self):
        r = Rig()
        r.out["qgc"].accept = False
        r.heard("rfd", heartbeat(1))
        self.assertEqual(r.hub.snapshot()["outputs"]["qgc"]["send_errors"], 1)
        self.assertEqual(len(r.got("drone_gui")), 1)

    def test_unvalidated_frames_are_forwarded(self):
        r = Rig()
        mid = next(i for i in range(59999, 0, -1) if i not in DIALECT)
        raw = build_frame(mid, b"abc", crc_extra=9, sysid=77)
        r.heard("rfd", raw)
        self.assertEqual(r.out["qgc"].sent, [raw])
        self.assertEqual(r.hub.snapshot()["links"]["rfd"]["unknown_id"], 1)

    def test_unvalidated_frames_teach_the_hub_nothing(self):
        r = Rig()
        mid = next(i for i in range(59999, 0, -1) if i not in DIALECT)
        r.heard("wifi", build_frame(mid, b"abc", crc_extra=9, sysid=77), BOAT_A)
        self.assertEqual(r.hub.snapshot()["sysids"], {})           # no invented vehicle
        r.from_consumer("qgc", command_long(77))
        self.assertEqual(r.hub.snapshot()["outputs"]["qgc"]["no_route"], 1)
        # and the junk's address is not where broadcast replies go
        r.from_consumer("qgc", gcs_heartbeat())
        self.assertEqual(r.wifi.sent, [])


class Dedupe(unittest.TestCase):
    def test_duplicate_from_a_second_source_port_is_dropped(self):
        r = Rig()
        raw = heartbeat(2, seq=5)
        r.heard("wifi", raw, BOAT_A)                    # the unicast copy
        r.heard("wifi", raw, BOAT_B)                    # the broadcast copy
        self.assertEqual(r.out["qgc"].sent, [raw])
        link = r.hub.snapshot()["links"]["wifi"]
        self.assertEqual((link["frames_in"], link["dups"]), (1, 1))

    def test_window_is_one_second(self):
        r = Rig()
        raw = heartbeat(2, seq=5)
        r.heard("wifi", raw, BOAT_A)
        r.clock.advance(1.0)
        r.heard("wifi", raw, BOAT_A)                    # exactly at the edge: still a dup
        self.assertEqual(len(r.out["qgc"].sent), 1)
        r.clock.advance(0.1)
        r.heard("wifi", raw, BOAT_A)                    # 1.1 s after the one we accepted
        self.assertEqual(len(r.out["qgc"].sent), 2)

    def test_a_duplicate_does_not_extend_the_window(self):
        d = Dedup(1.0)
        self.assertFalse(d.seen(b"x", 0.0))
        self.assertTrue(d.seen(b"x", 0.9))
        self.assertFalse(d.seen(b"x", 1.2))             # 0.9 did not refresh it

    def test_different_seq_is_not_a_duplicate(self):
        r = Rig()
        r.heard("wifi", heartbeat(2, seq=1), BOAT_A)
        r.heard("wifi", heartbeat(2, seq=2), BOAT_A)
        self.assertEqual(len(r.out["qgc"].sent), 2)

    def test_dedup_is_per_link(self):
        r = Rig()
        raw = heartbeat(42, seq=5)
        r.heard("wifi", raw, BOAT_A)
        r.heard("rfd", raw)                             # same bytes, other link: both are real
        self.assertEqual(len(r.out["qgc"].sent), 2)

    def test_memory_is_bounded(self):
        d = Dedup(1000.0, capacity=100)
        for i in range(1000):
            d.seen(i.to_bytes(4, "little"), i * 0.001)
        self.assertLessEqual(len(d), 100)

    def test_a_datagram_holding_a_frame_twice_forwards_it_once(self):
        r = Rig()
        raw = heartbeat(2, seq=3)
        r.heard("wifi", raw + raw, BOAT_A)
        self.assertEqual(r.out["qgc"].sent, [raw])


class ListenOnly(unittest.TestCase):
    def test_listen_only_outputs_drop_and_count_everything(self):
        r = Rig()
        r.heard("rfd", heartbeat(1))
        for name in ("drone_diag", "boat_gui"):
            r.from_consumer(name, command_long(1))
            r.from_consumer(name, gcs_heartbeat())
        self.assertEqual(r.rfd.sent + r.wifi.sent, [])
        snap = r.hub.snapshot()["outputs"]
        self.assertEqual(snap["drone_diag"]["dropped_listen_only"], 2)
        self.assertEqual(snap["drone_diag"]["frames_in_from_consumer"], 2)
        self.assertEqual(snap["boat_gui"]["dropped_listen_only"], 2)
        self.assertEqual(snap["qgc"]["dropped_listen_only"], 0)
        self.assertEqual(snap["drone_diag"]["no_route"], 0)


class ReverseRouting(unittest.TestCase):
    def setUp(self):
        self.r = Rig()
        self.r.heard("wifi", heartbeat(2), BOAT_A)       # the boat, on WiFi
        self.r.heard("rfd", heartbeat(1))                # Ekko, on the radio
        self.r.heard("rfd", heartbeat(42, seq=1))        # the boat's companion, on the radio too

    def out_counts(self, name="qgc"):
        return self.r.hub.snapshot()["outputs"][name]

    def test_targeted_frame_goes_only_to_the_link_that_heard_the_target(self):
        r = self.r
        r.from_consumer("qgc", command_long(2))
        self.assertEqual([a for _, a in r.wifi.sent], [BOAT_A])
        self.assertEqual(r.rfd.sent, [])
        r.wifi.sent.clear()
        r.from_consumer("qgc", command_long(1, seq=1))
        self.assertEqual(r.wifi.sent, [])
        self.assertEqual([d for d, _ in r.rfd.sent], [command_long(1, seq=1)])

    def test_sent_bytes_are_the_consumers_bytes_unchanged(self):
        raw = mission_request_list(1)
        self.r.from_consumer("qgc", raw)
        self.assertEqual(self.r.rfd.sent, [(raw, None)])

    def test_never_heard_sysid_is_dropped_and_counted(self):
        r = self.r
        r.from_consumer("qgc", command_long(9))
        self.assertEqual((r.wifi.sent, r.rfd.sent), ([], []))
        self.assertEqual(self.out_counts()["no_route"], 1)

    def test_route_goes_stale_after_30_seconds(self):
        r = self.r
        r.clock.advance(29.0)
        r.from_consumer("qgc", command_long(1))
        self.assertEqual(len(r.rfd.sent), 1)
        r.clock.advance(2.0)                              # 31 s since sysid 1 was heard
        r.from_consumer("qgc", command_long(1, seq=1))
        self.assertEqual(len(r.rfd.sent), 1)
        self.assertEqual(self.out_counts()["no_route"], 1)
        r.heard("rfd", heartbeat(1, seq=1))               # heard again -> routable again
        r.from_consumer("qgc", command_long(1, seq=2))
        self.assertEqual(len(r.rfd.sent), 2)

    def test_the_route_is_only_looked_up_among_the_outputs_own_links(self):
        r = self.r
        # The boat (sysid 2) is heard on wifi, but drone_gui is rfd-only.
        r.from_consumer("drone_gui", command_long(2))
        self.assertEqual((r.wifi.sent, r.rfd.sent), ([], []))
        self.assertEqual(self.out_counts("drone_gui")["no_route"], 1)
        r.from_consumer("drone_gui", mission_request_list(1))        # gcs_radio's fence read
        self.assertEqual(len(r.rfd.sent), 1)

    def test_target_zero_goes_to_every_link_of_the_output(self):
        r = self.r
        r.from_consumer("qgc", command_long(0))
        self.assertEqual(len(r.wifi.sent), 1)
        self.assertEqual(len(r.rfd.sent), 1)
        r.wifi.sent.clear()
        r.rfd.sent.clear()
        r.from_consumer("drone_gui", command_long(0))
        self.assertEqual((len(r.wifi.sent), len(r.rfd.sent)), (0, 1))   # drone_gui lists only rfd

    def test_message_without_a_target_field_goes_to_every_link(self):
        r = self.r
        r.from_consumer("qgc", gcs_heartbeat())
        self.assertEqual(len(r.wifi.sent), 1)
        self.assertEqual(len(r.rfd.sent), 1)

    def test_unknown_msgid_goes_to_every_link(self):
        r = self.r
        mid = next(i for i in range(59999, 0, -1) if i not in DIALECT)
        r.from_consumer("qgc", build_frame(mid, b"abc", crc_extra=9, sysid=255))
        self.assertEqual((len(r.wifi.sent), len(r.rfd.sent)), (1, 1))

    def test_most_recently_heard_link_wins_when_a_sysid_is_on_both(self):
        r = self.r
        r.clock.advance(1.0)
        r.heard("wifi", heartbeat(42, seq=2), BOAT_COMPANION)       # 42 now also on wifi, newer
        r.from_consumer("qgc", command_long(42))
        self.assertEqual([a for _, a in r.wifi.sent], [BOAT_COMPANION])
        self.assertEqual(r.rfd.sent, [])
        r.clock.advance(1.0)
        r.heard("rfd", heartbeat(42, seq=3))                        # and now the radio is newer
        r.wifi.sent.clear()
        r.from_consumer("qgc", command_long(42, seq=1))
        self.assertEqual((len(r.wifi.sent), len(r.rfd.sent)), (0, 1))

    def test_wifi_reply_goes_to_the_most_recent_source_address(self):
        r = self.r
        r.heard("wifi", heartbeat(2, seq=1), BOAT_B)
        r.from_consumer("qgc", gcs_heartbeat())                      # broadcast: link's last address
        self.assertEqual(r.wifi.sent[-1][1], BOAT_B)
        r.heard("wifi", heartbeat(2, seq=2), BOAT_A)
        r.from_consumer("qgc", gcs_heartbeat(seq=1))
        self.assertEqual(r.wifi.sent[-1][1], BOAT_A)

    def test_a_duplicate_does_not_move_the_reply_address(self):
        r = self.r
        raw = heartbeat(2, seq=9)
        r.heard("wifi", raw, BOAT_A)
        r.heard("wifi", raw, BOAT_B)                                 # dropped as a dup
        r.from_consumer("qgc", gcs_heartbeat())
        self.assertEqual(r.wifi.sent[-1][1], BOAT_A)

    def test_targeted_wifi_reply_goes_where_that_sysid_was_last_heard(self):
        r = self.r
        r.heard("wifi", heartbeat(42, seq=4), BOAT_COMPANION)        # a different socket, later
        r.from_consumer("qgc", command_long(2))
        self.assertEqual(r.wifi.sent[-1][1], BOAT_A)
        r.from_consumer("qgc", command_long(42, seq=1))
        self.assertEqual(r.wifi.sent[-1][1], BOAT_COMPANION)

    def test_broadcast_skips_a_wifi_link_nobody_has_spoken_on(self):
        r = Rig()
        r.heard("rfd", heartbeat(1))
        r.from_consumer("qgc", gcs_heartbeat())
        self.assertEqual(r.wifi.sent, [])                            # nowhere to send it
        self.assertEqual(len(r.rfd.sent), 1)
        self.assertEqual(r.hub.snapshot()["outputs"]["qgc"]["no_route"], 0)

    def test_nowhere_to_send_at_all_is_no_route(self):
        wifi = FakeLink("wifi", "udp", needs_addr=True)
        cfg = OutputCfg("gui", 1, ("wifi",), None, True)
        hub = Hub(DIALECT, [wifi], [(cfg, FakeOutput("gui"))], VehicleNames({}))
        hub.output_rx("gui", gcs_heartbeat())
        self.assertEqual(hub.snapshot()["outputs"]["gui"]["no_route"], 1)

    def test_link_that_refuses_is_counted_on_the_link(self):
        r = self.r
        r.rfd.accept = False                                          # port down
        r.from_consumer("qgc", command_long(1))
        self.assertEqual(r.hub.snapshot()["links"]["rfd"]["tx_dropped"], 1)
        self.assertEqual(self.out_counts()["no_route"], 0)

    def test_disabled_link_is_skipped_without_noise(self):
        r = self.r
        r.rfd.enabled = False
        r.from_consumer("qgc", gcs_heartbeat())
        self.assertEqual(len(r.wifi.sent), 1)
        self.assertEqual(r.hub.snapshot()["links"]["rfd"]["tx_dropped"], 0)

    def test_bad_crc_from_a_consumer_is_dropped(self):
        r = self.r
        raw = bytearray(command_long(1))
        raw[15] ^= 0xFF
        r.from_consumer("qgc", bytes(raw))
        self.assertEqual((r.wifi.sent, r.rfd.sent), ([], []))
        snap = self.out_counts()
        self.assertEqual((snap["bad_crc_in"], snap["frames_in_from_consumer"]), (1, 0))

    def test_two_frames_in_one_consumer_datagram_route_independently(self):
        r = self.r
        r.from_consumer("qgc", command_long(2) + command_long(1, seq=1))
        self.assertEqual(len(r.wifi.sent), 1)
        self.assertEqual(len(r.rfd.sent), 1)

    def test_write_failure_is_counted(self):
        r = self.r
        r.hub.link_tx("rfd", 1, 30, "boom")
        self.assertEqual(r.hub.snapshot()["links"]["rfd"]["write_errors"], 1)


class LinkState(unittest.TestCase):
    def test_a_reopened_stream_forgets_a_half_received_frame_and_the_dedup_window(self):
        r = Rig()
        raw = heartbeat(1, seq=1)
        r.heard("rfd", raw[:9])
        r.hub.link_state("rfd", False, "unplugged")
        r.hub.link_state("rfd", True)
        r.heard("rfd", raw[9:])                                       # tail of the old frame: garbage
        self.assertEqual(r.out["qgc"].sent, [])
        r.heard("rfd", raw)
        r.hub.link_state("rfd", False, "unplugged")
        r.hub.link_state("rfd", True)
        r.heard("rfd", raw)                                           # not suppressed by the old window
        self.assertEqual(len(r.out["qgc"].sent), 2)

    def test_state_changes_are_logged_once(self):
        r = Rig()
        with self.assertLogs("fleet", level="INFO") as cm:
            r.hub.link_state("rfd", False, "busy")
            r.hub.link_state("rfd", False, "busy")
            r.hub.link_state("rfd", False, "busy")
            r.hub.link_state("rfd", True)
            r.hub.link_state("rfd", True)
            r.hub.link_state("rfd", False, "gone")
        self.assertEqual(len(cm.output), 3, cm.output)
        link = r.hub.snapshot()["links"]["rfd"]
        self.assertEqual((link["open"], link["error"]), (False, "gone"))


if __name__ == "__main__":
    unittest.main()
