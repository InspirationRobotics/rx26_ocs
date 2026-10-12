"""The tunnel log keeps TUNNEL and robotx-dialect frames and NOTHING else, in both
directions, one flushed line each -- and can never take routing down, or wait on a disk."""
from __future__ import annotations

import base64
import json
import tempfile
import threading
import time
import unittest
from pathlib import Path

from fleet import mavframe, rxl_codec as rx
from fleet.hub import Hub
from fleet.tunnel_log import TunnelLog, logged_ids

from .helpers import (DIALECT, HAVE_ROBOTX, Rig, build_frame, command_long, encode, gcs_heartbeat, heartbeat,
                      load_robotx_module, position, tunnel_frame)


def read_lines(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines()]


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.rig = Rig()
        self.log = TunnelLog(self.tmp.name, DIALECT)
        self.path = self.log.open()
        self.addCleanup(self.log.close)
        self.rig.hub.observe(self.log.on_event, self.log.msgids)

    def lines(self):
        self.assertTrue(self.log.drain(), "the writer thread did not catch up")
        return read_lines(self.path)


class Filter(Base):
    def test_only_tunnel_and_robotx_ids_are_kept(self):
        r = self.rig
        r.heard("rfd", heartbeat(1))
        r.heard("rfd", position(1))
        r.heard("rfd", gcs_heartbeat())
        self.assertEqual(self.lines(), [])                      # stock MAVLink never reaches the file
        r.heard("rfd", tunnel_frame(0x8038, rx.pack_ack(msg_id=0x20, seq=3, src=1)))
        self.assertEqual(len(self.lines()), 1)

    @unittest.skipUnless(HAVE_ROBOTX, "robotx dialect not found")
    def test_native_robotx_messages_are_kept(self):
        ids = sorted(DIALECT.custom_ids())
        self.assertIn(42011, ids)
        self.assertIn(42018, ids)
        self.assertIn(42019, ids)
        self.assertTrue(set(ids) <= self.log.msgids)
        mod = load_robotx_module()
        mav = mod.MAVLink(None, srcSystem=42, srcComponent=191)
        raw = bytes(mav.rxl_usv_reached_gate_encode(123456, 5).pack(mav))
        self.rig.heard("rfd", raw)
        (e,) = self.lines()
        self.assertEqual((e["msgid"], e["msg"], e["dir"], e["link"]), (42018, "RXL_USV_REACHED_GATE", "rx", "rfd"))
        self.assertEqual((e["src_sysid"], e["src_compid"]), (42, 191))
        self.assertEqual(e["decoded"]["gate_seq"], 5)
        self.assertEqual(base64.b64decode(e["b64"]), mavframe.split_datagram(DIALECT, raw)[0].payload)

    def test_logged_ids_are_read_from_the_dialect(self):
        self.assertIn(385, logged_ids(DIALECT))
        self.assertNotIn(33, logged_ids(DIALECT))               # GLOBAL_POSITION_INT
        self.assertNotIn(0, logged_ids(DIALECT))                # HEARTBEAT

    def test_without_the_robotx_dialect_the_42xxx_block_is_logged_instead(self):
        bare = mavframe.load_dialect(None)
        self.assertEqual(bare.custom_ids(), frozenset())
        ids = logged_ids(bare)
        self.assertIn(42011, ids)
        self.assertIn(42999, ids)
        self.assertNotIn(41999, ids)


class Lines(Base):
    def test_a_known_rxl_tunnel_is_decoded(self):
        data = rx.pack_heartbeat(vehicle=3, robot_state=3, flight_phase=2, pos_valid=True, att_valid=True,
                                 alt_valid=True, lat=1.28, lon=103.85, hdg_deg=90.0)
        self.rig.heard("rfd", tunnel_frame(0x8010, data, sysid=200, compid=191, target=250, seq=17))
        (e,) = self.lines()
        for k, v in dict(dir="rx", link="rfd", origin="link", seq=17, msgid=385, msg="TUNNEL", src_sysid=200,
                         src_compid=191, target_sysid=250, target_compid=0, payload_type=0x8010,
                         payload_type_hex="0x8010", payload_type_name="HEARTBEAT", len=24,
                         validated=True).items():
            self.assertEqual(e[k], v, k)
        self.assertEqual(base64.b64decode(e["b64"]), data)
        self.assertEqual(e["decoded"]["vehicle"], 3)
        self.assertAlmostEqual(e["decoded"]["lat"], 1.28, 6)
        for k in ("t", "mono"):
            self.assertIn(k, e)

    def test_a_corrupt_known_payload_is_kept_with_the_error(self):
        self.rig.heard("rfd", tunnel_frame(0x8010, bytes(10)))
        (e,) = self.lines()
        self.assertIn("HEARTBEAT", e["decode_error"])
        self.assertNotIn("decoded", e)
        self.assertEqual(e["len"], 10)

    def test_foreign_payload_types_are_named_and_kept_raw(self):
        self.rig.heard("rfd", tunnel_frame(32772, b"\x01\x02\x03", sysid=200))
        self.rig.heard("rfd", tunnel_frame(33022, b"hello", sysid=42))
        a, b = self.lines()
        self.assertEqual((a["payload_type_name"], b["payload_type_name"]), ("POSITIONS", "TEST"))
        self.assertNotIn("decoded", a)
        self.assertEqual(base64.b64decode(b["b64"]), b"hello")

    def test_trailing_zero_bytes_survive_the_v2_trimming(self):
        """MAVLink 2 trims trailing zeros off the air; payload_length restores the real length."""
        self.rig.heard("rfd", tunnel_frame(33022, b"ab\x00\x00\x00"))
        (e,) = self.lines()
        self.assertEqual(e["len"], 5)
        self.assertEqual(base64.b64decode(e["b64"]), b"ab\x00\x00\x00")

    def test_the_clashing_id_carries_its_note(self):
        self.rig.heard("rfd", tunnel_frame(0x8013, bytes(3)))
        (e,) = self.lines()
        self.assertIn("Mesh", e["note"])

    def test_a_garbage_declared_length_is_flagged_not_trusted(self):
        self.rig.heard("rfd", tunnel_frame(0x8010, b"abc", declared_len=200))
        (e,) = self.lines()
        self.assertIn("exceeds", e["length_error"])
        self.assertNotIn("decoded", e)

    def test_lines_are_flushed_while_the_file_is_still_open(self):
        for i in range(5):
            self.rig.heard("rfd", tunnel_frame(33022, b"x%d" % i, seq=i))
            self.assertEqual(len(self.lines()), i + 1)           # readable once the writer has caught up, not at close()

    def test_one_file_per_start_named_by_utc(self):
        self.assertRegex(self.path.name, r"^\d{8}T\d{6}Z\.jsonl$")
        second = TunnelLog(self.tmp.name, DIALECT, wall=lambda: 1.7e9)
        third = TunnelLog(self.tmp.name, DIALECT, wall=lambda: 1.7e9)
        a, b = second.open(), third.open()
        self.addCleanup(second.close)
        self.addCleanup(third.close)
        self.assertNotEqual(a, b)                                # same second: -2 suffix, never overwritten
        self.assertTrue(b.name.endswith("-2.jsonl"))


class Directions(Base):
    def test_a_consumers_tunnel_to_a_vehicle_is_logged_as_tx_with_its_origin(self):
        r = self.rig
        r.heard("rfd", heartbeat(200, type=18, autopilot=8, base_mode=0))      # route to sysid 200
        r.from_consumer("drone_gui", tunnel_frame(33022, b"test", sysid=254, target=200))
        (e,) = self.lines()
        self.assertEqual((e["dir"], e["link"], e["origin"], e["sent"]), ("tx", "rfd", "output:drone_gui", True))
        self.assertEqual(e["src_sysid"], 254)
        self.assertEqual(e["target_sysid"], 200)

    def test_a_broadcast_tunnel_from_qgc_is_one_line_per_link_it_went_to(self):
        r = self.rig
        r.heard("wifi", heartbeat(2), addr=("10.0.0.2", 5000))
        r.from_consumer("qgc", tunnel_frame(33022, b"all", sysid=255, target=0))
        got = sorted((e["dir"], e["link"]) for e in self.lines())
        self.assertEqual(got, [("tx", "rfd"), ("tx", "wifi")])

    def test_a_refused_listen_only_tunnel_is_logged(self):
        self.rig.from_consumer("drone_diag", tunnel_frame(33022, b"nope", sysid=254, target=200))
        (e,) = self.lines()
        self.assertEqual((e["dir"], e["output"], e["reason"]), ("refused", "drone_diag", "listen_only"))
        self.assertNotIn("link", e)

    def test_a_tunnel_with_no_route_is_logged_as_refused(self):
        self.rig.from_consumer("drone_gui", tunnel_frame(33022, b"lost", sysid=254, target=77))
        (e,) = self.lines()
        self.assertEqual((e["dir"], e["reason"]), ("refused", "no_route"))

    def test_other_traffic_from_consumers_is_not_logged_in_any_direction(self):
        r = self.rig
        r.heard("rfd", heartbeat(1))
        r.from_consumer("drone_gui", command_long(1))
        r.from_consumer("drone_diag", command_long(1))
        r.from_consumer("drone_gui", gcs_heartbeat(source=254))
        self.assertEqual(self.lines(), [])

    def test_adapter_style_transmit_is_logged_with_its_origin(self):
        raw = mavframe.build_tunnel_frame(DIALECT, rx.pack_ack(msg_id=0x16, seq=4, src=3), payload_type=0x8038,
                                          sysid=250, compid=190, seq=9, target_system=200)
        self.assertTrue(self.rig.hub.transmit("rfd", raw, origin="adapter:UAV1"))
        (e,) = self.lines()
        self.assertEqual((e["dir"], e["link"], e["origin"], e["sent"]), ("tx", "rfd", "adapter:UAV1", True))
        self.assertEqual((e["src_sysid"], e["src_compid"], e["seq"]), (250, 190, 9))
        self.assertEqual(e["decoded"], {"msg_id": 0x16, "seq": 4, "src": 3})
        self.assertEqual(self.rig.rfd.sent[-1][0], raw)

    def test_a_refused_transmit_is_logged_as_not_sent(self):
        self.rig.rfd.accept = False
        raw = mavframe.build_tunnel_frame(DIALECT, b"\x01", payload_type=33022, sysid=250, compid=190, seq=0)
        self.assertFalse(self.rig.hub.transmit("rfd", raw, origin="adapter:UAV1"))
        (e,) = self.lines()
        self.assertFalse(e["sent"])
        self.assertEqual(self.rig.hub.snapshot()["links"]["rfd"]["tx_dropped"], 1)


class NeverBreaksRouting(Base):
    def test_a_dead_log_file_does_not_stop_frames_reaching_the_outputs(self):
        class Dead:
            def write(self, _):
                raise OSError("disk full")

            def flush(self):
                pass

            def close(self):
                pass
        self.log._fh.close()
        self.log._fh = Dead()
        frame = tunnel_frame(33022, b"still routed", sysid=200)
        self.rig.heard("rfd", frame)
        self.assertIn(frame, [f.raw for f in self.rig.got("qgc")])
        self.log.drain()
        self.assertEqual(self.log.errors, 1)
        self.assertIn("disk full", self.log.last_error)
        self.rig.heard("rfd", tunnel_frame(33022, b"again", sysid=200, seq=1))      # down: counted, not raised
        self.log.drain()
        self.assertEqual(self.log.dropped, 1)
        st = self.rig.hub.snapshot()                                  # status still serialises
        json.dumps(st, allow_nan=False)

    def test_the_log_reopens_after_a_pause_and_keeps_appending(self):
        self.log._fh.close()
        self.log._fh = None
        self.log._failed_at = self.log.mono() - 60                    # failed a minute ago
        self.rig.heard("rfd", tunnel_frame(33022, b"back"))
        self.assertEqual(len(self.lines()), 1)
        self.assertTrue(self.log.status()["open"])

    def test_an_observer_that_raises_is_counted_and_swallowed(self):
        def boom(*_):
            raise RuntimeError("observer bug")
        self.rig.hub.observe(boom)
        frame = heartbeat(1)
        self.rig.heard("rfd", frame)
        self.assertEqual(self.rig.got_sysids("qgc"), [1])
        self.assertEqual(self.rig.hub.observer_errors, 1)

    def test_an_unopenable_folder_is_an_oserror_for_the_caller_to_report(self):
        blocker = Path(self.tmp.name) / "file"
        blocker.write_text("x")
        log = TunnelLog(blocker / "sub", DIALECT)
        with self.assertRaises(OSError):
            log.open()
        log.on_event("rx", "rfd", mavframe.split_datagram(DIALECT, tunnel_frame(33022, b"x"))[0], 0.0, {})
        self.assertEqual(log.lines, 0)                                # never opened: silently nothing


class Gate:
    """A file handle whose write() waits until the test lets it go: a disk that has stalled."""

    def __init__(self):
        self.release = threading.Event()
        self.entered = threading.Event()
        self.written = []

    def write(self, text):
        self.entered.set()
        self.release.wait(10.0)
        self.written.append(text)

    def flush(self):
        pass

    def close(self):
        pass


class DiskStallNeverBlocksRouting(Base):
    """File I/O lives on the writer thread: a stalled disk (a OneDrive sync, a dying USB stick)
    must cost lines, never the hub's lock."""

    def stall(self, queue_max):
        self.log.close()                                    # replace Base's log with one on a stalled disk
        self.log = TunnelLog(self.tmp.name, DIALECT, queue_max=queue_max)
        self.path = self.log.open()
        self.addCleanup(self.log.close)
        self.rig.hub.observe(self.log.on_event, self.log.msgids)
        gate = Gate()
        self.log._fh.close()
        self.log._fh = gate
        self.addCleanup(gate.release.set)
        return gate

    def test_frames_keep_flowing_while_the_write_is_stuck(self):
        gate = self.stall(queue_max=3)
        sent = []
        t0 = time.monotonic()
        for i in range(40):
            frame = tunnel_frame(33022, b"s%d" % i, sysid=200, seq=i)
            sent.append(frame)
            self.rig.heard("rfd", frame)
        elapsed = time.monotonic() - t0
        self.assertTrue(gate.entered.wait(2.0), "the writer never reached the file")
        self.assertLess(elapsed, 2.0, "routing waited on the disk")
        self.assertEqual([f.raw for f in self.rig.got("qgc")], sent)       # every frame still routed
        st = self.log.status()
        self.assertGreaterEqual(st["queue_dropped"], 36)                    # 40 sent; at most 4 fit (3 queued + 1 in flight)
        self.assertLessEqual(st["queued"], 3)

    def test_what_was_accepted_is_written_when_the_disk_comes_back_and_nothing_is_unaccounted_for(self):
        gate = self.stall(queue_max=3)
        for i in range(10):
            self.rig.heard("rfd", tunnel_frame(33022, b"a%d" % i, sysid=200, seq=i))
        self.assertTrue(gate.entered.wait(2.0))
        gate.release.set()
        self.assertTrue(self.log.drain())
        st = self.log.status()
        self.assertEqual(st["lines"] + st["queue_dropped"], 10)             # each line was written or counted as lost
        self.assertEqual(len(gate.written), st["lines"])
        self.assertGreater(st["queue_dropped"], 0)

    def test_the_drop_is_reported_by_the_writer_not_the_routing_thread(self):
        gate = self.stall(queue_max=1)
        with self.assertLogs("fleet", level="WARNING") as captured:
            for i in range(5):
                self.rig.heard("rfd", tunnel_frame(33022, b"w%d" % i, sysid=200, seq=i))
            gate.release.set()
            self.log.drain()
        self.assertTrue(any("queue" in m and "dropped" in m for m in captured.output), captured.output)


class CleanShutdown(Base):
    def test_close_writes_out_everything_queued(self):
        for i in range(50):
            self.rig.heard("rfd", tunnel_frame(33022, b"c%d" % i, sysid=200, seq=i))
        self.log.close()                                    # no drain(): close itself must catch up
        self.assertEqual(len(read_lines(self.path)), 50)
        self.assertFalse(self.log._thread.is_alive())
        self.log.close()                                    # idempotent
        self.rig.heard("rfd", tunnel_frame(33022, b"late", sysid=200, seq=99))   # after close: ignored, not queued
        self.assertEqual(self.log.status()["queued"], 0)


class Status(Base):
    def test_status_reports_path_lines_and_errors(self):
        self.rig.heard("rfd", tunnel_frame(33022, b"x"))
        self.log.drain()
        st = self.log.status()
        self.assertEqual(st["path"], str(self.path))
        self.assertEqual((st["lines"], st["errors"], st["open"]), (1, 0, True))
        self.assertEqual((st["queued"], st["queue_dropped"]), (0, 0))


if __name__ == "__main__":
    unittest.main()
