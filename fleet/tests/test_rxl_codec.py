"""rxl_codec: every message round-trips, every decoder refuses short and garbage input
with RxlError (and nothing else), the enums are RoboCommand's, and the ids do not clash."""
from __future__ import annotations

import random
import subprocess
import unittest
from pathlib import Path

from fleet import rxl_codec as rx

OCS = Path(__file__).resolve().parents[2]
VENV_PYTHON = OCS / ".venv" / "Scripts" / "python.exe"

FENCE5 = [(1.2801, 103.8552), (1.2801, 103.85625), (1.2811, 103.85625), (1.2811, 103.8552), (1.2801, 103.8552)]


def hb(**kw):
    args = dict(vehicle=rx.NODE_UAV, robot_state=rx.STATE_AUTO, flight_phase=rx.PHASE_AIRBORNE,
                pos_valid=True, att_valid=True, alt_valid=True, lat=1.2806, lon=103.8557, spd_mps=6.5,
                hdg_deg=123.45, roll_deg=-3.25, pitch_deg=1.5, alt_m=17.4, task=rx.TASK_NONE, phase=2)
    args.update(kw)
    return rx.pack_heartbeat(**args)


# One valid payload per decodable id, for the table-driven tests below.
SAMPLES = {
    rx.HEARTBEAT: hb(),
    rx.SAFE_PASSAGE: rx.pack_safe_passage(time_boot_ms=123456, entry=(1.28, 103.85), exit_=(1.281, 103.851),
                                          buoys=[(1.2805, 103.8505, 2), (1.2806, 103.8506, 3)]),
    rx.PIPELINE_SURVEY: rx.pack_pipeline_survey(lat=1.28, lon=103.85, segments=[1, 2, 1, 1, 2, 1, 1, 2]),
    rx.RESOURCE_DELIVERY: rx.pack_resource_delivery(task=4, resource_color=1, delivery_circle_color=3),
    rx.DOCKING: rx.pack_docking(bay_id=2),
    rx.FIREFIGHTING: rx.pack_firefighting(window_id=3),
    rx.INCIDENT_ACK: rx.pack_incident_ack(cmd_seq=4242),
    rx.READINESS: rx.pack_readiness(cmd_seq=4242),
    rx.RUN_START: rx.pack_run_start(run_id=7),
    rx.ASSISTANCE_REQUEST: rx.pack_assistance_request(cmd_seq=9, lat=1.28, lon=103.85, vehicle_type=3),
    rx.KEEP_OUT_ZONE: rx.pack_keep_out_zone(cmd_seq=10, lat=1.28, lon=103.85, radius_m=25.5, affected=[3]),
    rx.ALL_CLEAR: rx.pack_all_clear(cmd_seq=11, vehicle_type=3),
    rx.MOVING_OBJECT_ALERT: rx.pack_moving_object_alert(lat=1.28, lon=103.85, heading_deg=270.0, speed_mps=2.5,
                                                        affected=[1, 3]),
    rx.READINESS_CONFIRM: rx.pack_readiness_confirm(report_seq=12, vehicle=rx.NODE_UAV),
    rx.COURSE: rx.pack_course(course_id="ALPHA", pinger_freq_hz=27000, corners=FENCE5),
    rx.GEOFENCE_INFO: rx.pack_geofence_parts(FENCE5)[0],
    rx.ACK: rx.pack_ack(msg_id=rx.RUN_START, seq=200, src=rx.NODE_OCS),
}
#: the catalogue's payload sizes (Boat/RXL_byte_budget.py) where this codec keeps them
CATALOGUE_SIZES = {rx.HEARTBEAT: 24, rx.PIPELINE_SURVEY: 11, rx.RESOURCE_DELIVERY: 3, rx.DOCKING: 1,
                   rx.FIREFIGHTING: 1, rx.INCIDENT_ACK: 2, rx.READINESS: 2, rx.RUN_START: 3,
                   rx.ASSISTANCE_REQUEST: 11, rx.ALL_CLEAR: 3, rx.MOVING_OBJECT_ALERT: 13,
                   rx.READINESS_CONFIRM: 3, rx.COURSE: 44, rx.ACK: 3}


class RoundTrips(unittest.TestCase):
    def test_every_id_has_a_sample_and_decodes(self):
        self.assertEqual(set(SAMPLES), set(rx.NAMES))
        for cid, data in SAMPLES.items():
            name, fields = rx.decode(rx.payload_type(cid), data)
            self.assertEqual(name, rx.NAMES[cid])
            self.assertIsInstance(fields, dict)

    def test_catalogue_sizes_are_kept(self):
        for cid, size in CATALOGUE_SIZES.items():
            self.assertEqual(len(SAMPLES[cid]), size, rx.NAMES[cid])

    def test_the_two_deliberate_size_deviations(self):
        self.assertEqual(len(SAMPLES[rx.KEEP_OUT_ZONE]), 14)          # catalogue 13: zone_id u8 -> cmd_seq u16
        self.assertEqual(len(rx.pack_safe_passage(time_boot_ms=0, entry=None, exit_=None, buoys=[])), 21)
        full = rx.pack_safe_passage(time_boot_ms=0, entry=None, exit_=None,
                                    buoys=[(1.0, 103.0, 2)] * rx.MAX_BUOYS)
        self.assertEqual(len(full), 21 + 9 * 10)                      # the Mesh bench layout, Mesh/rxl.py:211

    def test_heartbeat_layout_is_the_catalogue_table_offset_for_offset(self):
        raw = hb(lat=1.0, lon=2.0, spd_mps=3.0, hdg_deg=4.0, roll_deg=-5.0, pitch_deg=6.0, alt_m=7.0,
                 task=5, phase=3)
        self.assertEqual(raw[0], rx.NODE_UAV)                                       # vehicle @0
        self.assertEqual(int.from_bytes(raw[2:6], "little", signed=True), 10_000_000)    # lat @2
        self.assertEqual(int.from_bytes(raw[6:10], "little", signed=True), 20_000_000)   # lon @6
        self.assertEqual(int.from_bytes(raw[10:12], "little"), 300)                 # spd_cms @10
        self.assertEqual(int.from_bytes(raw[12:14], "little"), 400)                 # hdg_cdeg @12
        self.assertEqual(int.from_bytes(raw[14:16], "little", signed=True), -500)   # roll_cdeg @14
        self.assertEqual(int.from_bytes(raw[16:18], "little", signed=True), 600)    # pitch_cdeg @16
        self.assertEqual(int.from_bytes(raw[18:22], "little", signed=True), 700)    # alt_cm @18
        self.assertEqual(raw[22], 5)                                                # task @22
        self.assertEqual(raw[23], 3)                                                # phase @23
        # flags @1: bits 0-1 RobotState, 2-3 FlightPhase, 4 pos, 5 att, 6 alt, 7 dead-reckoned
        self.assertEqual(raw[1], 3 | (2 << 2) | 16 | 32 | 64)

    def test_heartbeat(self):
        f = rx.unpack_heartbeat(hb())
        self.assertEqual((f["vehicle"], f["robot_state"], f["flight_phase"]), (3, 3, 2))
        self.assertTrue(f["pos_valid"] and f["att_valid"] and f["alt_valid"])
        self.assertFalse(f["dead_reckoned"])
        self.assertAlmostEqual(f["lat"], 1.2806, 7)
        self.assertAlmostEqual(f["lon"], 103.8557, 7)
        self.assertAlmostEqual(f["spd_mps"], 6.5, 2)
        self.assertAlmostEqual(f["hdg_deg"], 123.45, 2)
        self.assertAlmostEqual(f["roll_deg"], -3.25, 2)
        self.assertAlmostEqual(f["alt_m"], 17.4, 2)
        self.assertEqual((f["task"], f["phase"]), (1, 2))

    def test_heartbeat_blanks_stay_blank(self):
        """A value behind a clear valid flag is None, not a number: blanks over guesses."""
        f = rx.unpack_heartbeat(hb(pos_valid=False, att_valid=False, alt_valid=False, hdg_deg=None))
        for k in ("lat", "lon", "roll_deg", "pitch_deg", "alt_m", "hdg_deg"):
            self.assertIsNone(f[k], k)

    def test_heartbeat_heading_wraps_and_never_hits_the_unresolved_marker(self):
        self.assertEqual(rx.unpack_heartbeat(hb(hdg_deg=360.0))["hdg_deg"], 0.0)
        self.assertEqual(rx.unpack_heartbeat(hb(hdg_deg=-90.0))["hdg_deg"], 270.0)
        self.assertEqual(rx.unpack_heartbeat(hb(hdg_deg=359.999))["hdg_deg"], 0.0)
        self.assertEqual(rx.unpack_heartbeat(hb(hdg_deg=None))["hdg_deg"], None)
        with self.assertRaises(rx.RxlError):
            hb(hdg_deg=float("nan"))

    def test_safe_passage(self):
        f = rx.unpack_safe_passage(SAMPLES[rx.SAFE_PASSAGE])
        self.assertEqual(f["time_boot_ms"], 123456)
        self.assertAlmostEqual(f["entry"]["lat"], 1.28, 7)
        self.assertAlmostEqual(f["exit"]["lon"], 103.851, 7)
        self.assertEqual([b["state"] for b in f["buoys"]], [2, 3])

    def test_safe_passage_unknown_entry_exit_are_absent_not_zero(self):
        f = rx.unpack_safe_passage(rx.pack_safe_passage(time_boot_ms=1, entry=None, exit_=None, buoys=[]))
        self.assertIsNone(f["entry"])
        self.assertIsNone(f["exit"])
        self.assertEqual(f["buoys"], [])

    def test_safe_passage_limits(self):
        with self.assertRaises(rx.RxlError):
            rx.pack_safe_passage(time_boot_ms=0, entry=None, exit_=None, buoys=[(1.0, 1.0, 2)] * 11)
        # a count byte above 10 is garbage, not a loop bound
        bad = bytearray(SAMPLES[rx.SAFE_PASSAGE])
        bad[20] = 200
        with self.assertRaises(rx.RxlError):
            rx.unpack_safe_passage(bytes(bad))

    def test_pipeline_survey(self):
        f = rx.unpack_pipeline_survey(SAMPLES[rx.PIPELINE_SURVEY])
        self.assertEqual(f["segments"], [1, 2, 1, 1, 2, 1, 1, 2])
        short = rx.unpack_pipeline_survey(rx.pack_pipeline_survey(lat=1.0, lon=2.0, segments=[2, 1]))
        self.assertEqual(short["segments"], [2, 1])
        with self.assertRaises(rx.RxlError):
            rx.pack_pipeline_survey(lat=1.0, lon=2.0, segments=[1] * 9)
        with self.assertRaises(rx.RxlError):                       # a segment status of 3 is not an enum value
            rx.pack_pipeline_survey(lat=1.0, lon=2.0, segments=[3])
        with self.assertRaises(rx.RxlError):                       # bits set beyond the declared count
            rx.unpack_pipeline_survey(rx.pack_pipeline_survey(lat=1.0, lon=2.0, segments=[1])[:-2] + b"\xff\xff")

    def test_small_reports(self):
        self.assertEqual(rx.unpack_resource_delivery(SAMPLES[rx.RESOURCE_DELIVERY]),
                         {"task": 4, "resource_color": 1, "delivery_circle_color": 3})
        self.assertEqual(rx.unpack_docking(SAMPLES[rx.DOCKING]), {"bay_id": 2})
        self.assertEqual(rx.unpack_firefighting(SAMPLES[rx.FIREFIGHTING]), {"window_id": 3})
        self.assertEqual(rx.unpack_incident_ack(SAMPLES[rx.INCIDENT_ACK]), {"cmd_seq": 4242})
        self.assertEqual(rx.unpack_readiness(SAMPLES[rx.READINESS]), {"cmd_seq": 4242})

    def test_downlink_messages(self):
        self.assertEqual(rx.unpack_run_start(SAMPLES[rx.RUN_START]), {"run_id": 7, "flags": 0})
        a = rx.unpack_assistance_request(SAMPLES[rx.ASSISTANCE_REQUEST])
        self.assertEqual((a["cmd_seq"], a["vehicle_type"]), (9, 3))
        self.assertAlmostEqual(a["lat"], 1.28, 7)
        z = rx.unpack_keep_out_zone(SAMPLES[rx.KEEP_OUT_ZONE])
        self.assertEqual((z["cmd_seq"], z["radius_m"], z["affected"], z["active"]), (10, 25.5, [3], True))
        self.assertEqual(rx.unpack_all_clear(SAMPLES[rx.ALL_CLEAR]), {"cmd_seq": 11, "vehicle_type": 3})
        m = rx.unpack_moving_object_alert(SAMPLES[rx.MOVING_OBJECT_ALERT])
        self.assertEqual((m["heading_deg"], m["speed_mps"], m["affected"]), (270.0, 2.5, [1, 3]))
        self.assertEqual(rx.unpack_readiness_confirm(SAMPLES[rx.READINESS_CONFIRM]),
                         {"report_seq": 12, "vehicle": rx.NODE_UAV})
        self.assertEqual(rx.unpack_ack(SAMPLES[rx.ACK]), {"msg_id": 0x20, "seq": 200, "src": 1})

    def test_keep_out_zone_inactive_flag(self):
        z = rx.unpack_keep_out_zone(rx.pack_keep_out_zone(cmd_seq=1, lat=1.0, lon=2.0, radius_m=5, affected=[1],
                                                          active=False))
        self.assertFalse(z["active"])

    def test_affected_mask_is_one_bit_per_vehicle_type(self):
        self.assertEqual(rx.affected_mask([rx.TYPE_USV]), 0b0010)
        self.assertEqual(rx.affected_mask([rx.TYPE_UUV]), 0b0100)
        self.assertEqual(rx.affected_mask([rx.TYPE_UAV]), 0b1000)
        self.assertEqual(rx.affected_types(0b1010), [1, 3])

    def test_course(self):
        f = rx.unpack_course(SAMPLES[rx.COURSE])
        self.assertEqual(f["pinger_freq_hz"], 27000)
        self.assertEqual(f["course_epoch"], rx.course_epoch("ALPHA"))
        self.assertEqual(len(f["corners"]), 5)
        self.assertAlmostEqual(f["corners"][2]["lat"], 1.2811, 7)

    def test_course_limits(self):
        corners15 = [(1.0 + i * 1e-4, 103.0) for i in range(15)]
        self.assertEqual(len(rx.pack_course(course_id="X", pinger_freq_hz=1, corners=corners15)), 4 + 8 * 15)
        for bad in (corners15 + [(1.0, 1.0)], corners15[:2]):
            with self.assertRaises(rx.RxlError):
                rx.pack_course(course_id="X", pinger_freq_hz=1, corners=bad)
        with self.assertRaises(rx.RxlError):
            rx.pack_course(course_id="", pinger_freq_hz=1, corners=FENCE5)
        with self.assertRaises(rx.RxlError):
            rx.pack_course(course_id="X", pinger_freq_hz=70000, corners=FENCE5)     # does not fit u16

    def test_course_epoch_is_the_documented_hash(self):
        self.assertEqual(rx.crc16_ccitt_false(b"123456789"), 0x29B1)               # the CRC's published check value
        self.assertEqual(rx.course_epoch("ALPHA"), rx.crc16_ccitt_false(b"ALPHA") & 0xFF)
        self.assertNotEqual(rx.course_epoch("ALPHA"), rx.course_epoch("BRAVO"))

    def test_value_overflows_are_refused_not_wrapped(self):
        with self.assertRaises(rx.RxlError):
            rx.pack_run_start(run_id=65536)
        with self.assertRaises(rx.RxlError):
            rx.pack_assistance_request(cmd_seq=65536, lat=1.0, lon=1.0, vehicle_type=3)
        with self.assertRaises(rx.RxlError):
            rx.pack_keep_out_zone(cmd_seq=1, lat=1.0, lon=1.0, radius_m=6553.6, affected=[3])
        with self.assertRaises(rx.RxlError):
            rx.pack_keep_out_zone(cmd_seq=1, lat=1.0, lon=1.0, radius_m=float("nan"), affected=[3])
        with self.assertRaises(rx.RxlError):
            rx.pack_moving_object_alert(lat=91.0, lon=1.0, heading_deg=0, speed_mps=1, affected=[3])
        with self.assertRaises(rx.RxlError):
            rx.pack_moving_object_alert(lat=1.0, lon=1.0, heading_deg=0, speed_mps=-1, affected=[3])
        with self.assertRaises(rx.RxlError):
            rx.pack_all_clear(cmd_seq=1, vehicle_type=9)
        with self.assertRaises(rx.RxlError):
            rx.pack_ack(msg_id=256, seq=0, src=1)
        # the boundary itself is fine
        rx.pack_run_start(run_id=65535)
        rx.pack_keep_out_zone(cmd_seq=65535, lat=-90.0, lon=-180.0, radius_m=6553.5, affected=[3])

    def test_a_range_refusal_names_the_field_and_the_value_and_carries_a_value_free_key(self):
        with self.assertRaises(rx.RxlError) as ctx:
            rx.pack_run_start(run_id=70000)
        self.assertIn("RUN_START.run_id", str(ctx.exception))
        self.assertIn("70000", str(ctx.exception))
        self.assertEqual(ctx.exception.key, "RUN_START.run_id does not fit u16")
        with self.assertRaises(rx.RxlError) as ctx:
            rx.pack_run_start(run_id=80000)
        self.assertEqual(ctx.exception.key, "RUN_START.run_id does not fit u16")      # same bucket, other value
        self.assertIsNone(rx.RxlError("anything else").key)


class Geofence(unittest.TestCase):
    def poly(self, n):
        pts = [(1.0 + 0.001 * i, 103.0 + 0.0007 * i) for i in range(n - 1)]
        return pts + [pts[0]]                                       # closed

    def test_five_vertices_is_one_part_of_45_bytes(self):
        parts = rx.pack_geofence_parts(FENCE5)
        self.assertEqual([len(p) for p in parts], [5 + 8 * 5])
        f = rx.unpack_geofence_info(parts[0])
        self.assertEqual((f["part"], f["n_parts"], f["closed"]), (0, 1, True))

    def test_fifteen_vertices_fill_one_part_and_sixteen_need_two(self):
        self.assertEqual([len(p) for p in rx.pack_geofence_parts(self.poly(15))], [5 + 8 * 15])
        parts = rx.pack_geofence_parts(self.poly(16))
        self.assertEqual([len(p) for p in parts], [5 + 8 * 15, 5 + 8 * 1])
        self.assertTrue(all(len(p) <= rx.TUNNEL_MAX for p in parts))

    def test_every_part_fits_a_tunnel_up_to_the_cap(self):
        parts = rx.pack_geofence_parts(self.poly(rx.MAX_FENCE_VERTICES))
        self.assertEqual(len(parts), rx.MAX_FENCE_PARTS)
        self.assertTrue(all(len(p) <= rx.TUNNEL_MAX for p in parts))
        with self.assertRaises(rx.RxlError):
            rx.pack_geofence_parts(self.poly(rx.MAX_FENCE_VERTICES + 1))
        with self.assertRaises(rx.RxlError):
            rx.pack_geofence_parts(FENCE5[:2])

    def test_assembler_rebuilds_the_polygon_in_any_order(self):
        poly = self.poly(40)
        parts = rx.pack_geofence_parts(poly)
        self.assertEqual(len(parts), 3)
        for order in ([0, 1, 2], [2, 0, 1], [1, 2, 0]):
            asm, got = rx.GeofenceAssembler(), None
            for i in order:
                got = asm.add(parts[i])
            self.assertEqual(len(got), 40)
            for (a, b), (c, d) in zip(got, poly):
                self.assertAlmostEqual(a, c, 7)
                self.assertAlmostEqual(b, d, 7)

    def test_a_newer_fence_discards_the_parts_of_the_older_one(self):
        asm = rx.GeofenceAssembler()
        old = rx.pack_geofence_parts(self.poly(30))
        new = rx.pack_geofence_parts([(p[0] + 0.01, p[1]) for p in self.poly(30)])
        self.assertIsNone(asm.add(old[0]))
        self.assertIsNone(asm.add(new[0]))                          # different epoch: restart
        self.assertIsNotNone(asm.add(new[1]))

    def test_epoch_changes_with_any_vertex(self):
        a = rx.unpack_geofence_info(rx.pack_geofence_parts(FENCE5)[0])["fence_epoch"]
        moved = list(FENCE5)
        moved[2] = (moved[2][0] + 1e-6, moved[2][1])
        b = rx.unpack_geofence_info(rx.pack_geofence_parts(moved)[0])["fence_epoch"]
        self.assertNotEqual(a, b)

    def test_part_numbers_are_validated(self):
        bad = bytearray(rx.pack_geofence_parts(FENCE5)[0])
        bad[2], bad[3] = 3, 2                                        # part 3 of 2
        with self.assertRaises(rx.RxlError):
            rx.unpack_geofence_info(bytes(bad))


class Garbage(unittest.TestCase):
    """The drone's own RX loop once died on a 10-byte frame. These decoders answer garbage
    with RxlError and with nothing else, for every length and for random bytes."""

    def test_every_wrong_length_is_a_clear_error(self):
        for cid, good in SAMPLES.items():
            name = rx.NAMES[cid]
            for n in range(0, len(good) + 4):
                data = (good + bytes(8))[:n]
                if n == len(good):
                    continue
                with self.assertRaises(rx.RxlError, msg="%s at %d bytes" % (name, n)):
                    rx.decode(rx.payload_type(cid), data)

    def test_error_text_names_the_message_and_the_sizes(self):
        with self.assertRaises(rx.RxlError) as cm:
            rx.decode(rx.payload_type(rx.HEARTBEAT), bytes(10))
        self.assertIn("HEARTBEAT", str(cm.exception))
        self.assertIn("10", str(cm.exception))
        self.assertIn("24", str(cm.exception))

    def test_random_bytes_never_raise_anything_but_rxlerror(self):
        rng = random.Random(20261008)
        for cid in rx.NAMES:
            for _ in range(400):
                data = bytes(rng.randrange(256) for _ in range(rng.choice((0, 1, 2, 3, 5, 11, 14, 24, 37, 44, 111, 128))))
                try:
                    rx.decode(rx.payload_type(cid), data)
                except rx.RxlError:
                    pass

    def test_mutated_valid_payloads_never_raise_anything_but_rxlerror(self):
        rng = random.Random(7)
        for cid, good in SAMPLES.items():
            for _ in range(300):
                data = bytearray(good)
                for _ in range(rng.randrange(1, 4)):
                    data[rng.randrange(len(data))] = rng.randrange(256)
                try:
                    rx.decode(rx.payload_type(cid), bytes(data))
                except rx.RxlError:
                    pass

    def test_oversize_payload_is_refused(self):
        with self.assertRaises(rx.RxlError):
            rx.decode(rx.payload_type(rx.SAFE_PASSAGE), bytes(129))

    def test_enum_values_outside_the_pinned_tables_are_refused(self):
        bad = bytearray(SAMPLES[rx.ASSISTANCE_REQUEST])
        bad[10] = 4                                                   # VehicleType 4 does not exist
        with self.assertRaises(rx.RxlError):
            rx.unpack_assistance_request(bytes(bad))
        bad = bytearray(SAMPLES[rx.RESOURCE_DELIVERY])
        bad[1] = 9
        with self.assertRaises(rx.RxlError):
            rx.unpack_resource_delivery(bytes(bad))
        bad = bytearray(SAMPLES[rx.SAFE_PASSAGE])
        bad[21 + 8] = 9                                               # BeaconState 9
        with self.assertRaises(rx.RxlError):
            rx.unpack_safe_passage(bytes(bad))
        bad = bytearray(SAMPLES[rx.MOVING_OBJECT_ALERT])
        bad[8:10] = (36000).to_bytes(2, "little")                    # heading must be below 36000 cdeg
        with self.assertRaises(rx.RxlError):
            rx.unpack_moving_object_alert(bytes(bad))
        bad = bytearray(SAMPLES[rx.ASSISTANCE_REQUEST])
        bad[2:6] = (950_000_000).to_bytes(4, "little", signed=True)  # 95 degrees latitude
        with self.assertRaises(rx.RxlError):
            rx.unpack_assistance_request(bytes(bad))

    def test_payloads_that_are_not_ours_are_not_decoded(self):
        for ptype in (32768, 32772, 33022, 0x8000, 0x8018, 0x8030, 0x8099, 0x0011, 200):
            self.assertIsNone(rx.decode(ptype, b"\x00\x01"), hex(ptype))


class Identity(unittest.TestCase):
    TAKEN = {32768, 32769, 32770, 32771, 32772, 32773, 32774, 33022}
    MESH = {0x8011, 0x8013, 0x8018, 0x8019, 0x8034}

    def test_ids_are_unique_and_in_the_catalogue_range(self):
        ids = list(rx.NAMES)
        self.assertEqual(len(ids), len(set(ids)))
        for cid in ids:
            self.assertTrue(0x10 <= cid <= 0x39, hex(cid))
            self.assertEqual(rx.catalogue_id(rx.payload_type(cid)), cid)

    def test_nothing_here_reuses_a_taken_payload_type(self):
        ours = {rx.payload_type(c) for c in rx.NAMES}
        self.assertFalse(ours & self.TAKEN)
        self.assertFalse(ours & set(rx.FOREIGN_PAYLOAD_TYPES))

    def test_overlap_with_the_mesh_bench_is_the_same_catalogue_id(self):
        # 0x8011 SAFE_PASSAGE and 0x8013 RESOURCE_DELIVERY are ids this codec uses too: the
        # Mesh bench builds them as 0x8000 | catalogue id, so for 0x11 that is consistent.
        # 0x13 is the one real clash: see KNOWN_CLASHES.
        self.assertEqual(rx.payload_type_name(0x8011), "SAFE_PASSAGE")
        self.assertEqual(rx.payload_type_name(0x8013), "RESOURCE_DELIVERY")
        self.assertIn(rx.RESOURCE_DELIVERY, rx.KNOWN_CLASHES)
        self.assertEqual(rx.payload_type_name(0x8018), "USV_REACHED_GATE")
        self.assertEqual(rx.payload_type_name(0x8019), "NEXT_BUOY_SET")
        self.assertEqual(rx.payload_type_name(0x8034), "SURVEY_START")
        for ptype in self.MESH - {0x8011, 0x8013}:
            self.assertFalse(rx.is_ours(ptype))            # named, never decoded as OCS traffic

    def test_geofence_info_id_is_first_free_after_course_and_free_everywhere(self):
        self.assertEqual(rx.GEOFENCE_INFO, rx.COURSE + 1)
        self.assertNotIn(rx.GEOFENCE_INFO, rx.INTER_VEHICLE_NAMES)
        self.assertNotIn(rx.payload_type(rx.GEOFENCE_INFO), self.TAKEN | self.MESH)
        self.assertEqual(rx.payload_type_name(0x8027), "GEOFENCE_INFO")

    def test_foreign_names(self):
        self.assertEqual(rx.payload_type_name(32768), "BOAT")
        self.assertEqual(rx.payload_type_name(33022), "TEST")
        self.assertEqual(rx.payload_type_name(32770), "retired v2 positions")
        self.assertEqual(rx.payload_type_name(0x7FFF), "UNKNOWN")

    def test_receiver_ack_set_is_the_catalogue_acked_messages(self):
        self.assertEqual(rx.RECEIVER_ACKS, {rx.INCIDENT_ACK, rx.READINESS, rx.RUN_START, rx.ALL_CLEAR})


class PinnedProtobuf(unittest.TestCase):
    """The enum tables are RoboCommand's. Read from the generated code in the OCS .venv
    (system python has no protobuf), never from memory."""

    DUMP = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
from rx_bridge import proto
out = {}
for mod in (proto.common_pb2, proto.rx_common_pb2):
    for en in mod.DESCRIPTOR.enum_types_by_name.values():
        out[en.name] = {v.number: v.name for v in en.values}
print(json.dumps(out))
'''

    @unittest.skipUnless(VENV_PYTHON.is_file(), "OCS .venv not present")
    def test_enums_match_the_generated_protobuf(self):
        import json
        res = subprocess.run([str(VENV_PYTHON), "-c", self.DUMP, str(OCS)], capture_output=True, text=True,
                             timeout=60)
        self.assertEqual(res.returncode, 0, res.stderr)
        pinned = {k: {int(n): v for n, v in d.items()} for k, d in json.loads(res.stdout).items()}
        mine = {"RobotState": rx.ROBOT_STATE, "FlightPhase": rx.FLIGHT_PHASE, "RxTask": rx.RX_TASK,
                "VehicleType": rx.VEHICLE_TYPE, "BeaconState": rx.BEACON_STATE,
                "PipelineSegmentStatus": rx.PIPELINE_SEGMENT, "Color": rx.COLOR}
        for name, table in mine.items():
            self.assertEqual(table, pinned[name], name)


if __name__ == "__main__":
    unittest.main()
