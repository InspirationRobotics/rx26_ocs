import json
import unittest
from unittest import mock

from fleet import vehicles
from fleet.fleet_link import summary_line
from fleet.tests.helpers import Rig, encode, gps_raw, heartbeat, position
from fleet.vehicles import RateMeter, VehicleNames

BOAT = ("192.168.100.5", 51001)


def entry(rig, sysid):
    """The /fleet entry for one sysid."""
    return rig.hub.snapshot()["sysids"][str(sysid)]


class Snapshot(unittest.TestCase):
    sysid = staticmethod(entry)

    def test_top_level_schema(self):
        r = Rig()
        snap = r.hub.snapshot()
        self.assertEqual(set(snap), {"schema", "time", "uptime_s", "stale_after_s", "dialect",
                                     "decode_errors", "links", "outputs", "sysids"})
        self.assertEqual(snap["decode_errors"], 0)
        self.assertEqual(snap["schema"], 1)
        self.assertEqual(set(snap["links"]), {"wifi", "rfd"})
        self.assertEqual(set(snap["outputs"]), {"qgc", "boat_gui", "drone_gui", "drone_diag"})
        self.assertEqual(snap["sysids"], {})
        json.dumps(snap, allow_nan=False)

    def test_link_and_output_keys(self):
        snap = Rig().hub.snapshot()
        for key in ("open", "error", "bytes_in", "frames_in", "bad_crc", "unknown_id", "dups",
                    "bytes_out", "frames_out", "rate_in_Bps"):
            self.assertIn(key, snap["links"]["rfd"])
        for key in ("port", "frames_out", "frames_in_from_consumer", "dropped_listen_only",
                    "no_route"):
            self.assertIn(key, snap["outputs"]["qgc"])
        self.assertEqual(snap["outputs"]["qgc"]["port"], 14570)
        self.assertIsNone(snap["links"]["rfd"]["last_frame_age_s"])      # never heard: a blank

    def test_copter_over_the_radio(self):
        r = Rig()
        r.heard("rfd", heartbeat(1))
        r.heard("rfd", position(1))
        s = self.sysid(r, 1)
        self.assertEqual((s["vehicle"], s["name"], s["link"], s["sysid"]),
                         ("drone", "Ekko", "rfd", 1))
        self.assertEqual(s["heartbeat"], {"age_s": 0.0, "type": "QUADROTOR",
                                          "autopilot": "ARDUPILOTMEGA", "mode": "GUIDED",
                                          "armed": True})
        self.assertEqual(s["position"], {"age_s": 0.0, "source": "GLOBAL_POSITION_INT",
                                         "lat": -35.3632621, "lon": 149.1652374,
                                         "alt_m": 584.09, "rel_alt_m": 10.5, "heading_deg": 90.0,
                                         "groundspeed_mps": 5.0})

    def test_rover_on_wifi(self):
        r = Rig()
        r.heard("wifi", heartbeat(2, type=11, custom_mode=15, base_mode=1), BOAT)
        s = self.sysid(r, 2)
        self.assertEqual((s["vehicle"], s["name"], s["link"]), ("boat", "Crusader", "wifi"))
        self.assertEqual(s["heartbeat"]["type"], "SURFACE_BOAT")
        self.assertEqual(s["heartbeat"]["mode"], "GUIDED")
        self.assertIs(s["heartbeat"]["armed"], False)

    def test_values_older_than_3s_are_null_but_the_age_is_kept(self):
        r = Rig()
        r.heard("rfd", heartbeat(1))
        r.heard("rfd", position(1))
        r.clock.advance(2.9)
        s = self.sysid(r, 1)
        self.assertEqual(s["heartbeat"]["mode"], "GUIDED")
        self.assertEqual(s["position"]["lat"], -35.3632621)
        r.clock.advance(0.2)                                          # 3.1 s
        s = self.sysid(r, 1)
        self.assertEqual(s["age_s"], 3.1)
        self.assertEqual(s["heartbeat"], {"age_s": 3.1, "type": None, "autopilot": None,
                                          "mode": None, "armed": None})
        self.assertEqual(s["position"], {"age_s": 3.1, "source": None, "lat": None, "lon": None,
                                         "alt_m": None, "rel_alt_m": None, "heading_deg": None,
                                         "groundspeed_mps": None})
        # a rate is a measurement too: it decays with silence, it is not frozen
        self.assertAlmostEqual(s["msgs_per_s"], 2 / 3.1, delta=0.1)
        r.clock.advance(10.0)
        self.assertEqual(self.sysid(r, 1)["msgs_per_s"], 0.0)

    def test_heard_but_no_position_yet_is_a_blank_with_no_age(self):
        r = Rig()
        r.heard("rfd", heartbeat(1))
        pos = self.sysid(r, 1)["position"]
        self.assertTrue(all(v is None for v in pos.values()), pos)

    def test_no_fix_is_a_blank_not_null_island(self):
        r = Rig()
        r.heard("rfd", position(1, lat=0, lon=0, alt=12345, rel=-250))
        pos = self.sysid(r, 1)["position"]
        self.assertIsNone(pos["lat"])
        self.assertIsNone(pos["lon"])
        self.assertIsNone(pos["alt_m"])
        self.assertEqual(pos["rel_alt_m"], -0.25)                     # a real baro measurement
        self.assertIsNone(pos["groundspeed_mps"])                     # an EKF with no GPS has no speed

    def test_unknown_heading_is_blank(self):
        r = Rig()
        r.heard("rfd", position(1, hdg=65535))
        self.assertIsNone(self.sysid(r, 1)["position"]["heading_deg"])

    def test_companion_heartbeat_has_no_mode_or_armed(self):
        r = Rig()
        r.heard("rfd", heartbeat(200, type=18, autopilot=8, base_mode=0, custom_mode=0))
        hb = self.sysid(r, 200)["heartbeat"]
        self.assertEqual((hb["type"], hb["autopilot"]), ("ONBOARD_CONTROLLER", "INVALID"))
        self.assertIsNone(hb["mode"])
        self.assertIsNone(hb["armed"])
        self.assertEqual(self.sysid(r, 200)["vehicle"], "drone")

    def test_a_companion_sharing_the_autopilots_sysid_does_not_displace_its_heartbeat(self):
        r = Rig()
        r.heard("rfd", heartbeat(1, compid=1))
        r.clock.advance(0.5)
        r.heard("rfd", heartbeat(1, compid=191, type=18, autopilot=8, base_mode=0, seq=1))
        hb = self.sysid(r, 1)["heartbeat"]
        self.assertEqual((hb["mode"], hb["armed"], hb["age_s"]), ("GUIDED", True, 0.5))

    def test_unknown_sysid_is_reported_as_unknown(self):
        r = Rig()
        r.heard("wifi", heartbeat(77, seq=1), BOAT)
        r.heard("rfd", heartbeat(51, type=0, autopilot=0, base_mode=0, custom_mode=0))
        self.assertEqual((self.sysid(r, 77)["vehicle"], self.sysid(r, 77)["name"]),
                         ("unknown", "sysid 77"))
        self.assertEqual(self.sysid(r, 51)["vehicle"], "radio")
        self.assertEqual(self.sysid(r, 51)["name"], "RFD900ux")

    def test_link_is_where_the_sysid_was_last_heard(self):
        r = Rig()
        r.heard("rfd", heartbeat(42, seq=1))
        r.clock.advance(1)
        r.heard("wifi", heartbeat(42, seq=2), BOAT)
        self.assertEqual(self.sysid(r, 42)["link"], "wifi")

    def test_sysid_rate(self):
        r = Rig()
        for i in range(30):
            r.heard("rfd", encode("attitude", 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, sysid=1, seq=i))
            r.clock.advance(0.1)
        r.clock.advance(-0.1)
        self.assertAlmostEqual(self.sysid(r, 1)["msgs_per_s"], 10.0, delta=1.0)

    def test_link_rate_and_counters(self):
        r = Rig()
        raw = heartbeat(1)
        for i in range(10):
            r.heard("rfd", heartbeat(1, seq=i))
            r.clock.advance(0.5)
        link = r.hub.snapshot()["links"]["rfd"]
        self.assertEqual((link["frames_in"], link["bytes_in"]), (10, 10 * len(raw)))
        self.assertAlmostEqual(link["rate_in_Bps"], 10 * len(raw) / 5.0, delta=10)
        self.assertEqual(link["last_frame_age_s"], 0.5)

    def test_bad_crc_and_garbage_are_counted_on_the_link(self):
        r = Rig()
        bad = bytearray(heartbeat(1))
        bad[12] ^= 0xFF
        r.heard("rfd", b"\x01\x02" + bytes(bad) + heartbeat(1, seq=1))
        link = r.hub.snapshot()["links"]["rfd"]
        self.assertEqual((link["frames_in"], link["bad_crc"]), (1, 1))
        self.assertGreater(link["garbage_bytes"], 2)

    def test_a_frame_pymavlink_cannot_decode_is_still_forwarded(self):
        r = Rig()
        raw = heartbeat(1)
        with mock.patch.object(vehicles._decoder, "decode", side_effect=ValueError("boom")):
            r.heard("rfd", raw)
        self.assertEqual(r.out["qgc"].sent, [raw])
        self.assertEqual(r.hub.vehicles.decode_errors, 1)
        s = self.sysid(r, 1)
        self.assertIsNone(s["heartbeat"]["age_s"])                       # no heartbeat was learned...
        self.assertEqual(s["age_s"], 0.0)                                # ...but the sysid was heard
        self.assertEqual(r.hub.snapshot()["decode_errors"], 1)           # ...and the failure is visible

    def test_snapshot_is_always_valid_json(self):
        r = Rig()
        r.heard("rfd", position(1, hdg=65535, lat=0, lon=0))
        r.heard("wifi", heartbeat(2, type=11), BOAT)
        text = json.dumps(r.hub.snapshot(), allow_nan=False)
        self.assertEqual(sorted(json.loads(text)["sysids"]), ["1", "2"])


class MapContract(unittest.TestCase):
    """What an OCS map page polling /fleet once a second needs from each entry."""

    ENTRY = {"sysid", "vehicle", "name", "link", "age_s", "msgs_per_s", "heartbeat", "position"}
    POSITION = {"age_s", "source", "lat", "lon", "alt_m", "rel_alt_m", "heading_deg",
                "groundspeed_mps"}

    def test_a_live_vehicle_has_everything_a_marker_needs(self):
        r = Rig()
        r.heard("wifi", heartbeat(2, type=11, custom_mode=15, base_mode=129), BOAT)
        r.heard("wifi", position(2, lat=13_500_000, lon=1_030_000_000, hdg=4500, vx=100, vy=0,
                                 seq=1), BOAT)
        e = entry(r, 2)
        self.assertEqual(set(e), self.ENTRY)
        self.assertEqual(set(e["position"]), self.POSITION)
        self.assertEqual((e["sysid"], e["name"], e["link"], e["age_s"]), (2, "Crusader", "wifi", 0.0))
        p = e["position"]
        self.assertEqual((p["lat"], p["lon"], p["heading_deg"], p["groundspeed_mps"]),
                         (1.35, 103.0, 45.0, 1.0))
        self.assertEqual((e["heartbeat"]["mode"], e["heartbeat"]["armed"]), ("GUIDED", True))

    def test_heard_but_never_positioned_has_null_lat_lon_and_no_age(self):
        r = Rig()
        r.heard("rfd", heartbeat(1))
        p = entry(r, 1)["position"]
        self.assertEqual(set(p), self.POSITION)
        self.assertEqual((p["lat"], p["lon"], p["age_s"], p["source"]), (None, None, None, None))

    def test_position_gone_stale_is_null_with_its_age_while_the_vehicle_is_still_heard(self):
        r = Rig()
        r.heard("rfd", position(1))
        r.clock.advance(2.0)
        r.heard("rfd", heartbeat(1, seq=1))                  # alive, but no new position
        r.clock.advance(1.5)
        e = entry(r, 1)
        self.assertEqual(e["age_s"], 1.5)
        self.assertEqual((e["position"]["lat"], e["position"]["lon"]), (None, None))
        self.assertEqual(e["position"]["age_s"], 3.5)
        self.assertEqual(e["heartbeat"]["mode"], "GUIDED")


class GpsFallback(unittest.TestCase):
    """GLOBAL_POSITION_INT first; GPS_RAW_INT only when that has no usable position and
    the GPS reports at least a 2D fix."""

    @staticmethod
    def pos(r, n=1):
        return entry(r, n)["position"]

    def test_gps_raw_stands_in_when_there_is_no_global_position(self):
        r = Rig()
        r.heard("rfd", gps_raw(1, fix=3))
        p = self.pos(r)
        self.assertEqual((p["source"], p["lat"], p["lon"], p["alt_m"]),
                         ("GPS_RAW_INT", -35.3632621, 149.1652374, 584.09))
        self.assertIsNone(p["rel_alt_m"])                    # GPS_RAW_INT has no such field
        self.assertEqual(p["groundspeed_mps"], 2.5)
        self.assertIsNone(p["heading_deg"])                  # COG is a course, not a heading

    def test_a_2d_fix_is_enough_and_a_1_fix_is_not(self):
        r = Rig()
        r.heard("rfd", gps_raw(1, fix=2))
        self.assertEqual(self.pos(r)["lat"], -35.3632621)
        r.heard("rfd", gps_raw(2, fix=1))
        p = self.pos(r, 2)
        self.assertEqual((p["lat"], p["lon"], p["alt_m"], p["groundspeed_mps"]),
                         (None, None, None, None))
        self.assertEqual(p["age_s"], 0.0)                    # heard, and says it has no fix

    def test_global_position_wins_when_both_are_good(self):
        r = Rig()
        r.heard("rfd", gps_raw(1, lat=100_000_000, lon=100_000_000))
        r.heard("rfd", position(1, seq=1))
        p = self.pos(r)
        self.assertEqual((p["source"], p["lat"]), ("GLOBAL_POSITION_INT", -35.3632621))

    def test_global_position_with_no_fix_falls_back_to_the_gps(self):
        r = Rig()
        r.heard("rfd", position(1, lat=0, lon=0))
        r.heard("rfd", gps_raw(1, seq=1))
        self.assertEqual(self.pos(r)["source"], "GPS_RAW_INT")

    def test_a_stale_global_position_does_not_beat_a_fresh_gps(self):
        r = Rig()
        r.heard("rfd", position(1, lat=100_000_000, lon=100_000_000))
        r.clock.advance(2.0)
        r.heard("rfd", gps_raw(1, seq=1))
        r.clock.advance(1.5)                                 # GLOBAL 3.5 s old, GPS 1.5 s
        p = self.pos(r)
        self.assertEqual((p["source"], p["lat"], p["age_s"]), ("GPS_RAW_INT", -35.3632621, 1.5))

    def test_both_stale_is_a_blank_aged_by_the_newest(self):
        r = Rig()
        r.heard("rfd", position(1))
        r.clock.advance(1.0)
        r.heard("rfd", gps_raw(1, seq=1))
        r.clock.advance(5.0)
        p = self.pos(r)
        self.assertEqual((p["lat"], p["lon"], p["source"], p["age_s"]), (None, None, None, 5.0))

    def test_gps_yaw_is_a_heading_only_when_the_gps_gives_one(self):
        r = Rig()
        for seq, (yaw, want) in enumerate([(0, None), (65535, None), (9000, 90.0), (36000, 0.0)]):
            r.heard("rfd", gps_raw(1, yaw=yaw, seq=seq))
            self.assertEqual(self.pos(r)["heading_deg"], want, yaw)

    def test_unknown_gps_speed_is_blank(self):
        r = Rig()
        r.heard("rfd", gps_raw(1, vel=65535))
        self.assertIsNone(self.pos(r)["groundspeed_mps"])


class RateMeterTests(unittest.TestCase):
    def test_idle_meter_reads_zero_and_forgets(self):
        m = RateMeter(5.0)
        self.assertEqual(m.rate(0), 0.0)
        for i in range(20):
            m.add(100, i * 0.25)
        self.assertGreater(m.rate(5.0), 100)
        self.assertEqual(m.rate(60.0), 0.0)

    def test_memory_is_constant_under_load(self):
        m = RateMeter(5.0)
        for i in range(100000):
            m.add(1, i * 0.0001)
        self.assertLessEqual(len(m._buckets), 25)


class VehicleNamesTests(unittest.TestCase):
    def test_duplicate_sysid_is_an_error(self):
        with self.assertRaises(ValueError):
            VehicleNames({"a": [1], "b": [1]})

    def test_display_names(self):
        n = VehicleNames({"drone": [1, 200], "boat": [2]}, {"drone": "Ekko"})
        self.assertEqual([n.display_of(i) for i in (1, 200, 2, 9)],
                         ["Ekko", "Ekko", "boat", "sysid 9"])
        self.assertEqual([n.name_of(i) for i in (1, 2, 9)], ["drone", "boat", "unknown"])


class Summary(unittest.TestCase):
    def test_one_line_with_ages_and_trouble(self):
        r = Rig()
        r.heard("rfd", heartbeat(1))
        r.heard("wifi", heartbeat(2, type=11), BOAT)
        r.clock.advance(12.0)
        r.heard("rfd", heartbeat(1, seq=1))
        r.from_consumer("drone_diag", heartbeat(255))
        line = summary_line(r.hub.snapshot())
        self.assertNotIn("\n", line)
        self.assertIn("1 drone 0.0s", line)
        self.assertIn("2 boat STALE 12s", line)
        self.assertIn("wifi", line)
        self.assertIn("drone_diag dropped_listen_only 1", line)

    def test_down_and_disabled_links(self):
        r = Rig()
        snap = r.hub.snapshot()
        self.assertIn("rfd DOWN", summary_line(snap))
        snap["links"]["wifi"]["enabled"] = False
        self.assertIn("wifi off", summary_line(snap))
        self.assertIn("no vehicles heard", summary_line(snap))


if __name__ == "__main__":
    unittest.main()
