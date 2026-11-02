"""ocs_translate: every RxCommand body, the course and the geofence go to the right RXL payload
and come back through the decoder as what was sent; every uplink report becomes the RxReport
JSON the bridge parses; and anything that cannot be said truthfully is a Refusal, never a guess."""
from __future__ import annotations

import unittest
from datetime import datetime, timezone

from fleet import ocs_translate as tr
from fleet import rxl_codec as rx

SINGAPORE = {"latitude": 1.2806, "longitude": 103.8557}
CORNERS = [{"latitude": 1.2801, "longitude": 103.8552}, {"latitude": 1.2801, "longitude": 103.85625},
           {"latitude": 1.2811, "longitude": 103.85625}, {"latitude": 1.2811, "longitude": 103.8552},
           {"latitude": 1.2801, "longitude": 103.8552}]


def decoded(out: tr.Outgoing):
    """What the receiving end reads out of the payload the translator produced."""
    return rx.decode(rx.payload_type(out.msg_id), out.payload)


def polygon(n: int):
    """An n-vertex closed-looking ring of LatLng JSON objects (all distinct, all in range)."""
    return [{"latitude": 1.28 + 0.0001 * i, "longitude": 103.85 + 0.0002 * i} for i in range(n)]


class RunStart(unittest.TestCase):
    def test_translates_and_decodes_back(self):
        out = tr.command_to_outgoing({"seq": 3, "run_start": {"run_id": 7}})
        self.assertEqual(decoded(out), ("RUN_START", {"run_id": 7, "flags": 0}))
        self.assertEqual((out.msg_id, out.key, out.cmd_seq, out.info), (rx.RUN_START, ("run_start",), 3, {"run_id": 7}))

    def test_camel_case_spelling_is_the_same_command(self):
        a = tr.command_to_outgoing({"seq": 3, "run_start": {"run_id": 7}})
        b = tr.command_to_outgoing({"seq": 3, "runStart": {"runId": 7}})
        self.assertEqual(a.payload, b.payload)

    def test_protobuf_omits_a_zero_run_id_and_a_zero_seq(self):
        out = tr.command_to_outgoing({"run_start": {}})
        self.assertEqual((decoded(out)[1]["run_id"], out.cmd_seq), (0, 0))


class AssistanceRequest(unittest.TestCase):
    def test_by_enum_name_and_by_number(self):
        for vt in ("TYPE_UAV", 3):
            out = tr.command_to_outgoing({"seq": 9, "assistance_request": {"position": SINGAPORE, "vehicle_type": vt}})
            name, msg = decoded(out)
            self.assertEqual((name, msg["cmd_seq"], msg["vehicle_type"]), ("ASSISTANCE_REQUEST", 9, 3))
            self.assertAlmostEqual(msg["lat"], 1.2806, 6)
            self.assertAlmostEqual(msg["lon"], 103.8557, 6)
            self.assertEqual((out.key, out.info), (("assist",), {"affected": [3]}))

    def test_an_absent_vehicle_type_is_UNKNOWN_which_means_everyone(self):
        out = tr.command_to_outgoing({"seq": 1, "assistance_request": {"position": SINGAPORE}})
        self.assertEqual(decoded(out)[1]["vehicle_type"], 0)

    def test_a_position_is_required(self):
        with self.assertRaises(tr.Refusal) as ctx:
            tr.command_to_outgoing({"seq": 1, "assistance_request": {"vehicle_type": "TYPE_UAV"}})
        self.assertIn("position", str(ctx.exception))

    def test_a_position_with_neither_coordinate_is_refused_not_read_as_0_0(self):
        with self.assertRaises(tr.Refusal):
            tr.command_to_outgoing({"seq": 1, "assistance_request": {"position": {}, "vehicle_type": "TYPE_UAV"}})

    def test_one_missing_coordinate_is_zero_because_protobuf_drops_zero(self):
        out = tr.command_to_outgoing({"seq": 1, "assistance_request": {"position": {"latitude": 1.5}}})
        self.assertEqual(decoded(out)[1]["lon"], 0.0)


class KeepOutZone(unittest.TestCase):
    def test_translates_and_decodes_back(self):
        out = tr.command_to_outgoing({"seq": 10, "keep_out_zone": {"center": SINGAPORE, "radius_m": 25.5,
                                                                      "vehicle_type": "TYPE_UAV"}})
        name, msg = decoded(out)
        self.assertEqual(name, "KEEP_OUT_ZONE")
        self.assertEqual((msg["cmd_seq"], msg["radius_m"], msg["affected"], msg["active"]), (10, 25.5, [3], True))
        self.assertEqual((out.key, out.cmd_seq, out.info), (("zone", 10), 10, {"affected": [3]}))

    def test_two_zones_have_two_keys(self):
        a = tr.command_to_outgoing({"seq": 10, "keep_out_zone": {"center": SINGAPORE, "radius_m": 5}})
        b = tr.command_to_outgoing({"seq": 11, "keep_out_zone": {"center": SINGAPORE, "radius_m": 5}})
        self.assertNotEqual(a.key, b.key)

    def test_a_radius_is_required_and_must_fit(self):
        with self.assertRaises(tr.Refusal):
            tr.command_to_outgoing({"seq": 10, "keep_out_zone": {"center": SINGAPORE}})
        with self.assertRaises(tr.Refusal) as ctx:
            tr.command_to_outgoing({"seq": 10, "keep_out_zone": {"center": SINGAPORE, "radius_m": 7000}})
        self.assertIn("radius_m", str(ctx.exception))

    def test_nan_radius_is_refused(self):
        with self.assertRaises(tr.Refusal):
            tr.command_to_outgoing({"seq": 10, "keep_out_zone": {"center": SINGAPORE, "radius_m": "NaN"}})


class AllClear(unittest.TestCase):
    def test_translates_and_decodes_back(self):
        out = tr.command_to_outgoing({"seq": 11, "all_clear": {"vehicle_type": "TYPE_UAV"}})
        self.assertEqual(decoded(out), ("ALL_CLEAR", {"cmd_seq": 11, "vehicle_type": 3}))
        self.assertEqual((out.key, out.info), (("clear", 11), {"affected": [3]}))

    def test_an_absent_vehicle_type_is_UNKNOWN(self):
        out = tr.command_to_outgoing({"seq": 11, "all_clear": {}})
        self.assertEqual(decoded(out)[1]["vehicle_type"], 0)
        self.assertEqual(out.info, {"affected": [0]})


class MovingObjectAlert(unittest.TestCase):
    def test_translates_and_decodes_back(self):
        out = tr.command_to_outgoing({"seq": 12, "moving_object_alert": {
            "position": SINGAPORE, "heading_deg": 270.0, "speed_mps": 2.5,
            "affected_vehicle_types": ["TYPE_USV", "TYPE_UAV"]}})
        name, msg = decoded(out)
        self.assertEqual((name, msg["heading_deg"], msg["speed_mps"], msg["affected"]),
                         ("MOVING_OBJECT_ALERT", 270.0, 2.5, [1, 3]))
        self.assertEqual((out.key, out.info),
                         (("moving",), {"affected": [1, 3], "keep_out_radius_m": None}))

    def test_the_keep_out_radius_is_kept_for_the_log_but_has_no_byte_on_the_radio(self):
        body = {"position": SINGAPORE, "heading_deg": 270.0, "speed_mps": 2.5}
        with_r = tr.command_to_outgoing({"seq": 12, "moving_object_alert": dict(body, keep_out_radius_m=7.0)})
        without = tr.command_to_outgoing({"seq": 12, "moving_object_alert": body})
        self.assertEqual(with_r.info["keep_out_radius_m"], 7.0)
        self.assertEqual(with_r.payload, without.payload)       # RXL 0x24 is 13 bytes either way
        with self.assertRaises(tr.Refusal):
            tr.command_to_outgoing({"seq": 12, "moving_object_alert": dict(body, keep_out_radius_m="NaN")})

    def test_protobuf_omits_zero_heading_speed_and_the_list_of_types(self):
        out = tr.command_to_outgoing({"seq": 12, "moving_object_alert": {"position": SINGAPORE}})
        name, msg = decoded(out)
        self.assertEqual((msg["heading_deg"], msg["speed_mps"], msg["affected"]), (0.0, 0.0, []))

    def test_an_unknown_type_name_is_refused(self):
        with self.assertRaises(tr.Refusal):
            tr.command_to_outgoing({"seq": 12, "moving_object_alert": {
                "position": SINGAPORE, "affected_vehicle_types": ["TYPE_BLIMP"]}})


class ReadinessConfirm(unittest.TestCase):
    def test_translates_and_decodes_back(self):
        out = tr.command_to_outgoing({"seq": 5, "readiness_confirm": {"report_seq": 12, "vehicle_id": "UAV1"}})
        self.assertEqual(decoded(out), ("READINESS_CONFIRM", {"report_seq": 12, "vehicle": rx.NODE_UAV}))
        self.assertEqual((out.key, out.cmd_seq), (("confirm", 12, rx.NODE_UAV), 5))

    def test_node_follows_the_vehicle_id_prefix(self):
        for vid, node in (("USV1", rx.NODE_USV), ("UAV2", rx.NODE_UAV), ("UUV1", rx.NODE_UUV)):
            out = tr.command_to_outgoing({"readiness_confirm": {"report_seq": 1, "vehicle_id": vid}})
            self.assertEqual(decoded(out)[1]["vehicle"], node)

    def test_without_a_vehicle_id_in_the_body_the_commands_own_is_used(self):
        out = tr.command_to_outgoing({"vehicle_id": "UAV1", "readiness_confirm": {"report_seq": 4}})
        self.assertEqual(decoded(out)[1]["vehicle"], rx.NODE_UAV)

    def test_the_body_wins_over_the_commands(self):
        out = tr.command_to_outgoing({"vehicle_id": "USV1",
                                      "readiness_confirm": {"report_seq": 4, "vehicle_id": "UAV1"}})
        self.assertEqual(decoded(out)[1]["vehicle"], rx.NODE_UAV)

    def test_without_either_the_links_own_vehicle_is_used(self):
        out = tr.command_to_outgoing({"readiness_confirm": {"report_seq": 4}}, default_vehicle_id="UAV1")
        self.assertEqual(decoded(out)[1]["vehicle"], rx.NODE_UAV)

    def test_camel_case_vehicle_id_at_the_top_level(self):
        out = tr.command_to_outgoing({"vehicleId": "UAV1", "readinessConfirm": {"reportSeq": 4}})
        self.assertEqual(decoded(out)[1], {"report_seq": 4, "vehicle": rx.NODE_UAV})

    def test_nobody_named_anywhere_is_refused(self):
        with self.assertRaises(tr.Refusal) as ctx:
            tr.command_to_outgoing({"readiness_confirm": {"report_seq": 4}})
        self.assertIn("no RXL node", str(ctx.exception))

    def test_an_id_with_no_known_prefix_is_refused(self):
        with self.assertRaises(tr.Refusal):
            tr.command_to_outgoing({"readiness_confirm": {"report_seq": 4, "vehicle_id": "BOAT7"}})

    def test_report_seq_is_mapped_to_the_cmd_seq_the_vehicle_used(self):
        out = tr.command_to_outgoing({"readiness_confirm": {"report_seq": 41, "vehicle_id": "UAV1"}},
                                     report_seq_of={41: 7}.get)
        self.assertEqual(decoded(out)[1]["report_seq"], 7)
        self.assertEqual(out.key, ("confirm", 7, rx.NODE_UAV))
        self.assertEqual(out.info, {"report_seq": 41})

    def test_a_seq_nobody_relayed_is_refused_not_passed_through(self):
        with self.assertRaises(tr.Refusal) as ctx:
            tr.command_to_outgoing({"readiness_confirm": {"report_seq": 41, "vehicle_id": "UAV1"}},
                                   report_seq_of={}.get)
        self.assertIn("41", str(ctx.exception))
        self.assertEqual(tr.reason_key(ctx.exception), "readiness_confirm.report_seq matches no relayed READINESS")

    def test_a_missing_report_seq_is_zero_and_so_unknown(self):
        with self.assertRaises(tr.Refusal):
            tr.command_to_outgoing({"readiness_confirm": {"vehicle_id": "UAV1"}}, report_seq_of={5: 1}.get)


class Refusals(unittest.TestCase):
    def test_no_body(self):
        with self.assertRaises(tr.Refusal):
            tr.command_to_outgoing({"seq": 1})

    def test_a_body_that_is_not_an_object(self):
        with self.assertRaises(tr.Refusal):
            tr.command_body({"run_start": 7})

    def test_enum_names_and_numbers_outside_the_pinned_enum(self):
        for bad in ("TYPE_BLIMP", 9, True, -1):
            with self.assertRaises(tr.Refusal, msg=repr(bad)):
                tr.command_to_outgoing({"seq": 1, "all_clear": {"vehicle_type": bad}})

    def test_negative_fractional_and_non_numeric_seq(self):
        for bad in (-1, 1.5, "abc", True):
            with self.assertRaises(tr.Refusal, msg=repr(bad)):
                tr.command_to_outgoing({"seq": bad, "run_start": {"run_id": 1}})

    def test_protobuf_json_writes_big_integers_as_strings_and_they_are_accepted(self):
        out = tr.command_to_outgoing({"seq": "12", "run_start": {"run_id": "7"}})
        self.assertEqual((out.cmd_seq, out.info["run_id"]), (12, 7))

    def test_u16_fields_are_refused_loudly_not_wrapped(self):
        """The proto says uint32; the wire says 16 bits. 70000 would arrive as 4464."""
        cases = {
            "RUN_START.run_id": {"seq": 1, "run_start": {"run_id": 70000}},
            "ASSISTANCE_REQUEST.cmd_seq": {"seq": 70000, "assistance_request": {"position": SINGAPORE}},
            "KEEP_OUT_ZONE.cmd_seq": {"seq": 70000, "keep_out_zone": {"center": SINGAPORE, "radius_m": 5}},
            "ALL_CLEAR.cmd_seq": {"seq": 70000, "all_clear": {"vehicle_type": 3}},
            "READINESS_CONFIRM.report_seq": {"readiness_confirm": {"report_seq": 70000, "vehicle_id": "UAV1"}},
        }
        for field, cmd in cases.items():
            with self.assertRaises(tr.Refusal, msg=field) as ctx:
                tr.command_to_outgoing(cmd)
            self.assertIn(field, str(ctx.exception))
            self.assertIn("70000", str(ctx.exception))
            self.assertEqual(tr.reason_key(ctx.exception), "%s does not fit u16" % field)

    def test_the_largest_u16_value_still_goes(self):
        out = tr.command_to_outgoing({"seq": 65535, "run_start": {"run_id": 65535}})
        self.assertEqual(decoded(out)[1]["run_id"], 65535)

    def test_reason_key_has_no_values_in_it(self):
        a = tr.Refusal("vehicle id 'X9' has no RXL node (expected a USV/UAV/UUV prefix)")
        b = tr.Refusal("vehicle id 'Q7' has no RXL node (expected a USV/UAV/UUV prefix)")
        self.assertEqual(tr.reason_key(a), tr.reason_key(b))
        self.assertNotIn("X9", tr.reason_key(a))
        c = tr.Refusal("report seq 12 is not 34")
        self.assertEqual(tr.reason_key(c), "report seq # is not #")

    def test_reason_key_is_bounded(self):
        self.assertLessEqual(len(tr.reason_key(tr.Refusal("x" * 1000))), 100)


class Course(unittest.TestCase):
    def test_translates_and_decodes_back(self):
        payload = tr.course_payload({"course_id": "ALPHA", "pinger_freq_hz": 27000, "corners": CORNERS})
        name, msg = rx.decode(rx.payload_type(rx.COURSE), payload)
        self.assertEqual((name, msg["course_epoch"], msg["pinger_freq_hz"], len(msg["corners"])),
                         ("COURSE", rx.course_epoch("ALPHA"), 27000, 5))
        self.assertAlmostEqual(msg["corners"][2]["lat"], 1.2811, 6)
        self.assertAlmostEqual(msg["corners"][2]["lon"], 103.85625, 6)

    def test_camel_case_and_pairs(self):
        a = tr.course_payload({"courseId": "ALPHA", "pingerFreqHz": 27000, "corners": CORNERS})
        b = tr.course_payload({"course_id": "ALPHA", "pinger_freq_hz": 27000,
                               "corners": [[c["latitude"], c["longitude"]] for c in CORNERS]})
        self.assertEqual(a, b)

    def test_a_course_of_fifteen_corners_is_124_bytes_and_sixteen_is_refused(self):
        self.assertEqual(len(tr.course_payload({"course_id": "C", "corners": polygon(15)})), 124)
        with self.assertRaises(tr.Refusal) as ctx:
            tr.course_payload({"course_id": "C", "corners": polygon(16)})
        self.assertIn("16 corners", str(ctx.exception))

    def test_refusals(self):
        with self.assertRaises(tr.Refusal):
            tr.course_payload({"corners": CORNERS})                       # no id, so no epoch
        with self.assertRaises(tr.Refusal):
            tr.course_payload({"course_id": "C", "corners": polygon(2)})
        with self.assertRaises(tr.Refusal):
            tr.course_payload({"course_id": "C", "pinger_freq_hz": 70000, "corners": CORNERS})
        with self.assertRaises(tr.Refusal):
            tr.course_payload("ALPHA")


class Geofence(unittest.TestCase):
    def test_a_small_fence_is_one_part(self):
        (part,) = tr.geofence_payloads(CORNERS)
        name, msg = rx.decode(rx.payload_type(rx.GEOFENCE_INFO), part)
        self.assertEqual((name, msg["part"], msg["n_parts"], msg["closed"], len(msg["vertices"])),
                         ("GEOFENCE_INFO", 0, 1, True, 5))

    def test_a_long_fence_is_split_into_parts_that_each_fit_a_tunnel(self):
        parts = tr.geofence_payloads(polygon(34))
        self.assertEqual(len(parts), 3)                                    # 15 + 15 + 4
        for p in parts:
            self.assertLessEqual(len(p), rx.TUNNEL_MAX)
        self.assertEqual([len(p) for p in parts], [125, 125, 5 + 4 * 8])

    def test_the_parts_rebuild_the_polygon_in_any_order(self):
        fence = polygon(34)
        parts = tr.geofence_payloads(fence)
        asm = rx.GeofenceAssembler()
        self.assertIsNone(asm.add(parts[2]))
        self.assertIsNone(asm.add(parts[0]))
        got = asm.add(parts[1])
        self.assertEqual(len(got), 34)
        for (lat, lon), want in zip(got, fence):
            self.assertAlmostEqual(lat, want["latitude"], 6)
            self.assertAlmostEqual(lon, want["longitude"], 6)

    def test_the_limits_are_15_per_part_and_225_in_all(self):
        self.assertEqual(len(tr.geofence_payloads(polygon(15))), 1)
        self.assertEqual(len(tr.geofence_payloads(polygon(16))), 2)
        self.assertEqual(len(tr.geofence_payloads(polygon(225))), 15)
        for p in tr.geofence_payloads(polygon(225)):
            self.assertLessEqual(len(p), rx.TUNNEL_MAX)
        with self.assertRaises(tr.Refusal) as ctx:
            tr.geofence_payloads(polygon(241))
        self.assertIn("vertices", str(ctx.exception))

    def test_two_vertices_are_not_a_fence(self):
        with self.assertRaises(tr.Refusal):
            tr.geofence_payloads(polygon(2))

    def test_all_parts_of_one_fence_share_an_epoch_and_a_change_changes_it(self):
        a = [rx.unpack_geofence_info(p)["fence_epoch"] for p in tr.geofence_payloads(polygon(34))]
        self.assertEqual(len(set(a)), 1)
        moved = polygon(34)
        moved[7]["latitude"] += 0.001
        self.assertNotEqual(rx.unpack_geofence_info(tr.geofence_payloads(moved)[0])["fence_epoch"], a[0])

    def test_accepted_shapes(self):
        pairs = [[c["latitude"], c["longitude"]] for c in CORNERS]
        self.assertEqual(tr.geofence_payloads(pairs), tr.geofence_payloads(CORNERS))
        self.assertEqual(tr.geofence_payloads({"uav_geofence": CORNERS}), tr.geofence_payloads(CORNERS))
        self.assertEqual(tr.geofence_payloads({"uavGeofence": CORNERS}), tr.geofence_payloads(CORNERS))
        self.assertEqual(tr.geofence_payloads({"points": CORNERS}), tr.geofence_payloads(CORNERS))

    def test_refusals(self):
        for bad in ("fence", 7, {"nothing": 1}, [{"latitude": 99.0, "longitude": 1.0}] * 4, [{}] * 4):
            with self.assertRaises(tr.Refusal, msg=repr(bad)):
                tr.geofence_payloads(bad)


def parse_utc(text: str) -> float:
    return datetime.strptime(text, "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc).timestamp()


class Uplink(unittest.TestCase):
    CTX = tr.UplinkContext("UAV1", "TYPE_UAV", rx.NODE_UAV)

    def report(self, cid, data, ctx=None):
        name, msg = rx.decode(rx.payload_type(cid), data)
        return tr.report_from_rxl(name, msg, ctx or self.CTX, 1_790_000_000.25)

    def test_every_uplink_id_has_a_translator(self):
        uplink = {rx.NAMES[c] for c in (rx.HEARTBEAT, rx.SAFE_PASSAGE, rx.PIPELINE_SURVEY, rx.RESOURCE_DELIVERY,
                                         rx.DOCKING, rx.FIREFIGHTING, rx.INCIDENT_ACK, rx.READINESS)}
        self.assertEqual(set(tr.UPLINK_BODIES), uplink)

    def test_the_envelope(self):
        r = self.report(rx.DOCKING, rx.pack_docking(bay_id=2))
        self.assertEqual(r["vehicle_id"], "UAV1")
        self.assertEqual(parse_utc(r["sent_at"]), 1_790_000_000.25)
        self.assertEqual(r["docking"], {"bay_id": 2})
        self.assertNotIn("seq", r)                                      # the adapter stamps its own

    def test_heartbeat(self):
        data = rx.pack_heartbeat(vehicle=rx.NODE_UAV, robot_state=rx.STATE_AUTO, flight_phase=rx.PHASE_AIRBORNE,
                                 pos_valid=True, att_valid=True, alt_valid=True, lat=1.2806, lon=103.8557,
                                 spd_mps=6.5, hdg_deg=123.45, roll_deg=-3.25, pitch_deg=1.5, alt_m=17.4,
                                 task=rx.TASK_NONE)
        hb = self.report(rx.HEARTBEAT, data)["heartbeat"]
        self.assertEqual(hb["state"], "STATE_AUTO")
        self.assertEqual(hb["flight_phase"], "FLIGHT_PHASE_AIRBORNE")
        self.assertEqual(hb["current_task"], "TASK_NONE")
        self.assertEqual(hb["vehicle_type"], "TYPE_UAV")
        self.assertEqual(hb["position"], {"latitude": 1.2806, "longitude": 103.8557})
        self.assertEqual((hb["spd_mps"], hb["heading_deg"], hb["roll_deg"], hb["pitch_deg"], hb["altitude_hae_m"],
                          hb["depth_m"]), (6.5, 123.45, -3.25, 1.5, 17.4, 0.0))

    def test_heartbeat_refusals_each_name_what_is_missing(self):
        good = dict(vehicle=rx.NODE_UAV, robot_state=rx.STATE_AUTO, flight_phase=rx.PHASE_AIRBORNE, pos_valid=True,
                    att_valid=True, alt_valid=True, lat=1.28, lon=103.85, hdg_deg=90.0, task=rx.TASK_NONE)
        cases = {
            "robot_state": dict(good, robot_state=rx.STATE_UNKNOWN),
            "flight_phase": dict(good, flight_phase=0),
            "node": dict(good, vehicle=rx.NODE_USV),
            "position": dict(good, pos_valid=False),
            "attitude": dict(good, att_valid=False),
            "altitude": dict(good, alt_valid=False),
            "heading": dict(good, hdg_deg=None),
            "task": dict(good, task=0),
        }
        for what, kw in cases.items():
            with self.assertRaises(tr.Refusal, msg=what) as ctx:
                self.report(rx.HEARTBEAT, rx.pack_heartbeat(**kw))
            self.assertIn(what, str(ctx.exception).lower(), what)

    def test_a_surface_vehicle_may_have_no_flight_phase(self):
        usv = tr.UplinkContext("USV1", "TYPE_USV", rx.NODE_USV)
        data = rx.pack_heartbeat(vehicle=rx.NODE_USV, robot_state=rx.STATE_AUTO, flight_phase=0, pos_valid=True,
                                 att_valid=True, alt_valid=True, lat=1.28, lon=103.85, hdg_deg=90.0)
        hb = self.report(rx.HEARTBEAT, data, usv)["heartbeat"]
        self.assertNotIn("flight_phase", hb)
        self.assertEqual(hb["vehicle_type"], "TYPE_USV")

    def test_safe_passage(self):
        data = rx.pack_safe_passage(time_boot_ms=1, entry=(1.28, 103.85), exit_=None,
                                    buoys=[(1.2805, 103.8505, 2), (1.2806, 103.8506, 4)])
        sp = self.report(rx.SAFE_PASSAGE, data)["safe_passage"]
        self.assertEqual(sp["entry_position"], {"latitude": 1.28, "longitude": 103.85})
        self.assertNotIn("exit_position", sp)                              # unknown stays absent, never 0,0
        self.assertEqual([b["state"] for b in sp["buoys"]], ["BEACON_STATE_FLASHING_RED", "BEACON_STATE_FLASHING_BLUE"])

    def test_safe_passage_with_an_unknown_beacon_state_is_refused(self):
        data = rx.pack_safe_passage(time_boot_ms=1, entry=None, exit_=None, buoys=[(1.28, 103.85, 0)])
        with self.assertRaises(tr.Refusal):
            self.report(rx.SAFE_PASSAGE, data)

    def test_pipeline_survey(self):
        data = rx.pack_pipeline_survey(lat=1.28, lon=103.85, segments=[1, 2, 1])
        ps = self.report(rx.PIPELINE_SURVEY, data)["pipeline_survey"]
        self.assertEqual(ps["segments"], ["PIPELINE_SEGMENT_INTACT", "PIPELINE_SEGMENT_DAMAGED",
                                          "PIPELINE_SEGMENT_INTACT"])
        self.assertEqual(ps["active_buoy_position"], {"latitude": 1.28, "longitude": 103.85})

    def test_resource_delivery(self):
        data = rx.pack_resource_delivery(task=4, resource_color=1, delivery_circle_color=3)
        self.assertEqual(self.report(rx.RESOURCE_DELIVERY, data)["resource_delivery"],
                         {"task": "TASK_COORDINATED_LOGISTICS", "resource_color": "COLOR_RED",
                          "delivery_circle_color": "COLOR_BLUE"})
        with self.assertRaises(tr.Refusal):
            self.report(rx.RESOURCE_DELIVERY, rx.pack_resource_delivery(task=4, resource_color=0,
                                                                       delivery_circle_color=3))

    def test_docking_firefighting_incident_ack_and_readiness(self):
        self.assertEqual(self.report(rx.FIREFIGHTING, rx.pack_firefighting(window_id=3))["firefighting"],
                         {"window_id": 3})
        self.assertEqual(self.report(rx.INCIDENT_ACK, rx.pack_incident_ack(cmd_seq=4242))["incident_ack"],
                         {"command_seq": 4242})
        self.assertEqual(self.report(rx.READINESS, rx.pack_readiness(cmd_seq=4242))["readiness"],
                         {"command_seq": 4242})

    def test_a_downlink_name_is_not_an_uplink_report(self):
        with self.assertRaises(tr.Refusal):
            tr.report_from_rxl("RUN_START", {"run_id": 1, "flags": 0}, self.CTX, 0.0)

    def test_stamp_seq_returns_a_copy_and_refuses_zero(self):
        base = {"vehicle_id": "UAV1", "docking": {"bay_id": 1}}
        stamped = tr.stamp_seq(base, 5)
        self.assertEqual((stamped["seq"], "seq" in base), (5, False))
        for bad in (0, -1, 2 ** 32, True, "3"):
            with self.assertRaises(tr.Refusal, msg=repr(bad)):
                tr.stamp_seq(base, bad)
        self.assertEqual(tr.stamp_seq(base, 2 ** 32 - 1)["seq"], 2 ** 32 - 1)


class Heartbeat(unittest.TestCase):
    def test_report_from_heartbeat_wraps_the_body_untouched(self):
        body = {"state": "STATE_AUTO"}
        r = tr.report_from_heartbeat(body, "UAV1", 1_790_000_000.0)
        self.assertEqual((r["vehicle_id"], r["heartbeat"]), ("UAV1", body))
        self.assertEqual(parse_utc(r["sent_at"]), 1_790_000_000.0)


if __name__ == "__main__":
    unittest.main()
