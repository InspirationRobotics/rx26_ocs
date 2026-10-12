"""ocs_adapter: the radio side of the RoboCommand bridge, driven with a real Hub over a fake
serial link, a fake bridge connection and one injectable clock -- no sockets, no sleeping.

What is pinned here: what the adapter says to the bridge (hello, reports, seq, sent_at), what it
puts on the radio (RXL TUNNELs, the one COMMAND_LONG), how long it keeps saying it (the repeat,
ACK and give-up policy of every downlink message), and that whatever it cannot say truthfully it
refuses loudly instead of repairing."""
from __future__ import annotations

import collections
import json
import socket
import tempfile
import time
import unittest
from datetime import datetime, timezone
from pathlib import Path
from unittest import mock

from pymavlink.dialects.v20 import ardupilotmega as ap

from fleet import ocs_adapter as oa
from fleet import rxl_codec as rx
from fleet.fleet_link import DEFAULT_CONFIG, load_config
from fleet.mavframe import MSG_COMMAND_LONG, MSG_TUNNEL, parse_tunnel
from fleet.ocs_adapter import BridgeLink, OcsAdapter, OcsCfg, OcsConfigError, RadioVehicleCfg
from rx_bridge import framing

from .helpers import FakeAutopilot, encode, frames_of, heartbeat, make_hub, position, tunnel_frame

WALL0 = 1_790_000_000.0
T0 = 1000.0                                    # helpers.Clock starts here
SINGAPORE = {"latitude": 1.2806, "longitude": 103.8557}
CORNERS = [{"latitude": 1.2801, "longitude": 103.8552}, {"latitude": 1.2801, "longitude": 103.85625},
           {"latitude": 1.2811, "longitude": 103.85625}, {"latitude": 1.2811, "longitude": 103.8552}]
COURSE = {"course_id": "ALPHA", "pinger_freq_hz": 27000,
          "corners": CORNERS + [CORNERS[0]]}


def ring(n):
    return [{"latitude": 1.28 + 0.0001 * i, "longitude": 103.85 + 0.0002 * i} for i in range(n)]


def parse_utc(text):
    return datetime.strptime(text, "%Y-%m-%dT%H:%M:%S.%fZ").replace(tzinfo=timezone.utc).timestamp()


Tx = collections.namedtuple("Tx", "t name msg target seq")


class FakeBridge:
    """Stands in for BridgeLink: records what the adapter hands the bridge."""

    def __init__(self, vehicle, on_frame):
        self.vehicle, self.on_frame = vehicle, on_frame
        self.connected = True
        self.sent = []

    def send_json(self, obj):
        if not self.connected:
            return False
        self.sent.append(json.loads(json.dumps(obj, allow_nan=False)))      # exactly what would be framed
        return True

    def status(self):
        return {"connected": self.connected}

    def reports(self, kind):
        return [r for r in self.sent if kind in r]


class Rig:
    """One radio vehicle (UAV1) on a Hub with a fake `rfd` serial link."""

    def __init__(self, vehicle=None, **ocs):
        self.hub, self.clock, self.rfd, _ = make_hub()
        self.hub.route_age_s = 1e9                  # long scenarios: a route never ages out unless a test says
        vk = dict(vehicle_id="UAV1", geoid_separation_m=-24.6)
        vk.update(vehicle or {})
        self.cfg = OcsCfg(vehicles=(RadioVehicleCfg(**vk),), **ocs)
        self.bridge = None
        self.adapter = OcsAdapter(self.cfg, self.hub, clock=self.clock,
                                  wall=lambda: WALL0 + (self.clock.t - T0), bridge_factory=self._bridge)
        self.v = self.adapter.vehicles["UAV1"]
        self.log = []                               # every Tx, stamped with the time it was seen
        self._n = 0
        self._cursor = 0

    def _bridge(self, vehicle, on_frame):
        self.bridge = FakeBridge(vehicle, on_frame)
        return self.bridge

    # ---- radio in
    def seq(self):
        self._n += 1
        return self._n & 0xFF

    def hear(self, raw):
        self.hub.link_rx("rfd", raw)

    def companion(self, cid, data, *, target=250):
        self.hear(tunnel_frame(rx.payload_type(cid), data, sysid=200, target=target, seq=self.seq()))
        return self._n & 0xFF

    def autopilot_heartbeat(self):
        self.hear(heartbeat(1, base_mode=129, custom_mode=4, seq=self.seq()))

    def autopilot(self, ext=1, lat=-353632621):
        """A complete, healthy autopilot picture (ext=None leaves EXTENDED_SYS_STATE out)."""
        self.autopilot_heartbeat()
        self.hear(position(1, lat=lat, seq=self.seq()))
        self.hear(encode("attitude", 0, 0.1, -0.05, 1.5, 0.0, 0.0, 0.0, sysid=1, seq=self.seq()))
        if ext is not None:
            self.ext_state(ext)

    def ext_state(self, landed=1):
        self.hear(encode("extended_sys_state", 0, landed, sysid=1, seq=self.seq()))

    # ---- bridge in
    def command(self, obj):
        self.bridge.on_frame(obj)
        self._collect()

    def info(self, **kw):
        self.command({"ocs_info": kw})

    # ---- time
    def poll(self):
        self.adapter.poll()
        self._collect()

    def step(self, dt):
        self.clock.advance(dt)
        self.poll()

    def run(self, seconds, step=0.25, autopilot=False):
        """Advance in `step`s, polling each time; `autopilot` keeps the autopilot's HEARTBEAT alive."""
        t_end = self.clock.t + seconds
        while self.clock.t < t_end - 1e-9:
            self.clock.advance(step)
            if autopilot and abs(self.clock.t % 1.0) < 1e-9:
                self.autopilot_heartbeat()
            self.poll()

    # ---- radio out
    def _collect(self):
        frames = frames_of([d for d, _ in self.rfd.sent])
        for f in frames[self._cursor:]:
            if f.msgid == MSG_TUNNEL:
                t = parse_tunnel(f)
                name, msg = rx.decode(t.payload_type, t.data)
                self.log.append(Tx(self.clock.t, name, msg, t.target_system, f.seq))
            else:
                self.log.append(Tx(self.clock.t, "MAV%d" % f.msgid, f, None, f.seq))
        self._cursor = len(frames)

    def times(self, name, since=T0):
        return [round(x.t - since, 3) for x in self.log if x.name == name]

    def of(self, name):
        return [x for x in self.log if x.name == name]

    def command_longs(self):
        return [ap.MAVLink(None).decode(bytearray(x.msg.raw)) for x in self.log if x.name == "MAV%d" % MSG_COMMAND_LONG]

    def status(self):
        st = self.adapter.status()
        json.dumps(st, allow_nan=False)             # whatever /fleet shows must be plain JSON
        return st["vehicles"]["UAV1"]


def heartbeat_body(**kw):
    args = dict(vehicle=rx.NODE_UAV, robot_state=rx.STATE_AUTO, flight_phase=rx.PHASE_AIRBORNE, pos_valid=True,
                att_valid=True, alt_valid=True, lat=1.28, lon=103.85, spd_mps=1.0, hdg_deg=90.0, roll_deg=1.0,
                pitch_deg=2.0, alt_m=30.0, task=rx.TASK_NONE)
    args.update(kw)
    return rx.pack_heartbeat(**args)


# ====================================================================== the hello

class Hello(unittest.TestCase):
    def test_the_hello_declares_the_synthesized_heartbeat(self):
        link = BridgeLink("UAV1", "127.0.0.1", 1, lambda o: None)
        self.assertEqual(link.hello(), {"ocs_hello": {"vehicle_id": "UAV1", "link": "radio", "agent": "fleet_link",
                                                       "synthesized": ["heartbeat"]}})

    def test_one_constant_names_the_kind(self):
        self.assertEqual(oa.SYNTHESIZED_KINDS, ("heartbeat",))
        self.assertEqual(BridgeLink("U", "h", 1, lambda o: None).hello()["ocs_hello"]["synthesized"],
                         list(oa.SYNTHESIZED_KINDS))

    def test_it_is_the_first_frame_on_the_wire_and_reports_follow_framed(self):
        srv = socket.socket()
        self.addCleanup(srv.close)
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        srv.settimeout(5.0)
        link = BridgeLink("UAV1", "127.0.0.1", srv.getsockname()[1], lambda o: None, reconnect_min_s=0.05)
        link.start()
        self.addCleanup(link.stop)
        conn, _ = srv.accept()
        self.addCleanup(conn.close)
        conn.settimeout(5.0)
        reader, frames = framing.FrameReader(), []
        while not frames:
            frames = reader.feed(conn.recv(4096))
        self.assertEqual(json.loads(frames[0])["ocs_hello"]["synthesized"], ["heartbeat"])
        self.assertEqual(json.loads(frames[0])["ocs_hello"]["link"], "radio")
        deadline = time.monotonic() + 5.0                   # the socket is published just after the hello is out
        while not link.connected and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(link.send_json({"vehicle_id": "UAV1", "seq": 1}))
        more = []
        while not more:
            more = reader.feed(conn.recv(4096))
        self.assertEqual(json.loads(more[0]), {"vehicle_id": "UAV1", "seq": 1})
        self.assertEqual(link.status()["synthesized"], ["heartbeat"])

    def test_nothing_is_sent_before_the_connection_exists(self):
        self.assertFalse(BridgeLink("UAV1", "127.0.0.1", 1, lambda o: None).send_json({"a": 1}))


# ====================================================================== config

class Config(unittest.TestCase):
    def test_an_empty_table_is_a_disabled_adapter(self):
        self.assertFalse(oa.parse_ocs_config({}).enabled)

    def test_the_shipped_fleet_toml(self):
        ocs = load_config(DEFAULT_CONFIG).ocs
        (v,) = ocs.vehicles
        self.assertEqual((v.vehicle_id, v.companion_sysid, v.autopilot_sysid, v.link, ocs.sysid, ocs.compid),
                         ("UAV1", 200, 1, "rfd", 250, 190))
        self.assertFalse(v.phase_fallback)                    # off by default: a heuristic is opt-in
        self.assertEqual(v.geoid_separation_m, -24.6)         # the user's value, deliberately untouched
        self.assertTrue(v.fence_upload)                       # the declared geofence goes onto the autopilot
        self.assertTrue(v.fence_may_block_arming)             # Chase: even if Ekko then cannot arm at home
        self.assertTrue(v.fence_clear_without_course)         # Chase: no RoboCommand course -> no polygon
        self.assertEqual(Path(ocs.fence_backup_dir), Path(oa.OCS_DIR) / "run" / "fence")

    def test_refusals(self):
        base = {"radio_vehicle": [{"vehicle_id": "UAV1"}]}
        bad = [
            {"radio_vehicle": [{"vehicle_id": "UAV1"}, {"vehicle_id": "UAV1"}]},
            {"radio_vehicle": [{"vehicle_id": "UAV1", "companion_sysid": 250}]},
            {"radio_vehicle": [{"vehicle_id": "UAV1", "companion_sysid": 1}]},
            {"radio_vehicle": [{"vehicle_id": "UAV1"}, {"vehicle_id": "UAV2"}]},
            {"radio_vehicle": [{"type": "TYPE_UAV"}]},
            {"radio_vehicle": [{"vehicle_id": "UAV1", "type": "TYPE_BLIMP"}]},
            {"radio_vehicle": [{"vehicle_id": "UAV1", "geoid_separation_m": 500}]},
            {"radio_vehicle": [{"vehicle_id": "UAV1", "node": 1}]},
            {"radio_vehicle": [{"vehicle_id": "UAV1", "fence_upload": "yes"}]},
            {"radio_vehicle": [{"vehicle_id": "UAV1", "fence_may_block_arming": 1}]},
            dict(base, bridge="nonsense"),
            dict(base, sysid=0),
        ]
        for raw in bad:
            with self.assertRaises(OcsConfigError, msg=repr(raw)):
                oa.parse_ocs_config(raw)

    def test_a_vehicle_on_a_link_the_hub_does_not_have(self):
        hub, clock, _rfd, _ = make_hub()
        with self.assertRaises(OcsConfigError):
            OcsAdapter(OcsCfg(vehicles=(RadioVehicleCfg("UAV1", link="nope"),)), hub)

    def test_types_overlap_is_the_one_rule_for_unknown(self):
        self.assertTrue(oa.types_overlap([], [3]))
        self.assertTrue(oa.types_overlap([0], [3]))
        self.assertTrue(oa.types_overlap([3], [0]))
        self.assertTrue(oa.types_overlap([1, 3], [3]))
        self.assertFalse(oa.types_overlap([1], [3]))
        self.assertFalse(oa.types_overlap([1, 2], [3]))


# ================================================================ uplink: reports to the bridge

class Uplink(unittest.TestCase):
    def test_a_companion_heartbeat_is_forwarded_under_our_vehicle_id(self):
        r = Rig()
        r.companion(rx.HEARTBEAT, heartbeat_body())
        r.poll()
        (rep,) = r.bridge.reports("heartbeat")
        self.assertEqual(rep["vehicle_id"], "UAV1")
        self.assertEqual(rep["heartbeat"]["position"], {"latitude": 1.28, "longitude": 103.85})
        self.assertEqual(r.status()["heartbeat"]["source"], "companion")

    def test_every_report_carries_the_adapters_own_seq_counting_up_from_one(self):
        r = Rig()
        r.companion(rx.DOCKING, rx.pack_docking(bay_id=1))
        r.companion(rx.FIREFIGHTING, rx.pack_firefighting(window_id=2))
        r.companion(rx.RESOURCE_DELIVERY, rx.pack_resource_delivery(task=4, resource_color=1, delivery_circle_color=2))
        r.poll()
        r.autopilot()
        r.step(0.5)                                         # the stand-in heartbeat is due again (2 Hz)
        seqs = [x["seq"] for x in r.bridge.sent]
        self.assertGreaterEqual(len(seqs), 4)
        self.assertEqual(seqs, list(range(1, len(seqs) + 1)))
        self.assertNotIn(0, seqs)

    def test_a_report_the_bridge_refused_still_burns_nothing_it_can_be_confused_with(self):
        r = Rig()
        r.bridge.connected = False
        r.companion(rx.DOCKING, rx.pack_docking(bay_id=1))
        r.poll()
        self.assertEqual(r.bridge.sent, [])
        r.bridge.connected = True
        r.companion(rx.DOCKING, rx.pack_docking(bay_id=2))
        r.poll()
        self.assertEqual(len(r.bridge.sent), 1)
        self.assertGreaterEqual(r.bridge.sent[0]["seq"], 1)

    def test_foreign_and_unaddressed_tunnels_are_counted_and_ignored(self):
        r = Rig()
        r.hear(tunnel_frame(32772, b"\x01\x02", sysid=200, seq=r.seq()))                  # boat_link POSITIONS
        r.companion(rx.DOCKING, rx.pack_docking(bay_id=1), target=77)                      # for another station
        r.poll()
        up = r.status()["uplink"]
        self.assertEqual(up["ignored"], {"foreign_tunnel": 1, "not_addressed_to_ocs": 1})
        self.assertEqual(r.bridge.sent, [])

    def test_a_corrupt_report_is_counted_not_forwarded(self):
        r = Rig()
        r.hear(tunnel_frame(rx.payload_type(rx.HEARTBEAT), bytes(10), sysid=200, seq=r.seq()))
        r.poll()
        up = r.status()["uplink"]
        self.assertEqual(up["decode_errors"], 1)
        self.assertEqual(r.bridge.sent, [])

    def test_an_untruthful_companion_heartbeat_is_refused_with_the_reason_and_the_stand_in_stays_quiet(self):
        r = Rig()
        r.autopilot()
        r.companion(rx.HEARTBEAT, heartbeat_body(pos_valid=False))
        r.step(0.1)
        self.assertEqual(r.bridge.reports("heartbeat"), [])                  # not the companion's, not a stand-in
        st = r.status()
        self.assertEqual(st["heartbeat"]["source"], "companion")
        self.assertIn("not forwarded", st["heartbeat"]["reason"])
        self.assertIn("position", st["heartbeat"]["reason"])
        self.assertEqual(st["uplink"]["refused"], 1)


class Acks(unittest.TestCase):
    def test_readiness_is_acked_with_the_frames_own_sequence_once_the_bridge_has_it(self):
        r = Rig()
        mav_seq = r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=7))
        r.poll()
        (ack,) = r.of("ACK")
        self.assertEqual(ack.msg, {"msg_id": rx.READINESS, "seq": mav_seq, "src": rx.NODE_UAV})
        self.assertEqual(ack.target, 200)
        self.assertEqual(len(r.bridge.reports("readiness")), 1)

    def test_a_report_the_bridge_could_not_take_is_not_acked(self):
        r = Rig()
        r.bridge.connected = False
        r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=7))
        r.poll()
        self.assertEqual(r.of("ACK"), [])
        self.assertEqual(r.status()["uplink"]["not_forwarded_no_bridge"], 1)

    def test_the_retry_of_a_report_the_bridge_missed_goes_through_and_is_then_acked(self):
        """The ACK is a promise. A retry must not be mistaken for a duplicate of something the
        bridge never got."""
        r = Rig()
        r.bridge.connected = False
        r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=7))
        r.poll()
        r.bridge.connected = True
        r.step(0.5)
        r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=7))
        r.poll()
        self.assertEqual(len(r.bridge.reports("readiness")), 1)
        self.assertEqual(len(r.of("ACK")), 1)

    def test_a_duplicate_is_acked_again_but_forwarded_once(self):
        r = Rig()
        r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=7))
        r.poll()
        r.step(2.0)
        r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=7))               # the first ACK was lost
        r.poll()
        self.assertEqual(len(r.bridge.reports("readiness")), 1)
        self.assertEqual(len(r.of("ACK")), 2)
        self.assertEqual(r.status()["uplink"]["duplicates"], 1)

    def test_the_same_content_after_the_horizon_is_a_new_report(self):
        r = Rig()
        r.companion(rx.INCIDENT_ACK, rx.pack_incident_ack(cmd_seq=3))
        r.poll()
        r.step(oa.DEDUP_HORIZON_S + 1)
        r.companion(rx.INCIDENT_ACK, rx.pack_incident_ack(cmd_seq=3))
        r.poll()
        self.assertEqual(len(r.bridge.reports("incident_ack")), 2)

    def test_heartbeats_and_task_reports_are_not_acked(self):
        r = Rig()
        r.companion(rx.HEARTBEAT, heartbeat_body())
        r.companion(rx.DOCKING, rx.pack_docking(bay_id=1))
        r.poll()
        self.assertEqual(r.of("ACK"), [])

    def test_no_route_no_ack_on_the_air(self):
        r = Rig()
        r.hub.route_age_s = 0.0
        r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=7))
        r.step(1.0)
        self.assertEqual(len(r.bridge.reports("readiness")), 1)
        self.assertEqual(r.of("ACK"), [])
        self.assertGreaterEqual(r.status()["downlink"]["tx_no_route"], 1)


class ReadinessRoundTrip(unittest.TestCase):
    """report_seq in a ReadinessConfirm is the seq WE stamped; Ekko knows its own cmd_seq."""

    def readiness(self, r, cmd_seq):
        r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=cmd_seq))
        r.poll()
        return r.bridge.reports("readiness")[-1]

    def test_the_confirm_goes_back_carrying_the_cmd_seq_ekko_used(self):
        r = Rig()
        r.autopilot_heartbeat()
        rep = self.readiness(r, cmd_seq=7)
        self.assertEqual(rep["readiness"], {"command_seq": 7})
        r.command({"vehicle_id": "UAV1", "seq": 5, "readiness_confirm": {"report_seq": rep["seq"],
                                                                         "vehicle_id": "UAV1"}})
        (conf,) = r.of("READINESS_CONFIRM")[:1]
        self.assertEqual(conf.msg, {"report_seq": 7, "vehicle": rx.NODE_UAV})
        self.assertEqual(conf.target, 200)

    def test_two_readiness_reports_keep_their_own_numbers(self):
        r = Rig()
        r.autopilot_heartbeat()
        a, b = self.readiness(r, 7), self.readiness(r, 9)
        self.assertNotEqual(a["seq"], b["seq"])
        r.command({"readiness_confirm": {"report_seq": b["seq"], "vehicle_id": "UAV1"}})
        r.command({"readiness_confirm": {"report_seq": a["seq"], "vehicle_id": "UAV1"}})
        self.assertEqual(sorted({x.msg["report_seq"] for x in r.of("READINESS_CONFIRM")}), [7, 9])

    def test_the_vehicle_id_may_be_missing_from_the_body_the_commands_own_is_used(self):
        r = Rig()
        r.autopilot_heartbeat()
        rep = self.readiness(r, 7)
        r.command({"vehicle_id": "UAV1", "readiness_confirm": {"report_seq": rep["seq"]}})
        self.assertEqual(r.of("READINESS_CONFIRM")[0].msg, {"report_seq": 7, "vehicle": rx.NODE_UAV})

    def test_with_no_vehicle_id_anywhere_the_adapters_own_vehicle_is_used(self):
        r = Rig()
        r.autopilot_heartbeat()
        rep = self.readiness(r, 7)
        r.command({"readiness_confirm": {"report_seq": rep["seq"]}})
        self.assertEqual(r.of("READINESS_CONFIRM")[0].msg, {"report_seq": 7, "vehicle": rx.NODE_UAV})

    def test_a_seq_we_never_relayed_is_refused_loudly_and_nothing_is_sent(self):
        r = Rig()
        r.autopilot_heartbeat()
        self.readiness(r, 7)
        with self.assertLogs("fleet", "WARNING") as logs:
            r.command({"readiness_confirm": {"report_seq": 4242, "vehicle_id": "UAV1"}})
        self.assertIn("READINESS_CONFIRM", logs.output[0])
        self.assertIn("4242", logs.output[0])
        self.assertEqual(r.of("READINESS_CONFIRM"), [])
        self.assertEqual(r.status()["downlink"]["refusals_by_reason"],
                         {"readiness_confirm.report_seq matches no relayed READINESS": 1})

    def test_a_confirm_with_no_report_seq_at_all_is_refused_too(self):
        r = Rig()
        r.autopilot_heartbeat()
        self.readiness(r, 7)
        with self.assertLogs("fleet", "WARNING"):
            r.command({"readiness_confirm": {"vehicle_id": "UAV1"}})
        self.assertEqual(r.of("READINESS_CONFIRM"), [])

    def test_the_confirm_is_three_sends_and_done(self):
        r = Rig()
        r.autopilot_heartbeat()
        rep = self.readiness(r, 7)
        t0 = r.clock.t
        r.command({"readiness_confirm": {"report_seq": rep["seq"], "vehicle_id": "UAV1"}})
        r.run(10)
        self.assertEqual(r.times("READINESS_CONFIRM", t0), [0.0, 0.5, 1.0])
        self.assertEqual(r.status()["downlink"]["queue_depth"], 0)

    def test_a_readiness_the_bridge_never_took_is_not_confirmable(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.bridge.connected = False
        r.companion(rx.READINESS, rx.pack_readiness(cmd_seq=7))
        r.poll()
        self.assertEqual(r.status()["uplink"]["readiness_seqs_held"], 0)

    def test_only_the_most_recent_reports_are_remembered(self):
        with mock.patch.object(oa, "READINESS_SEQS_KEPT", 3):
            r = Rig()
            r.autopilot_heartbeat()
            reps = [self.readiness(r, cmd_seq=100 + i) for i in range(5)]
            self.assertEqual(r.status()["uplink"]["readiness_seqs_held"], 3)
            with self.assertLogs("fleet", "WARNING"):
                r.command({"readiness_confirm": {"report_seq": reps[0]["seq"], "vehicle_id": "UAV1"}})
            r.command({"readiness_confirm": {"report_seq": reps[4]["seq"], "vehicle_id": "UAV1"}})
            self.assertEqual({x.msg["report_seq"] for x in r.of("READINESS_CONFIRM")}, {104})

    def test_the_seq_counter_wraps_past_uint32_without_ever_reaching_zero(self):
        r = Rig()
        r.v._report_seq = 0xFFFFFFFF - 1
        r.companion(rx.DOCKING, rx.pack_docking(bay_id=1))
        r.companion(rx.FIREFIGHTING, rx.pack_firefighting(window_id=1))
        r.companion(rx.RESOURCE_DELIVERY, rx.pack_resource_delivery(task=4, resource_color=1, delivery_circle_color=2))
        r.poll()
        self.assertEqual([x["seq"] for x in r.bridge.sent], [0xFFFFFFFF, 1, 2])


# =================================================================== the stand-in heartbeat

class StandIn(unittest.TestCase):
    def test_nothing_heard_nothing_sent_and_the_reason_is_in_fleet(self):
        r = Rig()
        r.step(0.1)
        self.assertEqual(r.bridge.sent, [])
        hb = r.status()["heartbeat"]
        self.assertEqual(hb["source"], "none")
        self.assertIn("GLOBAL_POSITION_INT", hb["reason"])
        self.assertEqual(hb["suppressed"], 1)

    def test_each_blank_input_sends_nothing_and_names_itself(self):
        cases = [
            ("position", dict(), "GLOBAL_POSITION_INT"),
            ("attitude", dict(), "ATTITUDE"),
            ("autopilot heartbeat", dict(), "HEARTBEAT"),
            ("ext", dict(), "EXTENDED_SYS_STATE"),
        ]
        for what, _, word in cases:
            r = Rig()
            if what != "autopilot heartbeat":
                r.autopilot_heartbeat()
            if what != "position":
                r.hear(position(1, seq=r.seq()))
            if what != "attitude":
                r.hear(encode("attitude", 0, 0.1, -0.05, 1.5, 0.0, 0.0, 0.0, sysid=1, seq=r.seq()))
            if what != "ext":
                r.ext_state()
            r.step(0.1)
            self.assertEqual(r.bridge.sent, [], what)
            self.assertIn(word, r.status()["heartbeat"]["reason"], what)

    def test_no_geoid_configured_nothing_sent(self):
        r = Rig(vehicle=dict(geoid_separation_m=None))
        r.autopilot()
        r.step(0.1)
        self.assertEqual(r.bridge.sent, [])
        self.assertIn("geoid_separation_m", r.status()["heartbeat"]["reason"])

    def test_a_complete_picture_sends_a_stand_in_at_the_configured_rate(self):
        r = Rig()
        r.autopilot()
        r.step(0.0)
        self.assertEqual(len(r.bridge.reports("heartbeat")), 1)
        st = r.status()["heartbeat"]
        self.assertEqual((st["source"], st["reason"], st["phase_source"], st["current_task"]),
                         ("synthesized", None, "autopilot", "TASK_NONE"))
        for _ in range(9):                                  # 0.9 s at 0.1 s steps; 2 Hz = one per 0.5 s
            r.autopilot()
            r.step(0.1)
        self.assertEqual(len(r.bridge.reports("heartbeat")), 2)
        hb = r.bridge.reports("heartbeat")[0]["heartbeat"]
        self.assertEqual(hb["current_task"], "TASK_NONE")
        self.assertEqual(hb["vehicle_type"], "TYPE_UAV")
        self.assertEqual(hb["flight_phase"], "FLIGHT_PHASE_GROUNDED")

    def test_sent_at_is_when_the_data_was_in_hand_not_when_the_tick_ran(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.ext_state()
        r.step(0.1)                                         # attitude at +0.1
        r.hear(encode("attitude", 0, 0.1, -0.05, 1.5, 0.0, 0.0, 0.0, sysid=1, seq=r.seq()))
        r.step(0.1)                                         # position at +0.2: the newest input
        r.hear(position(1, seq=r.seq()))
        r.clock.advance(0.7)                                # the tick runs at +0.9; pose is still < 1.0 s old
        r.poll()
        (rep,) = r.bridge.reports("heartbeat")
        self.assertAlmostEqual(parse_utc(rep["sent_at"]), WALL0 + 0.2, places=3)
        self.assertLess(parse_utc(rep["sent_at"]), WALL0 + 0.9 - 0.5)       # clearly not the tick time

    def test_a_stand_in_built_on_aging_data_says_so(self):
        r = Rig()
        r.autopilot()
        r.run(0.0)
        r.step(0.0)
        first = parse_utc(r.bridge.reports("heartbeat")[0]["sent_at"])
        self.assertAlmostEqual(first, WALL0, places=3)

    def test_a_fresh_companion_heartbeat_replaces_the_stand_in(self):
        r = Rig()
        for _ in range(6):
            r.autopilot(lat=-353632621)
            r.companion(rx.HEARTBEAT, heartbeat_body(lat=1.28))
            r.step(0.5)
        lats = [h["heartbeat"]["position"]["latitude"] for h in r.bridge.reports("heartbeat")]
        self.assertTrue(lats)
        self.assertEqual(set(lats), {1.28})                 # never the autopilot's -35.36
        self.assertEqual(r.status()["heartbeat"]["source"], "companion")

    def test_the_stand_in_resumes_when_the_companion_goes_quiet(self):
        r = Rig()
        r.companion(rx.HEARTBEAT, heartbeat_body())
        r.step(0.0)
        for _ in range(8):
            r.autopilot()
            r.step(0.5)
        self.assertEqual(r.status()["heartbeat"]["source"], "synthesized")

    def test_the_reason_when_the_bridge_is_down(self):
        r = Rig()
        r.autopilot()
        r.bridge.connected = False
        r.step(0.0)
        self.assertEqual(r.bridge.sent, [])
        hb = r.status()["heartbeat"]
        self.assertEqual(hb["sent"], 0)
        self.assertIn("bridge", hb["reason"])
        r.bridge.connected = True
        r.autopilot()
        r.step(0.5)
        self.assertIsNone(r.status()["heartbeat"]["reason"])
        self.assertEqual(r.status()["heartbeat"]["sent"], 1)

    def test_the_fallback_is_off_until_configured(self):
        r = Rig()
        r.autopilot(ext=None)
        r.step(0.0)
        self.assertEqual(r.bridge.sent, [])
        r2 = Rig(vehicle=dict(phase_fallback=True))
        r2.autopilot(ext=None)
        r2.step(0.0)
        (rep,) = r2.bridge.reports("heartbeat")
        self.assertEqual(rep["heartbeat"]["flight_phase"], "FLIGHT_PHASE_AIRBORNE")
        self.assertEqual(r2.status()["heartbeat"]["phase_source"], "fallback")


# ========================================================== asking for EXTENDED_SYS_STATE

class FlightStateRequest(unittest.TestCase):
    def test_the_request_is_exactly_set_message_interval_245_at_one_hertz_to_the_autopilot(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.poll()
        (m,) = r.command_longs()
        self.assertEqual((m.command, m.param1, m.param2, m.param3, m.param7), (511, 245.0, 1e6, 0.0, 0.0))
        self.assertEqual((m.target_system, m.target_component), (1, 1))
        self.assertEqual((m.get_srcSystem(), m.get_srcComponent()), (250, 190))
        self.assertEqual(m.confirmation, 0)

    def test_it_uses_gcs_radios_own_interval_so_the_two_never_fight(self):
        self.assertEqual(oa.REQUEST_INTERVAL_US, 1_000_000)
        self.assertEqual(oa.REQUEST_STALE_S, 10.0)

    def test_it_waits_for_the_autopilot_to_be_alive(self):
        r = Rig()
        r.run(5)
        self.assertEqual(r.command_longs(), [])
        self.assertIn("not alive", r.status()["heartbeat"]["flight_state_request"]["state"])
        r.autopilot_heartbeat()
        r.run(3.0)                                          # the HEARTBEAT has gone stale (status timeout 2.5 s)
        self.assertEqual(len(r.command_longs()), 1)         # ... but it was asked at once while it was alive

    def test_a_stale_autopilot_heartbeat_is_not_asked(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.hub.route_age_s = 1e9
        r.clock.advance(5.0)
        r.poll()
        self.assertEqual(r.command_longs(), [])

    def test_at_most_once_per_ten_seconds_while_it_stays_unheard(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.poll()
        r.run(25, autopilot=True)
        self.assertEqual(r.times("MAV%d" % MSG_COMMAND_LONG), [0.0, 10.0, 20.0])
        self.assertEqual(r.status()["heartbeat"]["flight_state_request"]["sent"], 3)

    def test_a_message_that_is_arriving_is_not_asked_for(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.ext_state()
        r.run(9.5, autopilot=True)
        self.assertEqual(r.command_longs(), [])
        self.assertIn("arriving", r.status()["heartbeat"]["flight_state_request"]["state"])

    def test_it_is_asked_for_again_once_unheard_for_ten_seconds(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.ext_state()                                       # heard at +0
        r.run(12, autopilot=True)
        self.assertEqual(r.times("MAV%d" % MSG_COMMAND_LONG), [10.25])     # first poll with an age > 10 s

    def test_a_flowing_message_keeps_it_quiet_for_good(self):
        r = Rig()
        r.autopilot_heartbeat()
        for _ in range(60):
            r.ext_state()
            r.run(1.0, autopilot=True)
        self.assertEqual(r.command_longs(), [])
        self.assertEqual(r.status()["heartbeat"]["flight_state_request"]["sent"], 0)

    def test_a_companion_heartbeat_makes_the_stand_in_and_so_the_request_unnecessary(self):
        r = Rig()
        r.autopilot_heartbeat()
        for _ in range(30):
            r.companion(rx.HEARTBEAT, heartbeat_body())
            r.run(0.5, autopilot=True)
        self.assertEqual(r.command_longs(), [])
        self.assertIn("not needed", r.status()["heartbeat"]["flight_state_request"]["state"])

    def test_a_refused_write_is_counted_and_retried_after_the_same_ten_seconds(self):
        r = Rig()
        r.rfd.accept = False
        r.autopilot_heartbeat()
        r.poll()
        st = r.status()["heartbeat"]["flight_state_request"]
        self.assertEqual((st["sent"], st["refused"]), (0, 1))
        r.rfd.accept = True
        r.run(9.5, autopilot=True)
        self.assertEqual(r.command_longs(), [])
        r.run(1.0, autopilot=True)
        self.assertEqual(len(r.command_longs()), 1)

    def test_the_request_is_counted_in_tx(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.poll()
        self.assertEqual(r.adapter.status()["tx"]["frames"], 1)

    def test_end_to_end_the_missing_input_is_requested_and_the_heartbeat_follows_it(self):
        """The P0: with EXTENDED_SYS_STATE in no stream group, the stand-in never spoke."""
        r = Rig()
        r.autopilot(ext=None)
        r.step(0.0)
        self.assertEqual(r.bridge.sent, [])                                      # nothing invented in the meantime
        self.assertIn("EXTENDED_SYS_STATE", r.status()["heartbeat"]["reason"])
        self.assertEqual(len(r.command_longs()), 1)                              # ... and it asked
        r.ext_state()                                                            # the autopilot answers
        r.autopilot()
        r.step(0.5)
        self.assertEqual(len(r.bridge.reports("heartbeat")), 1)
        self.assertEqual(len(r.command_longs()), 1)                              # no more asking

    def test_the_frame_that_leaves_is_a_valid_mavlink_frame_the_hub_accepts_back(self):
        r = Rig()
        r.autopilot_heartbeat()
        r.poll()
        raw = r.rfd.sent[0][0]
        self.assertTrue(r.hub.dialect.get(MSG_COMMAND_LONG) is not None)
        self.assertEqual(frames_of([raw])[0].msgid, MSG_COMMAND_LONG)
        self.assertEqual(frames_of([raw])[0].validated, True)


# ======================================================================= the downlink policies

class Policies(unittest.TestCase):
    def rig(self, **kw):
        r = Rig(**kw)
        r.autopilot_heartbeat()                             # a route to the aircraft
        r.poll()
        r.log.clear()                                       # the SET_MESSAGE_INTERVAL is not under test here
        return r

    def ack(self, r, msg_id, tx):
        r.companion(rx.ACK, rx.pack_ack(msg_id=msg_id, seq=tx.seq, src=rx.NODE_OCS))
        r.poll()

    # ---- RUN_START
    def test_run_start_five_at_2_hz_then_every_ten_seconds(self):
        r = self.rig()
        t0 = r.clock.t
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        r.run(30)
        self.assertEqual(r.times("RUN_START", t0), [0.0, 0.5, 1.0, 1.5, 2.0, 12.0, 22.0])
        self.assertEqual(r.of("RUN_START")[0].msg, {"run_id": 7, "flags": 0})
        st = r.status()["downlink"]
        self.assertEqual((st["sends"], st["retries"]), (7, 6))

    def test_an_ack_ends_it(self):
        r = self.rig()
        t0 = r.clock.t
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        r.step(0.5)
        self.ack(r, rx.RUN_START, r.of("RUN_START")[-1])
        r.run(30)
        self.assertEqual(r.times("RUN_START", t0), [0.0, 0.5])
        st = r.status()["downlink"]
        self.assertEqual((st["acks_matched"], st["queue_depth"]), (1, 0))

    def test_an_ack_for_an_attempt_that_never_happened_is_not_matched(self):
        r = self.rig()
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        r.companion(rx.ACK, rx.pack_ack(msg_id=rx.RUN_START, seq=200, src=rx.NODE_OCS))
        r.companion(rx.ACK, rx.pack_ack(msg_id=rx.ALL_CLEAR, seq=r.of("RUN_START")[0].seq, src=rx.NODE_OCS))
        r.poll()
        st = r.status()["downlink"]
        self.assertEqual((st["acks_matched"], st["acks_unmatched"], st["queue_depth"]), (0, 2, 1))

    def test_it_gives_up_without_an_ack_and_says_so(self):
        r = self.rig(retry_give_up_s=15.0)
        t0 = r.clock.t
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        with self.assertLogs("fleet", "WARNING") as logs:
            r.run(30)
        self.assertEqual(r.times("RUN_START", t0), [0.0, 0.5, 1.0, 1.5, 2.0, 12.0])
        self.assertEqual(r.status()["downlink"]["give_ups"], 1)
        self.assertIn("gave up on RUN_START", logs.output[0])

    def test_the_same_run_start_again_is_a_duplicate_not_a_restart(self):
        r = self.rig()
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        r.step(0.5)
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        self.assertEqual(r.status()["downlink"]["duplicates_from_bridge"], 1)
        self.assertEqual(r.status()["downlink"]["queue_depth"], 1)

    def test_a_new_run_drops_the_previous_runs_messages_but_keeps_the_course_and_fence(self):
        r = self.rig()
        r.command({"seq": 1, "run_start": {"run_id": 1}})
        r.command({"seq": 5, "all_clear": {"vehicle_type": "TYPE_UAV"}})          # before the zone: it must not end it
        r.command({"seq": 2, "keep_out_zone": {"center": SINGAPORE, "radius_m": 20, "vehicle_type": "TYPE_UAV"}})
        r.command({"seq": 3, "moving_object_alert": {"position": SINGAPORE, "affected_vehicle_types": ["TYPE_UAV"]}})
        r.command({"seq": 4, "assistance_request": {"position": SINGAPORE, "vehicle_type": "TYPE_UAV"}})
        r.info(course=COURSE, uav_geofence=CORNERS)
        before = {p["msg"] for p in r.status()["downlink"]["queue"]}
        self.assertEqual(before, {"RUN_START", "KEEP_OUT_ZONE", "MOVING_OBJECT_ALERT", "ASSISTANCE_REQUEST",
                                  "ALL_CLEAR", "COURSE", "GEOFENCE_INFO"})
        r.command({"seq": 7, "run_start": {"run_id": 2}})
        after = [p["msg"] for p in r.status()["downlink"]["queue"]]
        self.assertEqual(sorted(after), ["COURSE", "GEOFENCE_INFO", "RUN_START"])
        self.assertEqual(r.status()["downlink"]["cancelled"], 4)
        self.assertEqual(r.status()["downlink"]["run_id"], 2)

    # ---- ASSISTANCE_REQUEST
    def test_assistance_request_repeats_until_the_vehicles_incident_ack_answers_it(self):
        r = self.rig()
        t0 = r.clock.t
        r.command({"seq": 9, "assistance_request": {"position": SINGAPORE, "vehicle_type": "TYPE_UAV"}})
        r.run(7)
        self.assertEqual(r.times("ASSISTANCE_REQUEST", t0), [0.0, 0.5, 1.0, 6.0])
        self.assertEqual(r.of("ASSISTANCE_REQUEST")[0].msg["cmd_seq"], 9)
        r.companion(rx.INCIDENT_ACK, rx.pack_incident_ack(cmd_seq=9))
        r.run(20)
        self.assertEqual(len(r.of("ASSISTANCE_REQUEST")), 4)
        self.assertEqual(r.status()["downlink"]["queue_depth"], 0)
        self.assertEqual(len(r.bridge.reports("incident_ack")), 1)                # and the bridge heard it too

    def test_an_incident_ack_for_another_command_does_not_answer_it(self):
        r = self.rig()
        r.command({"seq": 9, "assistance_request": {"position": SINGAPORE, "vehicle_type": "TYPE_UAV"}})
        r.companion(rx.INCIDENT_ACK, rx.pack_incident_ack(cmd_seq=10))
        r.poll()
        self.assertEqual(r.status()["downlink"]["queue_depth"], 1)

    def test_a_command_for_another_domain_is_not_sent_to_this_vehicle(self):
        r = self.rig()
        r.command({"seq": 9, "assistance_request": {"position": SINGAPORE, "vehicle_type": "TYPE_USV"}})
        r.command({"seq": 10, "keep_out_zone": {"center": SINGAPORE, "radius_m": 5, "vehicle_type": "TYPE_UUV"}})
        r.command({"seq": 11, "moving_object_alert": {"position": SINGAPORE,
                                                       "affected_vehicle_types": ["TYPE_USV"]}})
        r.run(3)
        self.assertEqual(r.log, [])
        self.assertEqual(r.status()["downlink"]["not_for_this_vehicle"], 3)

    def test_an_unknown_domain_reaches_everyone(self):
        r = self.rig()
        r.command({"seq": 9, "assistance_request": {"position": SINGAPORE}})
        r.command({"seq": 10, "moving_object_alert": {"position": SINGAPORE}})
        r.step(0.1)
        self.assertEqual({x.name for x in r.log}, {"ASSISTANCE_REQUEST", "MOVING_OBJECT_ALERT"})

    # ---- KEEP_OUT_ZONE
    def zone(self, seq, radius=20, vt="TYPE_UAV"):
        return {"seq": seq, "keep_out_zone": {"center": SINGAPORE, "radius_m": radius, "vehicle_type": vt}}

    def test_a_zone_is_a_burst_of_three_then_every_five_seconds_for_its_life(self):
        r = self.rig(zone_ttl_s=20.0)
        t0 = r.clock.t
        r.command(self.zone(20))
        with self.assertLogs("fleet", "INFO"):
            r.run(25)
        self.assertEqual(r.times("KEEP_OUT_ZONE", t0), [0.0, 0.5, 1.0, 6.0, 11.0, 16.0])      # and none after the TTL
        self.assertEqual(r.status()["downlink"]["expired"], 1)
        self.assertEqual(r.of("KEEP_OUT_ZONE")[0].msg["radius_m"], 20.0)

    def test_an_ack_ends_only_the_burst_the_zone_goes_on_repeating(self):
        r = self.rig()
        t0 = r.clock.t
        r.command(self.zone(20))
        r.step(0.5)
        self.ack(r, rx.KEEP_OUT_ZONE, r.of("KEEP_OUT_ZONE")[-1])
        r.run(12)
        self.assertEqual(r.times("KEEP_OUT_ZONE", t0), [0.0, 0.5, 5.5, 10.5])

    def test_a_changed_zone_with_the_same_seq_supersedes_and_is_sent_at_once(self):
        r = self.rig()
        r.command(self.zone(20, radius=20))
        r.command(self.zone(20, radius=35))
        self.assertEqual([x.msg["radius_m"] for x in r.of("KEEP_OUT_ZONE")], [20.0, 35.0])
        self.assertEqual(r.status()["downlink"]["superseded"], 1)
        self.assertEqual(r.status()["downlink"]["queue_depth"], 1)

    def test_at_most_eight_zones_the_ninth_is_refused_loudly(self):
        r = self.rig()
        for seq in range(1, 9):
            r.command(self.zone(seq))
        self.assertEqual(r.status()["downlink"]["queue_depth"], 8)
        with self.assertLogs("fleet", "WARNING") as logs:
            r.command(self.zone(9))
        self.assertIn("KEEP_OUT_ZONE NOT sent", logs.output[0])
        self.assertIn("8 keep-out zones", logs.output[0])
        self.assertNotIn(9, [x.msg["cmd_seq"] for x in r.of("KEEP_OUT_ZONE")])
        self.assertEqual(r.status()["downlink"]["queue_depth"], 8)
        self.assertEqual(sum(r.status()["downlink"]["refusals_by_reason"].values()), 1)
        r.command(self.zone(3, radius=99))                                         # an existing zone may still change
        self.assertEqual(r.of("KEEP_OUT_ZONE")[-1].msg["radius_m"], 99.0)

    # ---- ALL_CLEAR
    def test_all_clear_three_at_2_hz_then_every_five_seconds_until_acked(self):
        r = self.rig()
        t0 = r.clock.t
        r.command({"seq": 30, "all_clear": {"vehicle_type": "TYPE_UAV"}})
        r.run(12)
        self.assertEqual(r.times("ALL_CLEAR", t0), [0.0, 0.5, 1.0, 6.0, 11.0])
        self.ack(r, rx.ALL_CLEAR, r.of("ALL_CLEAR")[-1])
        r.run(10)
        self.assertEqual(len(r.of("ALL_CLEAR")), 5)

    def test_all_clear_ends_the_zones_and_alerts_for_that_type(self):
        r = self.rig()
        r.command(self.zone(1))
        r.command(self.zone(2, vt="TYPE_UNKNOWN"))                                 # every vehicle
        r.command({"seq": 3, "moving_object_alert": {"position": SINGAPORE, "affected_vehicle_types": ["TYPE_UAV"]}})
        r.command({"seq": 4, "all_clear": {"vehicle_type": "TYPE_UAV"}})
        left = [p["msg"] for p in r.status()["downlink"]["queue"]]
        self.assertEqual(left, ["ALL_CLEAR"])
        self.assertEqual(r.status()["downlink"]["cancelled"], 3)

    def test_all_clear_with_the_vehicle_type_unknown_ends_the_uav_zones_too(self):
        """TYPE_UNKNOWN means every type -- on the ALL_CLEAR as much as on the zone."""
        r = self.rig()
        r.command(self.zone(1))
        r.command({"seq": 3, "moving_object_alert": {"position": SINGAPORE, "affected_vehicle_types": ["TYPE_UAV"]}})
        r.command({"seq": 4, "all_clear": {}})
        self.assertEqual([p["msg"] for p in r.status()["downlink"]["queue"]], ["ALL_CLEAR"])
        self.assertEqual(r.status()["downlink"]["cancelled"], 2)

    def test_all_clear_for_another_type_neither_reaches_nor_ends_anything_here(self):
        r = self.rig()
        r.command(self.zone(1))
        r.command({"seq": 4, "all_clear": {"vehicle_type": "TYPE_USV"}})
        self.assertEqual([p["msg"] for p in r.status()["downlink"]["queue"]], ["KEEP_OUT_ZONE"])
        self.assertEqual(r.status()["downlink"]["not_for_this_vehicle"], 1)

    # ---- MOVING_OBJECT_ALERT
    def alert(self, heading=90.0):
        return {"seq": 40, "moving_object_alert": {"position": SINGAPORE, "heading_deg": heading, "speed_mps": 2.0,
                                                    "affected_vehicle_types": ["TYPE_UAV"]}}

    def test_a_moving_object_is_repeated_at_1_hz_while_active_then_forgotten(self):
        r = self.rig(moving_ttl_s=5.0)
        t0 = r.clock.t
        r.command(self.alert())
        r.run(10)
        self.assertEqual(r.times("MOVING_OBJECT_ALERT", t0), [0.0, 1.0, 2.0, 3.0, 4.0])
        self.assertEqual(r.status()["downlink"]["expired"], 1)

    def test_the_same_alert_again_keeps_it_alive(self):
        r = self.rig(moving_ttl_s=5.0)
        t0 = r.clock.t
        r.command(self.alert())
        r.run(3)
        r.command(self.alert())
        r.run(10)
        self.assertEqual(r.times("MOVING_OBJECT_ALERT", t0), [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
        self.assertEqual(r.status()["downlink"]["duplicates_from_bridge"], 1)

    def test_a_changed_alert_replaces_the_old_one_and_stays_at_1_hz(self):
        r = self.rig()
        t0 = r.clock.t
        r.command(self.alert(heading=90.0))
        r.run(2.5)
        r.command(self.alert(heading=180.0))
        r.run(2.5)
        self.assertEqual([x.msg["heading_deg"] for x in r.of("MOVING_OBJECT_ALERT")],
                         [90.0, 90.0, 90.0, 180.0, 180.0, 180.0])
        times = r.times("MOVING_OBJECT_ALERT", t0)
        self.assertEqual(times, [0.0, 1.0, 2.0, 3.0, 4.0, 5.0])
        self.assertTrue(all(b - a >= 1.0 - 1e-9 for a, b in zip(times, times[1:])), times)
        self.assertEqual(r.status()["downlink"]["superseded"], 1)

    # ---- COURSE and GEOFENCE
    def test_the_course_goes_on_receipt_then_every_ten_seconds(self):
        r = self.rig()
        t0 = r.clock.t
        r.info(course=COURSE)
        r.run(25)
        self.assertEqual(r.times("COURSE", t0), [0.0, 10.0, 20.0])
        self.assertEqual(r.of("COURSE")[0].msg["course_epoch"], rx.course_epoch("ALPHA"))
        self.assertEqual(r.status()["downlink"]["course"]["corners"], 5)

    def test_the_same_course_again_is_a_duplicate_a_changed_one_supersedes(self):
        r = self.rig()
        r.info(course=COURSE)
        r.info(course=COURSE)
        self.assertEqual(r.status()["downlink"]["duplicates_from_bridge"], 1)
        r.info(course=dict(COURSE, course_id="BRAVO"))
        self.assertEqual(r.status()["downlink"]["superseded"], 1)
        self.assertEqual([x.msg["course_epoch"] for x in r.of("COURSE")],
                         [rx.course_epoch("ALPHA"), rx.course_epoch("BRAVO")])

    def test_a_null_course_withdraws_it(self):
        r = self.rig()
        r.info(course=COURSE)
        r.info(course=None)
        r.run(30)
        self.assertEqual(len(r.of("COURSE")), 1)
        self.assertFalse(r.status()["downlink"]["course"]["known"])

    def test_a_long_fence_goes_as_parts_each_inside_a_tunnel(self):
        r = self.rig()
        t0 = r.clock.t
        r.info(uav_geofence=ring(34))
        r.run(12)
        parts = r.of("GEOFENCE_INFO")
        self.assertEqual(r.times("GEOFENCE_INFO", t0)[:3], [0.0, 0.25, 0.5])              # staggered by 0.25 s
        self.assertEqual([p.msg["part"] for p in parts[:3]], [0, 1, 2])
        self.assertEqual({p.msg["n_parts"] for p in parts}, {3})
        for f in frames_of([d for d, _ in r.rfd.sent]):
            if f.msgid == MSG_TUNNEL:
                self.assertLessEqual(parse_tunnel(f).payload_length, 128)
        self.assertEqual(len(parts), 6)                                                    # twice: 0 s and 10 s
        self.assertEqual(r.status()["downlink"]["geofence"]["parts"], 3)

    def test_the_same_fence_is_a_duplicate_a_changed_one_replaces_every_part_and_none_withdraws_it(self):
        r = self.rig()
        r.info(uav_geofence=ring(34))
        r.info(uav_geofence=ring(34))
        self.assertEqual(r.status()["downlink"]["duplicates_from_bridge"], 1)
        r.info(uav_geofence=ring(20))
        st = r.status()["downlink"]
        self.assertEqual((st["superseded"], st["geofence"]["parts"]), (3, 2))
        r.info(uav_geofence=None)
        self.assertFalse(r.status()["downlink"]["geofence"]["known"])

    def test_the_camel_case_fence_key_works(self):
        r = self.rig()
        r.info(uavGeofence=CORNERS)
        self.assertEqual(len(r.of("GEOFENCE_INFO")), 1)

    # ---- a bad course must not cost the geofence, and the other way round
    def test_a_bad_course_does_not_block_the_fence_in_either_key_order(self):
        bad_course = dict(COURSE, corners=ring(16))                    # 16 corners: the message holds 15
        for order in (("course", "uav_geofence"), ("uav_geofence", "course")):
            r = self.rig()
            info = {"course": bad_course, "uav_geofence": CORNERS}
            with self.assertLogs("fleet", "WARNING") as logs:
                r.command({"ocs_info": {k: info[k] for k in order}})
            self.assertEqual(len(r.of("GEOFENCE_INFO")), 1, order)
            self.assertEqual(r.of("COURSE"), [], order)
            self.assertIn("COURSE NOT sent", logs.output[0])
            self.assertIn("16 corners", logs.output[0])
            self.assertEqual(r.status()["downlink"]["encode_errors"], 1)

    def test_a_bad_fence_does_not_block_the_course_in_either_key_order(self):
        bad_fence = ring(2)
        for order in (("course", "uav_geofence"), ("uav_geofence", "course")):
            r = self.rig()
            info = {"course": COURSE, "uav_geofence": bad_fence}
            with self.assertLogs("fleet", "WARNING") as logs:
                r.command({"ocs_info": {k: info[k] for k in order}})
            self.assertEqual(len(r.of("COURSE")), 1, order)
            self.assertEqual(r.of("GEOFENCE_INFO"), [], order)
            self.assertIn("GEOFENCE_INFO NOT sent", logs.output[0])
            self.assertEqual(r.status()["downlink"]["encode_errors"], 1)

    def test_a_refused_fence_leaves_the_old_one_in_force(self):
        r = self.rig()
        r.info(uav_geofence=CORNERS)
        with self.assertLogs("fleet", "WARNING"):
            r.info(uav_geofence=ring(2))
        self.assertTrue(r.status()["downlink"]["geofence"]["known"])

    def test_both_bad_is_two_refusals(self):
        r = self.rig()
        with self.assertLogs("fleet", "WARNING"):
            r.info(course="nope", uav_geofence=ring(2))
        self.assertEqual(r.status()["downlink"]["encode_errors"], 2)

    # ---- refusals are loud
    def test_a_run_id_the_wire_cannot_hold_is_refused_loudly_and_never_truncated(self):
        r = self.rig()
        with self.assertLogs("fleet", "WARNING") as logs:
            r.command({"seq": 1, "run_start": {"run_id": 70000}})
        line = logs.output[0]
        self.assertIn("RUN_START", line)
        self.assertIn("run_id", line)
        self.assertIn("70000", line)
        self.assertEqual(r.log, [])                                  # not even as 4464
        st = r.status()["downlink"]
        self.assertEqual(st["refusals_by_reason"], {"RUN_START.run_id does not fit u16": 1})
        self.assertEqual((st["encode_errors"], st["queue_depth"]), (1, 0))
        self.assertIn("70000", st["last_encode_error"])

    def test_the_counter_groups_by_reason_not_by_value(self):
        r = self.rig()
        with self.assertLogs("fleet", "WARNING"):
            for bad in (70000, 80000, 90000):
                r.command({"seq": 1, "run_start": {"run_id": bad}})
            r.command({"seq": 70000, "all_clear": {"vehicle_type": "TYPE_UAV"}})
        self.assertEqual(r.status()["downlink"]["refusals_by_reason"],
                         {"RUN_START.run_id does not fit u16": 3, "ALL_CLEAR.cmd_seq does not fit u16": 1})

    def test_the_number_of_distinct_reasons_is_bounded(self):
        r = self.rig()
        with self.assertLogs("fleet", "WARNING"):
            for i in range(oa.MAX_REFUSAL_REASONS + 10):
                r.v._refuse("TEST", ValueError("reason %s" % ("x" * i)))
        self.assertLessEqual(len(r.status()["downlink"]["refusals_by_reason"]), oa.MAX_REFUSAL_REASONS + 1)
        self.assertIn("(other reasons)", r.status()["downlink"]["refusals_by_reason"])

    def test_directives_have_no_rxl_message_and_are_counted(self):
        r = self.rig()
        r.command({"ocs_directive": {"action": "hold"}})
        r.command({"ocs_directive": {"action": "hold"}})
        self.assertEqual(r.status()["downlink"]["directives_ignored"], {"hold": 2})
        self.assertEqual(r.log, [])

    def test_unrecognised_frames_are_counted(self):
        r = self.rig()
        with self.assertLogs("fleet", "WARNING"):
            r.command({"mystery": 1})
            r.command([1, 2])
        self.assertEqual(r.status()["downlink"]["unknown_frames"], 2)

    # ---- the radio
    def test_nothing_is_keyed_into_an_empty_radio_and_it_goes_when_the_aircraft_is_heard(self):
        r = Rig()                                                    # nobody heard yet
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        r.run(2)
        self.assertEqual(r.log, [])
        self.assertGreaterEqual(r.status()["downlink"]["tx_no_route"], 2)
        r.autopilot_heartbeat()
        r.run(1)
        self.assertEqual(r.times("RUN_START", T0)[0], 2.5)          # the first retry (every 0.5 s) after the route appeared
        self.assertEqual(r.status()["downlink"]["queue_depth"], 1)

    def test_a_link_that_refuses_the_write_is_retried_half_a_second_later(self):
        r = self.rig()
        r.rfd.accept = False
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        r.run(1.0)
        self.assertEqual(r.log, [])
        self.assertGreaterEqual(r.status()["downlink"]["tx_refused"], 2)
        r.rfd.accept = True
        r.run(1.0)
        self.assertEqual(r.of("RUN_START")[0].msg["run_id"], 7)

    def test_frames_go_from_sysid_250_to_the_companion_and_nobody_else(self):
        r = self.rig()
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        (f,) = [x for x in frames_of([d for d, _ in r.rfd.sent]) if x.msgid == MSG_TUNNEL]
        t = parse_tunnel(f)
        self.assertEqual((f.sysid, f.compid, t.target_system), (250, 190, 200))
        self.assertEqual(t.payload_type, 0x8000 | rx.RUN_START)

    def test_the_mavlink_sequence_is_one_counter_across_message_types(self):
        r = self.rig()
        r.command({"seq": 1, "run_start": {"run_id": 7}})
        r.info(course=COURSE)
        seqs = [f.seq for f in frames_of([d for d, _ in r.rfd.sent])]
        self.assertEqual(seqs, list(range(seqs[0], seqs[0] + len(seqs))))


class Plumbing(unittest.TestCase):
    def test_status_is_plain_json_with_the_new_blocks(self):
        r = Rig()
        r.autopilot()
        r.step(0.0)
        st = r.status()
        self.assertIn("flight_state_request", st["heartbeat"])
        self.assertIn("refusals_by_reason", st["downlink"])
        self.assertEqual(r.adapter.status()["tx"]["frames"], r.adapter.sender.frames)

    def test_a_frame_that_raises_is_counted_and_the_adapter_carries_on(self):
        r = Rig()
        with mock.patch.object(r.v, "_command", side_effect=RuntimeError("bug")):
            with self.assertLogs("fleet", "ERROR"):
                r.command({"seq": 1, "run_start": {"run_id": 1}})
        self.assertEqual(r.status()["downlink"]["unknown_frames"], 1)
        r.command({"seq": 2, "run_start": {"run_id": 2}})
        self.assertEqual(r.status()["downlink"]["queue_depth"], 1)

    def test_the_hub_side_is_a_bounded_append(self):
        with mock.patch.object(oa, "INBOX_MAX", 4):
            r = Rig()
            for _ in range(10):
                r.autopilot_heartbeat()
            self.assertEqual(len(r.adapter.inbox), 4)
            self.assertGreaterEqual(r.adapter.inbox_overflow, 6)

    def test_frames_from_other_vehicles_are_not_this_adapters_business(self):
        r = Rig()
        r.hear(heartbeat(2, seq=r.seq()))
        r.hear(tunnel_frame(rx.payload_type(rx.DOCKING), rx.pack_docking(bay_id=1), sysid=42, target=250, seq=r.seq()))
        r.poll()
        self.assertEqual(len(r.adapter.inbox), 0)
        self.assertEqual(r.bridge.sent, [])



# ================================================================ the geofence, onto the autopilot

FENCE_MSGS = {"MAV%d" % m for m in (20, 23, 43, 44, 47, 51, 73)}     # PARAM_SET (23) must never appear at all


class FenceUploadOverTheRadio(unittest.TestCase):
    """fence_upload.py through the real hub: ocs_info from the bridge -> MISSION_* on the rfd link ->
    a fake ArduPilot answering on the same link (its own tests are test_fence_upload.py)."""

    def rig(self, on=True, autopilot=None, **flags):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        r = Rig(dict(fence_upload=True, **flags) if on else None, fence_backup_dir=tmp.name)
        return r, autopilot or FakeAutopilot()

    def fly(self, r, a, seconds, *, armed=False):
        """An autopilot on the radio -- HEARTBEAT and position every 0.1 s, at the Sarasota
        field, far outside the Singapore fence -- whose answers to the adapter come back on rfd."""
        seen = len(r.log)
        t_end = r.clock.t + seconds
        while r.clock.t < t_end - 1e-9:
            r.clock.advance(0.1)
            r.hear(heartbeat(1, base_mode=129 if armed else 1, seq=r.seq()))
            r.hear(position(1, lat=273364000, lon=-825307000, seq=r.seq()))
            r.poll()
            for x in r.log[seen:]:
                if x.name.startswith("MAV"):
                    a.hear(x.msg.raw, r.clock.t)
            seen = len(r.log)
            a.tick(r.clock.t)
            for raw in a.take():
                r.hear(raw)

    def test_off_by_default_nothing_is_said_to_the_autopilot(self):
        r, a = self.rig(on=False)
        r.info(uav_geofence=CORNERS + [CORNERS[0]])
        self.fly(r, a, 10)
        self.assertEqual({x.name for x in r.log} & FENCE_MSGS, set())
        self.assertEqual(r.status()["fence_upload"], {"enabled": False})
        self.assertTrue(r.of("GEOFENCE_INFO"))                       # the RXL relay is unchanged

    def test_on_the_declared_fence_is_written_to_the_autopilot_over_the_radio(self):
        r, a = self.rig()
        r.info(uav_geofence=CORNERS + [CORNERS[0]])
        self.fly(r, a, 10)
        st = r.status()["fence_upload"]
        self.assertEqual(st["state"], "done", st)
        self.assertEqual([(round(i["lat"], 6), round(i["lon"], 6), i["param1"]) for i in a.fence],
                         [(c["latitude"], c["longitude"], 4.0) for c in CORNERS])
        mission = [x.msg for x in r.log if x.name in FENCE_MSGS]
        self.assertTrue(mission)
        self.assertEqual({(f.sysid, f.compid) for f in mission}, {(250, 190)})
        self.assertNotIn("MAV23", {x.name for x in r.log})
        self.assertTrue(r.of("GEOFENCE_INFO"))                       # and Ekko's companion still gets its copy

    def test_an_armed_aircraft_is_told_nothing(self):
        r, a = self.rig()
        r.info(uav_geofence=CORNERS + [CORNERS[0]])
        self.fly(r, a, 10, armed=True)
        self.assertEqual({x.name for x in r.log} & FENCE_MSGS, set())
        self.assertIn("armed", r.status()["fence_upload"]["reason"])

    def test_a_fence_that_is_not_a_polygon_is_refused_loudly(self):
        r, a = self.rig()
        with self.assertLogs("fleet", "WARNING") as logs:
            r.info(uav_geofence=[CORNERS[0], CORNERS[1], CORNERS[0]])
        self.assertTrue(any("FENCE_UPLOAD NOT sent" in line for line in logs.output))
        self.fly(r, a, 5)
        self.assertEqual(r.status()["fence_upload"]["state"], "off")
        self.assertEqual({x.name for x in r.log} & FENCE_MSGS, set())

    # ---- no course -> cleared (fence_clear_without_course); a refused course fence -> left alone

    def held_fence(self):
        return FakeAutopilot(fence=[{"command": 5001, "param1": 4.0, "lat": c["latitude"], "lon": c["longitude"]}
                                    for c in CORNERS])

    def test_no_course_clears_the_autopilots_fence_after_the_grace(self):
        r, a = self.rig(autopilot=self.held_fence(), fence_clear_without_course=True)
        r.info(course=None, uav_geofence=None)
        self.fly(r, a, 15)
        self.assertEqual(len(a.fence), 4)                            # the grace: a course may still come
        self.fly(r, a, 15)
        st = r.status()["fence_upload"]
        self.assertEqual((st["state"], a.fence, st["clearing"]), ("done", [], True))
        self.assertIsNotNone(st["backup"])

    def test_without_the_switch_no_course_leaves_it_alone(self):
        r, a = self.rig(autopilot=self.held_fence())
        r.info(course=None, uav_geofence=None)
        self.fly(r, a, 30)
        self.assertEqual(len(a.fence), 4)
        self.assertEqual({x.name for x in r.log} & FENCE_MSGS, set())

    def test_a_course_whose_fence_was_refused_leaves_the_old_fence(self):
        r, a = self.rig(autopilot=self.held_fence(), fence_clear_without_course=True)
        r.info(course=COURSE, uav_geofence=None)
        self.fly(r, a, 30)
        self.assertEqual(len(a.fence), 4)
        self.assertEqual({x.name for x in r.log} & FENCE_MSGS, set())

    def test_the_bridge_is_told_what_the_autopilot_was_confirmed_to_hold(self):
        r, a = self.rig()
        r.info(uav_geofence=CORNERS + [CORNERS[0]])
        self.fly(r, a, 10)
        last = r.bridge.reports("ocs_fence")[-1]["ocs_fence"]
        self.assertEqual((last["vehicle_id"], last["state"], last["verified"]), ("UAV1", "done", True))
        self.assertEqual(last["polygon"], [[c["latitude"], c["longitude"]] for c in CORNERS])
        self.assertLess(abs(parse_utc(last["verified_at"]) - (WALL0 + r.clock.t - T0)), 10)
        n = len(r.bridge.reports("ocs_fence"))
        self.fly(r, a, 5)
        self.assertEqual(len(r.bridge.reports("ocs_fence")), n)     # nothing changed: nothing re-sent
        self.fly(r, a, 60)
        self.assertGreater(len(r.bridge.reports("ocs_fence")), n)   # the re-check refreshed verified_at


if __name__ == "__main__":
    unittest.main()
