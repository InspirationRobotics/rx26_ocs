"""rcfind: finding RoboCommand's broker by its course topic (`[robocommand] host = "auto"`).

Pure parts first (the routing tables, the order hosts are tried in), then a round with the
network injected, then real probes against an in-process broker holding a retained course,
then the search thread driven by hand, then the bridge.
"""
from __future__ import annotations

import asyncio
import socket
import sys
import threading
import time
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import paho.mqtt.client as mqtt                                  # noqa: E402

from rx_bridge import rcfind                                     # noqa: E402
from rx_bridge.bridge import Bridge                              # noqa: E402
from rx_bridge.config import ConfigError, load                   # noqa: E402
from rx_bridge.proto import rx_course_pb2                         # noqa: E402

from tests.harness import CONFIG, TEAM, BridgeCase               # noqa: E402

TOPIC = "robocommand/robotx/course"

ROUTE_PRINT = """\
===========================================================================
Interface List
 12...00 e0 4c 68 01 23 ......Realtek USB GbE Family Controller
  7...a4 b1 c1 22 33 44 ......Intel(R) Wi-Fi 6 AX201 160MHz
===========================================================================

IPv4 Route Table
===========================================================================
Active Routes:
Network Destination        Netmask          Gateway       Interface  Metric
          0.0.0.0          0.0.0.0      192.168.1.1     192.168.1.75     35
          0.0.0.0          0.0.0.0     192.168.69.1    192.168.69.20     25
        127.0.0.0        255.0.0.0         On-link         127.0.0.1    331
      192.168.1.0    255.255.255.0         On-link      192.168.1.75    291
===========================================================================
Persistent Routes:
  Network Address          Netmask  Gateway Address  Metric
          0.0.0.0          0.0.0.0      192.168.1.1  Default
===========================================================================
"""


class TestPure(unittest.TestCase):

    def test_route_print_gives_each_active_default_route_with_its_interface(self):
        self.assertEqual(rcfind.parse_route_print(ROUTE_PRINT),
                         [("192.168.1.1", "192.168.1.75"), ("192.168.69.1", "192.168.69.20")])

    def test_ip_route_gives_the_source_when_there_is_one(self):
        text = ("default via 192.168.1.1 dev wlp2s0 proto dhcp src 192.168.1.20 metric 600\n"
                "default via 10.0.0.1 dev eth0\n")
        self.assertEqual(rcfind.parse_ip_route(text),
                         [("192.168.1.1", "192.168.1.20"), ("10.0.0.1", "")])

    def test_only_addresses_that_can_be_on_a_course_network(self):
        self.assertEqual(rcfind.usable(["127.0.0.1", "169.254.3.4", "0.0.0.0", "224.0.0.1",
                                        "fe80::1", "host", "192.168.69.20", "192.168.69.20 ",
                                        "10.1.2.3"]),
                         ["192.168.69.20", "10.1.2.3"])

    def test_the_order_hosts_are_tried_in(self):
        got = rcfind.candidates("192.168.65.2",
                                [("192.168.69.1", "192.168.69.20"), ("192.168.1.1", "")],
                                ["192.168.69.20", "192.168.1.75"])
        self.assertEqual(got, [
            ("192.168.65.2", "where RoboCommand was last found"),
            ("192.168.69.1", "the default gateway of 192.168.69.20"),
            ("192.168.1.1", "the default gateway"),
            ("192.168.69.2", ".2 of 192.168.69.0/24"),
            ("192.168.1.2", ".2 of 192.168.1.0/24"),
            ("192.168.69.3", ".3 of 192.168.69.0/24"),
            ("192.168.1.3", ".3 of 192.168.1.0/24"),
        ])

    def test_each_host_once_and_never_this_laptop(self):
        got = rcfind.candidates("192.168.69.3", [("192.168.69.3", "192.168.69.2")],
                                ["192.168.69.2"])
        self.assertEqual([h for h, _ in got], ["192.168.69.3"],
                         "the gateway was the last host, and .2 is the laptop itself")
        self.assertEqual(rcfind.candidates(None, [], []), [])


def fake_probe(robocommand: dict):
    """A probe that finds RoboCommand at the hosts in `robocommand` (host -> course id) and
    a plain broker anywhere else it is asked."""
    asked: list[str] = []

    def probe(host, port, topic, client_id):
        asked.append(host)
        if host in robocommand:
            return rcfind.Probe(host, True, "RoboCommand", robocommand[host], "192.168.69.20")
        return rcfind.Probe(host, False, "an MQTT broker with nothing on %s" % topic)

    probe.asked = asked
    return probe


class TestRound(unittest.TestCase):
    LOCAL = ["192.168.69.20", "192.168.1.75"]
    GATEWAYS = [("192.168.69.1", "192.168.69.20")]

    def run_round(self, *, answering, robocommand, sweep=(), last=None, stop=None):
        self.probe = fake_probe(robocommand)
        self.answer_calls: list[list[str]] = []

        def answer(hosts, port):
            self.answer_calls.append(list(hosts))
            return [h for h in hosts if h in answering]

        return rcfind.search_round(self.LOCAL, 1883, TOPIC, "ocs-ASTA-find-x", last=last,
                                   sweep=sweep, gateways=lambda: self.GATEWAYS,
                                   probe_host=self.probe, answer=answer, stop=stop)

    def test_the_gateway_first_and_no_sweep_once_found(self):
        nets = rcfind.networks(self.LOCAL)
        rnd = self.run_round(answering={"192.168.69.1"}, robocommand={"192.168.69.1": "Alpha"},
                             sweep=nets)
        self.assertEqual(rnd.found, rcfind.Found("192.168.69.1", 1883, "192.168.69.20", "Alpha",
                                                 "the default gateway of 192.168.69.20"))
        self.assertEqual(rnd.swept, [])
        self.assertEqual(len(self.answer_calls), 1, "the sweep never ran")

    def test_a_host_that_does_not_answer_is_not_probed_but_is_listed(self):
        rnd = self.run_round(answering=set(), robocommand={})
        self.assertIsNone(rnd.found)
        self.assertEqual(self.probe.asked, [])
        self.assertEqual(rnd.tried[0], {"host": "192.168.69.1",
                                        "why": "the default gateway of 192.168.69.20",
                                        "verdict": "no answer on TCP 1883"})

    def test_found_in_the_sweep_skipping_this_laptop_and_what_was_tried(self):
        nets = rcfind.networks(self.LOCAL)
        rnd = self.run_round(answering={"192.168.69.2", "192.168.69.57"},
                             robocommand={"192.168.69.57": "Bravo"}, sweep=nets)
        self.assertEqual(self.probe.asked, ["192.168.69.2", "192.168.69.57"],
                         "the .2 answered but is a plain broker; the sweep found .57")
        self.assertEqual(rnd.found.host, "192.168.69.57")
        self.assertEqual(rnd.found.why, "answered on TCP 1883 in the sweep of 192.168.69.0/24")
        swept = self.answer_calls[1]
        self.assertEqual(len(swept), 2 * 254 - 2 - 5,
                         "two /24s, less the laptop's two addresses and the five tried first")
        self.assertNotIn("192.168.69.20", swept)
        self.assertNotIn("192.168.69.1", swept)
        self.assertEqual(rnd.swept, ["192.168.69.0/24", "192.168.1.0/24"])

    def test_nothing_anywhere(self):
        rnd = self.run_round(answering={"192.168.1.50"}, robocommand={},
                             sweep=rcfind.networks(self.LOCAL))
        self.assertIsNone(rnd.found)
        self.assertEqual(rnd.tried[-1]["host"], "192.168.1.50")
        self.assertIn("nothing on " + TOPIC, rnd.tried[-1]["verdict"])

    def test_the_host_found_last_time_is_tried_first(self):
        rnd = self.run_round(answering={"10.9.9.9"}, robocommand={"10.9.9.9": "Charlie"},
                             last="10.9.9.9")
        self.assertEqual(rnd.found.why, "where RoboCommand was last found")

    def test_a_stopped_round_ends_without_probing(self):
        stop = threading.Event()
        stop.set()
        rnd = self.run_round(answering={"192.168.69.1"}, robocommand={"192.168.69.1": "A"},
                             stop=stop)
        self.assertIsNone(rnd.found)
        self.assertEqual(self.probe.asked, [])


# ---- real probes, against an in-process broker ------------------------------------------

def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_until(cond, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.05)
    return cond()


class Broker:
    """amqtt in a thread of its own, as the bench runs it, holding one retained RxCourse on
    the course topic: a stand-in for RoboCommand's broker."""

    def __init__(self, host: str, port: int, course_id: str) -> None:
        from amqtt.broker import Broker as Amqtt
        cfg = {"listeners": {"default": {"type": "tcp", "bind": "%s:%d" % (host, port)}},
               "plugins": {"amqtt.plugins.authentication.AnonymousAuthPlugin":
                           {"allow_anonymous": True}}}
        ready, self.loop = threading.Event(), None

        def run():
            async def main():
                broker = Amqtt(cfg)
                await broker.start()
                self.loop, self.done = asyncio.get_running_loop(), asyncio.Event()
                ready.set()
                await self.done.wait()
                await broker.shutdown()
            asyncio.run(main())

        self.thread = threading.Thread(target=run, daemon=True)
        self.thread.start()
        if not ready.wait(20):
            raise unittest.SkipTest("amqtt never started")
        pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="test-rc")
        pub.connect(host, port, 15)
        pub.loop_start()
        pub.publish(TOPIC, rx_course_pb2.RxCourse(course_id=course_id).SerializeToString(),
                    qos=1, retain=True).wait_for_publish(5)
        pub.disconnect()
        pub.loop_stop()

    def stop(self) -> None:
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self.done.set)
            self.thread.join(5)
            self.loop = None


class TestProbe(unittest.TestCase):
    """Real probes against a broker holding a retained course."""

    @classmethod
    def setUpClass(cls):
        cls.port = _free_port()
        cls.broker = Broker("127.0.0.1", cls.port, "Alpha")

    @classmethod
    def tearDownClass(cls):
        cls.broker.stop()

    def probe(self, topic=TOPIC, port=None, wait_s=1.0):
        return rcfind.probe("127.0.0.1", port or self.port, topic, "ocs-TEST-find-1",
                            connect_s=1.0, wait_s=wait_s)

    def test_the_retained_course_is_robocommand(self):
        p = self.probe()
        self.assertTrue(p.is_robocommand, p.verdict)
        self.assertEqual((p.course_id, p.via), ("Alpha", "127.0.0.1"))
        self.assertEqual(p.verdict, "RoboCommand: course Alpha on " + TOPIC)

    def test_a_broker_with_nothing_on_the_topic_is_not(self):
        t0 = time.monotonic()
        p = self.probe(topic="nobody/robotx/course", wait_s=0.5)
        self.assertFalse(p.is_robocommand)
        self.assertIn("an MQTT broker with nothing on nobody/robotx/course", p.verdict)
        self.assertLess(time.monotonic() - t0, 3.0)

    def test_nothing_listening(self):
        p = self.probe(port=_free_port())
        self.assertFalse(p.is_robocommand)
        # Windows retries a refused loopback connect for about 2 s, past the 1 s allowed here.
        self.assertIn(p.verdict, ("no MQTT connection: refused", "no MQTT connection: timed out"))

    def test_something_that_is_not_mqtt(self):
        srv = socket.socket()
        srv.bind(("127.0.0.1", 0))
        srv.listen(1)
        self.addCleanup(srv.close)

        def hang_up():
            conn, _ = srv.accept()
            conn.close()

        threading.Thread(target=hang_up, daemon=True).start()
        p = self.probe(port=srv.getsockname()[1])
        self.assertFalse(p.is_robocommand)
        self.assertIn("closed it without an MQTT answer", p.verdict)

    def test_a_whole_round_finds_it_on_loopback_by_the_last_host(self):
        rnd = rcfind.search_round([], self.port, TOPIC, "ocs-TEST-find-2", last="127.0.0.1",
                                  gateways=lambda: [])
        # 127.0.0.1 is not usable as a course address, so it is never a candidate: a round
        # from nowhere finds nothing. The bridge never searches loopback.
        self.assertIsNone(rnd.found)
        self.assertEqual(rnd.tried, [])


# ---- the search thread, by hand -----------------------------------------------------------

class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class TestFinder(unittest.TestCase):

    def setUp(self):
        self.clock, self.logs, self.found, self.rounds = Clock(), [], [], []
        self.addrs = ["192.168.69.20", "192.168.1.75"]
        self.robocommand: dict = {}
        self.paused = True
        self.searches: list[dict] = []

        def search(local, port, topic, client_id, *, last, sweep, stop):
            self.searches.append({"local": local, "last": last, "client_id": client_id,
                                  "sweep": [str(n) for n in sweep]})
            rnd = rcfind.Round(at=time.time(), addresses=list(local),
                               swept=[str(n) for n in sweep])
            for host, cid in self.robocommand.items():
                rnd.found = rcfind.Found(host, port, "192.168.69.20", cid, "the default gateway")
            return rnd

        self.f = rcfind.Finder(1883, TOPIC, "ASTA", pause=lambda: self.paused,
                               on_found=self.found.append, on_round=self.rounds.append,
                               log=self.logs.append, addresses=lambda: list(self.addrs),
                               search=search, clock=self.clock)

    def later(self, s):
        self.clock.t += s

    def test_the_first_tick_searches_and_sweeps_every_network(self):
        rnd = self.f.tick()
        self.assertIsNotNone(rnd)
        self.assertEqual(self.searches[0]["sweep"], ["192.168.69.0/24", "192.168.1.0/24"])
        self.assertTrue(self.searches[0]["client_id"].startswith("ocs-ASTA-find-"))
        self.assertNotEqual(self.searches[0]["client_id"], "ocs-ASTA")
        self.assertEqual(self.found, [])
        self.assertEqual(len(self.rounds), 1)
        self.assertTrue(any("RoboCommand search: not found" in m for m in self.logs))

    def test_rounds_every_ROUND_S_and_sweeps_only_every_SWEEP_S(self):
        self.f.tick()
        self.later(rcfind.ROUND_S - 1)
        self.assertIsNone(self.f.tick(), "not due yet")
        self.later(1)
        self.assertIsNotNone(self.f.tick())
        self.assertEqual(self.searches[1]["sweep"], [], "swept a moment ago")
        self.later(rcfind.SWEEP_S)
        self.f.tick()
        self.assertEqual(len(self.searches[2]["sweep"]), 2)

    def test_sweeps_back_off_to_SWEEP_MAX_S(self):
        swept_at = []
        for _ in range(int(3 * rcfind.SWEEP_MAX_S / rcfind.ROUND_S)):
            self.later(rcfind.ROUND_S)
            self.f.tick()
            if self.searches[-1]["sweep"]:
                swept_at.append(self.clock.t)
        gaps = [b - a for a, b in zip(swept_at, swept_at[1:])]
        self.assertEqual(gaps[:4], [60.0, 120.0, 240.0, 480.0])
        self.assertTrue(all(g == rcfind.SWEEP_MAX_S for g in gaps[4:]), gaps)

    def test_a_network_that_comes_back_is_swept_at_once(self):
        self.f.tick()
        self.addrs.remove("192.168.1.75")
        self.later(1)
        self.f.tick()
        self.assertIn("192.168.1.0/24 not yet searched", self.f.hold_reason("192.168.1.80"))
        self.addrs.append("192.168.1.80")                 # re-plugged, a new lease
        self.later(1)
        self.f.tick()
        self.assertEqual(self.searches[-1]["sweep"], ["192.168.1.0/24"])

    def test_a_new_address_is_searched_at_once_and_its_network_swept(self):
        self.f.tick()
        self.addrs.append("192.168.70.5")
        self.later(1)
        self.assertIsNotNone(self.f.tick())
        self.assertEqual(self.searches[1]["sweep"], ["192.168.70.0/24"])

    def test_the_same_nothing_is_logged_once(self):
        self.f.tick()
        n = len(self.logs)
        for _ in range(3):
            self.later(rcfind.ROUND_S)
            self.f.tick()
        self.assertEqual(len(self.logs), n)

    def test_the_beacon_is_held_off_networks_not_yet_swept_until_found(self):
        self.assertIn("not yet searched for RoboCommand", self.f.hold_reason("192.168.1.75"))
        self.f.tick()
        self.assertIsNone(self.f.hold_reason("192.168.1.75"))
        self.assertIn("192.168.70.0/24 not yet searched", self.f.hold_reason("192.168.70.5"))
        self.robocommand["192.168.69.1"] = "Alpha"
        self.later(rcfind.ROUND_S)
        self.f.tick()
        self.assertIsNone(self.f.hold_reason("192.168.70.5"),
                          "found: the beacon's own rules take over")

    def test_found_then_quiet_while_connected_then_redialled_then_searched_again(self):
        self.robocommand["192.168.69.1"] = "Alpha"
        self.f.tick()
        self.assertEqual([f.host for f in self.found], ["192.168.69.1"])
        self.assertTrue(any(m.startswith("RoboCommand found at 192.168.69.1:1883") for m in self.logs))
        self.assertEqual(self.f.snapshot()["state"], "dialling")
        self.f.link(True)
        self.later(600)
        self.assertIsNone(self.f.tick(), "connected: no search")
        self.assertEqual(self.f.snapshot()["state"], "connected")
        self.f.link(False)
        self.later(rcfind.REDISCOVER_S - 1)
        self.assertIsNone(self.f.tick(), "paho redials the broker it had first")
        self.assertEqual(self.f.snapshot()["state"], "dialling")
        self.later(1)
        self.assertIsNotNone(self.f.tick())
        self.assertEqual(self.searches[-1]["last"], "192.168.69.1")

    def test_a_round_is_skipped_when_the_client_connected_meanwhile(self):
        self.paused = False
        self.assertIsNone(self.f.tick())
        self.assertEqual(self.searches, [])
        self.assertEqual(self.f.snapshot()["state"], "waiting")

    def test_the_snapshot(self):
        s = self.f.snapshot()
        self.assertEqual((s["state"], s["host"], s["rounds"], s["last_round"]),
                         ("waiting", None, 0, None))
        self.f.tick()
        s = self.f.snapshot()
        self.assertEqual((s["rounds"], s["search_in_s"]), (1, rcfind.ROUND_S))
        self.assertEqual(s["searched"], ["192.168.1.0/24", "192.168.69.0/24"])
        self.assertEqual(s["last_round"]["swept"], ["192.168.69.0/24", "192.168.1.0/24"])


class TestBridgeFindsAndMoves(unittest.TestCase):
    """The real bridge and its real paho client. The round's network is stood in for (a
    search that answers with a loopback broker, which a real round never tries); the rest
    is real: the client is stopped before each round, dialled at what was found, takes the
    retained course; that broker goes away, and after REDISCOVER_S the client is moved to
    the one the next round finds."""

    def setUp(self):
        import tempfile
        self.port = _free_port()
        self.brokers = {"127.0.0.1": Broker("127.0.0.1", self.port, "Alpha"),
                        "127.0.0.2": Broker("127.0.0.2", self.port, "Bravo")}
        for b in self.brokers.values():
            self.addCleanup(b.stop)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        d = Path(tmp.name)
        text = CONFIG.format(team=TEAM, usv_extra="", uav_extra="", limits="", geofence="",
                             callsign="Test UA0000", rc_host="auto",
                             seq=(d / "seq.json").as_posix(), wire=(d / "wire").as_posix())
        rc = '[robocommand]\nhost = "auto"\nport = 1\n'
        self.assertEqual(text.count(rc), 1)
        (d / "bridge.toml").write_text(text.replace(rc, rc.replace("port = 1", "port = %d"
                                                                   % self.port)),
                                       encoding="utf-8")
        self.logs: list[str] = []
        self.bridge = Bridge(load(d / "bridge.toml"), log=lambda m: self.logs.append(str(m)))
        self.where = ["127.0.0.1"]

        def search(local, port, topic, client_id, *, last, sweep, stop):
            rnd = rcfind.Round(at=time.time(), addresses=list(local))
            rnd.found = rcfind.Found(self.where[0], port, self.where[0], "", "the stand-in round")
            return rnd

        finder = self.bridge.rc_find
        finder._addresses, finder._search = (lambda: ["192.0.2.10"]), search
        saved = rcfind.REDISCOVER_S
        rcfind.REDISCOVER_S = 1.0
        self.addCleanup(setattr, rcfind, "REDISCOVER_S", saved)
        self.bridge.start()
        self.addCleanup(self.bridge.stop)

    def course(self):
        return (self.bridge.snapshot().get("course") or {}).get("course_id")

    def test_found_connected_then_moved_when_that_broker_goes(self):
        self.assertTrue(wait_until(lambda: self.bridge.links["robocommand"]
                                   and self.course() == "Alpha", 15), self.logs)
        self.assertEqual(self.bridge.snapshot()["endpoints"]["robocommand"],
                         "127.0.0.1:%d (auto: found by its course topic)" % self.port)
        self.where[0] = "127.0.0.2"
        self.brokers["127.0.0.1"].stop()
        self.assertTrue(wait_until(lambda: self.course() == "Bravo", 30), self.logs)
        self.assertTrue(self.bridge.links["robocommand"])
        self.assertEqual(self.bridge.rc_find.snapshot()["state"], "connected")
        self.assertTrue(any(m.startswith("RoboCommand found at 127.0.0.2") for m in self.logs))


# ---- configuration and the bridge -----------------------------------------------------------

class TestConfig(unittest.TestCase):
    BASE = """
team_id = "ASTA"
[[vehicle]]
id = "USV1"
type = "TYPE_USV"
[team]
host = "{team}"
[robocommand]
{rc}
[tiers]
task1 = "TIER_CORE"
"""

    def load(self, rc, team="127.0.0.1"):
        import tempfile
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "bridge.toml"
            p.write_text(self.BASE.format(rc=rc, team=team), encoding="utf-8")
            return load(p)

    def test_auto_or_no_host_searches_and_an_address_pins(self):
        self.assertTrue(self.load('host = "auto"').robocommand.auto)
        self.assertTrue(self.load("port = 1883").robocommand.auto)
        cfg = self.load('host = "192.168.69.3"')
        self.assertFalse(cfg.robocommand.auto)
        self.assertEqual(cfg.robocommand.host, "192.168.69.3")

    def test_only_robocommand_is_searched_for(self):
        with self.assertRaisesRegex(ConfigError, "only RoboCommand"):
            self.load('host = "auto"', team="auto")

    def test_a_host_that_is_not_one(self):
        for bad in ('host = ""', "host = 5", 'host = "  "'):
            with self.subTest(bad), self.assertRaisesRegex(ConfigError, "address or \"auto\""):
                self.load(bad)

    def test_the_shipped_bridge_toml_searches(self):
        cfg = load(Path(__file__).resolve().parents[1] / "bridge.toml")
        self.assertTrue(cfg.robocommand.auto)
        self.assertEqual(cfg.robocommand.port, 1883)


class TestBridgeAuto(BridgeCase):
    rc_host = "auto"

    def found(self, host="192.168.69.3"):
        """A round that found RoboCommand, recorded the way the search thread does."""
        rnd = rcfind.Round(at=time.time(), addresses=["192.168.69.20"])
        rnd.found = rcfind.Found(host, 1, "192.168.69.20", "Alpha",
                                 "the default gateway of 192.168.69.20")
        self.bridge.rc_find._record(rnd)

    def test_it_says_it_is_searching(self):
        s = self.bridge.snapshot()
        self.assertEqual(s["endpoints"]["robocommand"],
                         "auto: searching for robocommand/robotx/course on TCP 1")
        self.assertEqual(s["rc_search"]["state"], "waiting")
        self.assertEqual(s["rc_search"]["topic"], TOPIC)

    def test_what_is_found_is_dialled_and_said(self):
        self.found()
        self.assertEqual(self.bridge.rc.calls, [("connect_async", "192.168.69.3", 1),
                                                ("loop_start",)])
        self.assertEqual(self.bridge.snapshot()["endpoints"]["robocommand"],
                         "192.168.69.3:1 (auto: found by its course topic)")
        events = [l for l in self.wire_lines() if l.get("kind") == "rc_found"]
        self.assertEqual(len(events), 1)
        self.assertIn("192.168.69.3:1 from 192.168.69.20, course Alpha", str(events[0]))

    def test_the_client_is_stopped_before_a_round_and_left_alone_if_connected(self):
        self.assertTrue(self.bridge._rc_pause())
        self.assertEqual(self.bridge.rc.calls, [("loop_stop",)])
        self.bridge.rc.connected = True
        self.assertFalse(self.bridge._rc_pause())
        self.assertEqual(self.bridge.rc.calls[-1], ("loop_start",))

    def test_the_link_tells_the_search(self):
        self.found()
        self.bridge._on_rc_connect(self.bridge.rc, None, None, 0)
        self.assertEqual(self.bridge.rc_find.snapshot()["state"], "connected")
        self.bridge._on_rc_disconnect(self.bridge.rc, None, None, 7)
        self.assertEqual(self.bridge.rc_find.snapshot()["state"], "dialling")


class TestBridgePinned(BridgeCase):

    def test_a_pinned_host_never_searches(self):
        self.assertIsNone(self.bridge.rc_find)
        s = self.bridge.snapshot()
        self.assertEqual(s["endpoints"]["robocommand"], "127.0.0.1:1")
        self.assertIsNone(s["rc_search"])


if __name__ == "__main__":
    unittest.main()
