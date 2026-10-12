"""The discovery beacon: the datagram, the interface filter, the config and the sender.

Nothing here touches a real network. The sender test runs the real socket path on
loopback to an ephemeral port (never 37565), and the Bridge wiring test replaces
the sender with a recorder, so a unit-test run cannot broadcast to a boat.

    .venv\\Scripts\\python.exe -m unittest tests.test_beacon
"""
from __future__ import annotations

import importlib.util
import os
import socket
import sys
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rx_bridge import beacon                                   # noqa: E402
from rx_bridge.bridge import Bridge                            # noqa: E402
from rx_bridge.config import ConfigError, load                 # noqa: E402

#: What this laptop really listed on 2026-10-10, with the course network up.
LAPTOP = ["169.254.83.107", "192.168.100.107", "192.168.11.1", "192.168.65.184"]


def wait_until(cond, timeout: float = 5.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.02)
    return cond()


class TestDatagram(unittest.TestCase):
    def test_build_then_parse_round_trips(self):
        got = beacon.parse(beacon.build("ASTA", 37564))
        self.assertEqual(got, {"team": "ASTA", "port": 37564, "v": 1})

    def test_the_wire_shape_is_exactly_the_documented_one(self):
        self.assertEqual(beacon.build("ASTA", 37564),
                         b'{"ocs_beacon":{"team":"ASTA","port":37564,"v":1}}')

    def test_build_refuses_what_it_could_not_advertise(self):
        for team, port in (("", 37564), ("  ", 37564), (None, 37564),
                           ("ASTA", 0), ("ASTA", 70000), ("ASTA", "37564"),
                           ("ASTA", True)):
            with self.subTest(team=team, port=port), self.assertRaises(ValueError):
                beacon.build(team, port)

    def test_parse_rejects_anything_that_is_not_exactly_a_v1_beacon(self):
        good = {"team": "ASTA", "port": 37564, "v": 1}
        bad = [
            b"", b"not json", b"\xff\xfe", b"[]", b'"x"', b"{}",
            b'{"ocs_beacon": 5}', b'{"ocs_beacon": {}}',
            b'{"ocs_hello": {"team":"ASTA","port":1,"v":1}}',
            b'{"ocs_beacon": {"team":"ASTA","port":37564,"v":1}, "extra": 1}',
            b'{"ocs_beacon": {"team":"","port":37564,"v":1}}',
            b'{"ocs_beacon": {"team":7,"port":37564,"v":1}}',
            b'{"ocs_beacon": {"team":"ASTA","port":0,"v":1}}',
            b'{"ocs_beacon": {"team":"ASTA","port":65536,"v":1}}',
            b'{"ocs_beacon": {"team":"ASTA","port":"37564","v":1}}',
            b'{"ocs_beacon": {"team":"ASTA","port":true,"v":1}}',
            b'{"ocs_beacon": {"team":"ASTA","port":37564,"v":2}}',
            b'{"ocs_beacon": {"team":"ASTA","port":37564}}',
            b'{"ocs_beacon": {"team":"ASTA","port":37564,"v":1}}' + b" " * 600,
        ]
        for data in bad:
            with self.subTest(data=data[:40]):
                self.assertIsNone(beacon.parse(data))
        self.assertEqual(beacon.parse(
            b'{"ocs_beacon": {"team":"ASTA","port":37564,"v":1}}'), good)

    def test_parse_does_not_choke_on_non_bytes(self):
        self.assertIsNone(beacon.parse(None))
        self.assertIsNone(beacon.parse("text"))


class TestEligibility(unittest.TestCase):
    def split(self, addrs, **kw):
        sending, skipped = beacon.eligible(addrs, **kw)
        return sending, {s["addr"]: s["why"] for s in skipped}

    def test_loopback_and_link_local_are_never_used(self):
        sending, why = self.split(["127.0.0.1", "127.5.5.5", "169.254.1.1",
                                   "0.0.0.0", "192.168.100.107"], never_on=())
        self.assertEqual(sending, ["192.168.100.107"])
        self.assertIn("loopback", why["127.0.0.1"])
        self.assertIn("loopback", why["127.5.5.5"])
        self.assertIn("link-local", why["169.254.1.1"])
        self.assertIn("unspecified", why["0.0.0.0"])

    def test_every_course_subnet_is_skipped_by_default(self):
        for third in (65, 66, 67, 68):
            addr = "192.168.%d.184" % third
            with self.subTest(addr):
                sending, why = self.split([addr, "192.168.100.107"])
                self.assertEqual(sending, ["192.168.100.107"])
                self.assertIn("never_on 192.168.%d.0/24" % third, why[addr])

    def test_the_neighbouring_subnets_are_not_swept_up(self):
        sending, _ = self.split(["192.168.64.9", "192.168.69.9", "192.168.6.5"])
        self.assertEqual(sending, ["192.168.64.9", "192.168.69.9", "192.168.6.5"])

    def test_a_custom_never_on_replaces_the_default(self):
        sending, why = self.split(["10.1.2.3", "192.168.65.184"],
                                  never_on=["10.0.0.0/8"])
        self.assertEqual(sending, ["192.168.65.184"])
        self.assertIn("never_on 10.0.0.0/8", why["10.1.2.3"])

    def test_the_brokers_slash_24_is_skipped_even_when_it_is_not_a_course_subnet(self):
        sending, why = self.split(["172.16.9.50", "172.16.10.50"], never_on=(),
                                  broker_host="172.16.9.2")
        self.assertEqual(sending, ["172.16.10.50"])
        self.assertIn("RoboCommand broker 172.16.9.2", why["172.16.9.50"])

    def test_a_broker_hostname_adds_no_rule_but_never_on_still_holds(self):
        sending, why = self.split(["192.168.65.184", "192.168.100.107"],
                                  broker_host="robocommand.example")
        self.assertEqual(sending, ["192.168.100.107"])
        self.assertIn("never_on", why["192.168.65.184"])

    def test_this_laptop_with_the_course_network_up(self):
        """The real list: WiFi and VMware speak, Tailscale and the RoboCommand NIC do not."""
        sending, why = self.split(LAPTOP, broker_host="192.168.65.2")
        self.assertEqual(sending, ["192.168.100.107", "192.168.11.1"])
        self.assertEqual(sorted(why), ["169.254.83.107", "192.168.65.184"])

    def test_the_broker_rule_alone_covers_the_course_nic(self):
        """With never_on emptied, the broker's /24 still keeps the course NIC silent."""
        sending, _ = self.split(LAPTOP, never_on=(), broker_host="192.168.65.2")
        self.assertNotIn("192.168.65.184", sending)

    def test_garbage_duplicates_and_order(self):
        sending, why = self.split(["192.168.100.107", "banana", "192.168.100.107",
                                   " 10.0.0.5 ", "fe80::1"], never_on=())
        self.assertEqual(sending, ["192.168.100.107", "10.0.0.5"])
        self.assertEqual(sorted(why), ["banana", "fe80::1"])

    def test_missing_course_networks(self):
        self.assertEqual(beacon.missing_course_networks(beacon.DEFAULT_NEVER_ON), [])
        self.assertEqual(beacon.missing_course_networks(["192.168.0.0/16"]), [])
        self.assertEqual(beacon.missing_course_networks(["192.168.65.0/24"]),
                         ["192.168.66.0/24", "192.168.67.0/24", "192.168.68.0/24"])
        self.assertEqual(len(beacon.missing_course_networks([])), 4)


class TestEnumeration(unittest.TestCase):
    def test_this_machines_addresses_are_ipv4_strings_without_loopback(self):
        try:
            addrs = beacon.local_ipv4_addresses()
        except OSError as exc:               # a box whose own name does not resolve
            self.skipTest("cannot list this machine's addresses: %s" % exc)
        self.assertIsInstance(addrs, list)
        self.assertEqual(len(set(addrs)), len(addrs))
        for a in addrs:
            with self.subTest(a):
                socket.inet_aton(a)                      # a real dotted quad
                self.assertNotEqual(beacon.eligible([a], (), None)[1][:1],
                                    [{"addr": a, "why": "not an IPv4 address"}])


class TestConfig(unittest.TestCase):
    BASE = """
team_id = "ASTA"
[[vehicle]]
id = "USV1"
type = "TYPE_USV"
[team]
host = "127.0.0.1"
[robocommand]
host = "192.168.65.2"
"""

    def load(self, extra: str = ""):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bridge.toml"
            p.write_text(self.BASE + extra, encoding="utf-8")
            return load(p).discovery

    def test_without_the_section_it_is_off(self):
        d = self.load()
        self.assertFalse(d.enabled)
        self.assertEqual((d.port, d.interval_s), (37565, 1.0))
        self.assertEqual(d.never_on, beacon.COURSE_NETWORKS)

    def test_a_section_that_does_not_say_enabled_is_still_off(self):
        self.assertFalse(self.load("[discovery]\nport = 40000\n").enabled)

    def test_every_key_is_read(self):
        d = self.load('[discovery]\nenabled = true\nport = 40001\ninterval_s = 0.5\n'
                      'never_on = ["10.0.0.0/8", "172.16.9.7/24"]\n')
        self.assertEqual((d.enabled, d.port, d.interval_s), (True, 40001, 0.5))
        self.assertEqual(d.never_on, ("10.0.0.0/8", "172.16.9.0/24"))   # normalised

    def test_integers_are_accepted_as_an_interval(self):
        self.assertEqual(self.load("[discovery]\ninterval_s = 1\n").interval_s, 1.0)

    def test_bad_values_stop_the_load_with_the_key_named(self):
        cases = {
            "enabled": ['enabled = "yes"', "enabled = 1"],
            "port": ["port = 0", "port = 70000", 'port = "37565"', "port = true",
                     "port = 1.5"],
            "interval_s": ["interval_s = 0", "interval_s = -1", "interval_s = 2.5",
                           'interval_s = "1"', "interval_s = true"],
            "never_on": ['never_on = "192.168.65.0/24"', 'never_on = ["nope"]',
                         "never_on = [65]", 'never_on = ["fd00::/8"]'],
        }
        for key, bodies in cases.items():
            for body in bodies:
                with self.subTest(body), self.assertRaisesRegex(ConfigError, key):
                    self.load("[discovery]\n" + body + "\n")

    def test_the_shipped_bridge_toml_turns_it_on_and_keeps_all_four_course_subnets(self):
        cfg = load(Path(__file__).resolve().parents[1] / "bridge.toml")
        d = cfg.discovery
        self.assertTrue(d.enabled)
        self.assertEqual(d.port, 37565)
        self.assertEqual(beacon.missing_course_networks(d.never_on), [])
        # ...and so the RoboCommand-facing address is skipped on the real config.
        sending, _ = beacon.eligible(LAPTOP, d.never_on, cfg.robocommand.host)
        self.assertEqual(sending, ["192.168.100.107", "192.168.11.1"])


class TestSender(unittest.TestCase):
    """The real socket path, on loopback, to an ephemeral port."""

    def setUp(self):
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.rx.close)
        self.rx.bind(("127.0.0.1", 0))
        self.rx.settimeout(3.0)
        self.logs: list[str] = []

    def sender(self, addresses, **kw):
        kw.setdefault("interval_s", 0.05)
        kw.setdefault("rescan_s", 0.1)
        s = beacon.BeaconSender(
            "ASTA", port=self.rx.getsockname()[1], log=self.logs.append,
            addresses=addresses, dest=("127.0.0.1", self.rx.getsockname()[1]),
            select=kw.pop("select", lambda found: (list(found), [])), **kw)
        self.addCleanup(s.stop)
        return s

    def test_it_sends_parseable_beacons_from_the_chosen_address(self):
        s = self.sender(lambda: ["127.0.0.1"])
        s.start(40123)
        data, src = self.rx.recvfrom(2048)
        self.assertEqual(beacon.parse(data), {"team": "ASTA", "port": 40123, "v": 1})
        self.assertEqual(src[0], "127.0.0.1")
        self.assertTrue(wait_until(lambda: s.snapshot()["sent"] >= 2))
        snap = s.snapshot()
        self.assertEqual(snap["sending_on"], ["127.0.0.1"])
        self.assertEqual((snap["enabled"], snap["errors"], snap["tcp_port"]),
                         (True, 0, 40123))

    def test_stop_ends_the_beacons(self):
        s = self.sender(lambda: ["127.0.0.1"])
        s.start(40123)
        self.rx.recvfrom(2048)
        s.stop()
        self.rx.settimeout(0.3)
        try:
            while True:
                self.rx.recvfrom(2048)      # drain what was already in flight
        except socket.timeout:
            pass
        with self.assertRaises(socket.timeout):
            self.rx.recvfrom(2048)
        self.assertEqual(s.snapshot()["sending_on"], [])

    def test_addresses_are_listed_again_and_sockets_follow_them(self):
        current = {"v": ["127.0.0.1"]}
        s = self.sender(lambda: list(current["v"]))
        s.start(40123)
        self.assertTrue(wait_until(lambda: s.snapshot()["sending_on"] == ["127.0.0.1"]))
        current["v"] = []                                   # WiFi went away
        self.assertTrue(wait_until(lambda: s.snapshot()["sending_on"] == []))
        current["v"] = ["127.0.0.1"]                        # ...and came back
        self.assertTrue(wait_until(lambda: s.snapshot()["sending_on"] == ["127.0.0.1"]))

    def test_an_address_that_cannot_be_bound_is_reported_not_fatal(self):
        # 203.0.113.0/24 is TEST-NET-3: not an address of this machine.
        s = self.sender(lambda: ["203.0.113.9", "127.0.0.1"])
        s.start(40123)
        self.assertTrue(wait_until(lambda: s.snapshot()["sending_on"] == ["127.0.0.1"]))
        snap = s.snapshot()
        self.assertEqual([x["addr"] for x in snap["skipped"]], ["203.0.113.9"])
        self.assertIn("bind failed", snap["skipped"][0]["why"])
        self.assertGreaterEqual(snap["errors"], 1)
        self.assertIn("203.0.113.9", snap["last_error"])

    def test_a_resolver_failure_keeps_what_is_open_and_says_so(self):
        state = {"fail": False}

        def addresses():
            if state["fail"]:
                raise OSError("name not resolved")
            return ["127.0.0.1"]

        s = self.sender(addresses)
        s.start(40123)
        self.assertTrue(wait_until(lambda: s.snapshot()["sending_on"] == ["127.0.0.1"]))
        state["fail"] = True
        self.assertTrue(wait_until(lambda: s.snapshot()["errors"] >= 1))
        self.assertEqual(s.snapshot()["sending_on"], ["127.0.0.1"])
        self.assertIn("name not resolved", s.snapshot()["last_error"])

    def test_the_default_filter_never_lets_loopback_through(self):
        s = beacon.BeaconSender("ASTA", port=40124, log=self.logs.append,
                                addresses=lambda: ["127.0.0.1", "169.254.1.1"],
                                dest=("127.0.0.1", self.rx.getsockname()[1]),
                                interval_s=0.05, rescan_s=0.1)
        self.addCleanup(s.stop)
        s.start(40123)
        self.assertTrue(wait_until(lambda: len(s.snapshot()["skipped"]) == 2))
        self.assertEqual(s.snapshot()["sending_on"], [])
        self.rx.settimeout(0.3)
        with self.assertRaises(socket.timeout):
            self.rx.recvfrom(2048)

    def test_a_never_on_that_forgets_a_course_subnet_is_warned_about(self):
        s = beacon.BeaconSender("ASTA", never_on=["192.168.65.0/24"], log=self.logs.append,
                                addresses=lambda: [], interval_s=0.05,
                                dest=("127.0.0.1", self.rx.getsockname()[1]))
        self.addCleanup(s.stop)
        s.start(40123)
        warned = [m for m in self.logs if "never_on does not cover" in m]
        self.assertEqual(len(warned), 3)

    def test_a_change_in_the_set_is_logged_once_not_every_scan(self):
        s = self.sender(lambda: ["127.0.0.1"], rescan_s=0.05)
        s.start(40123)
        self.assertTrue(wait_until(lambda: s.snapshot()["sent"] >= 6))
        self.assertEqual(len([m for m in self.logs if m.startswith("beacon: UDP")]), 1)


def _boat_ocs_link():
    """The boat's ocs_link.py as a module, or None if its checkout is not here.

    Found the way bench_stage2 finds it (RX26_ASV, then a sibling, then Boat/).
    Loaded by path rather than put on sys.path: it is stdlib-only, and the boat's
    package name must not shadow anything of ours.
    """
    env = os.environ.get("RX26_ASV")
    root = Path(__file__).resolve().parents[2]
    for base in ([Path(env)] if env else []) + [root / "rx26_asv",
                                                root / "Boat" / "rx26_asv"]:
        path = base / "crusader_groundstation" / "crusader_groundstation" / "ocs_link.py"
        if path.is_file():
            spec = importlib.util.spec_from_file_location("boat_ocs_link", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return module
    return None


class TestBoatCopyAgrees(unittest.TestCase):
    """The beacon format is duplicated in the boat's ocs_link.py by necessity (a ROS
    package must not import from here). The one failure worth catching is the two
    copies drifting apart: each would pass its own tests while the boat never saw a
    beacon. Same reason bench_stage2 runs the boat's own client."""

    @classmethod
    def setUpClass(cls):
        cls.boat = _boat_ocs_link()
        if cls.boat is None:
            raise unittest.SkipTest("the boat checkout is not beside this one "
                                    "(set RX26_ASV)")

    def test_the_constants_agree(self):
        b = self.boat
        self.assertEqual((b.BEACON_KEY, b.BEACON_VERSION),
                         (beacon.BEACON_KEY, beacon.BEACON_VERSION))
        self.assertEqual(b.DEFAULT_DISCOVERY_PORT, beacon.DEFAULT_PORT)
        self.assertEqual([str(n) for n in b.COURSE_NETWORKS],
                         list(beacon.COURSE_NETWORKS))

    def test_the_boat_forgets_a_beacon_no_sooner_than_two_could_have_been_sent(self):
        self.assertGreaterEqual(self.boat.BEACON_FRESH_S, 2 * beacon.MAX_INTERVAL_S)

    def test_the_boat_reads_what_we_write_and_refuses_what_we_refuse(self):
        corpus = [beacon.build("ASTA", 37564), beacon.build("X", 1),
                  b"", b"junk", b"[]", b"{}", b"\xff", None, "text",
                  b'{"ocs_beacon":{"team":"ASTA","port":0,"v":1}}',
                  b'{"ocs_beacon":{"team":"ASTA","port":true,"v":1}}',
                  b'{"ocs_beacon":{"team":"ASTA","port":37564,"v":2}}',
                  b'{"ocs_beacon":{"team":"ASTA","port":37564,"v":1},"x":1}',
                  b'{"ocs_beacon":{"team":"","port":37564,"v":1}}']
        for data in corpus:
            with self.subTest(data=data if data is None else data[:30]):
                self.assertEqual(self.boat.parse_beacon(data), beacon.parse(data))
        self.assertEqual(self.boat.parse_beacon(beacon.build("ASTA", 37564))["port"],
                         37564)

    def test_the_boat_never_trusts_a_course_address_we_would_not_use(self):
        for third in (65, 66, 67, 68):
            addr = "192.168.%d.184" % third
            self.assertTrue(self.boat.on_course_network(addr))
            self.assertEqual(beacon.eligible([addr], beacon.DEFAULT_NEVER_ON)[0], [])

    def test_a_beacon_off_our_sender_reaches_the_boats_chooser(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(rx.close)
        rx.bind(("127.0.0.1", 0))
        rx.settimeout(3.0)
        sender = beacon.BeaconSender(
            "ASTA", addresses=lambda: ["127.0.0.1"], interval_s=0.05,
            dest=("127.0.0.1", rx.getsockname()[1]),
            select=lambda found: (list(found), []), log=lambda m: None)
        self.addCleanup(sender.stop)
        sender.start(40123)
        data, (ip, _port) = rx.recvfrom(2048)
        chooser = self.boat.TargetChooser([], 37564)
        listener = self.boat.BeaconListener(37565, "ASTA", chooser, lambda m: None)
        self.assertTrue(listener.handle(data, ip))
        self.assertEqual(chooser.choose(), ("127.0.0.1", 40123, "beacon from 127.0.0.1"))
        # ...and a boat of another team is not moved by it.
        other = self.boat.BeaconListener(37565, "OTHER", chooser, lambda m: None)
        self.assertFalse(other.handle(data, ip))


class _FakeSender:
    """Records what the bridge does to its beacon; sends nothing."""

    made: list = []

    def __init__(self, team, **kw):
        self.team, self.kw = team, kw
        self.calls: list = []
        _FakeSender.made.append(self)

    def start(self, tcp_port):
        self.calls.append(("start", tcp_port))

    def stop(self):
        self.calls.append(("stop",))

    def snapshot(self):
        return beacon.off_snapshot(self.kw["port"], "fake") | {"enabled": True}


class TestBridgeWiring(unittest.TestCase):
    CONFIG = """
team_id = "ASTA"
[[vehicle]]
id = "USV1"
type = "TYPE_USV"
[team]
host = "127.0.0.1"
port = 1
[robocommand]
host = "192.168.65.2"
port = 1
[tcp]
host = "127.0.0.1"
port = 0
{tcp_extra}
[limits]
min_declared = 1
[paths]
seq_store = "{tmp}/seq.json"
wire_log = "{tmp}/wire"
{discovery}
"""

    def bridge(self, discovery: str = "", tcp_extra: str = "") -> Bridge:
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name) / "bridge.toml"
        path.write_text(self.CONFIG.format(tmp=Path(tmp.name).as_posix(),
                                           discovery=discovery, tcp_extra=tcp_extra),
                        encoding="utf-8")
        b = Bridge(load(path), log=lambda m: None)
        self.addCleanup(b.wire.close)
        return b

    def test_off_by_default_and_the_page_is_told_why(self):
        b = self.bridge()
        self.assertIsNone(b.beacon)
        view = b.snapshot()["beacon"]
        self.assertFalse(view["enabled"])
        self.assertIn("enabled", view["why"])
        self.assertIn("off (", b.status())

    def test_the_tcp_link_being_off_turns_the_beacon_off_with_it(self):
        b = self.bridge("[discovery]\nenabled = true\n", tcp_extra="enabled = false")
        self.assertIsNone(b.beacon)
        self.assertIn("TCP", b.snapshot()["beacon"]["why"])

    def test_it_starts_with_the_tcp_server_on_the_bound_port_and_stops_before_it(self):
        _FakeSender.made.clear()
        with mock.patch("rx_bridge.bridge.BeaconSender", _FakeSender):
            b = self.bridge("[discovery]\nenabled = true\nport = 40125\n")
        (fake,) = _FakeSender.made
        self.assertEqual(fake.team, "ASTA")
        self.assertEqual(fake.kw["port"], 40125)
        self.assertEqual(fake.kw["broker_host"], "192.168.65.2")
        self.assertEqual(fake.kw["never_on"], beacon.COURSE_NETWORKS)
        b.start_vehicle_link()
        self.addCleanup(b.stop_vehicle_link)
        self.assertEqual(fake.calls, [("start", b.tcp.bound_port)])
        self.assertNotEqual(b.tcp.bound_port, 0)             # the real port, not the config's 0
        self.assertTrue(b.snapshot()["beacon"]["enabled"])
        b.stop_vehicle_link()
        self.assertEqual(fake.calls[-1], ("stop",))


if __name__ == "__main__":
    unittest.main()
