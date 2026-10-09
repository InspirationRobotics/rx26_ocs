import copy
import tempfile
import unittest
from pathlib import Path

from fleet.fleet_link import (DEFAULT_CONFIG, ConfigError, apply_overrides, load_config,
                              parse_config)

BASE = {
    "links": {"wifi": {"type": "udp", "port": 14550},
              "rfd": {"type": "serial", "device": "COM8"}},
    "outputs": {"a": {"port": 14570, "links": ["wifi", "rfd"], "transmit": True}},
}


def cfg_with(**changes):
    raw = copy.deepcopy(BASE)
    for key, value in changes.items():
        raw[key] = value
    return parse_config(raw)


class Shipped(unittest.TestCase):
    def test_default_config_is_the_spec(self):
        cfg = load_config(DEFAULT_CONFIG)
        self.assertEqual((cfg.status_host, cfg.status_port), ("127.0.0.1", 8098))
        wifi, rfd = cfg.links["wifi"], cfg.links["rfd"]
        self.assertEqual((wifi.type, wifi.bind, wifi.port), ("udp", "0.0.0.0", 14550))
        self.assertEqual((rfd.type, rfd.device, rfd.baud, rfd.reopen_s), ("serial", "COM8", 115200, 5.0))
        by_name = {o.name: o for o in cfg.outputs}
        self.assertEqual(list(by_name), ["qgc", "boat_gui", "drone_gui", "drone_diag"])
        q, b, g, d = (by_name[n] for n in ("qgc", "boat_gui", "drone_gui", "drone_diag"))
        self.assertEqual((q.port, q.links, q.sysids, q.transmit), (14570, ("wifi", "rfd"), None, True))
        self.assertEqual((b.port, b.links, b.sysids, b.transmit), (14554, ("wifi",), frozenset({2}), False))
        self.assertEqual((g.port, g.links, g.sysids, g.transmit), (14543, ("rfd",), None, True))
        self.assertEqual((d.port, d.links, d.sysids, d.transmit), (14544, ("rfd",), None, False))
        self.assertEqual(cfg.vehicles, {"boat": [2, 42], "drone": [1, 200], "radio": [51]})
        self.assertEqual(cfg.names, {"boat": "Crusader", "drone": "Ekko", "radio": "RFD900ux"})
        self.assertEqual([cfg.vehicle_names().display_of(i) for i in (1, 2, 42, 200, 51, 7)],
                         ["Ekko", "Crusader", "Crusader", "Ekko", "RFD900ux", "sysid 7"])

    def test_aircraft_and_boat_port_discipline(self):
        # outputs on 1454x are the aircraft's tooling; 14554 is the boat's; qgc is neither
        cfg = load_config(DEFAULT_CONFIG)
        ports = {o.name: o.port for o in cfg.outputs}
        self.assertTrue(str(ports["drone_gui"]).startswith("1454"))
        self.assertTrue(str(ports["drone_diag"]).startswith("1454"))
        self.assertTrue(str(ports["boat_gui"]).startswith("1455"))


class Validation(unittest.TestCase):
    def test_output_listing_an_unknown_link(self):
        with self.assertRaisesRegex(ConfigError, "unknown link 'lte'"):
            cfg_with(outputs={"a": {"port": 14570, "links": ["lte"]}})

    def test_output_aimed_at_a_port_we_bind_would_loop(self):
        with self.assertRaisesRegex(ConfigError, "sending to itself"):
            cfg_with(outputs={"a": {"port": 14550, "links": ["wifi"]}})

    def test_two_outputs_on_one_port(self):
        with self.assertRaisesRegex(ConfigError, "both target UDP 14570"):
            cfg_with(outputs={"a": {"port": 14570, "links": ["wifi"]},
                              "b": {"port": 14570, "links": ["rfd"]}})

    def test_bad_sysid_lists(self):
        for bad in ([], [256], ["2"], 2):
            with self.assertRaises(ConfigError, msg=repr(bad)):
                cfg_with(outputs={"a": {"port": 14570, "links": ["wifi"], "sysids": bad}})

    def test_missing_keys(self):
        with self.assertRaisesRegex(ConfigError, "missing 'port'"):
            cfg_with(outputs={"a": {"links": ["wifi"]}})
        with self.assertRaisesRegex(ConfigError, "missing 'device'"):
            cfg_with(links={"rfd": {"type": "serial"}})

    def test_transmit_defaults_to_listen_only(self):
        cfg = cfg_with(outputs={"a": {"port": 14570, "links": ["wifi"]}})
        self.assertFalse(cfg.outputs[0].transmit)

    def test_sysid_under_two_vehicles(self):
        with self.assertRaisesRegex(ConfigError, "sysid 2"):
            cfg_with(vehicles={"boat": [2], "other": [2]})

    def test_a_display_name_for_a_group_that_does_not_exist(self):
        with self.assertRaisesRegex(ConfigError, r"\[names\] bot: no such"):
            cfg_with(vehicles={"boat": [2]}, names={"bot": "Crusader"})

    def test_names_are_optional(self):
        cfg = cfg_with(vehicles={"boat": [2]})
        self.assertEqual(cfg.vehicle_names().display_of(2), "boat")

    def test_bad_toml_and_missing_file(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.toml"
            p.write_text("[links.wifi\n", encoding="utf-8")
            with self.assertRaises(ConfigError):
                load_config(p)
            with self.assertRaisesRegex(ConfigError, "not found"):
                load_config(Path(d) / "nope.toml")


class Overrides(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(DEFAULT_CONFIG)

    def test_serial_port_and_baud(self):
        c = apply_overrides(self.cfg, serial="COM9", baud=57600)
        self.assertEqual((c.links["rfd"].device, c.links["rfd"].baud, c.links["rfd"].enabled),
                         ("COM9", 57600, True))
        self.assertEqual(self.cfg.links["rfd"].device, "COM8")          # the original is untouched

    def test_none_switches_a_link_off(self):
        self.assertFalse(apply_overrides(self.cfg, serial="NONE").links["rfd"].enabled)
        self.assertFalse(apply_overrides(self.cfg, serial="none").links["rfd"].enabled)
        self.assertFalse(apply_overrides(self.cfg, wifi_port="NONE").links["wifi"].enabled)

    def test_wifi_port_and_status_port(self):
        c = apply_overrides(self.cfg, wifi_port="14999", status_port=18098)
        self.assertEqual((c.links["wifi"].port, c.status_port), (14999, 18098))

    def test_wifi_port_cannot_be_moved_onto_an_output(self):
        with self.assertRaisesRegex(ConfigError, "sending to itself"):
            apply_overrides(self.cfg, wifi_port="14570")

    def test_override_of_a_link_the_config_lacks(self):
        c = cfg_with(links={"wifi": {"type": "udp", "port": 14550}},
                     outputs={"a": {"port": 14570, "links": ["wifi"]}})
        with self.assertRaisesRegex(ConfigError, "no link named 'rfd'"):
            apply_overrides(c, serial="COM9")


if __name__ == "__main__":
    unittest.main()
