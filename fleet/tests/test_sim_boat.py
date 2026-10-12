"""--sim-boat: a simulated boat's radio port (the Crusader sim in WSL2) mirrored from the
real rfd link, whose replies are transmitted on the real radio. Command-line only.

Three layers: the option and every refusal (pure), the routing on a Hub with fake
transports, and real loopback sockets + the real command line. The sim is a UDP socket on
loopback here; the non-loopback bind is covered by patching the routing-table lookup. No
hardware, and nothing here binds 14550 or opens COM8."""
import contextlib
import dataclasses
import io
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from subprocess import CompletedProcess
from unittest import mock

from fleet import endpoints, wsl_ip
from fleet.endpoints import EndpointError, UdpOutputIO
from fleet.fleet_link import (DEFAULT_CONFIG, SIM_BOAT, ConfigError, SimBoatCfg, _parse_args,
                              apply_overrides, build_hub, load_config, parse_config,
                              parse_sim_boat, sim_boat_output_cfg, summary_line)
from fleet.hub import Hub
from fleet.tests import test_endpoints as te
from fleet.tests.helpers import (DIALECT, Clock, FakeLink, FakeOutput, command_long, free_tcp_port,
                                 free_udp_port, frames_of, heartbeat, position, tunnel_frame)

WSL_HOST = "172.20.74.12"            # what `python -m fleet.wsl_ip` would print
SIM = "%s:14555" % WSL_HOST
LOOPBACK = "127.0.0.1"


def default_cfg(**kw):
    return apply_overrides(load_config(DEFAULT_CONFIG), **kw)


# --------------------------------------------------------------------- the option

class Parsing(unittest.TestCase):
    def test_host_and_port(self):
        self.assertEqual(parse_sim_boat(SIM), (WSL_HOST, 14555))
        self.assertEqual(parse_sim_boat(" 10.0.0.1:65535 "), ("10.0.0.1", 65535))
        self.assertEqual(parse_sim_boat("127.0.0.1:1"), ("127.0.0.1", 1))

    def test_every_malformed_value_is_refused_in_one_line(self):
        bad = ["", "14555", WSL_HOST, WSL_HOST + ":", ":14555",
               "localhost:14555", "wsl.local:14555", "[::1]:14555", "::1:14555",   # not IPv4 literals
               "172.20.74:14555", "172.20.74.256:14555", "172.020.74.12:14555",
               WSL_HOST + ":14555:1",
               "0.0.0.0:14555", "255.255.255.255:14555", "224.0.0.251:14555",      # not somewhere to send
               WSL_HOST + ":0", WSL_HOST + ":65536", WSL_HOST + ":-1", WSL_HOST + ":abc",
               WSL_HOST + ":1.5", WSL_HOST + ":1_000", WSL_HOST + ":１４５"]
        for value in bad:
            with self.subTest(value=value):
                with self.assertRaises(ConfigError) as cm:
                    parse_sim_boat(value)
                self.assertTrue(str(cm.exception).startswith("--sim-boat"), str(cm.exception))
                self.assertNotIn("\n", str(cm.exception))

    def test_the_flags(self):
        args = _parse_args([])
        self.assertEqual((args.sim_boat, args.sim_boat_rx_only), (None, False))
        args = _parse_args(["--sim-boat", SIM])
        self.assertEqual((args.sim_boat, args.sim_boat_rx_only), (SIM, False))
        args = _parse_args(["--sim-boat-rx-only", "--sim-boat", SIM])
        self.assertEqual((args.sim_boat, args.sim_boat_rx_only), (SIM, True))
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as cm:
            _parse_args(["--sim-boat"])                                  # no value
        self.assertEqual(cm.exception.code, 2)

    def test_overrides_build_the_config(self):
        cfg = default_cfg(sim_boat=SIM)
        self.assertEqual(cfg.sim_boat, SimBoatCfg(WSL_HOST, 14555, rx_only=False))
        self.assertEqual(default_cfg(sim_boat=SIM, sim_boat_rx_only=True).sim_boat,
                         SimBoatCfg(WSL_HOST, 14555, rx_only=True))
        self.assertIsNone(default_cfg().sim_boat)
        self.assertEqual([o.name for o in cfg.outputs], ["qgc", "boat_gui", "drone_gui", "drone_diag"])


class NotAConfigKey(unittest.TestCase):
    """The competition fleet.toml must not be able to carry a simulated boat."""

    BASE = {"links": {"wifi": {"type": "udp", "port": 14550},
                      "rfd": {"type": "serial", "device": "COM8"}},
            "outputs": {"a": {"port": 14570, "links": ["wifi", "rfd"], "transmit": True}}}

    def raw(self, **changes):
        raw = json.loads(json.dumps(self.BASE))
        raw.update(changes)
        return raw

    def test_a_sim_boat_table_is_refused_not_ignored(self):
        with self.assertRaisesRegex(ConfigError, "command line only"):
            parse_config(self.raw(sim_boat={"host": WSL_HOST, "port": 14555}))

    def test_a_host_key_on_an_output_is_refused_not_ignored(self):
        raw = self.raw()
        raw["outputs"]["a"]["host"] = WSL_HOST
        with self.assertRaisesRegex(ConfigError, r"\[outputs.a\]: 'host' is not a key"):
            parse_config(raw)

    def test_the_shipped_config_has_none(self):
        self.assertIsNone(load_config(DEFAULT_CONFIG).sim_boat)
        text = Path(DEFAULT_CONFIG).read_text(encoding="utf-8").lower()
        self.assertNotIn("sim_boat", text)
        self.assertNotIn("sim-boat", text)


class Refusals(unittest.TestCase):
    def refused(self, pattern, **kw):
        with self.assertRaisesRegex(ConfigError, pattern):
            default_cfg(**kw)

    def test_rx_only_without_a_sim_boat(self):
        self.refused(r"--sim-boat-rx-only needs --sim-boat", sim_boat_rx_only=True)

    def test_a_malformed_value_through_the_overrides(self):
        self.refused("not HOST:PORT", sim_boat="14555")
        self.refused("not an IPv4 address", sim_boat="localhost:14555")
        self.refused(r"not a UDP port \(1-65535\)", sim_boat=WSL_HOST + ":70000")

    def test_a_port_fleet_link_binds_itself(self):
        self.refused("binds itself .link wifi.", sim_boat=WSL_HOST + ":14550")
        # ...wherever the WiFi link has been moved to
        self.refused("binds itself", sim_boat=WSL_HOST + ":14999", wifi_port="14999")
        # but a port nothing binds is fine, and so is 14550 once WiFi is off
        default_cfg(sim_boat=WSL_HOST + ":14999")
        default_cfg(sim_boat=WSL_HOST + ":14550", wifi_port="NONE")

    def test_no_rfd_link_to_mirror(self):
        no_rfd = parse_config({"links": {"wifi": {"type": "udp", "port": 14550}},
                               "outputs": {"a": {"port": 14570, "links": ["wifi"]}}})
        with self.assertRaisesRegex(ConfigError, r"no \[links.rfd\]"):
            apply_overrides(no_rfd, sim_boat=SIM)

    def test_rfd_switched_off(self):
        self.refused("rfd link, which is switched off", sim_boat=SIM, serial="NONE")

    def test_the_output_name_is_reserved_only_while_a_sim_boat_is_asked_for(self):
        raw = NotAConfigKey.BASE
        named = parse_config({**raw, "outputs": {SIM_BOAT: {"port": 14570, "links": ["rfd"]}}})
        self.assertEqual(apply_overrides(named).outputs[0].name, SIM_BOAT)       # an ordinary loopback output
        with self.assertRaisesRegex(ConfigError, "clashes with --sim-boat"):
            apply_overrides(named, sim_boat=SIM)

    def test_an_override_never_mutates_the_config_it_was_given(self):
        base = load_config(DEFAULT_CONFIG)
        apply_overrides(base, sim_boat=SIM)
        self.assertIsNone(base.sim_boat)


# ------------------------------------------------------------- routing, fake transports

class SimRig:
    """The shipped outputs plus the sim boat, on fake links and a fake clock."""

    def __init__(self, *, rx_only=False):
        self.cfg = default_cfg(sim_boat=SIM, sim_boat_rx_only=rx_only)
        self.clock = Clock()
        self.wifi = FakeLink("wifi", "udp", needs_addr=True, stream=False)
        self.rfd = FakeLink("rfd", "serial", needs_addr=False, stream=True)
        self.out = {o.name: FakeOutput(o.name) for o in self.cfg.outputs}
        self.sim = self.out[SIM_BOAT] = FakeOutput(SIM_BOAT)
        outs = [(o, self.out[o.name]) for o in self.cfg.outputs]
        outs.append((sim_boat_output_cfg(self.cfg.sim_boat), self.sim))
        self.hub = Hub(DIALECT, [self.wifi, self.rfd], outs, self.cfg.vehicle_names(),
                       clock=self.clock, wall=lambda: 1.7e9)

    def status(self):
        return self.hub.snapshot()["outputs"][SIM_BOAT]


class Routing(unittest.TestCase):
    def test_the_output_the_option_adds(self):
        cfg = sim_boat_output_cfg(SimBoatCfg(WSL_HOST, 14555))
        self.assertEqual((cfg.name, cfg.port, cfg.links, cfg.sysids, cfg.transmit),
                         (SIM_BOAT, 14555, ("rfd",), None, True))
        self.assertFalse(sim_boat_output_cfg(SimBoatCfg(WSL_HOST, 14555, rx_only=True)).transmit)

    def test_every_sysid_heard_on_rfd_reaches_the_sim_unchanged(self):
        r = SimRig()
        sent = [heartbeat(1, seq=1), position(1, seq=2), heartbeat(200, type=18, autopilot=8, seq=3),
                heartbeat(42, type=11, seq=4), heartbeat(51, seq=5)]
        for raw in sent:
            r.hub.link_rx("rfd", raw)
        self.assertEqual(r.sim.sent, sent)                          # byte-identical, in order
        self.assertEqual(r.sim.sent, r.out["drone_diag"].sent)      # what the other rfd output gets
        self.assertEqual(r.status()["frames_out"], len(sent))

    def test_wifi_traffic_does_not_reach_the_sim(self):
        r = SimRig()
        r.hub.link_rx("wifi", heartbeat(2), ("192.168.100.2", 5000))
        self.assertEqual(r.sim.sent, [])
        self.assertEqual(len(r.out["boat_gui"].sent), 1)

    def test_a_reply_from_the_sim_is_transmitted_on_rfd(self):
        r = SimRig()
        reply = tunnel_frame(42000, b"hello", sysid=42, compid=191)      # no target: every link of the output
        r.hub.output_rx(SIM_BOAT, reply)
        self.assertEqual(r.rfd.sent, [(reply, None)])
        self.assertEqual(r.wifi.sent, [])                                # never the boat's WiFi link
        st = r.status()
        self.assertEqual((st["frames_in_from_consumer"], st["dropped_listen_only"], st["no_route"]), (1, 0, 0))
        self.assertEqual(r.hub.snapshot()["links"]["rfd"]["frames_out"], 1)

    def test_a_reply_aimed_at_a_vehicle_follows_the_ordinary_route_rule(self):
        r = SimRig()
        r.hub.link_rx("rfd", heartbeat(1))                               # Ekko heard on the radio
        r.hub.link_rx("wifi", heartbeat(2), ("192.168.100.2", 5000))    # the real boat heard on WiFi only
        to_ekko, to_unheard, to_boat = command_long(1), command_long(7), command_long(2)
        for raw in (to_ekko, to_unheard, to_boat):
            r.hub.output_rx(SIM_BOAT, raw)
        self.assertEqual([d for d, _ in r.rfd.sent], [to_ekko])
        self.assertEqual(r.wifi.sent, [])
        self.assertEqual(r.status()["no_route"], 2)
        r.clock.advance(31.0)                                            # Ekko not heard for over 30 s
        r.hub.output_rx(SIM_BOAT, to_ekko)
        self.assertEqual(len(r.rfd.sent), 1)
        self.assertEqual(r.status()["no_route"], 3)

    def test_rx_only_drops_and_counts_the_reply(self):
        r = SimRig(rx_only=True)
        r.hub.link_rx("rfd", heartbeat(1))
        self.assertEqual(len(r.sim.sent), 1)                             # it still hears the radio
        r.hub.output_rx(SIM_BOAT, tunnel_frame(42000, b"hello", sysid=42, compid=191))
        r.hub.output_rx(SIM_BOAT, command_long(1))
        self.assertEqual(r.rfd.sent, [])
        st = r.status()
        self.assertEqual((st["transmit"], st["frames_in_from_consumer"], st["dropped_listen_only"]),
                         (False, 2, 2))

    def test_it_shows_in_fleet_status_with_its_counters(self):
        r = SimRig()
        r.sim.host = WSL_HOST
        r.hub.link_rx("rfd", heartbeat(1))
        snap = r.hub.snapshot()
        json.dumps(snap, allow_nan=False)
        self.assertEqual(set(snap["outputs"]), {"qgc", "boat_gui", "drone_gui", "drone_diag", SIM_BOAT})
        st = snap["outputs"][SIM_BOAT]
        self.assertEqual((st["host"], st["port"], st["links"], st["sysids"], st["transmit"]),
                         (WSL_HOST, 14555, ["rfd"], None, True))
        for key in ("frames_out", "bytes_out", "send_errors", "frames_in_from_consumer",
                    "dropped_listen_only", "no_route", "reply_port"):
            self.assertIn(key, st)
        self.assertEqual(st["frames_out"], 1)

    def test_the_console_summary_says_so(self):
        for rx_only, word in ((False, "KEYING rfd"), (True, "listen-only")):
            r = SimRig(rx_only=rx_only)
            r.sim.host = WSL_HOST
            line = summary_line(r.hub.snapshot())
            self.assertIn("SIM BOAT %s:14555 %s" % (WSL_HOST, word), line)

    def test_build_hub_wires_the_same_output(self):
        hub = build_hub(default_cfg(sim_boat=SIM))
        self.assertEqual(list(hub.outputs), ["qgc", "boat_gui", "drone_gui", "drone_diag", SIM_BOAT])
        sim = hub.outputs[SIM_BOAT]
        self.assertEqual((sim.cfg, type(sim.io), sim.io.host, sim.io.port),
                         (sim_boat_output_cfg(SimBoatCfg(WSL_HOST, 14555)), UdpOutputIO, WSL_HOST, 14555))
        for name in ("qgc", "boat_gui", "drone_gui", "drone_diag"):
            self.assertEqual(hub.outputs[name].io.host, LOOPBACK, name)
        self.assertEqual(list(build_hub(default_cfg()).outputs),
                         ["qgc", "boat_gui", "drone_gui", "drone_diag"])


# ------------------------------------------------------------------ the output socket

class OutputSocket(unittest.TestCase):
    def test_default_host_is_loopback(self):
        self.assertEqual(UdpOutputIO("x", 14555).host, LOOPBACK)

    def test_a_loopback_output_never_asks_the_routing_table(self):
        io = UdpOutputIO("x", free_udp_port())
        with mock.patch.object(endpoints, "local_address_toward") as lookup:
            io.open()
        try:
            lookup.assert_not_called()
            self.assertEqual(io._sock.getsockname()[0], LOOPBACK)
            self.assertEqual(io.local_host, LOOPBACK)
        finally:
            io._release()

    def test_a_remote_output_binds_the_local_address_that_routes_to_it(self):
        io = UdpOutputIO(SIM_BOAT, 14555, "192.0.2.7")                   # TEST-NET-1: nothing there
        with mock.patch.object(endpoints, "local_address_toward", return_value=LOOPBACK) as lookup:
            io.open()
        try:
            lookup.assert_called_once_with("192.0.2.7", 14555)
            self.assertEqual(io._sock.getsockname()[0], LOOPBACK)        # the stand-in; never a wildcard
            self.assertNotEqual(io._sock.getsockname()[0], "0.0.0.0")
            self.assertEqual((io.local_host, io.local_port), io._sock.getsockname()[:2])
        finally:
            io._release()

    def test_no_route_is_a_startup_error_naming_the_host(self):
        io = UdpOutputIO(SIM_BOAT, 14555, "192.0.2.7")
        with mock.patch.object(endpoints, "local_address_toward", side_effect=OSError("unreachable")):
            with self.assertRaisesRegex(EndpointError, "output sim_boat: no route to 192.0.2.7"):
                io.open()

    def test_the_routing_lookup_itself(self):
        self.assertEqual(endpoints.local_address_toward(LOOPBACK, 14555), LOOPBACK)


# ------------------------------------------------------------ real sockets, fake serial

class _SimBase(te.Harness):
    """An App on loopback with the sim boat's UDP socket standing where the WSL sim's
    udpin:0.0.0.0:14555 would be."""
    rx_only = False

    def setUp(self):
        super().setUp()
        self.sim = te.udp_socket()
        self.addCleanup(self.sim.close)
        self.sim_port = self.sim.getsockname()[1]
        self.cfg = apply_overrides(self.cfg, sim_boat="%s:%d" % (LOOPBACK, self.sim_port),
                                   sim_boat_rx_only=self.rx_only)

    def status(self):
        return te.get_json(self.status_port)

    def sim_status(self):
        return self.status()["outputs"][SIM_BOAT]


class SimSockets(_SimBase):
    def test_radio_frames_reach_the_sim_and_its_reply_keys_the_radio(self):
        self.start()
        ser = self.serials[0]
        ser.feed(heartbeat(1) + position(1, seq=1) + heartbeat(200, type=18, autopilot=8, seq=2))
        data, source = te.recv(self.sim)
        self.assertEqual([f.sysid for f in frames_of([data])], [1, 1, 200])
        self.assertEqual(data, heartbeat(1) + position(1, seq=1) + heartbeat(200, type=18, autopilot=8, seq=2))
        self.assertEqual(source, (LOOPBACK, self.sim_status()["reply_port"]))

        reply = tunnel_frame(42000, b"from the sim", sysid=42, compid=191)
        self.sim.sendto(reply, source)                                   # pymavlink udpin answers the sender
        self.assertTrue(te.wait_until(lambda: reply in ser.written), "the reply never reached the radio")
        self.assertEqual(ser.write_threads, {"link-rfd"})                # through the one thread that owns the port
        self.assertTrue(te.wait_until(lambda: self.status()["links"]["rfd"]["frames_out"] == 1))
        st = self.status()
        self.assertEqual((st["links"]["rfd"]["frames_out"], st["links"]["rfd"]["bytes_out"]), (1, len(reply)))
        sim = st["outputs"][SIM_BOAT]
        self.assertEqual((sim["host"], sim["port"], sim["transmit"], sim["links"], sim["sysids"]),
                         (LOOPBACK, self.sim_port, True, ["rfd"], None))
        self.assertEqual((sim["frames_out"], sim["frames_in_from_consumer"], sim["dropped_listen_only"]), (3, 1, 0))

    def test_wifi_traffic_never_reaches_the_sim(self):
        self.start()
        self.to_wifi(self.boat_socket(), heartbeat(2))
        self.assertIsNotNone(te.recv(self.consumer["qgc"]))
        self.assertIsNone(te.recv(self.sim, 0.3))
        self.serials[0].feed(heartbeat(1))                               # while the radio's does
        self.assertIsNotNone(te.recv(self.sim))

    def test_config_outputs_are_still_loopback_beside_a_remote_sim(self):
        remote = "192.0.2.7"                                             # TEST-NET-1; the lookup is stubbed
        self.cfg = apply_overrides(self.cfg, sim_boat="%s:%d" % (remote, self.sim_port))
        with mock.patch.object(endpoints, "local_address_toward", return_value=LOOPBACK) as lookup:
            app = self.start()
        lookup.assert_called_once_with(remote, self.sim_port)            # only the sim asked
        for name, out in app.hub.outputs.items():
            self.assertEqual(out.io.host, remote if name == SIM_BOAT else LOOPBACK, name)
            self.assertEqual(out.io._sock.getsockname()[0], LOOPBACK, name)
        self.assertEqual([o.name for o in self.cfg.outputs], ["qgc", "boat_gui", "drone_gui", "drone_diag"])
        self.assertEqual(list(app.hub.outputs)[-1], SIM_BOAT)
        self.assertEqual(self.sim_status()["host"], remote)

    def test_the_banner_is_loud(self):
        lines = self.start().banner()
        self.assertEqual(lines[0], "SIM BOAT: rfd -> %s:%d; its replies are KEYED ON THE REAL RADIO"
                         % (LOOPBACK, self.sim_port))
        self.assertTrue(any(l.startswith("  out  sim_boat") and "may-transmit" in l and "links=rfd" in l
                            and "sysids=all" in l for l in lines), lines)

    def test_without_the_option_there_is_no_banner_line_and_no_output(self):
        self.cfg = dataclasses.replace(self.cfg, sim_boat=None)
        app = self.start()
        self.assertNotIn(SIM_BOAT, app.hub.outputs)
        self.assertFalse(any("SIM BOAT" in l for l in app.banner()))


class SimSocketsRxOnly(_SimBase):
    rx_only = True

    def test_it_hears_the_radio_but_its_replies_are_dropped_and_counted(self):
        self.start()
        ser = self.serials[0]
        ser.feed(heartbeat(1))
        data, source = te.recv(self.sim)
        self.assertEqual(data, heartbeat(1))
        self.sim.sendto(tunnel_frame(42000, b"x", sysid=42, compid=191), source)
        self.sim.sendto(command_long(1), source)
        self.assertTrue(te.wait_until(lambda: self.sim_status()["dropped_listen_only"] == 2))
        self.assertEqual(ser.written, [])
        self.assertEqual(self.status()["links"]["rfd"]["frames_out"], 0)
        self.assertFalse(self.sim_status()["transmit"])

    def test_the_banner_says_listen_only(self):
        self.assertEqual(self.start().banner()[0], "SIM BOAT: rfd -> %s:%d; listen-only (its replies are "
                         "dropped and counted)" % (LOOPBACK, self.sim_port))


# ------------------------------------------------------------------- the real command line

def run_cli(config, *extra):
    return subprocess.Popen([sys.executable, "-B", "-m", "fleet.fleet_link", "--config", config, *extra],
                            cwd=str(te.OCS_DIR), env=dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
                                     FLEET_LINK_PAUSE=""),
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if te.WINDOWS else 0)


def write_config(directory, wifi_port, status_port, qgc_port):
    path = Path(directory) / "t.toml"
    path.write_text("""
[status]
port = %d
summary_s = 1
[links.wifi]
type = "udp"
bind = "127.0.0.1"
port = %d
[links.rfd]
type = "serial"
device = "COM_NOT_USED"
[outputs.qgc]
port = %d
links = ["wifi", "rfd"]
transmit = true
[vehicles]
boat = [2, 42]
""" % (status_port, wifi_port, qgc_port), encoding="utf-8")
    return str(path)


class CommandLine(unittest.TestCase):
    def setUp(self):
        self.wifi_port, self.status_port = free_udp_port(), free_tcp_port()
        self.qgc = te.udp_socket()
        self.sim = te.udp_socket()
        self.sim_port = self.sim.getsockname()[1]
        for s in (self.qgc, self.sim):
            self.addCleanup(s.close)
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.config = write_config(tmp.name, self.wifi_port, self.status_port, self.qgc.getsockname()[1])

    def finish(self, proc):
        proc.send_signal(signal.CTRL_BREAK_EVENT if te.WINDOWS else signal.SIGINT)
        try:
            out, _ = proc.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
            out, _ = proc.communicate()
            self.fail("did not stop on the break signal:\n" + out)
        return out

    def test_a_refused_option_exits_2_with_one_line_naming_it(self):
        for extra, words in ((["--serial", "loop://", "--sim-boat", "%s:%d" % (LOOPBACK, self.wifi_port)], "binds itself"),
                             (["--serial", "loop://", "--sim-boat", "localhost:14555"], "not an IPv4 address")):
            with self.subTest(extra=extra):
                proc = run_cli(self.config, "--tunnel-dir", "NONE", *extra)
                out, _ = proc.communicate(timeout=30)
                self.assertEqual(proc.returncode, 2, out)
                refusal = [l for l in out.splitlines() if "configuration error" in l]
                self.assertEqual(len(refusal), 1, out)
                self.assertIn("--sim-boat", refusal[0])
                self.assertIn(words, refusal[0])
                self.assertNotIn("fleet_link up", out)               # nothing was opened

    def test_runs_with_the_sim_boat_and_its_reply_goes_out_on_the_radio(self):
        # pyserial's loop:// hands back whatever is written, so a frame transmitted on rfd comes
        # straight back in as one heard on rfd -- and is then mirrored to the sim.
        proc = run_cli(self.config, "--tunnel-dir", "NONE", "--serial", "loop://",
                       "--sim-boat", "%s:%d" % (LOOPBACK, self.sim_port))
        try:
            self.assertTrue(te.wait_until(lambda: self._status() is not None, 15.0), "status never answered")
            reply = tunnel_frame(42000, b"from the sim", sysid=42, compid=191)
            reply_port = self._status()["outputs"][SIM_BOAT]["reply_port"]
            self.sim.sendto(reply, (LOOPBACK, reply_port))
            data, _ = te.recv(self.sim, 5.0)
            self.assertEqual(data, reply)                            # out on rfd, round the loop, back to the sim
            snap = self._status()
            self.assertEqual(snap["links"]["rfd"]["frames_out"], 1)
            self.assertEqual(snap["outputs"][SIM_BOAT]["frames_in_from_consumer"], 1)
            self.assertEqual(snap["outputs"][SIM_BOAT]["host"], LOOPBACK)
        finally:
            out = self.finish(proc)
        self.assertEqual(proc.returncode, 0, out)
        self.assertIn("SIM BOAT: rfd -> %s:%d; its replies are KEYED ON THE REAL RADIO" % (LOOPBACK, self.sim_port), out)
        self.assertIn("out  sim_boat", out)

    def _status(self):
        try:
            return te.get_json(self.status_port)
        except Exception:                                            # noqa: BLE001
            return None


# ------------------------------------------------------------------------- wsl_ip.py

ETH0 = ("2: eth0    inet 172.20.74.12/20 brd 172.20.79.255 scope global eth0\\       "
        "valid_lft forever preferred_lft forever\n")


def completed(stdout=b"", stderr=b"", code=0):
    return CompletedProcess([], code, stdout, stderr)


class WslIp(unittest.TestCase):
    def test_parse(self):
        self.assertEqual(wsl_ip.parse_eth0_ipv4(ETH0), "172.20.74.12")
        self.assertEqual(wsl_ip.parse_eth0_ipv4("1: lo inet 127.0.0.1/8\n" + ETH0), "172.20.74.12")
        self.assertEqual(wsl_ip.parse_eth0_ipv4("2: eth0 inet 169.254.1.2/16\n2: eth0 inet 10.1.2.3/24\n"), "10.1.2.3")
        for text in ("", "Device \"eth0\" does not exist.\n", "2: eth0 inet6 fe80::1/64\n", "inet 999.1.1.1/8"):
            self.assertIsNone(wsl_ip.parse_eth0_ipv4(text), text)

    def test_asks_wsl_for_eth0_in_the_default_distro(self):
        run = mock.Mock(return_value=completed(ETH0.encode()))
        self.assertEqual(wsl_ip.wsl_eth0_ipv4(run=run), "172.20.74.12")
        cmd = run.call_args.args[0]
        self.assertEqual(cmd, ["wsl.exe", "-d", "Ubuntu-22.04", "-e", "sh", "-c", "ip -4 -o addr show eth0"])
        run = mock.Mock(return_value=completed(ETH0.encode()))
        wsl_ip.wsl_eth0_ipv4("Debian", run=run)
        self.assertEqual(run.call_args.args[0][2], "Debian")

    def test_wsl_exe_failures_are_reported_in_words(self):
        no_distro = completed(stdout="There is no distribution with the supplied name.\r\n".encode("utf-16-le"),
                              code=4294967295)
        with self.assertRaisesRegex(wsl_ip.WslIpError, "NoSuch.*no distribution with the supplied name"):
            wsl_ip.wsl_eth0_ipv4("NoSuch", run=mock.Mock(return_value=no_distro))
        with self.assertRaisesRegex(wsl_ip.WslIpError, "no IPv4 address on eth0"):
            wsl_ip.wsl_eth0_ipv4(run=mock.Mock(return_value=completed(b"")))
        with self.assertRaisesRegex(wsl_ip.WslIpError, "wsl.exe not found"):
            wsl_ip.wsl_eth0_ipv4(run=mock.Mock(side_effect=FileNotFoundError()))
        with self.assertRaisesRegex(wsl_ip.WslIpError, "did not answer"):
            wsl_ip.wsl_eth0_ipv4(run=mock.Mock(side_effect=subprocess.TimeoutExpired("wsl.exe", 1)))

    def test_main_prints_only_the_address_or_fails_on_stderr(self):
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(wsl_ip, "wsl_eth0_ipv4", return_value="172.20.74.12"), \
                contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            self.assertEqual(wsl_ip.main([]), 0)
        self.assertEqual((out.getvalue(), err.getvalue()), ("172.20.74.12\n", ""))
        out, err = io.StringIO(), io.StringIO()
        with mock.patch.object(wsl_ip, "wsl_eth0_ipv4", side_effect=wsl_ip.WslIpError("boom")), \
                contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            self.assertEqual(wsl_ip.main(["--distro", "X"]), 1)
        self.assertEqual((out.getvalue(), err.getvalue()), ("", "wsl_ip: boom\n"))


if __name__ == "__main__":
    unittest.main()
