"""Real sockets on loopback, a scripted fake serial port, the real HTTP server, and
the real command line. No hardware."""
import ast
import json
import os
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

from fleet import endpoints
from fleet.endpoints import EndpointError, SerialLinkIO, UdpLinkIO
from fleet.fleet_link import App, parse_config
from fleet.tests.helpers import (command_long, free_tcp_port, free_udp_port, frames_of,
                                 heartbeat, mission_request_list, position)

LOOPBACK = "127.0.0.1"

OCS_DIR = Path(__file__).resolve().parents[2]
WINDOWS = sys.platform == "win32"


def wait_until(predicate, timeout=3.0, step=0.01):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(step)
    return predicate()


def udp_socket(port=0, host="127.0.0.1"):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))
    return s


def recv(sock, timeout=2.0):
    sock.settimeout(timeout)
    try:
        return sock.recvfrom(65535)
    except (socket.timeout, ConnectionResetError):
        return None


def recv_frame(sock, predicate, timeout=3.0):
    """The first frame on `sock` that satisfies `predicate`, skipping the rest (a
    consumer is fed several vehicles at once)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        got = recv(sock, max(0.05, deadline - time.monotonic()))
        if got is None:
            return None
        for f in frames_of([got[0]]):
            if predicate(f):
                return f
    return None


def get_json(port, path="/fleet"):
    with urllib.request.urlopen("http://127.0.0.1:%d%s" % (port, path), timeout=3) as r:
        return json.loads(r.read())


class FakeSerial:
    """Just enough of pyserial.Serial, scripted."""

    def __init__(self):
        self._rx = bytearray()
        self._lock = threading.Lock()
        self.written = []
        self.read_threads = set()
        self.write_threads = set()
        self.closed = False
        self.fail_read = None
        self.fail_write = None

    def feed(self, data):
        with self._lock:
            self._rx += data

    @property
    def in_waiting(self):
        if self.fail_read:
            raise self.fail_read
        with self._lock:
            return len(self._rx)

    def read(self, n=1):
        self.read_threads.add(threading.current_thread().name)
        if self.fail_read:
            raise self.fail_read
        with self._lock:
            out = bytes(self._rx[:n])
            del self._rx[:n]
        if not out:
            time.sleep(0.005)
        return out

    def write(self, data):
        self.write_threads.add(threading.current_thread().name)
        if self.fail_write:
            raise self.fail_write
        self.written.append(bytes(data))
        return len(data)

    def close(self):
        self.closed = True


def make_config(wifi_port, status_port, out_ports, wifi_host="127.0.0.1"):
    return parse_config({
        "status": {"port": status_port},
        "dialect": {},
        "links": {"wifi": {"type": "udp", "bind": wifi_host, "port": wifi_port},
                  "rfd": {"type": "serial", "device": "FAKE", "reopen_s": 0.05}},
        "outputs": {
            "qgc": {"port": out_ports["qgc"], "links": ["wifi", "rfd"], "transmit": True},
            "boat_gui": {"port": out_ports["boat_gui"], "links": ["wifi"], "sysids": [2]},
            "drone_gui": {"port": out_ports["drone_gui"], "links": ["rfd"], "transmit": True},
            "drone_diag": {"port": out_ports["drone_diag"], "links": ["rfd"]},
        },
        "vehicles": {"boat": [2, 42], "drone": [1, 200], "radio": [51]},
        "names": {"boat": "Crusader", "drone": "Ekko", "radio": "RFD900ux"},
    })


class Harness(unittest.TestCase):
    """An App on real loopback sockets with a fake serial port, and a consumer socket
    standing on each output port."""

    def setUp(self):
        self.serials = []
        self.open_calls = 0
        self.open_script = []           # exceptions to raise on successive opens

        def opener(device, baud):
            self.open_calls += 1
            if self.open_script:
                raise self.open_script.pop(0)
            ser = FakeSerial()
            self.serials.append(ser)
            return ser

        self.wifi_port = free_udp_port()
        self.status_port = free_tcp_port()
        self.consumer = {}
        out_ports = {}
        for name in ("qgc", "boat_gui", "drone_gui", "drone_diag"):
            sock = udp_socket()
            self.consumer[name] = sock
            out_ports[name] = sock.getsockname()[1]
        self.cfg = make_config(self.wifi_port, self.status_port, out_ports)
        self.opener = opener
        self.app = None
        self.boats = []

    def start(self):
        self.app = App(self.cfg, serial_opener=self.opener)
        self.app.start()
        self.assertTrue(wait_until(lambda: self.serials), "serial never opened")
        return self.app

    def boat_socket(self):
        s = udp_socket()
        self.boats.append(s)
        return s

    def to_wifi(self, sock, data):
        sock.sendto(data, ("127.0.0.1", self.wifi_port))

    def tearDown(self):
        if self.app is not None:
            self.app.stop()
        for s in list(self.consumer.values()) + self.boats:
            s.close()


class WifiPath(Harness):
    def test_frames_reach_the_right_consumers_and_a_reply_comes_back(self):
        self.start()
        boat = self.boat_socket()
        raw = heartbeat(2, type=11, custom_mode=15)
        self.to_wifi(boat, raw)
        data, qgc_reply_addr = recv(self.consumer["qgc"])
        self.assertEqual(data, raw)
        self.assertEqual(recv(self.consumer["boat_gui"])[0], raw)
        self.assertIsNone(recv(self.consumer["drone_gui"], 0.2))
        self.assertIsNone(recv(self.consumer["drone_diag"], 0.2))

        cmd = command_long(2)
        self.consumer["qgc"].sendto(cmd, qgc_reply_addr)           # QGC answers the datagram's source
        data, source = recv(boat)
        self.assertEqual(data, cmd)
        self.assertEqual(source, ("127.0.0.1", self.wifi_port))    # it left from the port the boat sent to

    def test_each_output_has_its_own_attributable_source_port(self):
        self.start()
        boat = self.boat_socket()
        self.to_wifi(boat, heartbeat(2))
        sources = {n: recv(self.consumer[n])[1] for n in ("qgc", "boat_gui")}
        self.assertNotEqual(sources["qgc"], sources["boat_gui"])
        status = get_json(self.status_port)
        self.assertEqual(sources["qgc"][1], status["outputs"]["qgc"]["reply_port"])
        self.assertEqual(sources["boat_gui"][1], status["outputs"]["boat_gui"]["reply_port"])

    def test_unicast_and_broadcast_copies_arrive_once(self):
        self.start()
        a, b = self.boat_socket(), self.boat_socket()              # two source ports, as MAVProxy has
        raw = heartbeat(2, seq=7)
        self.to_wifi(a, raw)
        self.to_wifi(b, raw)
        self.assertEqual(recv(self.consumer["qgc"])[0], raw)
        self.assertIsNone(recv(self.consumer["qgc"], 0.3))
        wifi = get_json(self.status_port)["links"]["wifi"]
        self.assertEqual((wifi["frames_in"], wifi["dups"]), (1, 1))

    def test_a_listen_only_consumer_cannot_talk_to_the_boat(self):
        self.start()
        boat = self.boat_socket()
        self.to_wifi(boat, heartbeat(2))
        _, gui_addr = recv(self.consumer["boat_gui"])
        self.consumer["boat_gui"].sendto(command_long(2), gui_addr)
        self.assertTrue(wait_until(
            lambda: get_json(self.status_port)["outputs"]["boat_gui"]["dropped_listen_only"] == 1))
        self.assertIsNone(recv(boat, 0.3))

    def test_a_consumer_that_went_away_does_not_stop_the_loops(self):
        # Windows turns the ICMP "port unreachable" from sending to a closed port into
        # ConnectionResetError on the NEXT recvfrom of that same socket.
        self.start()
        boat = self.boat_socket()
        self.to_wifi(boat, heartbeat(2, seq=1))
        _, qgc_addr = recv(self.consumer["qgc"])
        self.consumer["drone_gui"].close()                          # the consumer exits...
        for i in range(10):                                         # ...while frames keep arriving
            self.serials[0].feed(heartbeat(1, seq=i))
            time.sleep(0.01)
        boat.close()                                                # the boat's socket goes away too
        self.consumer["qgc"].sendto(command_long(2), qgc_addr)      # reply into the void
        time.sleep(0.2)
        boat2 = self.boat_socket()
        self.to_wifi(boat2, heartbeat(2, seq=2))                    # the wifi thread must still be alive
        self.assertIsNotNone(recv_frame(self.consumer["qgc"], lambda f: (f.sysid, f.seq) == (2, 2)))
        self.assertTrue(self.app.hub.outputs["drone_gui"].io._thread.is_alive())
        self.serials[0].feed(heartbeat(1, seq=99))                  # ...and so must the serial one
        self.assertIsNotNone(recv_frame(self.consumer["qgc"], lambda f: (f.sysid, f.seq) == (1, 99)))


class SerialPath(Harness):
    def test_radio_frames_fan_out_and_commands_go_back_on_the_one_thread(self):
        self.start()
        ser = self.serials[0]
        ser.feed(heartbeat(1) + position(1) + heartbeat(200, type=18, autopilot=8, seq=1))
        for name in ("qgc", "drone_gui", "drone_diag"):
            self.assertEqual(len(frames_of([recv(self.consumer[name])[0]])), 3, name)
        self.assertIsNone(recv(self.consumer["boat_gui"], 0.2))

        self.consumer["drone_gui"].sendto(mission_request_list(1), ("127.0.0.1",
                                          self.app.hub.outputs["drone_gui"].io.local_port))
        fence = mission_request_list(1)
        self.assertTrue(wait_until(lambda: fence in ser.written), "command never reached the radio")
        self.assertEqual(ser.write_threads, {"link-rfd"})
        self.assertEqual(ser.read_threads, {"link-rfd"})            # the SAME thread reads and writes
        link = get_json(self.status_port)["links"]["rfd"]
        self.assertEqual((link["frames_out"], link["bytes_out"]), (1, len(fence)))

        diag_port = self.app.hub.outputs["drone_diag"].io.local_port
        self.consumer["drone_diag"].sendto(mission_request_list(1, seq=1), ("127.0.0.1", diag_port))
        self.assertTrue(wait_until(
            lambda: get_json(self.status_port)["outputs"]["drone_diag"]["dropped_listen_only"] == 1))
        self.assertEqual(ser.written, [fence])

    def test_open_failures_are_logged_once_and_retried(self):
        denied = OSError("could not open port 'COM8': PermissionError(13, 'Access is denied.', None, 5)")
        self.open_script = [denied, denied, denied]
        with self.assertLogs("fleet", level="INFO") as cm:
            self.app = App(self.cfg, serial_opener=self.opener)
            self.app.start()
            self.assertTrue(wait_until(lambda: self.serials), "never reopened")
            self.assertTrue(wait_until(lambda: get_json(self.status_port)["links"]["rfd"]["open"]))
        self.assertEqual(self.open_calls, 4)
        self.assertEqual(len([l for l in cm.output if "rfd" in l and "DOWN" in l]), 1, cm.output)
        self.assertEqual(len([l for l in cm.output if "rfd: open" in l]), 1, cm.output)
        self.assertIn("another program has it open", "\n".join(cm.output))

    def test_port_disappearing_mid_stream_reopens_and_never_replays_commands(self):
        self.start()
        first = self.serials[0]
        first.feed(heartbeat(1))
        recv(self.consumer["drone_gui"])
        port = self.app.hub.outputs["drone_gui"].io.local_port
        self.open_script = [OSError("could not open port 'COM8': FileNotFoundError(2, 'gone')")] * 8
        with self.assertLogs("fleet", level="INFO") as cm:
            first.fail_read = OSError("ClearCommError failed (PermissionError(13, 'The device "
                                      "does not recognize the command.', None, 22))")
            self.assertTrue(wait_until(lambda: not get_json(self.status_port)["links"]["rfd"]["open"]))
            # While the port is down a command is refused, not parked for later.
            self.consumer["drone_gui"].sendto(mission_request_list(1), ("127.0.0.1", port))
            self.assertTrue(wait_until(
                lambda: get_json(self.status_port)["links"]["rfd"]["tx_dropped"] == 1))
            self.assertTrue(wait_until(lambda: len(self.serials) == 2, 5.0), "never reopened")
            self.assertTrue(wait_until(lambda: get_json(self.status_port)["links"]["rfd"]["open"]))
        self.assertTrue(first.closed)
        self.assertEqual(len([l for l in cm.output if "DOWN" in l]), 2, cm.output)    # unplugged, then busy
        time.sleep(0.15)
        self.assertEqual(self.serials[1].written, [])                  # nothing replayed

    def test_write_failure_is_counted_and_the_link_carries_on(self):
        self.start()
        ser = self.serials[0]
        ser.feed(heartbeat(1))
        recv(self.consumer["drone_gui"])
        port = self.app.hub.outputs["drone_gui"].io.local_port
        ser.fail_write = OSError("write timeout")
        self.consumer["drone_gui"].sendto(mission_request_list(1), ("127.0.0.1", port))
        self.assertTrue(wait_until(lambda: get_json(self.status_port)["links"]["rfd"]["write_errors"] == 1))
        ser.fail_write = None
        self.consumer["drone_gui"].sendto(mission_request_list(1, seq=1), ("127.0.0.1", port))
        self.assertTrue(wait_until(lambda: mission_request_list(1, seq=1) in ser.written))
        self.assertTrue(get_json(self.status_port)["links"]["rfd"]["open"])

    def test_a_full_tx_queue_refuses_rather_than_blocking(self):
        io = SerialLinkIO("rfd", "FAKE", 115200, tx_queue_max=2)
        io._is_open = True
        self.assertEqual([io.send(b"a", 1, None) for _ in range(3)], [True, True, False])

    def test_pyserial_loop_url_opens(self):
        ser = endpoints.open_serial("loop://", 115200)
        try:
            ser.write(b"abc")
            self.assertEqual(ser.read(3), b"abc")
        finally:
            ser.close()


class Status(Harness):
    def test_fleet_endpoint(self):
        self.start()
        self.serials[0].feed(heartbeat(1) + position(1))
        recv(self.consumer["qgc"])
        snap = get_json(self.status_port)
        self.assertEqual(snap["sysids"]["1"]["vehicle"], "drone")
        self.assertEqual(snap["sysids"]["1"]["position"]["lat"], -35.3632621)
        self.assertTrue(snap["links"]["rfd"]["open"])
        self.assertTrue(snap["links"]["wifi"]["open"])
        self.assertEqual(snap["outputs"]["qgc"]["port"], self.cfg.outputs[0].port)
        self.assertEqual(snap["dialect"]["notes"], [])

    def test_pretty_and_unknown_paths(self):
        self.start()
        url = "http://127.0.0.1:%d/fleet?pretty=1" % self.status_port
        with urllib.request.urlopen(url, timeout=3) as r:
            self.assertEqual(r.headers["Content-Type"], "application/json")
            self.assertEqual(r.headers["Cache-Control"], "no-store")
            self.assertIn(b"\n  ", r.read())
        with self.assertRaises(urllib.error.HTTPError) as cm:
            get_json(self.status_port, "/")
        self.assertEqual(cm.exception.code, 404)

    def test_a_page_on_another_port_may_read_it(self):
        self.start()
        with urllib.request.urlopen("http://127.0.0.1:%d/fleet" % self.status_port, timeout=3) as r:
            self.assertEqual(r.headers["Access-Control-Allow-Origin"], "*")
        with self.assertRaises(urllib.error.HTTPError) as cm:      # even the error is readable
            get_json(self.status_port, "/nope")
        self.assertEqual(cm.exception.headers["Access-Control-Allow-Origin"], "*")

    def test_what_a_map_page_reads_over_http(self):
        self.start()
        boat = self.boat_socket()
        self.to_wifi(boat, heartbeat(2, type=11, custom_mode=15))
        self.to_wifi(boat, position(2, lat=13_500_000, lon=1_030_000_000, hdg=4500, seq=1))
        self.assertTrue(wait_until(lambda: get_json(self.status_port)["sysids"].get("2", {})
                                   .get("position", {}).get("lat") is not None))
        boat_entry = get_json(self.status_port)["sysids"]["2"]
        self.assertEqual((boat_entry["name"], boat_entry["link"]), ("Crusader", "wifi"))
        self.assertEqual((boat_entry["position"]["lat"], boat_entry["position"]["lon"],
                          boat_entry["position"]["heading_deg"]), (1.35, 103.0, 45.0))
        self.assertEqual(boat_entry["heartbeat"]["mode"], "GUIDED")
        self.serials[0].feed(heartbeat(1) + position(1, seq=1))
        self.assertTrue(wait_until(lambda: "1" in get_json(self.status_port)["sysids"]))
        self.assertEqual(get_json(self.status_port)["sysids"]["1"]["link"], "rfd")

    def test_every_socket_is_loopback_only(self):
        app = self.start()
        self.assertEqual(app.status.httpd.server_address[0], LOOPBACK)
        for name, out in app.hub.outputs.items():
            self.assertEqual(out.io._sock.getsockname()[0], LOOPBACK, name)
            self.assertEqual(out.io.host, LOOPBACK, name)


class Claiming(Harness):
    def test_wifi_port_taken_fails_loudly_and_releases_everything(self):
        squatter = udp_socket(self.wifi_port)                       # same 127.0.0.1:port
        try:
            self.app = App(self.cfg, serial_opener=self.opener)
            with self.assertRaises(EndpointError) as cm:
                self.app.start()
        finally:
            squatter.close()
        text = str(cm.exception)
        self.assertIn("UDP %d is taken" % self.wifi_port, text)
        self.assertIn("QGroundControl", text)
        self.assertIn("set its UDP link to %d" % self.cfg.outputs[0].port, text)
        # nothing was left holding the status port
        tcp = socket.socket()
        tcp.bind(("127.0.0.1", self.status_port))
        tcp.close()

    def test_second_instance_says_it_is_already_running(self):
        self.start()
        other = App(make_config(free_udp_port(), self.status_port,
                                {n: s.getsockname()[1] for n, s in self.consumer.items()}))
        with self.assertRaisesRegex(EndpointError, "already running"):
            other.start()

    def test_a_plain_holder_of_the_port_fails_us_loudly(self):
        for holder_host in ("0.0.0.0", LOOPBACK):
            port = free_udp_port()
            holder = udp_socket(port, holder_host)                # no options at all
            try:
                with self.assertRaisesRegex(EndpointError, "is taken", msg=holder_host):
                    _open_link(_Bound(), port)
            finally:
                holder.close()

    def test_a_so_reuseaddr_holder_does_not_let_us_share_the_port(self):
        # SO_REUSEADDR is what Qt's UDP sockets (QGroundControl) ask for by default.
        for holder_host in ("0.0.0.0", LOOPBACK):
            port = free_udp_port()
            qt_like = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            qt_like.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            qt_like.bind((holder_host, port))
            try:
                with self.assertRaisesRegex(EndpointError, "is taken", msg=holder_host):
                    _open_link(_Bound(), port)
            finally:
                qt_like.close()

    @unittest.skipUnless(WINDOWS, "Windows UDP sharing semantics")
    def test_once_we_hold_the_port_nobody_can_bind_it_to_steal_it(self):
        # On Windows a socket with SO_REUSEADDR bound to a MORE SPECIFIC address takes
        # datagrams away from a wildcard bind, silently. Ours must make every one fail.
        port = free_udp_port()
        link = _open_link(_Bound(), port)
        try:
            for host in ("0.0.0.0", LOOPBACK):
                for reuse in (False, True):
                    thief = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    if reuse:
                        thief.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    try:
                        with self.assertRaises(OSError, msg="host=%s reuse=%s" % (host, reuse)):
                            thief.bind((host, port))
                    finally:
                        thief.close()
        finally:
            link.stop()

    def test_the_bind_asks_for_exclusive_use_and_never_for_sharing(self):
        sock = endpoints._bind_udp(LOOPBACK, 0)
        try:
            self.assertFalse(sock.getsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR))
            if hasattr(socket, "SO_EXCLUSIVEADDRUSE"):
                self.assertTrue(sock.getsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE))
        finally:
            sock.close()

    def test_no_code_in_the_package_asks_to_share_a_port(self):
        """A regression net over the source itself: the first time someone 'fixes' a
        bind error by adding SO_REUSEADDR, two programs split one stream and neither
        notices."""
        for path in sorted((OCS_DIR / "fleet").glob("*.py")):
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
                name = (node.attr if isinstance(node, ast.Attribute)
                        else node.id if isinstance(node, ast.Name) else None)
                where = "%s:%d" % (path.name, getattr(node, "lineno", 0))
                self.assertNotIn(name, ("SO_REUSEADDR", "SO_REUSEPORT"), where)
                if isinstance(node, ast.Assign) and any(
                        isinstance(t, ast.Name) and t.id == "allow_reuse_address"
                        for t in node.targets):
                    self.assertIs(getattr(node.value, "value", None), False, where)


class _Bound:
    """The few hub methods UdpLinkIO.open() calls."""
    def link_state(self, *a):
        pass


def _open_link(hub, port):
    link = UdpLinkIO("wifi", "0.0.0.0", port, "QGroundControl hint")
    link.hub = hub
    link.open()
    return link


class CommandLine(unittest.TestCase):
    def write_config(self, directory, wifi_port, status_port, out_ports):
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
[outputs.boat_gui]
port = %d
links = ["wifi"]
sysids = [2]
[vehicles]
boat = [2, 42]
""" % (status_port, wifi_port, out_ports[0], out_ports[1]), encoding="utf-8")
        return str(path)

    def run_cli(self, config, *extra):
        env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1")
        env.pop("FLEET_LINK_PAUSE", None)
        flags = subprocess.CREATE_NEW_PROCESS_GROUP if WINDOWS else 0
        return subprocess.Popen([sys.executable, "-B", "-m", "fleet.fleet_link", "--config", config,
                                 *extra], cwd=str(OCS_DIR), env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, creationflags=flags)

    def test_runs_end_to_end_and_stops_cleanly(self):
        wifi_port, status_port = free_udp_port(), free_tcp_port()
        consumers = [udp_socket(), udp_socket()]
        try:
            with tempfile.TemporaryDirectory() as d:
                cfg = self.write_config(d, wifi_port, status_port,
                                        [s.getsockname()[1] for s in consumers])
                proc = self.run_cli(cfg, "--serial", "NONE")
                try:
                    self.assertTrue(wait_until(lambda: self._answers(status_port), 15.0),
                                    "status never answered")
                    boat = udp_socket()
                    raw = heartbeat(2, type=11)
                    boat.sendto(raw, ("127.0.0.1", wifi_port))
                    self.assertEqual(recv(consumers[0])[0], raw)
                    self.assertEqual(recv(consumers[1])[0], raw)
                    snap = get_json(status_port)
                    self.assertFalse(snap["links"]["rfd"]["enabled"])
                    self.assertEqual(snap["links"]["rfd"]["error"], "disabled")
                    self.assertEqual(snap["sysids"]["2"]["vehicle"], "boat")
                    boat.close()
                finally:
                    proc.send_signal(signal.CTRL_BREAK_EVENT if WINDOWS else signal.SIGINT)
                    try:
                        out, _ = proc.communicate(timeout=15)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        out, _ = proc.communicate()
                        self.fail("did not stop on the break signal:\n" + out)
            self.assertEqual(proc.returncode, 0, out)
            self.assertIn("fleet_link stopped", out)
            self.assertIn("rfd", out)
            self.assertIn("disabled", out)
        finally:
            for s in consumers:
                s.close()

    def test_taken_wifi_port_exits_2_naming_the_culprit(self):
        wifi_port, status_port = free_udp_port(), free_tcp_port()
        squatter = udp_socket(wifi_port, "127.0.0.1")
        consumers = [udp_socket(), udp_socket()]
        try:
            with tempfile.TemporaryDirectory() as d:
                cfg = self.write_config(d, wifi_port, status_port,
                                        [s.getsockname()[1] for s in consumers])
                proc = self.run_cli(cfg, "--serial", "NONE")
                out, _ = proc.communicate(timeout=20)
            self.assertEqual(proc.returncode, 2, out)
            self.assertIn("UDP %d is taken" % wifi_port, out)
            self.assertIn("QGroundControl", out)
        finally:
            squatter.close()
            for s in consumers:
                s.close()

    def test_bad_config_exits_2(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "bad.toml"
            p.write_text("[links.wifi]\ntype = 'udp'\n", encoding="utf-8")
            proc = self.run_cli(str(p))
            out, _ = proc.communicate(timeout=20)
        self.assertEqual(proc.returncode, 2, out)
        self.assertIn("configuration error", out)

    @staticmethod
    def _answers(port):
        try:
            get_json(port)
            return True
        except Exception:
            return False


if __name__ == "__main__":
    unittest.main()
