"""fleet_link -- owns the OCS laptop's two physical MAVLink inputs and fans them out.

    python -m fleet.fleet_link --config fleet\\fleet.toml        (from the OCS directory)

  wifi  UDP 0.0.0.0:14550   the boat's MAVProxy (sysid 2), unicast AND broadcast
  rfd   COM8 @ 115200       the RFD900ux base station: Ekko, Ekko's companion, the
                            boat's companion, the radio's own RADIO_STATUS

Both go to loopback outputs (QGroundControl, the boat GUI, gcs_radio, diagnostics)
and /fleet on 127.0.0.1:8098 reports what is alive. This REPLACES radio_link.py on
this laptop. Run it on the SYSTEM python (pymavlink and pyserial live there).

The routing rules are written up at the top of hub.py; the wire handling at the top
of mavframe.py; the schema of /fleet in README.md.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
import signal
import socketserver
import sys
import threading
import time
import tomllib
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional, Sequence
from urllib.parse import parse_qs, urlsplit

from . import mavframe
from .endpoints import (DisabledLinkIO, EndpointError, SerialLinkIO, UdpLinkIO,
                        UdpOutputIO, claim_exclusively, open_serial)
from .hub import Hub, LinkIO, OutputCfg
from .vehicles import STALE_S, VehicleNames

log = logging.getLogger("fleet")

DEFAULT_CONFIG = Path(__file__).with_name("fleet.toml")
NONE_WORDS = ("none", "off", "disabled")        # --serial NONE / --wifi-port NONE


class ConfigError(ValueError):
    pass


# ----------------------------------------------------------------------- config

@dataclass(frozen=True)
class LinkCfg:
    name: str
    type: str                            # "udp" | "serial"
    enabled: bool = True
    bind: str = "0.0.0.0"                # udp
    port: int = 0                        # udp
    device: str = ""                     # serial
    baud: int = 115200                   # serial
    reopen_s: float = 5.0                # serial


@dataclass(frozen=True)
class Config:
    status_host: str
    status_port: int
    links: Dict[str, LinkCfg]
    outputs: List[OutputCfg]
    vehicles: Dict[str, List[int]]
    robotx_dialect: Optional[str]        # "auto" | path | None
    stale_s: float = STALE_S
    summary_s: float = 10.0
    names: Dict[str, str] = dataclasses.field(default_factory=dict)   # [names]: role -> display

    def vehicle_names(self) -> VehicleNames:
        return VehicleNames(self.vehicles, self.names)


def _need(table: dict, key: str, where: str):
    if key not in table:
        raise ConfigError("%s: missing '%s'" % (where, key))
    return table[key]


def _port(value, where: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 65535:
        raise ConfigError("%s: %r is not a UDP/TCP port (1-65535)" % (where, value))
    return value


def _sysid_list(value, where: str) -> List[int]:
    if (not isinstance(value, list) or not value or
            any(not isinstance(i, int) or isinstance(i, bool) or not 0 <= i <= 255
                for i in value)):
        raise ConfigError("%s: must be a non-empty list of sysids 0-255 (leave the key out "
                          "for 'all sysids')" % where)
    return value


def parse_config(raw: dict) -> Config:
    status = raw.get("status", {})
    links: Dict[str, LinkCfg] = {}
    for name, t in raw.get("links", {}).items():
        where = "[links.%s]" % name
        ltype = _need(t, "type", where)
        enabled = bool(t.get("enabled", True))
        if ltype == "udp":
            links[name] = LinkCfg(name, "udp", enabled,
                                  bind=str(t.get("bind", "0.0.0.0")),
                                  port=_port(_need(t, "port", where), where))
        elif ltype == "serial":
            links[name] = LinkCfg(name, "serial", enabled,
                                  device=str(_need(t, "device", where)),
                                  baud=int(t.get("baud", 115200)),
                                  reopen_s=float(t.get("reopen_s", 5.0)))
        else:
            raise ConfigError("%s: type must be 'udp' or 'serial', not %r" % (where, ltype))

    outputs: List[OutputCfg] = []
    for name, t in raw.get("outputs", {}).items():
        where = "[outputs.%s]" % name
        out_links = _need(t, "links", where)
        if not isinstance(out_links, list) or not out_links:
            raise ConfigError("%s: links must be a non-empty list of link names" % where)
        for ln in out_links:
            if ln not in links:
                raise ConfigError("%s: unknown link %r (defined: %s)"
                                  % (where, ln, ", ".join(links) or "none"))
        sysids = _sysid_list(t["sysids"], where + " sysids") if "sysids" in t else None
        outputs.append(OutputCfg(name, _port(_need(t, "port", where), where),
                                 tuple(out_links),
                                 None if sysids is None else frozenset(sysids),
                                 bool(t.get("transmit", False))))

    vehicles = {name: _sysid_list(ids, "[vehicles] %s" % name)
                for name, ids in raw.get("vehicles", {}).items()}
    names = {str(role): str(label) for role, label in raw.get("names", {}).items()}
    for role in names:
        if role not in vehicles:
            raise ConfigError("[names] %s: no such [vehicles] group (defined: %s)"
                              % (role, ", ".join(vehicles) or "none"))

    dialect = raw.get("dialect", {})
    cfg = Config(status_host=str(status.get("host", "127.0.0.1")),
                 status_port=_port(status.get("port", 8098), "[status] port"),
                 links=links, outputs=outputs, vehicles=vehicles,
                 robotx_dialect=str(dialect["robotx"]) if "robotx" in dialect else "auto",
                 stale_s=float(status.get("stale_s", STALE_S)),
                 summary_s=float(status.get("summary_s", 10.0)),
                 names=names)
    validate(cfg)
    return cfg


def validate(cfg: Config) -> None:
    """Cross-checks that a typo should not get past."""
    if not cfg.links:
        raise ConfigError("no [links.*] defined")
    if not cfg.outputs:
        raise ConfigError("no [outputs.*] defined")
    try:
        cfg.vehicle_names()
    except ValueError as e:
        raise ConfigError("[vehicles]: %s" % e)
    udp_ports = {l.port for l in cfg.links.values() if l.type == "udp" and l.enabled}
    seen = {}
    for out in cfg.outputs:
        # An output aimed at a port we bind ourselves would feed our own fan-out
        # straight back in as input.
        if out.port in udp_ports:
            raise ConfigError("output %r targets UDP %d, which a link binds: fleet_link "
                              "would be sending to itself" % (out.name, out.port))
        if out.port in seen:
            raise ConfigError("outputs %r and %r both target UDP %d"
                              % (seen[out.port], out.name, out.port))
        seen[out.port] = out.name


def load_config(path) -> Config:
    try:
        with open(path, "rb") as fh:
            raw = tomllib.load(fh)
    except FileNotFoundError:
        raise ConfigError("config file not found: %s" % path)
    except tomllib.TOMLDecodeError as e:
        raise ConfigError("%s: %s" % (path, e))
    return parse_config(raw)


def _is_none(value: str) -> bool:
    return str(value).strip().lower() in NONE_WORDS


def _switch(link: LinkCfg, value: str, **changes) -> LinkCfg:
    """Apply a CLI value to a link: NONE switches it off, anything else enables it
    with `changes`."""
    if _is_none(value):
        return dataclasses.replace(link, enabled=False)
    return dataclasses.replace(link, enabled=True, **changes)


def apply_overrides(cfg: Config, *, serial: Optional[str] = None, baud: Optional[int] = None,
                    wifi_port: Optional[str] = None, status_port: Optional[int] = None) -> Config:
    """CLI overrides. `NONE` for --serial / --wifi-port switches that link off."""
    links = dict(cfg.links)

    def need(name: str) -> LinkCfg:
        if name not in links:
            raise ConfigError("the config has no link named %r to override" % name)
        return links[name]

    if serial is not None:
        links["rfd"] = _switch(need("rfd"), serial, device=serial)
    if baud is not None:
        links["rfd"] = dataclasses.replace(need("rfd"), baud=baud)
    if wifi_port is not None:
        wifi = need("wifi")
        links["wifi"] = _switch(wifi, wifi_port, port=wifi.port if _is_none(wifi_port)
                                else _port(int(wifi_port), "--wifi-port"))
    new = dataclasses.replace(cfg, links=links,
                              status_port=cfg.status_port if status_port is None
                              else _port(status_port, "--status-port"))
    validate(new)
    return new


# ----------------------------------------------------------------------- wiring

def build_link_io(cfg: LinkCfg, taken_hint: str = "", serial_opener=open_serial) -> LinkIO:
    if not cfg.enabled:
        return DisabledLinkIO(cfg.name, cfg.type)
    if cfg.type == "udp":
        return UdpLinkIO(cfg.name, cfg.bind, cfg.port, taken_hint)
    return SerialLinkIO(cfg.name, cfg.device, cfg.baud, cfg.reopen_s, opener=serial_opener)


def build_hub(cfg: Config, dialect: Optional[mavframe.Dialect] = None,
              serial_opener=open_serial) -> Hub:
    if dialect is None:
        dialect = mavframe.load_dialect(cfg.robotx_dialect)
    # The text the operator reads when 14550 is taken names the port to move QGC to.
    qgc = next((o.port for o in cfg.outputs if o.name == "qgc"), None)
    hint = ("QGroundControl with UDP autoconnect? Close it or set its UDP link to %d" % qgc
            if qgc else "")
    ios = [build_link_io(l, hint, serial_opener) for l in cfg.links.values()]
    outs = [(o, UdpOutputIO(o.name, o.port)) for o in cfg.outputs]
    return Hub(dialect, ios, outs, cfg.vehicle_names(), stale_s=cfg.stale_s)


# --------------------------------------------------------------- status server

class _StatusHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False      # on Windows SO_REUSEADDR lets a second server steal the port

    def server_bind(self):
        claim_exclusively(self.socket)
        socketserver.TCPServer.server_bind(self)     # HTTPServer's adds a slow getfqdn()
        host, port = self.server_address[:2]
        self.server_name, self.server_port = host, port


def _make_handler(hub: Hub):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def _reply(self, code: int, body: bytes) -> None:
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            # Read-only and loopback-only: a map page served from another localhost
            # port polls this straight from the browser.
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):                                            # noqa: N802
            url = urlsplit(self.path)
            if url.path.rstrip("/") != "/fleet":
                self._reply(404, b'{"error":"only GET /fleet exists"}')
                return
            pretty = "pretty" in parse_qs(url.query)
            body = json.dumps(hub.snapshot(), indent=2 if pretty else None,
                              separators=None if pretty else (",", ":"),
                              allow_nan=False).encode()
            self._reply(200, body)

        def log_message(self, fmt, *args):                           # keep the console for fleet
            pass

    return Handler


class StatusServer:
    def __init__(self, hub: Hub, host: str, port: int):
        self.host, self.port = host, port
        try:
            self.httpd = _StatusHTTPServer((host, port), _make_handler(hub))
        except OSError as e:
            raise EndpointError("status port %s:%d is taken - is fleet_link already running? "
                                "(%s)" % (host, port, e)) from e
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self.httpd.serve_forever, kwargs={"poll_interval": 0.25},
                                        name="status", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._thread is not None:
            self.httpd.shutdown()
            self._thread.join(2.0)
        self.httpd.server_close()


# ---------------------------------------------------------------------- console

def _fmt_rate(bps: float) -> str:
    return "%.1f kB/s" % (bps / 1000.0) if bps >= 1000 else "%.0f B/s" % bps


def summary_line(snap: dict) -> str:
    """One line for the console: per-link rate and state, per-sysid age, then any
    trouble counters that are non-zero."""
    stale = snap["stale_after_s"]
    links = []
    trouble = []
    for name, l in snap["links"].items():
        if not l["enabled"]:
            links.append("%s off" % name)
            continue
        state = _fmt_rate(l["rate_in_Bps"]) if l["open"] else "DOWN"
        links.append("%s %s" % (name, state))
        for key in ("bad_crc", "unknown_id", "tx_dropped", "write_errors"):
            if l[key]:
                trouble.append("%s %s %d" % (name, key, l[key]))
    sysids = []
    for s in snap["sysids"].values():
        age = s["age_s"]
        sysids.append("%d %s %s" % (s["sysid"], s["vehicle"],
                                    ("%.1fs" % age) if age <= stale else "STALE %.0fs" % age))
    for name, o in snap["outputs"].items():
        for key in ("no_route", "dropped_listen_only"):
            if o[key]:
                trouble.append("%s %s %d" % (name, key, o[key]))
    parts = ["fleet", " | ".join(links), ", ".join(sysids) or "no vehicles heard"]
    if trouble:
        parts.append(", ".join(trouble))
    return " | ".join(parts)


# -------------------------------------------------------------------------- app

class App:
    """Everything wired and ready; start() claims the ports, stop() releases them."""

    def __init__(self, cfg: Config, dialect: Optional[mavframe.Dialect] = None,
                 serial_opener=open_serial):
        self.cfg = cfg
        self.hub = build_hub(cfg, dialect, serial_opener)
        self.status: Optional[StatusServer] = None

    def start(self) -> None:
        try:
            # Status first: if another fleet_link is running its 8098 is the cleanest
            # tell, whereas its 14550 would blame QGroundControl.
            self.status = StatusServer(self.hub, self.cfg.status_host, self.cfg.status_port)
            self.hub.open()
            self.hub.start()
            self.status.start()
        except BaseException:
            self.stop()
            raise

    def stop(self) -> None:
        self.hub.stop()
        if self.status is not None:
            self.status.stop()

    def banner(self) -> List[str]:
        lines = []
        for name, l in self.cfg.links.items():
            if not l.enabled:
                lines.append("  link %-5s disabled" % name)
            elif l.type == "udp":
                lines.append("  link %-5s UDP %s:%d" % (name, l.bind, l.port))
            else:
                lines.append("  link %-5s %s @ %d (reopen every %g s)" % (name, l.device, l.baud, l.reopen_s))
        for o in self.cfg.outputs:
            io = self.hub.outputs[o.name].io
            lines.append("  out  %-10s -> 127.0.0.1:%-5d links=%s sysids=%s %s (replies on 127.0.0.1:%s)" % (
                o.name, o.port, "+".join(o.links),
                "all" if o.sysids is None else sorted(o.sysids),
                "may-transmit" if o.transmit else "LISTEN-ONLY", io.local_port))
        lines.append("  status http://%s:%d/fleet" % (self.cfg.status_host, self.cfg.status_port))
        d = self.hub.dialect
        lines.append("  dialect ardupilotmega + robotx (%s): %d messages%s" % (
            d.robotx_path or "NOT LOADED", len(d), "".join("\n  NOTE: " + n for n in d.notes)))
        return lines


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="python -m fleet.fleet_link",
                                 description=__doc__.split("\n\n")[0])
    ap.add_argument("--config", default=str(DEFAULT_CONFIG), help="fleet.toml (default: next to this file)")
    ap.add_argument("--serial", help="the RFD900ux's serial port (COM8), or NONE to switch it off")
    ap.add_argument("--baud", type=int, help="the radio's SERIAL_SPEED (default 115200)")
    ap.add_argument("--wifi-port", help="UDP port to listen on for the boat (14550), or NONE to switch it off")
    ap.add_argument("--status-port", type=int, help="port for GET /fleet (8098)")
    return ap.parse_args(argv)


def _pause_if_launched_from_cmd() -> None:
    """FLEET_LINK.cmd opens this in its own window; when the program dies at start-up
    that window would close before anyone read the reason. The .cmd sets
    FLEET_LINK_PAUSE so only that launch path waits."""
    if os.environ.get("FLEET_LINK_PAUSE") and sys.stdin and sys.stdin.isatty():
        try:
            input("\nfleet_link stopped. Press Enter to close this window. ")
        except (EOFError, KeyboardInterrupt):
            pass


def main(argv: Optional[Sequence[str]] = None) -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s", datefmt="%H:%M:%S")
    args = _parse_args(argv)
    try:
        cfg = apply_overrides(load_config(args.config), serial=args.serial, baud=args.baud,
                              wifi_port=args.wifi_port, status_port=args.status_port)
        app = App(cfg)
    except (ConfigError, mavframe.DialectError, ValueError) as e:
        log.error("configuration error: %s", e)
        _pause_if_launched_from_cmd()
        return 2
    try:
        app.start()
    except EndpointError as e:
        log.error("FATAL: %s", e)
        _pause_if_launched_from_cmd()
        return 2

    stop = threading.Event()
    for sig in ("SIGINT", "SIGTERM", "SIGBREAK"):
        if hasattr(signal, sig):
            signal.signal(getattr(signal, sig), lambda *_: stop.set())
    log.info("fleet_link up:\n%s\nctrl-C to stop", "\n".join(app.banner()))
    try:
        next_summary = time.monotonic() + cfg.summary_s
        while not stop.wait(0.5):
            if time.monotonic() >= next_summary:
                log.info(summary_line(app.hub.snapshot()))
                next_summary += cfg.summary_s
    finally:
        app.stop()
        log.info("fleet_link stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
