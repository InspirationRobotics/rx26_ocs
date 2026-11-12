"""fleet_link -- owns the OCS laptop's two physical MAVLink inputs and fans them out.

    python -m fleet.fleet_link --config fleet\\fleet.toml        (from the OCS directory)

  wifi  UDP 0.0.0.0:14550   the boat's MAVProxy (sysid 2), unicast AND broadcast
  rfd   COM8 @ 115200       the RFD900ux base station: Ekko, Ekko's companion, the
                            boat's companion, the radio's own RADIO_STATUS

Both go to loopback outputs (QGroundControl, the boat GUI, gcs_radio, diagnostics)
and /fleet on 127.0.0.1:8098 reports what is alive. This REPLACES radio_link.py on
this laptop. Run it on the SYSTEM python (pymavlink and pyserial live there).

Two things ride on top of the routing without being part of it: the tunnel log
(tunnel_log.py: every TUNNEL and robotx-dialect frame, to run/tunnel/*.jsonl) and the OCS
adapter (ocs_adapter.py: Ekko's radio link to the RoboCommand bridge). Both are hub
observers; neither can stop a frame being routed.

The routing rules are written up at the top of hub.py; the wire handling at the top
of mavframe.py; the schema of /fleet in README.md.

SIMULATION ONLY, COMMAND LINE ONLY: --sim-boat HOST:PORT adds one output, "sim_boat", that
sends every frame heard on rfd to a simulated boat's radio port (the Crusader Gazebo sim in
WSL2) and TRANSMITS WHATEVER IT SENDS BACK ON THE REAL RADIO, as a real boat's radio would.
It is not a fleet.toml key, so the competition config cannot carry it. See README.md.
"""
from __future__ import annotations

import argparse
import dataclasses
import ipaddress
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
from typing import Dict, List, Optional, Sequence, Tuple
from urllib.parse import parse_qs, urlsplit

from . import mavframe
from .endpoints import (DisabledLinkIO, EndpointError, SerialLinkIO, UdpLinkIO,
                        UdpOutputIO, claim_exclusively, open_serial)
from .hub import Hub, LinkIO, OutputCfg
from .ocs_adapter import OcsAdapter, OcsCfg, OcsConfigError, disabled_status, parse_hostport, parse_ocs_config
from .tunnel_log import TunnelLog
from .vehicles import STALE_S, VehicleNames

log = logging.getLogger("fleet")

DEFAULT_CONFIG = Path(__file__).with_name("fleet.toml")
OCS_DIR = Path(__file__).resolve().parents[1]       # relative paths in fleet.toml are relative to this
NONE_WORDS = ("none", "off", "disabled")        # --serial NONE / --wifi-port NONE
SIM_BOAT = "sim_boat"                           # the name of the --sim-boat output in /fleet


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
class TunnelLogCfg:
    enabled: bool = True
    dir: str = "run/tunnel"              # relative to the OCS folder; run/ is gitignored

    def path(self) -> Path:
        p = Path(self.dir)
        return p if p.is_absolute() else OCS_DIR / p


@dataclass(frozen=True)
class SimBoatCfg:
    """--sim-boat: a simulated boat's radio port (the Crusader Gazebo sim in WSL2). Set
    ONLY by apply_overrides, from the command line; parse_config never builds one, so
    fleet.toml -- the competition config -- cannot carry it."""
    host: str                            # an IPv4 literal: the WSL VM's eth0 address
    port: int
    rx_only: bool = False                # --sim-boat-rx-only: hear the radio, never key it


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
    ocs: OcsCfg = OcsCfg(enabled=False)                                # [ocs]: Ekko's radio link to the bridge
    tunnel_log: TunnelLogCfg = TunnelLogCfg()                          # [tunnel_log]
    sim_boat: Optional[SimBoatCfg] = None                              # --sim-boat only, never a file key

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
    if SIM_BOAT in raw:
        raise ConfigError("[%s]: not a config key -- --sim-boat exists on the command line only, "
                          "so this file cannot carry a simulated boat" % SIM_BOAT)
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
        if "host" in t:
            raise ConfigError("%s: 'host' is not a key -- every output is loopback (the one remote "
                              "output is --sim-boat, command line only)" % where)
        sysids = _sysid_list(t["sysids"], where + " sysids") if "sysids" in t else None
        blocked = t.get("block_stream_control_links", [])
        if not isinstance(blocked, list) or any(x not in out_links for x in blocked):
            raise ConfigError("%s: block_stream_control_links must list only this output's links"
                              % where)
        outputs.append(OutputCfg(name, _port(_need(t, "port", where), where),
                                 tuple(out_links),
                                 None if sysids is None else frozenset(sysids),
                                 bool(t.get("transmit", False)), tuple(blocked)))

    vehicles = {name: _sysid_list(ids, "[vehicles] %s" % name)
                for name, ids in raw.get("vehicles", {}).items()}
    names = {str(role): str(label) for role, label in raw.get("names", {}).items()}
    for role in names:
        if role not in vehicles:
            raise ConfigError("[names] %s: no such [vehicles] group (defined: %s)"
                              % (role, ", ".join(vehicles) or "none"))

    try:
        ocs = parse_ocs_config(raw.get("ocs", {}))
    except OcsConfigError as e:
        raise ConfigError(str(e))
    tl = raw.get("tunnel_log", {})
    tunnel_log = TunnelLogCfg(enabled=bool(tl.get("enabled", True)), dir=str(tl.get("dir", "run/tunnel")))

    dialect = raw.get("dialect", {})
    cfg = Config(status_host=str(status.get("host", "127.0.0.1")),
                 status_port=_port(status.get("port", 8098), "[status] port"),
                 links=links, outputs=outputs, vehicles=vehicles,
                 robotx_dialect=str(dialect["robotx"]) if "robotx" in dialect else "auto",
                 stale_s=float(status.get("stale_s", STALE_S)),
                 summary_s=float(status.get("summary_s", 10.0)),
                 names=names, ocs=ocs, tunnel_log=tunnel_log)
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
    if cfg.sim_boat is not None:
        _validate_sim_boat(cfg, udp_ports)


def _validate_sim_boat(cfg: Config, udp_ports: set) -> None:
    sb = cfg.sim_boat
    rfd = cfg.links.get("rfd")
    if rfd is None:
        raise ConfigError("--sim-boat mirrors the rfd link and this config has no [links.rfd]")
    if not rfd.enabled:
        raise ConfigError("--sim-boat mirrors the rfd link, which is switched off "
                          "(--serial NONE or enabled = false)")
    if sb.port in udp_ports:
        binder = next(l.name for l in cfg.links.values() if l.type == "udp" and l.enabled and l.port == sb.port)
        raise ConfigError("--sim-boat port %d is a UDP port fleet_link binds itself (link %s): "
                          "use the sim's own radio port, normally 14555" % (sb.port, binder))
    if any(o.name == SIM_BOAT for o in cfg.outputs):
        raise ConfigError("[outputs.%s] clashes with --sim-boat, which owns that output name" % SIM_BOAT)


def parse_sim_boat(value: str) -> Tuple[str, int]:
    """--sim-boat HOST:PORT. HOST must be an IPv4 literal, the WSL VM's eth0 address
    (`python -m fleet.wsl_ip`); a name or an IPv6 address is refused rather than resolved."""
    text = str(value).strip()
    host, sep, port = text.rpartition(":")
    if not sep or not host or not port:
        raise ConfigError("--sim-boat: %r is not HOST:PORT (HOST = the WSL VM's eth0 IPv4 from "
                          "python -m fleet.wsl_ip, PORT = 14555)" % text)
    try:
        addr = ipaddress.IPv4Address(host)
    except ValueError:
        raise ConfigError("--sim-boat: host %r is not an IPv4 address (use the WSL VM's eth0 "
                          "address: python -m fleet.wsl_ip)" % host)
    if addr.is_unspecified or addr.is_multicast or addr == ipaddress.IPv4Address("255.255.255.255"):
        raise ConfigError("--sim-boat: %s is not a unicast address to send to" % host)
    if not (port.isascii() and port.isdigit() and 1 <= int(port) <= 65535):
        raise ConfigError("--sim-boat: port %r is not a UDP port (1-65535)" % port)
    return str(addr), int(port)


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
                    wifi_port: Optional[str] = None, status_port: Optional[int] = None,
                    ocs_bridge: Optional[str] = None, tunnel_dir: Optional[str] = None,
                    sim_boat: Optional[str] = None, sim_boat_rx_only: bool = False) -> Config:
    """CLI overrides. `NONE` for --serial / --wifi-port / --ocs-bridge / --tunnel-dir
    switches that off. `sim_boat` ("HOST:PORT") is the only way a SimBoatCfg comes to
    exist."""
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
    ocs = cfg.ocs
    if ocs_bridge is not None:
        try:
            ocs = (dataclasses.replace(ocs, enabled=False) if _is_none(ocs_bridge) else
                   dataclasses.replace(ocs, enabled=True, **dict(zip(("bridge_host", "bridge_port"),
                                                                      parse_hostport(ocs_bridge, "--ocs-bridge")))))
        except OcsConfigError as e:
            raise ConfigError(str(e))
    tunnel_log = cfg.tunnel_log
    if tunnel_dir is not None:
        tunnel_log = (dataclasses.replace(tunnel_log, enabled=False) if _is_none(tunnel_dir)
                      else dataclasses.replace(tunnel_log, enabled=True, dir=tunnel_dir))
    if sim_boat is None and sim_boat_rx_only:
        raise ConfigError("--sim-boat-rx-only needs --sim-boat HOST:PORT")
    sim = cfg.sim_boat if sim_boat is None else SimBoatCfg(*parse_sim_boat(sim_boat), rx_only=sim_boat_rx_only)
    new = dataclasses.replace(cfg, links=links, ocs=ocs, tunnel_log=tunnel_log, sim_boat=sim,
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


def sim_boat_output_cfg(sb: SimBoatCfg) -> OutputCfg:
    """The one output --sim-boat adds: every sysid heard on rfd goes to the sim, and, unless
    rx_only, whatever the sim sends back goes out on rfd by the ordinary reverse path."""
    return OutputCfg(SIM_BOAT, sb.port, ("rfd",), None, transmit=not sb.rx_only)


def build_hub(cfg: Config, dialect: Optional[mavframe.Dialect] = None,
              serial_opener=open_serial) -> Hub:
    if dialect is None:
        dialect = mavframe.load_dialect(cfg.robotx_dialect)
    # The text the operator reads when 14550 is taken names the port to move QGC to.
    qgc = next((o.port for o in cfg.outputs if o.name == "qgc"), None)
    hint = ("QGroundControl with UDP autoconnect? Close it or set its UDP link to %d" % qgc
            if qgc else "")
    ios = [build_link_io(l, hint, serial_opener) for l in cfg.links.values()]
    outs = [(o, UdpOutputIO(o.name, o.port)) for o in cfg.outputs]       # loopback, always
    if cfg.sim_boat is not None:
        outs.append((sim_boat_output_cfg(cfg.sim_boat), UdpOutputIO(SIM_BOAT, cfg.sim_boat.port, cfg.sim_boat.host)))
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
    sim = snap["outputs"].get(SIM_BOAT)
    if sim:
        parts.append("SIM BOAT %s:%d %s" % (sim["host"], sim["port"],
                                            "KEYING rfd" if sim["transmit"] else "listen-only"))
    ocs = snap.get("ocs") or {}
    for vid, v in (ocs.get("vehicles") or {}).items():
        hb = v["heartbeat"]
        parts.append("ocs %s bridge %s hb %s%s" % (
            vid, "up" if v["bridge"]["connected"] else "DOWN", hb["source"],
            "" if hb["source"] != "none" else " (%s)" % (hb["reason"] or "?")[:70]))
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
        self.tunnel_log: Optional[TunnelLog] = None
        self.adapter: Optional[OcsAdapter] = None
        if cfg.tunnel_log.enabled:
            self.tunnel_log = TunnelLog(cfg.tunnel_log.path(), self.hub.dialect)
            self.hub.observe(self.tunnel_log.on_event, self.tunnel_log.msgids)
            self.hub.add_status_section("tunnel_log", self.tunnel_log.status)
        if cfg.ocs.enabled and cfg.ocs.vehicles:
            self.adapter = OcsAdapter(cfg.ocs, self.hub, tunnel_log=self.tunnel_log)
        else:
            reason = "switched off" if not cfg.ocs.enabled else "no [[ocs.radio_vehicle]] configured"
            self.hub.add_status_section("ocs", lambda: disabled_status(reason))

    def start(self) -> None:
        try:
            # Status first: if another fleet_link is running its 8098 is the cleanest
            # tell, whereas its 14550 would blame QGroundControl.
            self.status = StatusServer(self.hub, self.cfg.status_host, self.cfg.status_port)
            self._open_tunnel_log()
            self.hub.open()
            self.hub.start()
            if self.adapter is not None:
                self.adapter.start()
            self.status.start()
        except BaseException:
            self.stop()
            raise

    def _open_tunnel_log(self) -> None:
        """A log that cannot be opened is reported, never fatal: routing comes first."""
        if self.tunnel_log is None:
            return
        try:
            self.tunnel_log.open()
        except OSError as e:
            self.tunnel_log.errors += 1
            self.tunnel_log.last_error = "cannot open %s: %s" % (self.cfg.tunnel_log.path(), e)
            log.error("tunnel log NOT recording - %s (routing is unaffected)", self.tunnel_log.last_error)

    def stop(self) -> None:
        if self.adapter is not None:
            self.adapter.stop()
        self.hub.stop()
        if self.tunnel_log is not None:
            self.tunnel_log.close()
        if self.status is not None:
            self.status.stop()

    def banner(self) -> List[str]:
        lines = []
        sb = self.cfg.sim_boat
        if sb is not None:      # unindented on purpose: nothing else in the banner may read as routine
            lines.append("SIM BOAT: rfd -> %s:%d; %s" % (
                sb.host, sb.port, "listen-only (its replies are dropped and counted)" if sb.rx_only
                else "its replies are KEYED ON THE REAL RADIO"))
        for name, l in self.cfg.links.items():
            if not l.enabled:
                lines.append("  link %-5s disabled" % name)
            elif l.type == "udp":
                lines.append("  link %-5s UDP %s:%d" % (name, l.bind, l.port))
            else:
                lines.append("  link %-5s %s @ %d (reopen every %g s)" % (name, l.device, l.baud, l.reopen_s))
        for out in self.hub.outputs.values():
            o, io = out.cfg, out.io
            lines.append("  out  %-10s -> %s:%-5d links=%s sysids=%s %s (replies on %s:%s)" % (
                o.name, io.host, o.port, "+".join(o.links),
                "all" if o.sysids is None else sorted(o.sysids),
                "may-transmit" if o.transmit else "LISTEN-ONLY", io.local_host, io.local_port))
        o = self.cfg.ocs
        if self.adapter is None:
            lines.append("  ocs  adapter off")
        else:
            lines.append("  ocs  bridge %s:%d as sysid %d/%d for %s" % (
                o.bridge_host, o.bridge_port, o.sysid, o.compid,
                ", ".join("%s (companion %d, autopilot %d, %s)" % (v.vehicle_id, v.companion_sysid,
                                                                   v.autopilot_sysid, v.link)
                          for v in o.vehicles)))
        lines.append("  tunnel log %s" % (
            (self.tunnel_log.path or self.cfg.tunnel_log.path()) if self.tunnel_log else "off"))
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
    ap.add_argument("--ocs-bridge", help="HOST:PORT of the rx_bridge vehicle link (127.0.0.1:37564), or NONE "
                                         "to switch the OCS adapter off")
    ap.add_argument("--tunnel-dir", help="folder for the tunnel log (run/tunnel), or NONE to switch it off")
    ap.add_argument("--sim-boat", metavar="HOST:PORT",
                    help="SIMULATION ONLY, command line only: forward every rfd frame to a simulated boat's radio "
                         "port (the Crusader sim in WSL2, udpin 14555; HOST = the WSL eth0 IPv4, python -m "
                         "fleet.wsl_ip). Its replies are TRANSMITTED ON THE REAL RADIO. Never at competition")
    ap.add_argument("--sim-boat-rx-only", action="store_true",
                    help="with --sim-boat: the sim only listens; its replies are dropped and counted")
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
                              wifi_port=args.wifi_port, status_port=args.status_port,
                              ocs_bridge=args.ocs_bridge, tunnel_dir=args.tunnel_dir,
                              sim_boat=args.sim_boat, sim_boat_rx_only=args.sim_boat_rx_only)
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
