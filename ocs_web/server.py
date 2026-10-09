"""ocs_web — the operator's page.

    python -m ocs_web --config bridge.toml
    then open http://127.0.0.1:8099

stdlib http.server, no framework. Same choice both vehicle ground stations
made, and it means the OCS laptop needs nothing installed that the bridge does
not already need.

Endpoints:
    GET  /                 the page
    GET  /map              the fleet map page (routes live in maproutes.py)
    GET  /api/state        everything, as JSON (the page polls this)
    GET  /api/fleet        fleet_link's /fleet, proxied (or {"error": ...})
    GET  /api/map-config   map centre and the vehicle-GUI links
    POST /api/declare      publish the RunDeclaration
    POST /api/active       force a vehicle in/out of the declaration
    POST /api/end          end the run
    POST /api/inject       publish one fake heartbeat as a chosen vehicle

INJECT IS FOR TESTING AND SAYS SO. It publishes to the TEAM broker, which is
exactly what a real vehicle does -- so the bridge cannot tell the difference,
which is the point. Nothing about it is safe to leave running during a scored
run, so it is off unless --inject is passed, and the page shows a banner
whenever it is on.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import replace
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import paho.mqtt.client as mqtt

from rx_bridge.bridge import Bridge
from rx_bridge.config import ConfigError, load
from rx_bridge.proto import common_pb2, rx_common_pb2, rx_reports_pb2

from . import maproutes

HERE = Path(__file__).resolve().parent

#: The middle of RoboNation's own test course (rc_test/test_server.py corners,
#: Singapore). Injected vehicles crawl a slow circle so every field changes --
#: a stream of constants hides the frozen-cache and stale-forwarding bugs this
#: is meant to surface -- and sitting inside the real course means the position
#: on the page is sane rather than mid-Atlantic.
LAT0, LON0 = 1.28060, 103.85570


class Injector:
    """Publishes fake heartbeats onto the team broker, on demand."""

    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                             client_id="ocs-inject-" + cfg.team_id)
        self.c.connect_async(cfg.team.host, cfg.team.port, cfg.team.keepalive)
        self.c.loop_start()
        self._running: dict[str, threading.Event] = {}
        self._t0 = time.time()

    def frame(self, vid: str, *, fault: str = "") -> bytes:
        spec = self.cfg.vehicle(vid)
        theta = (time.time() - self._t0) * 0.05

        r = rx_reports_pb2.RxReport()
        r.team_id = self.cfg.team_id
        r.vehicle_id = vid
        r.sent_at.FromNanoseconds(time.time_ns())

        hb = r.heartbeat
        hb.state = common_pb2.RobotState.Value("STATE_AUTO")
        hb.position.latitude = LAT0 + 0.0004 * math.sin(theta)
        hb.position.longitude = LON0 + 0.0004 * math.cos(theta)
        hb.spd_mps = 1.4
        hb.heading_deg = (float("nan") if fault == "nan"
                          else math.degrees(theta) % 360.0)
        hb.roll_deg = 3.0 * math.sin(theta * 7.0)
        hb.pitch_deg = 1.5 * math.sin(theta * 5.0)

        if spec.type == "TYPE_UUV":
            hb.depth_m = 4.5 + 0.5 * math.sin(theta * 3.0)
        elif spec.type == "TYPE_UAV":
            hb.altitude_hae_m = 42.0 + 3.0 * math.sin(theta * 2.0)
        else:
            hb.altitude_hae_m = -24.6          # geoid separation

        # "mistype" claims to be the wrong craft, which is what a reporter
        # copy-pasted between vehicles looks like on the wire.
        claimed = spec.type
        if fault == "mistype":
            claimed = "TYPE_UAV" if spec.type != "TYPE_UAV" else "TYPE_USV"
        hb.vehicle_type = rx_common_pb2.VehicleType.Value(claimed)

        if fault != "unknown_task":
            hb.current_task = rx_common_pb2.RxTask.TASK_NONE
        # A UAV must always carry a flight phase; "no_phase" is the fault that
        # proves the validator catches an aircraft that forgot it.
        if spec.type == "TYPE_UAV" and fault != "no_phase":
            hb.flight_phase = rx_common_pb2.FlightPhase.FLIGHT_PHASE_AIRBORNE

        return r.SerializeToString()

    def once(self, vid: str, fault: str = "") -> None:
        topic = "team/robotx/%s/%s/report" % (self.cfg.team_id, vid)
        self.c.publish(topic, self.frame(vid, fault=fault), qos=0)

    def stream(self, vid: str, on: bool, fault: str = "") -> None:
        """2 Hz, the rate the handbook mandates, until told to stop."""
        if not on:
            ev = self._running.pop(vid, None)
            if ev:
                ev.set()
            return
        if vid in self._running:
            return
        stop = threading.Event()
        self._running[vid] = stop

        def loop():
            while not stop.is_set():
                self.once(vid, fault)
                stop.wait(0.5)

        threading.Thread(target=loop, daemon=True).start()

    def streaming(self) -> list[str]:
        return sorted(self._running)

    def stop(self) -> None:
        for ev in self._running.values():
            ev.set()
        self._running.clear()
        self.c.loop_stop()
        self.c.disconnect()


def _start_local_broker(host: str, port: int, log) -> None:
    """An MQTT broker in this process, for the team side only.

    So the page can be run on a laptop with nothing installed. It is the same
    amqtt the bench uses, and it is NOT a substitute for Mosquitto on the day:
    it binds wherever the config says, but real vehicles need a broker they can
    route to, and this one dies with the page.
    """
    import asyncio

    from amqtt.broker import Broker

    conf = {
        "listeners": {"default": {"type": "tcp", "bind": "%s:%d" % (host, port),
                                  "max_connections": 50}},
        "sys_interval": 0,
        # amqtt 0.12 refuses every connection without this, and the refusal
        # looks like a client that simply never connects.
        "plugins": {
            "amqtt.plugins.authentication.AnonymousAuthPlugin":
                {"allow_anonymous": True},
        },
    }
    ready = threading.Event()

    def run():
        async def main():
            broker = Broker(conf)      # must be constructed inside the loop
            await broker.start()
            ready.set()
            await asyncio.Event().wait()
        asyncio.run(main())

    threading.Thread(target=run, daemon=True).start()
    if not ready.wait(20):
        raise SystemExit("local broker never started")
    time.sleep(0.6)
    log("LOCAL BROKER on %s:%d -- dev only, real vehicles cannot reach it"
        % (host, port))


def make_handler(bridge: Bridge, injector: Injector | None, logs: list, *,
                 fleet_url: str = maproutes.DEFAULT_FLEET_URL,
                 map_cfg: dict | None = None):
    if map_cfg is None:
        map_cfg = maproutes.build_map_config(None, fleet_url)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):            # quiet; the page is polling
            pass

        def _send(self, code, body: bytes, ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode("utf-8"))

        def do_GET(self):
            # The map page's routes first; None means "not mine" and every
            # path they do not claim falls through to the routes below.
            hit = maproutes.route(self.path, fleet_url=fleet_url,
                                  map_cfg=map_cfg)
            if hit is not None:
                self._send(*hit)
                return
            if self.path in ("/", "/index.html"):
                self._send(200, (HERE / "index.html").read_bytes(), "text/html")
            elif self.path.startswith("/api/state"):
                state = bridge.snapshot()
                state["inject"] = {
                    "enabled": injector is not None,
                    "streaming": injector.streaming() if injector else [],
                }
                state["log"] = logs[-60:]
                self._json(state)
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            body = json.loads(self.rfile.read(n) or b"{}") if n else {}

            if self.path == "/api/declare":
                self._json({"result": bridge.declare()})
            elif self.path == "/api/end":
                self._json({"result": bridge.end()})
            elif self.path == "/api/active":
                # on: true = force into the declaration, false = force out,
                # null = back to deciding from whether it is reporting.
                on = body.get("on")
                self._json({"result": bridge.set_active(
                    body.get("vehicle", ""),
                    None if on is None else bool(on))})
            elif self.path == "/api/inject":
                if injector is None:
                    self._json({"result": "injection is off -- restart with "
                                          "--inject to enable it"}, 403)
                    return
                vid = body.get("vehicle", "")
                if bridge.cfg.vehicle(vid) is None:
                    self._json({"result": "%s is not a configured vehicle" % vid}, 400)
                    return
                fault = body.get("fault", "")
                if body.get("mode") == "stream":
                    injector.stream(vid, bool(body.get("on")), fault)
                    on = bool(body.get("on"))
                    logs.append("inject: %s stream %s%s"
                                % (vid, "ON" if on else "off",
                                   " fault=" + fault if fault and on else ""))
                else:
                    injector.once(vid, fault)
                    logs.append("inject: %s one frame%s"
                                % (vid, " fault=" + fault if fault else ""))
                self._json({"result": "ok", "streaming": injector.streaming()})
            else:
                self._send(404, b"not found", "text/plain")

    return Handler


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="ocs_web", description=__doc__)
    ap.add_argument("--config", default="bridge.toml", type=Path)
    ap.add_argument("--port", type=int, default=8099)
    ap.add_argument("--inject", action="store_true",
                    help="enable the fake-data buttons. Testing only -- the "
                         "page shows a banner while this is on.")
    ap.add_argument("--local-broker", nargs="?", const=11883, type=int,
                    default=None, metavar="PORT",
                    help="run an in-process MQTT broker for the TEAM side on "
                         "PORT (default 11883) and point the team client at "
                         "it, so the page works on a laptop with no Mosquitto. "
                         "Dev only -- real vehicles cannot reach it. It does "
                         "NOT default to 1883 because Windows reserves that "
                         "port in the Hyper-V range and the bind fails with "
                         "WinError 10013.")
    ap.add_argument("--fleet-url", default=maproutes.DEFAULT_FLEET_URL,
                    metavar="URL",
                    help="where the map page's /api/fleet proxy reads "
                         "fleet_link's GET /fleet (default %(default)s)")
    args = ap.parse_args(argv)

    fleet = urlsplit(args.fleet_url)
    if fleet.scheme not in ("http", "https") or not fleet.netloc:
        ap.error("--fleet-url must be an http(s) URL, got %r" % args.fleet_url)

    try:
        cfg = load(args.config)
    except ConfigError as exc:
        print("config: %s" % exc, file=sys.stderr)
        return 2

    logs: list[str] = []

    def log(m):
        line = time.strftime("%H:%M:%S ") + str(m)
        logs.append(line)
        print(line, flush=True)

    if args.local_broker:
        # Override the team broker in the loaded config. The RoboCommand side
        # is untouched: this flag changes where OUR vehicles publish, never
        # where we report.
        cfg = replace(cfg, team=replace(cfg.team, host="127.0.0.1",
                                        port=args.local_broker))
        _start_local_broker(cfg.team.host, cfg.team.port, log)

    bridge = Bridge(cfg, log=log)
    bridge.start()

    injector = Injector(cfg) if args.inject else None
    if injector:
        log("INJECTION ENABLED -- fake data can be published as any vehicle")

    map_cfg = maproutes.build_map_config(
        maproutes.load_map_table(args.config), args.fleet_url)
    for w in map_cfg["warnings"]:
        log("map config: " + w)

    srv = ThreadingHTTPServer(("127.0.0.1", args.port),
                              make_handler(bridge, injector, logs,
                                           fleet_url=args.fleet_url,
                                           map_cfg=map_cfg))
    log("OCS page on http://127.0.0.1:%d   map on /map   fleet_link at %s"
        % (args.port, args.fleet_url))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.shutdown()
        if injector:
            injector.stop()
        bridge.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
