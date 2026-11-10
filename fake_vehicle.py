"""fake_vehicle — a heartbeat source for any of the three, so step 1 needs no boat.

Publishes RxReport heartbeats at 2 Hz to the TEAM broker on the topic a real
robocommand_reporter will use. That is the whole point: the bridge cannot tell
this apart from the Jetson, so everything downstream of the radio can be
rehearsed, broken and fixed on a desk.

`--type` picks which craft it pretends to be, and the differences are not
cosmetic -- a UUV fills depth and a UAV fills altitude and flight phase, which
is exactly the axis along which the bridge's validation policy now varies.
Run three of these at once to rehearse the whole fleet:

    python fake_vehicle.py --vehicle USV1 --type TYPE_USV
    python fake_vehicle.py --vehicle UUV1 --type TYPE_UUV
    python fake_vehicle.py --vehicle UAV1 --type TYPE_UAV

It also carries the faults that are otherwise impossible to provoke on purpose,
because they come from hardware or software states you cannot ask for:

  --nan       heading goes NaN, the way telemetry_bridge.py:210 emits it when
              GPS yaw is unresolved (msg.hdg == 65535)
  --unknown   current_task left at TASK_UNKNOWN, the proto3 zero value that any
              forgotten field produces
  --no-phase  flight_phase left UNKNOWN. Harmless from a boat and refused from
              an aircraft, which is the per-type policy in one flag.
  --mistype   report a vehicle_type that contradicts the config, the signature
              of a reporter copy-pasted from one vehicle to another

A fake UAV is not declarable on heartbeats alone. A declaration with an aircraft carries
the fence and ceiling read off its autopilot (docs/VEHICLE_LINK.md, `ocs_fence`), and that
read travels on the TCP vehicle link, not on the MQTT topic these heartbeats use.
`--ocs-tcp` plays the radio adapter's part, and the autopilot's and pilot's with it: it
connects to the bridge, says hello as `--vehicle`, and answers every `ocs_info` with an
`ocs_fence` read "now" -- RoboCommand's polygon written to a stand-in autopilot, a stand-in
pilot's polygon drawn inside it (RoboCommand's pulled toward its centre to half size), a
FENCE_ALT_MAX and a home altitude (PARAMS, HOME_HAE_M). It reads again every
30 s, as the adapter does. A null polygon leaves the stand-in autopilot as it is.

    python fake_vehicle.py --vehicle UAV1 --type TYPE_UAV --ocs-tcp 127.0.0.1:37564

Run each and watch the bridge refuse the frame. A safety net you have never
seen catch anything is a safety net you are guessing about.

    python fake_vehicle.py --nan                            # DROP heading_deg: NaN
    python fake_vehicle.py --unknown                        # DROP current_task
    python fake_vehicle.py --vehicle UAV1 --type TYPE_UAV --no-phase
    python fake_vehicle.py --mistype                        # DROP ... mistyped
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time
from datetime import datetime, timezone

import paho.mqtt.client as mqtt

from rx_bridge.framing import FrameReader, encode
from rx_bridge.proto import common_pb2, rx_common_pb2, rx_reports_pb2

# Sarasota, near enough for a bench. The boat drives a slow circle so that every
# field in the heartbeat actually changes -- a stream of constants hides exactly
# the bugs (frozen cache, stale forwarding) this is meant to surface.
LAT0, LON0 = 27.3364, -82.5307
RADIUS_DEG = 0.0004


def _rfc3339() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


# ---- a stand-in autopilot's fence, as the radio adapter reads it ------------------------
# Shared with bench_stage3.py and rc_e2e.py, whose stand-in aircraft read the same way.

#: What a stand-in autopilot reports: polygon and altitude fencing (FENCE_TYPE 5),
#: auto-enabled on arming, FENCE_ALT_MAX 30 m above home.
PARAMS = {"FENCE_TYPE": 5.0, "FENCE_ENABLE": 0.0, "FENCE_AUTOENABLE": 1.0,
          "FENCE_ALT_MAX": 30.0, "FENCE_OPTIONS": 0.0}
#: Its home altitude above the WGS84 ellipsoid: the ceiling declared is this + FENCE_ALT_MAX.
HOME_HAE_M = 9.0
#: The stand-in pilot's polygon: RoboCommand's, pulled toward its centre to this fraction.
PILOT_SCALE = 0.5


def open_polygon(fence: list) -> list:
    """`ocs_info.uav_geofence` -- closed, {latitude, longitude} points -- as the OPEN
    [lat, lon] list the adapter reads an autopilot's polygons in."""
    return [[p["latitude"], p["longitude"]] for p in fence[:-1]]


def pilot_polygon(rc: list) -> list:
    """The polygon a stand-in pilot draws inside RoboCommand's OPEN polygon `rc`."""
    return _shrunk(rc, PILOT_SCALE)


def fence_read(vid: str, rc: list | None, held: list, reason: str) -> dict:
    """The `ocs_fence` the radio adapter sends after reading a stand-in autopilot, read now:
    `rc` RoboCommand's OPEN polygon it was told to put there (None: none), `held` the
    autopilot's OPEN inclusion polygons, with PARAMS and HOME_HAE_M."""
    on = rc is not None and rc in held
    return {"ocs_fence": {
        "vehicle_id": vid, "state": "done" if on else "off", "polygon": rc,
        "verified": on, "verified_at": _rfc3339() if on else None, "reason": reason,
        "lockout": None, "read_at": _rfc3339(),
        "shapes": [{"type": "polygon", "inclusion": True, "points": p} for p in held],
        "params": dict(PARAMS), "home_hae_m": HOME_HAE_M}}


class FenceAdapter(threading.Thread):
    """The radio adapter's half of the geofence workflow, over the TCP vehicle link -- with a
    stand-in autopilot and a stand-in pilot behind it, because a bench has neither.

    Not a vehicle and not a report source: its heartbeats still go over MQTT. It opens a
    socket as `vehicle` (hello, link "radio"), reads `ocs_info`, and answers with an
    `ocs_fence` the way fleet_link does after reading the autopilot: RoboCommand's polygon
    it was told, whether that is on the autopilot, every inclusion polygon there, the FENCE_*
    parameters and home HAE. Told a polygon, the stand-in autopilot gets it and the stand-in
    pilot draws one inside it (PILOT_SCALE); a null one leaves the autopilot as it is. Reads
    again every READ_S so the OCS's freshness limit keeps being met. Reconnects until stopped.
    """

    READ_S = 30.0

    def __init__(self, vehicle: str, host: str, port: int) -> None:
        super().__init__(daemon=True, name="fence-adapter")
        self.vehicle, self.host, self.port = vehicle, host, port
        self.halt = threading.Event()
        self.rc: list | None = None           # RoboCommand's polygon it was told (open), or None
        self.held: list = []                  # the stand-in autopilot's inclusion polygons (open)
        self.sock: socket.socket | None = None

    def _send(self, obj: dict) -> None:
        self.sock.sendall(encode(json.dumps(obj).encode("utf-8")))

    def _read(self, reason: str) -> None:
        self._send(fence_read(self.vehicle, self.rc, self.held, reason))
        print("  ocs_fence: %s" % reason)

    def _on_info(self, info: dict) -> None:
        if "uav_geofence" not in info:
            return                                  # not told a polygon: not for an aircraft
        fence = info["uav_geofence"]
        if not fence:
            self.rc = None
            self._read("no RoboCommand polygon: the autopilot's fence is left as it is")
            return
        rc = open_polygon(fence)
        if rc != self.rc:
            self.rc = rc
            self.held = [rc, pilot_polygon(rc)]
            self._read("RoboCommand's polygon written and read back (%d vertices); the stand-in "
                       "pilot's drawn inside it" % len(rc))

    def run(self) -> None:
        while not self.halt.is_set():
            try:
                self.sock = socket.create_connection((self.host, self.port), timeout=5)
                self.sock.settimeout(1.0)
                self._send({"ocs_hello": {"vehicle_id": self.vehicle, "link": "radio",
                                          "agent": "fake_vehicle"}})
                print("fence adapter: connected to %s:%d as %s"
                      % (self.host, self.port, self.vehicle))
                reader, last = FrameReader(), time.monotonic()
                while not self.halt.is_set():
                    try:
                        chunk = self.sock.recv(4096)
                    except socket.timeout:
                        chunk = None
                    if chunk == b"":
                        break
                    for payload in reader.feed(chunk or b""):
                        msg = json.loads(payload.decode("utf-8"))
                        if "ocs_info" in msg:
                            self._on_info(msg["ocs_info"])
                            last = time.monotonic()
                    if time.monotonic() - last >= self.READ_S:
                        self._read("read again: %d inclusion polygon(s)" % len(self.held))
                        last = time.monotonic()
            except (OSError, ValueError) as exc:
                print("fence adapter: %s -- retrying" % exc)
            finally:
                if self.sock is not None:
                    self.sock.close()
            self.halt.wait(2.0)


def _shrunk(polygon: list, k: float) -> list:
    """`polygon` pulled toward the mean of its vertices to `k` of its size: inside it when it
    is convex, which RoboCommand's always is."""
    clat = sum(p[0] for p in polygon) / len(polygon)
    clon = sum(p[1] for p in polygon) / len(polygon)
    return [[round(clat + k * (lat - clat), 8), round(clon + k * (lon - clon), 8)]
            for lat, lon in polygon]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--team", default="ASTA")
    ap.add_argument("--vehicle", default="USV1")
    ap.add_argument("--type", dest="vtype", default="TYPE_USV",
                    choices=["TYPE_USV", "TYPE_UUV", "TYPE_UAV"])
    ap.add_argument("--rate", type=float, default=2.0, help="Hz; the handbook says 2")
    ap.add_argument("--nan", action="store_true", help="emit a NaN heading")
    ap.add_argument("--unknown", action="store_true", help="leave current_task UNKNOWN")
    ap.add_argument("--no-phase", dest="no_phase", action="store_true",
                    help="leave flight_phase UNKNOWN -- legal from a boat, "
                         "refused from an aircraft")
    ap.add_argument("--mistype", action="store_true",
                    help="report a vehicle_type that contradicts --type, the "
                         "signature of a copy-pasted reporter")
    ap.add_argument("--state", default="STATE_AUTO",
                    choices=["STATE_KILLED", "STATE_MANUAL", "STATE_AUTO"])
    ap.add_argument("--ocs-tcp", metavar="HOST:PORT", default=None,
                    help="play the radio adapter's part in the geofence handshake over "
                         "the OCS's TCP vehicle link (127.0.0.1:37564): answer ocs_info "
                         "with ocs_fence, so a fake UAV can be declared. UAV only.")
    args = ap.parse_args()
    if args.ocs_tcp and args.vtype != "TYPE_UAV":
        ap.error("--ocs-tcp is the aircraft's fence handshake; --type must be TYPE_UAV")

    topic = "team/robotx/%s/%s/report" % (args.team, args.vehicle)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id="fake-" + args.vehicle)
    client.connect(args.host, args.port, 15)
    client.loop_start()
    print("publishing %.1f Hz to %s on %s:%d" % (args.rate, topic, args.host, args.port))
    adapter = None
    if args.ocs_tcp:
        host, _, port = args.ocs_tcp.rpartition(":")
        adapter = FenceAdapter(args.vehicle, host or "127.0.0.1", int(port))
        adapter.start()

    period = 1.0 / args.rate
    t0 = time.time()
    n = 0
    try:
        while True:
            n += 1
            theta = (time.time() - t0) * 0.05

            report = rx_reports_pb2.RxReport()
            report.team_id = args.team
            report.vehicle_id = args.vehicle
            # seq and sent_at are the BRIDGE's to own. seq is left at 0 because
            # only the bridge has a durable counter, and sent_at is set here
            # purely so the bridge can measure how stale we were -- it will
            # overwrite it from the OCS clock before publishing.
            report.sent_at.FromNanoseconds(time.time_ns())

            hb = report.heartbeat
            hb.state = common_pb2.RobotState.Value(args.state)
            hb.position.latitude = LAT0 + RADIUS_DEG * math.sin(theta)
            hb.position.longitude = LON0 + RADIUS_DEG * math.cos(theta)
            hb.spd_mps = 1.4
            hb.heading_deg = float("nan") if args.nan else (math.degrees(theta) % 360.0)
            hb.roll_deg = 3.0 * math.sin(theta * 7.0)
            hb.pitch_deg = 1.5 * math.sin(theta * 5.0)
            # The three craft differ here, and the differences are the point.
            # A boat sits at the geoid separation with no depth; a UUV has real
            # depth and no meaningful height above ellipsoid; an aircraft has
            # real altitude and a flight phase it must not omit.
            if args.vtype == "TYPE_UUV":
                hb.altitude_hae_m = 0.0
                hb.depth_m = 4.5 + 0.5 * math.sin(theta * 3.0)
            elif args.vtype == "TYPE_UAV":
                hb.altitude_hae_m = 42.0 + 3.0 * math.sin(theta * 2.0)
                hb.depth_m = 0.0
            else:
                hb.altitude_hae_m = -24.6      # geoid separation; see the README
                hb.depth_m = 0.0

            claimed = args.vtype
            if args.mistype:
                # Claim to be something else entirely. TYPE_USV is the value a
                # reporter copied off Crusader would carry, so a boat claims to
                # be an aircraft and everything else claims to be the boat.
                claimed = "TYPE_UAV" if args.vtype == "TYPE_USV" else "TYPE_USV"
            hb.vehicle_type = rx_common_pb2.VehicleType.Value(claimed)

            if not args.unknown:
                hb.current_task = rx_common_pb2.RxTask.TASK_NONE

            # flight_phase is left UNKNOWN for a surface or submerged vehicle:
            # neither has an honest value, and the bridge's per-type allowance
            # covers them. An aircraft gets a real one unless --no-phase asks
            # for the failure, because for a UAV that field is relayed onward
            # for Singapore's Network Remote ID.
            if args.vtype == "TYPE_UAV" and not args.no_phase:
                hb.flight_phase = rx_common_pb2.FlightPhase.FLIGHT_PHASE_AIRBORNE

            client.publish(topic, report.SerializeToString(), qos=0)
            if n % 20 == 0:
                print("  %d heartbeats" % n)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n%d heartbeats sent" % n)
    finally:
        if adapter is not None:
            adapter.halt.set()
        client.loop_stop()
        client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
