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

Run each and watch the bridge refuse the frame. A safety net you have never
seen catch anything is a safety net you are guessing about.

    python fake_vehicle.py --nan                            # DROP heading_deg: NaN
    python fake_vehicle.py --unknown                        # DROP current_task
    python fake_vehicle.py --vehicle UAV1 --type TYPE_UAV --no-phase
    python fake_vehicle.py --mistype                        # DROP ... mistyped
"""
from __future__ import annotations

import argparse
import math
import time

import paho.mqtt.client as mqtt

from rx_bridge.proto import common_pb2, rx_common_pb2, rx_reports_pb2

# Sarasota, near enough for a bench. The boat drives a slow circle so that every
# field in the heartbeat actually changes -- a stream of constants hides exactly
# the bugs (frozen cache, stale forwarding) this is meant to surface.
LAT0, LON0 = 27.3364, -82.5307
RADIUS_DEG = 0.0004


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
    args = ap.parse_args()

    topic = "team/robotx/%s/%s/report" % (args.team, args.vehicle)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id="fake-" + args.vehicle)
    client.connect(args.host, args.port, 15)
    client.loop_start()
    print("publishing %.1f Hz to %s on %s:%d" % (args.rate, topic, args.host, args.port))

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
        client.loop_stop()
        client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
