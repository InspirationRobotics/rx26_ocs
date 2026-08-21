"""fake_vehicle — a USV heartbeat source, so step 1 needs no boat.

Publishes RxReport heartbeats at 2 Hz to the TEAM broker on the topic a real
robocommand_reporter will use. That is the whole point: the bridge cannot tell
this apart from the Jetson, so everything downstream of the radio can be
rehearsed, broken and fixed on a desk.

It also carries the two faults that are otherwise impossible to provoke on
purpose, because both of them come from hardware states you cannot ask for:

  --nan      heading goes NaN, the way telemetry_bridge.py:210 emits it when
             GPS yaw is unresolved (msg.hdg == 65535)
  --unknown  current_task left at TASK_UNKNOWN, the proto3 zero value that any
             forgotten field produces

Run either and watch the bridge refuse the frame. A safety net you have never
seen catch anything is a safety net you are guessing about.

    python fake_vehicle.py --host 127.0.0.1 --vehicle USV1
    python fake_vehicle.py --nan          # expect: DROP ... heading_deg: NaN
    python fake_vehicle.py --unknown      # expect: DROP ... current_task: UNKNOWN
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
    ap.add_argument("--team", default="INSPIRATION")
    ap.add_argument("--vehicle", default="USV1")
    ap.add_argument("--rate", type=float, default=2.0, help="Hz; the handbook says 2")
    ap.add_argument("--nan", action="store_true", help="emit a NaN heading")
    ap.add_argument("--unknown", action="store_true", help="leave current_task UNKNOWN")
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
            hb.altitude_hae_m = -24.6          # geoid separation; see the README
            hb.depth_m = 0.0
            hb.vehicle_type = rx_common_pb2.VehicleType.TYPE_USV
            if not args.unknown:
                hb.current_task = rx_common_pb2.RxTask.TASK_NONE
            # flight_phase deliberately left UNKNOWN: a surface vehicle has no
            # honest value for it, and the bridge's allow_unknown covers it.

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
