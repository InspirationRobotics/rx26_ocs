"""bad_request -- send RoboCommand a deliberately wrong Run Request, for the assessment's
system checks SYS 2 (fence outside the course) and SYS 3 (height above the limit).

Laptop, Windows PowerShell, in OCS\\, with ocs_web STOPPED:

    .venv\\Scripts\\python.exe bad_request.py fence  --callsign <first name + UAPL number>
    .venv\\Scripts\\python.exe bad_request.py height --callsign <first name + UAPL number>

It exists because the OCS page refuses both of these itself, so through the page they never
reach RoboCommand. It uses bridge.toml's address and team, and the OCS's own sequence counter
so a later real declaration still climbs. RoboCommand answers a refused declaration with
nothing at all: this prints the reason the OCS's own copy of their check gives, publishes,
then watches for a RunStart. A RunStart means it was wrongly accepted. No RunStart proves
nothing by itself, because a good request also gets none until the vehicles report AUTO, so
the refusal has to be confirmed on RoboCommand's side (the test server's log, tonight).
"""
from __future__ import annotations

import argparse
import socket
import sys
import threading
import time

import paho.mqtt.client as mqtt

from rx_bridge import config, geofence
from rx_bridge.proto import common_pb2, rx_commands_pb2, rx_course_pb2, rx_requests_pb2
from rx_bridge.seqstore import SeqStore

OCS_PAGE_PORT = 8099
OUTSIDE_BY_M = 100.0            # how far beyond the course and transit area the bad fence sits
ABOVE_LIMIT_BY_M = 10.0         # how far above RoboCommand's ceiling limit the bad height is
METRES_PER_DEG_LAT = 111320.0


def ocs_is_running() -> bool:
    try:
        socket.create_connection(("127.0.0.1", OCS_PAGE_PORT), timeout=0.5).close()
        return True
    except OSError:
        return False


def latlons(points) -> list[tuple[float, float]]:
    return [(p.latitude, p.longitude) for p in points]


def moved_north_of_course(fence, corners, transit) -> tuple:
    """The fence, slid north until all of it is well outside the course and transit area."""
    everything = list(corners) + list(transit)
    slide = (max(p[0] for p in everything) - min(p[0] for p in fence)
             + OUTSIDE_BY_M / METRES_PER_DEG_LAT)
    return tuple((lat + slide, lon) for lat, lon in fence)


def build_request(cfg, seqs, uav_id, fence, ceiling, callsign):
    seqs.ensure_epoch("%s-%d" % (cfg.team_id, int(time.time())))
    req = rx_requests_pb2.RxRequest()
    req.team_id = cfg.team_id
    req.seq = seqs.next_request()
    req.sent_at.FromNanoseconds(time.time_ns())
    decl = req.run_declaration
    decl.vehicle_ids.append(uav_id)
    tier = common_pb2.TaskTier
    decl.task1_tier = tier.Value(cfg.tiers[0])
    decl.task2_tier = tier.Value(cfg.tiers[1])
    decl.task3_tier = tier.Value(cfg.tiers[2])
    decl.task4_tier = tier.Value(cfg.tiers[3])
    for lat, lon in fence:
        decl.uav_geofence.add(latitude=lat, longitude=lon)
    decl.uav_geofence_max_altitude_hae_m = ceiling
    decl.uav_pilot_callsign = callsign
    return req


def main() -> int:
    ap = argparse.ArgumentParser(description="Send RoboCommand a deliberately wrong Run Request.")
    ap.add_argument("what", choices=("fence", "height"))
    ap.add_argument("--callsign", required=True, help="first name + UAPL number")
    ap.add_argument("--config", default="bridge.toml")
    ap.add_argument("--wait", type=float, default=10.0, help="seconds to watch for a RunStart")
    args = ap.parse_args()

    if ocs_is_running():
        print("REFUSED: ocs_web is running. Stop it first, or the two share one team's "
              "connection and sequence counter.")
        return 2

    cfg = config.load(args.config)
    uav_id = next((v.id for v in cfg.vehicles if v.is_uav), None)
    if uav_id is None:
        print("REFUSED: bridge.toml has no UAV")
        return 2

    got_course, got_start = threading.Event(), threading.Event()
    held = {}

    def on_connect(client, _ud, _flags, reason, _props=None):
        if reason != 0:
            print("RoboCommand refused the connection: %s" % reason)
            return
        client.subscribe([(cfg.rc_course_sub(), 1), (cfg.rc_command_sub(), 1)])

    def on_message(_client, _ud, msg):
        if msg.topic == cfg.rc_course_sub():
            held["course"] = rx_course_pb2.RxCourse.FromString(msg.payload)
            got_course.set()
        elif msg.topic == cfg.rc_command_sub():
            cmd = rx_commands_pb2.RxCommand.FromString(msg.payload)
            if cmd.WhichOneof("body") == "run_start":
                held["run_start"] = cmd.run_start
                got_start.set()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id="ocs-badrequest-" + cfg.team_id)
    client.on_connect, client.on_message = on_connect, on_message
    print("connecting to RoboCommand at %s:%d as team %s"
          % (cfg.robocommand.host, cfg.robocommand.port, cfg.team_id))
    client.connect(cfg.robocommand.host, cfg.robocommand.port, keepalive=30)
    client.loop_start()
    try:
        if not got_course.wait(30.0):
            print("FAILED: no RxCourse in 30 s, so nothing was sent")
            return 1
        course = held["course"]
        corners = latlons(course.corners)
        transit = latlons(course.uav_transit_area)
        good = geofence.for_course(course.course_id, corners, cfg.geofence_table,
                                   cfg.geofence_margin_m, transit)
        if good.polygon is None:
            print("FAILED: the OCS has no valid fence for %s: %s" % (course.course_id, good.reason))
            return 1

        ceiling = cfg.geofence_ceiling_hae_m
        fence = good.polygon
        if args.what == "fence":
            fence = moved_north_of_course(fence, corners, transit)
        else:
            ceiling = cfg.geofence_ceiling_limit_hae_m + ABOVE_LIMIT_BY_M

        print("course %s; sending %s, ceiling %g m HAE, %d fence points"
              % (course.course_id, uav_id, ceiling, len(fence)))
        problems = geofence.rc_declaration_problems(fence, corners, transit, ceiling,
                                                    cfg.geofence_ceiling_limit_hae_m)
        for problem in problems:
            print("expected refusal: " + problem)
        if not problems:
            print("WARNING: the OCS's copy of RoboCommand's check finds nothing wrong with this")

        req = build_request(cfg, SeqStore(cfg.seq_store), uav_id, fence, ceiling, args.callsign)
        info = client.publish(cfg.rc_request_topic(), req.SerializeToString(), qos=1)
        info.wait_for_publish(timeout=5.0)
        print("seq %d published: broker acknowledged = %s" % (req.seq, info.is_published()))
        if not info.is_published():
            print("FAILED: the broker never acknowledged it, so this proves nothing")
            return 1

        if got_start.wait(args.wait):
            print("ACCEPTED, which is WRONG: RunStart run_id %d" % held["run_start"].run_id)
            return 1
        print("No RunStart in %g s. That alone does NOT prove a refusal: RoboCommand also stays "
              "silent on a good request until every declared vehicle reports AUTO. Confirm the "
              "refusal on RoboCommand's side." % args.wait)
        return 0
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    sys.exit(main())
