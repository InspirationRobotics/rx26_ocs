"""fake_course — publish a retained RxCourse, so the bridge can get unstuck.

The bridge cannot declare until it has a course: that is the handbook's order
and runstate.py enforces it. RoboCommand normally publishes RxCourse retained,
so it arrives the instant you subscribe.

If RoboNation's stub is not publishing one -- or you want to test the bridge
without bringing the whole stub up -- run this against the same broker and the
bridge will move CONNECTED -> COURSE_RX and let you declare.

RETAINED is the entire point. Published without the retain flag, the message
only reaches clients already subscribed, so a bridge that reconnects later gets
nothing and sits in CONNECTED looking healthy. That is the single most
confusing failure mode in the whole bench setup.

    python fake_course.py --host 127.0.0.1
    python fake_course.py --host 127.0.0.1 --clear   # remove the retained message
"""
from __future__ import annotations

import argparse
import time

import paho.mqtt.client as mqtt

from rx_bridge.proto import rx_course_pb2

TOPIC = "robocommand/robotx/course"

# A rectangle off Sarasota, matching fake_vehicle.py's circle so the boat sits
# inside its own course. Corner order is the polygon's winding, not a bbox.
CORNERS = [
    (27.3350, -82.5325),
    (27.3350, -82.5290),
    (27.3378, -82.5290),
    (27.3378, -82.5325),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--course-id", default="ALPHA")
    ap.add_argument("--pinger-hz", type=int, default=25000)
    ap.add_argument("--clear", action="store_true",
                    help="clear the retained message instead of publishing one")
    args = ap.parse_args()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake-course")
    client.connect(args.host, args.port, 15)
    client.loop_start()

    if args.clear:
        # A zero-length retained payload is how MQTT deletes a retained message.
        client.publish(TOPIC, b"", qos=1, retain=True).wait_for_publish()
        print("cleared the retained course on " + TOPIC)
    else:
        course = rx_course_pb2.RxCourse()
        course.course_id = args.course_id
        course.pinger_freq_hz = args.pinger_hz
        for lat, lon in CORNERS:
            course.corners.add(latitude=lat, longitude=lon)
        course.sent_at.FromNanoseconds(time.time_ns())

        payload = course.SerializeToString()
        client.publish(TOPIC, payload, qos=1, retain=True).wait_for_publish()
        print("published RETAINED course %s (%d Hz, %d corners, %d bytes) to %s"
              % (args.course_id, args.pinger_hz, len(CORNERS), len(payload), TOPIC))

    client.loop_stop()
    client.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
