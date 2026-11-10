"""graey_ocs_client -- Graey (UUV1) on the OCS vehicle link, run from the laptop.

    python graey_ocs_client.py                    # graey.local -> OCS on 127.0.0.1:37564, 2 Hz
    python graey_ocs_client.py --once             # print the heartbeat it would send, connect nothing
    python graey_ocs_client.py --graey 192.168.8.105 --ocs 127.0.0.1:37564

Windows laptop, next to ocs_web; stdlib only, any python. Graey has no OCS client of its own (its
link design waits on docs/ROBONATION_QUESTIONS.md Q1), so this reads Graey's own control GUI --
GET http://<graey>:8090/api/navigation, READ-ONLY: never /api/start, /api/stop, /api/power,
/api/mission/*, which act on the sub -- and turns what Graey's stack measured into a UUV1
heartbeat on TCP 37564, in the 4-byte length + JSON frames the boat's ocs_link.py sends. Its hello
says `link=wifi synthesized=[heartbeat]`: the heartbeat is built here, not by Graey.

For bench and team-village tests with Graey at the surface on the team network. NOT the competition
design: submerged, Graey has neither GPS nor WiFi, and what it owes then is Q1.

NEVER INVENT A VALUE (OCS/CLAUDE.md). Every field is one of Graey's own streams, each with its own
age; a stream older than --max-age is not used.
  position      streams.gps, the Cube's GPS: valid, 3D fix, not 0,0.    Missing -> NO heartbeat.
  heading_deg   streams.vn_heading, the VN-100 Graey steers by.          Missing -> NO heartbeat.
  state         streams.heartbeat, the Cube's armed flag and mode.        Missing -> NO heartbeat.
                Armed in AUTO/GUIDED, or disarmed in GUIDED (waiting for RunStart -- Ekko's rule,
                heartbeat_core.py), is STATE_AUTO; anything else STATE_MANUAL. Graey's missions
                fly GUIDED (mission_base.py). Its kill is hardware and invisible here, so
                STATE_KILLED is never sent: a killed Graey reads MANUAL.
  roll/pitch    streams.attitude (the Cube), omitted when stale.
  spd_mps       the nav bridge's GPS ground speed, only when it is the same fix as the position.
  depth_m       the nav bridge's depth_m, omitted while Graey's own stack says null.
  current_task  TASK_NONE, literally: Graey has no task-state source (Ekko's reporter does the same).
sent_at is the laptop's clock at the observation (fetch midpoint minus the oldest age used), so the
OCS's staleness check measures the data, not this loop. The OCS re-stamps it before RoboCommand.

Downlink (RxCommand, ocs_info, ocs_directive, ocs_receipt) is printed and never acted on.
"""
from __future__ import annotations

import argparse
import json
import math
import socket
import struct
import threading
import time
import urllib.request
from collections import Counter
from datetime import datetime, timezone

HEADER = struct.Struct(">I")                 # rx_bridge/framing.py
VEHICLE_ID = "UUV1"
#: ArduSub custom_mode -> name (ArduSub mode.h)
MODES = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 7: "CIRCLE",
         9: "SURFACE", 16: "POSHOLD", 19: "MANUAL", 20: "MOTOR_DETECT", 21: "SURFTRAK"}
AUTONOMOUS_MODES = frozenset({"AUTO", "GUIDED"})      # armed
WAITING_AUTONOMOUS_MODES = frozenset({"GUIDED"})      # disarmed, waiting for RunStart


def say(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def rfc3339(t: float) -> str:
    return datetime.fromtimestamp(t, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


def finite(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def stream(snap: dict, name: str, max_age: float):
    """(data, age) of one of Graey's navigation streams; data is None when stale or absent."""
    s = (snap.get("streams") or {}).get(name) or {}
    age = s.get("age")
    if not s.get("fresh") or not finite(age) or age > max_age or not s.get("data"):
        return None, age
    return s["data"], age


def robot_state(mode: str | None, armed: bool) -> str:
    return "STATE_AUTO" if mode in (AUTONOMOUS_MODES if armed else WAITING_AUTONOMOUS_MODES) \
        else "STATE_MANUAL"


def build(snap: dict, max_age: float):
    """(heartbeat, oldest age used, mode, armed) or (None, why, None, None)."""
    beat, age = stream(snap, "heartbeat", max_age)
    if beat is None or not isinstance(beat.get("armed"), bool):
        return None, "no fresh autopilot HEARTBEAT (age %s)" % age, None, None
    ages = [age]
    mode, armed = MODES.get(beat.get("mode_id")), beat["armed"]

    gps, age = stream(snap, "gps", max_age)
    if gps is None:
        return None, "no fresh GPS (age %s)" % age, mode, armed
    lat, lon = gps.get("lat"), gps.get("lon")
    if (gps.get("valid") is not True or not finite(gps.get("fix")) or gps["fix"] < 3
            or not finite(lat) or not finite(lon) or (lat == 0 and lon == 0)):
        return None, "GPS not usable (valid %s, fix %s)" % (gps.get("valid"), gps.get("fix")), \
            mode, armed
    ages.append(age)

    hdg, age = stream(snap, "vn_heading", max_age)
    if hdg is None or not finite(hdg.get("degrees")):
        return None, "no fresh VN-100 heading (age %s)" % age, mode, armed
    ages.append(age)

    hb = {"state": robot_state(mode, armed),
          "position": {"latitude": lat, "longitude": lon},
          "heading_deg": hdg["degrees"] % 360.0,
          "vehicle_type": "TYPE_UUV",
          "current_task": "TASK_NONE"}

    att, age = stream(snap, "attitude", max_age)
    rpy = (att or {}).get("rpy")
    if isinstance(rpy, list) and len(rpy) >= 2 and finite(rpy[0]) and finite(rpy[1]):
        hb["roll_deg"], hb["pitch_deg"] = rpy[0], rpy[1]
        ages.append(age)

    nav, age = stream(snap, "bridge", max_age)
    if nav is not None:
        obs = nav.get("gps_observation") or {}
        if (finite(obs.get("speed_mps")) and obs.get("cube_fix_time_us") is not None
                and obs.get("cube_fix_time_us") == gps.get("receiver_time_us")):
            hb["spd_mps"] = obs["speed_mps"]
        if finite(nav.get("depth_m")):
            hb["depth_m"] = nav["depth_m"]
            ages.append(age)
    return hb, max(ages), mode, armed


class OcsLink:
    """One TCP connection to the bridge as UUV1: hello first, reports up, downlink printed."""

    def __init__(self, host: str, port: int, agent: str):
        self.addr = (host, port)
        self.hello = {"ocs_hello": {"vehicle_id": VEHICLE_ID, "link": "wifi", "agent": agent,
                                    "synthesized": ["heartbeat"]}}
        self.sock = None
        self.next_try = 0.0
        self.lock = threading.Lock()
        self.down = Counter()

    def _frame(self, obj) -> bytes:
        body = json.dumps(obj, allow_nan=False).encode("utf-8")
        return HEADER.pack(len(body)) + body

    def connected(self) -> bool:
        with self.lock:
            if self.sock is not None:
                return True
            if time.monotonic() < self.next_try:
                return False
            self.next_try = time.monotonic() + 2.0
            try:
                s = socket.create_connection(self.addr, timeout=3.0)
                s.settimeout(3.0)
                s.sendall(self._frame(self.hello))
            except OSError as e:
                say("OCS %s:%d not reachable: %s" % (self.addr + (e,)))
                return False
            self.sock = s
        say("connected to the OCS at %s:%d as %s (hello sent)" % (self.addr + (VEHICLE_ID,)))
        threading.Thread(target=self._read, args=(s,), daemon=True).start()
        return True

    def send(self, report: dict) -> bool:
        if not self.connected():
            return False
        try:
            self.sock.sendall(self._frame(report))
            return True
        except (OSError, AttributeError) as e:
            self._drop("send failed: %s" % e)
            return False

    def _drop(self, why: str, s=None):
        with self.lock:
            if self.sock is None or (s is not None and s is not self.sock):
                return
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        say("OCS link closed: %s -- reconnecting" % why)

    def _read(self, s):
        buf = b""
        while True:
            try:
                chunk = s.recv(65536)
            except socket.timeout:
                continue
            except OSError as e:
                return self._drop("recv: %s" % e, s)
            if not chunk:
                return self._drop("the OCS closed the socket", s)
            buf += chunk
            while len(buf) >= HEADER.size:
                (n,) = HEADER.unpack_from(buf)
                if len(buf) < HEADER.size + n:
                    break
                body, buf = buf[HEADER.size:HEADER.size + n], buf[HEADER.size + n:]
                self._downlink(body)

    def _downlink(self, body: bytes):
        try:
            d = json.loads(body)
        except ValueError:
            say("DOWNLINK undecodable frame of %d bytes" % len(body))
            return
        keys = [k for k in d if k not in ("team_id", "vehicle_id", "seq", "sent_at", "header")]
        kind = keys[0] if keys else "?"
        self.down[kind] += 1
        if kind == "ocs_info":
            info = d["ocs_info"] or {}
            say("DOWNLINK ocs_info: %s" % ", ".join(sorted(info))[:200])
        else:
            say("DOWNLINK %s: %s   (printed only -- not acted on)" % (kind, json.dumps(d)[:300]))


def fetch(url: str, timeout: float) -> dict:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return json.loads(r.read().decode("utf-8"))


def resolve(host: str) -> str:
    return socket.getaddrinfo(host, 8090, socket.AF_INET, socket.SOCK_STREAM)[0][4][0]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--graey", default="graey.local", help="Graey's host (GUI on :8090)")
    ap.add_argument("--ocs", default="127.0.0.1:37564", help="the bridge's vehicle link")
    ap.add_argument("--team", default="ASTA")
    ap.add_argument("--rate", type=float, default=2.0, help="heartbeats per second")
    ap.add_argument("--max-age", type=float, default=2.0, help="oldest usable stream, s")
    ap.add_argument("--once", action="store_true", help="print one heartbeat, connect nothing")
    args = ap.parse_args()

    ip = resolve(args.graey)
    url = "http://%s:8090/api/navigation" % ip
    say("Graey %s = %s; reading %s" % (args.graey, ip, url))
    if args.once:
        t0 = time.time()
        snap = fetch(url, 3.0)
        hb, oldest, mode, armed = build(snap, args.max_age)
        say("mode %s armed %s" % (mode, armed))
        if hb is None:
            say("NO heartbeat: %s" % oldest)
            return 1
        say("would send (oldest stream %.2f s):\n%s" % (oldest, json.dumps(hb, indent=1)))
        return 0

    host, port = args.ocs.rsplit(":", 1)
    link = OcsLink(host, int(port), "graey_ocs_client on the laptop, from Graey's GUI :8090")
    period, sent, skipped, fails = 1.0 / args.rate, 0, Counter(), 0
    last_hb, last_status, next_t = None, 0.0, time.monotonic()
    while True:
        next_t += period
        t0 = time.time()
        try:
            snap = fetch(url, 1.5)
            fails = 0
        except Exception as e:  # noqa: BLE001 -- URLError, timeout, IncompleteRead, bad JSON
            snap, fails = None, fails + 1
            skipped["Graey GUI unreachable: %s" % type(e).__name__] += 1
            if fails % 10 == 0:
                try:
                    ip = resolve(args.graey)
                    url = "http://%s:8090/api/navigation" % ip
                except OSError:
                    pass
        t1 = time.time()
        if snap is not None:
            hb, oldest, mode, armed = build(snap, args.max_age)
            if hb is None:
                skipped[oldest] += 1
            else:
                report = {"team_id": args.team, "vehicle_id": VEHICLE_ID,
                          "sent_at": rfc3339((t0 + t1) / 2.0 - oldest), "heartbeat": hb}
                if link.send(report):
                    if sent == 0:
                        say("first heartbeat:\n%s" % json.dumps(report, indent=1))
                    sent += 1
                    last_hb = (hb, oldest, mode, armed)
                else:
                    skipped["OCS not connected"] += 1
        if time.monotonic() - last_status >= 5.0:
            last_status = time.monotonic()
            line = "sent %d" % sent
            if last_hb:
                hb, oldest, mode, armed = last_hb
                line += " | last: %s (%s, %s) %.6f,%.6f hdg %.1f%s, oldest %.2f s" % (
                    hb["state"], mode, "armed" if armed else "disarmed",
                    hb["position"]["latitude"], hb["position"]["longitude"], hb["heading_deg"],
                    "" if "depth_m" not in hb else " depth %.2f" % hb["depth_m"], oldest)
            if skipped:
                line += " | skipped: " + "; ".join("%s x%d" % kv for kv in skipped.most_common(3))
            if link.down:
                line += " | downlink: " + ", ".join("%s %d" % kv for kv in link.down.items())
            say(line)
            skipped.clear()
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_t = time.monotonic()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        say("stopped")
