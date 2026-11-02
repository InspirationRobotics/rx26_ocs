"""rc_watch -- what reached RoboCommand, live, on the test box's screen.

Runs on the RoboCommand stand-in (the Linux box at 192.168.65.2), beside RoboNation's own
test server, and puts everything that crosses its broker on one page, port 8088:

  * heartbeats as ONE live line per vehicle -- at 2 Hz each they bury everything else in
    the test server's log, which is why task reports were hard to see;
  * a card per task report, declaration and command, task reports loudest;
  * the test server's own verdicts, read from its log: a refused RunDeclaration gets NO
    reply on MQTT, so its log is the only place a refusal ever shows;
  * the test being run right now: what is being done, what should happen, PASS or FAIL.

    server   python rc_watch.py --broker 127.0.0.1 --http 8088 --logs /app/logs
    client   python rc_watch.py post /api/step < step.json      (on the box itself)
    relay    python rc_watch.py relay      a JSON line {"path", "body"} per request on stdin,
                                           one reply line out: rc_e2e.py's one ssh session

The server half needs paho-mqtt 2.x, protobuf 7.x and RoboNation's generated code on
PYTHONPATH -- exactly what the rc-test image holds, so it runs in a container built from
that image (rc_watch.compose.yml, copied to the box as docker-compose.override.yml). The
client half is standard library only. Tests are driven from outside: POST /api/plan, /step,
/result and /note, accepted from 127.0.0.1 only, so only someone on the box can drive it.
"""
from __future__ import annotations

import argparse
import collections
import glob
import json
import os
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

TOPIC_ROOT = "robocommand/robotx"
EVENTS_MAX = 400
HB_WINDOW_S = 10.0
#: Test-server log lines worth a card. Everything else it logs is the decoded message
#: itself, which the MQTT side already shows better.
SERVER_KEYS = ("[ERROR]", "[geofence]", "[auto-start]", "Published RxCourse")


# ---- matching a test's expectations ------------------------------------------------------

def matches(spec: dict, ev: dict) -> bool:
    """`spec` is {"type": "report", "kind": "docking", "vehicle": "USV1", ...}: every key must
    agree with the event. `contains` is a substring of the event's text, `min_hz` a floor on
    a heartbeat's rate."""
    for key, want in spec.items():
        if key == "contains":
            if str(want).lower() not in ev.get("text", "").lower():
                return False
        elif key == "min_hz":
            if float(ev.get("hz") or 0.0) < float(want):
                return False
        elif str(ev.get(key, "")).lower() != str(want).lower():
            return False
    return True


def describe(spec: dict) -> str:
    return " ".join("%s=%s" % kv for kv in spec.items())


# ---- turning messages into rows ------------------------------------------------------------

def latlng(d) -> str:
    return "%.6f, %.6f" % (d.get("latitude", 0.0), d.get("longitude", 0.0))


def flatten(obj, prefix="") -> list:
    """[(key, text)] for a decoded message body. LatLng collapses to "lat, lon"; a list of
    LatLng to its count, because forty coordinates on a card say nothing."""
    out = []
    if isinstance(obj, dict) and set(obj) <= {"latitude", "longitude"} and obj:
        return [(prefix or "position", latlng(obj))]
    if isinstance(obj, dict):
        for k, v in obj.items():
            out.extend(flatten(v, "%s.%s" % (prefix, k) if prefix else k))
        return out
    if isinstance(obj, list):
        if obj and all(isinstance(v, dict) and set(v) <= {"latitude", "longitude"} for v in obj):
            return [(prefix, "%d points" % len(obj))]
        if all(not isinstance(v, (dict, list)) for v in obj):
            return [(prefix, ", ".join(str(v) for v in obj))]
        for i, v in enumerate(obj):
            out.extend(flatten(v, "%s[%d]" % (prefix, i)))
        return out
    return [(prefix, str(obj))]


def points(lst) -> list:
    return [[p.get("latitude", 0.0), p.get("longitude", 0.0)] for p in lst or []]


class Watch:
    """All state behind one lock: the MQTT, log and ticker threads write, HTTP threads read."""

    def __init__(self, record_dir: str) -> None:
        self.lock = threading.Lock()
        self.events = collections.deque(maxlen=EVENTS_MAX)
        self.next_id = 1
        self.course = None
        self.course_key = None
        self.heartbeats = {}
        self.hb_times = collections.defaultdict(collections.deque)
        self.fence = None           # the last declared uav_geofence, for the map
        self.ceiling = None
        self.alerts = {}            # "moving_object_alert" / "keep_out_zone" -> map circle
        self.tests = collections.OrderedDict()
        self.current = None
        self.broker = {"connected": False, "host": "", "since": None}
        self.server_log = None
        self.boot = time.time()     # a page that sees this change reloads: new ids, new page
        self.record = None
        if record_dir:
            os.makedirs(record_dir, exist_ok=True)
            self.record = os.path.join(record_dir, time.strftime("events-%Y%m%d.jsonl"))

    # ---- events ---------------------------------------------------------------------

    def _add(self, ev: dict) -> dict:
        """Caller holds the lock."""
        ev["id"] = self.next_id
        ev.setdefault("at", time.time())
        ev.setdefault("text", " ".join([ev.get("title", "")] +
                                       ["%s=%s" % kv for kv in ev.get("fields", [])]))
        self.next_id += 1
        self.events.append(ev)
        if self.record:
            try:
                with open(self.record, "a", encoding="utf-8") as fh:
                    fh.write(json.dumps(ev, default=str) + "\n")
            except OSError:
                pass
        self._evaluate(ev)
        return ev

    def mqtt_state(self, connected: bool, host: str) -> None:
        with self.lock:
            if connected != self.broker["connected"]:
                self.broker = {"connected": connected, "host": host, "since": time.time()}
                self._add({"type": "server", "kind": "broker",
                           "title": ("watching %s/# on %s" if connected else
                                     "LOST the broker at %s") % ((TOPIC_ROOT, host) if connected
                                                                 else (host,))})

    def message(self, topic: str, kind_of_topic: str, team: str, vehicle: str,
                body: str, decoded: dict) -> None:
        with self.lock:
            if kind_of_topic == "course":
                self._course(decoded)
            elif kind_of_topic == "report" and body == "heartbeat":
                self._heartbeat(team, vehicle, decoded)
            elif kind_of_topic == "report":
                self._add({"type": "report", "kind": body, "team": team,
                           "vehicle": decoded.get("vehicle_id", vehicle),
                           "seq": decoded.get("seq"), "title": "TASK REPORT · " + body,
                           "fields": flatten(decoded.get(body, {}))})
            elif kind_of_topic == "request":
                self._request(team, body, decoded)
            elif kind_of_topic == "command":
                self._command(team, body, decoded)
            else:
                self._add({"type": "other", "title": topic, "fields": []})

    def _course(self, d: dict) -> None:
        key = json.dumps({k: v for k, v in d.items() if k != "sent_at"}, sort_keys=True)
        self.course = d
        if key == self.course_key:
            return                      # a re-delivery of the retained course is not news
        self.course_key = key
        self._add({"type": "course", "kind": "course", "title": "RxCourse " + d.get("course_id", "?"),
                   "fields": [("corners", "%d points" % len(d.get("corners", []))),
                              ("uav_transit_area", "%d points" % len(d.get("uav_transit_area", []))
                               if d.get("uav_transit_area") else "none"),
                              ("pinger_freq_hz", str(d.get("pinger_freq_hz", "")))]})

    def _heartbeat(self, team: str, vehicle: str, d: dict) -> None:
        hb = d.get("heartbeat", {})
        vid = d.get("vehicle_id", vehicle)
        now = time.time()
        times = self.hb_times[(team, vid)]
        times.append(now)
        while times and now - times[0] > HB_WINDOW_S:
            times.popleft()
        hz = round(len(times) / HB_WINDOW_S, 1)
        pos = hb.get("position", {})
        row = {"team": team, "vehicle": vid, "state": hb.get("state", ""),
               "task": hb.get("current_task", ""), "type": hb.get("vehicle_type", ""),
               "lat": pos.get("latitude"), "lon": pos.get("longitude"),
               "spd": hb.get("spd_mps"), "hdg": hb.get("heading_deg"),
               "alt": hb.get("altitude_hae_m"), "phase": hb.get("flight_phase", ""),
               "reg": hb.get("uav_registration", ""), "seq": d.get("seq"), "last": now, "hz": hz}
        first = (team, vid) not in self.heartbeats
        self.heartbeats[(team, vid)] = row
        if first:
            self._add({"type": "server", "kind": "first_heartbeat", "team": team, "vehicle": vid,
                       "title": "first heartbeat from %s/%s" % (team, vid),
                       "fields": [("state", row["state"]), ("task", row["task"])]})
        # Heartbeats are matched against the running test but never become cards.
        self._evaluate({"type": "hb", "team": team, "vehicle": vid, "state": row["state"],
                        "task": row["task"], "reg": row["reg"], "hz": hz,
                        "text": "%s %s %s %s" % (vid, row["state"], row["task"], row["reg"])})

    def _request(self, team: str, body: str, d: dict) -> None:
        fields = []
        if body == "run_declaration":
            rd = d.get("run_declaration", {})
            self.fence = points(rd.get("uav_geofence"))
            self.ceiling = rd.get("uav_geofence_max_altitude_hae_m")
            fields = [("vehicles", ", ".join(rd.get("vehicle_ids", []))),
                      ("tiers", " ".join(rd.get("task%d_tier" % i, "-").replace("TIER_", "")
                                         for i in range(1, 5))),
                      ("uav_geofence", "%d points" % len(self.fence) if self.fence else "none"),
                      ("ceiling_hae_m", "%s" % self.ceiling if self.ceiling is not None else "NOT SENT"),
                      ("pilot_callsign", rd.get("uav_pilot_callsign") or "NOT SENT")]
        else:
            fields = flatten(d.get(body, {}))
        self._add({"type": "request", "kind": body, "team": team, "seq": d.get("seq"),
                   "title": "REQUEST · " + body, "fields": fields})

    def _command(self, team: str, body: str, d: dict) -> None:
        inner = d.get(body, {})
        fields = flatten(inner)
        if body == "moving_object_alert":
            r = inner.get("keep_out_radius_m")
            self.alerts[body] = {"at": points([inner.get("position", {})])[0],
                                 "r": r, "label": "moving object %s" % ("%g m" % r if r else "radius NOT SENT")}
            if r is None:
                fields.append(("keep_out_radius_m", "NOT SENT"))
        elif body == "keep_out_zone":
            self.alerts[body] = {"at": points([inner.get("center", {})])[0],
                                 "r": inner.get("radius_m"), "label": "keep-out zone"}
        elif body == "all_clear":
            self.alerts.pop("keep_out_zone", None)
        self._add({"type": "command", "kind": body, "team": team,
                   "vehicle": d.get("vehicle_id", ""), "seq": d.get("seq"),
                   "title": "COMMAND · " + body, "fields": fields})

    def server_line(self, line: str) -> None:
        msg = line.split(" ", 3)[-1]                  # drop "date time LEVEL"
        low = msg.lower()
        if "refused" in low:
            kind, title = "refused", "RoboCommand REFUSED the declaration"
        elif "[error]" in low:
            kind, title = "error", "RoboCommand could not read a message"
        elif "[geofence]" in low:
            kind, title = "geofence_ok", "RoboCommand: geofence and ceiling OK"
        elif "publishing runstart" in low:
            kind, title = "run_start", "RoboCommand: all declared vehicles AUTO, RunStart sent"
        elif "[auto-start]" in low:
            kind, title = "tracking", "RoboCommand: declaration accepted, waiting for AUTO"
        else:
            kind, title = "course", "RoboCommand: course published"
        with self.lock:
            self._add({"type": "refused" if kind == "refused" else "server", "kind": kind,
                       "title": title, "detail": msg, "fields": [], "text": title + " " + msg})

    # ---- tests ----------------------------------------------------------------------

    def plan(self, req: dict) -> dict:
        with self.lock:
            for t in req.get("tests", []):
                self.tests.setdefault(t["id"], {"id": t["id"], "title": t.get("title", ""),
                                                "status": "pending", "note": ""})
            return {"ok": True, "tests": len(self.tests)}

    def step(self, req: dict) -> dict:
        with self.lock:
            prev = self.tests.get(self.current)
            if prev and prev["status"] == "running":
                self._finish(prev, "stopped", "superseded by %s" % req["id"])
            t = self.tests.setdefault(req["id"], {"id": req["id"]})
            t.update({"title": req.get("title", t.get("title", "")), "doing": req.get("doing", ""),
                      "expect": req.get("expect", ""), "match": req.get("match", []),
                      "absent": req.get("absent", []), "seen": [], "status": "running", "note": "",
                      "started": time.time(),
                      "deadline": time.time() + float(req.get("timeout_s", 20))})
            self.current = t["id"]
            self._add({"type": "step", "kind": "start", "title": "▶ %s  %s" % (t["id"], t["title"]),
                       "fields": [("doing", t["doing"]), ("expect", t["expect"])]})
            return {"ok": True}

    def result(self, req: dict) -> dict:
        with self.lock:
            t = self.tests.get(req["id"])
            if t is None:
                return {"ok": False, "error": "no test %s" % req["id"]}
            self._finish(t, req.get("status", "info"), req.get("note", ""))
            return {"ok": True}

    def note(self, req: dict) -> dict:
        with self.lock:
            self._add({"type": "note", "kind": req.get("level", "info"),
                       "title": req.get("text", ""), "fields": []})
            return {"ok": True}

    def _finish(self, t: dict, status: str, note: str) -> None:
        t["status"], t["note"], t["ended"] = status, note, time.time()
        mark = {"pass": "✔", "fail": "✖"}.get(status, "■")
        self._add({"type": "result", "kind": status,
                   "title": "%s %s %s" % (mark, t["id"], status.upper()),
                   "fields": [("", note)] if note else []})

    def _evaluate(self, ev: dict) -> None:
        """Caller holds the lock. Results are events too, so they are never judged."""
        t = self.tests.get(self.current)
        if not t or t.get("status") != "running" or ev["type"] in ("step", "result", "note"):
            return
        for spec in t["absent"]:
            if matches(spec, ev):
                self._finish(t, "fail", "saw what must not happen: %s" % describe(spec))
                return
        for i, spec in enumerate(t["match"]):
            if i not in t["seen"] and matches(spec, ev):
                t["seen"].append(i)
        # With nothing that must stay absent, the last expected sighting is the verdict.
        # Otherwise absence can only be judged when the time is up (tick).
        if t["match"] and len(t["seen"]) == len(t["match"]) and not t["absent"]:
            self._finish(t, "pass", "saw everything expected")

    def tick(self) -> None:
        with self.lock:
            t = self.tests.get(self.current)
            if not t or t.get("status") != "running" or time.time() < t["deadline"]:
                return
            missing = [describe(s) for i, s in enumerate(t["match"]) if i not in t["seen"]]
            if missing:
                self._finish(t, "fail", "timed out without: " + "; ".join(missing))
            else:
                self._finish(t, "pass", "everything expected, nothing forbidden")

    # ---- the page's view ------------------------------------------------------------

    def snapshot(self, since: int) -> dict:
        with self.lock:
            now = time.time()
            hbs = []
            for row in self.heartbeats.values():
                r = dict(row)
                r["age_s"] = round(now - row["last"], 1)
                times = self.hb_times[(row["team"], row["vehicle"])]
                while times and now - times[0] > HB_WINDOW_S:
                    times.popleft()
                r["hz"] = round(len(times) / HB_WINDOW_S, 1)
                hbs.append(r)
            tests = [{k: v for k, v in t.items() if k in ("id", "title", "status", "note", "doing",
                                                          "expect", "started", "deadline")}
                     for t in self.tests.values()]
            course = None
            if self.course:
                course = {"id": self.course.get("course_id", ""),
                          "corners": points(self.course.get("corners")),
                          "transit": points(self.course.get("uav_transit_area")),
                          "pinger": self.course.get("pinger_freq_hz")}
            return {"now": now, "boot": self.boot, "broker": self.broker, "server_log": self.server_log,
                    "course": course, "fence": self.fence, "ceiling": self.ceiling,
                    "alerts": list(self.alerts.values()), "heartbeats": hbs, "tests": tests,
                    "current": self.current, "last_id": self.next_id - 1,
                    "events": [e for e in self.events if e["id"] > since]}


# ---- the threads ---------------------------------------------------------------------------

def run_mqtt(watch: Watch, host: str, port: int) -> None:
    import paho.mqtt.client as mqtt
    from google.protobuf import json_format
    from robotx import rx_commands_pb2, rx_course_pb2, rx_reports_pb2, rx_requests_pb2

    classes = {"report": rx_reports_pb2.RxReport, "request": rx_requests_pb2.RxRequest,
               "command": rx_commands_pb2.RxCommand, "course": rx_course_pb2.RxCourse}

    def on_connect(client, _ud, _flags, reason, _props=None):
        if reason == 0:
            client.subscribe(TOPIC_ROOT + "/#", 0)
            watch.mqtt_state(True, "%s:%d" % (host, port))

    def on_disconnect(_client, _ud, _flags, _reason, _props=None):
        watch.mqtt_state(False, "%s:%d" % (host, port))

    def on_message(_client, _ud, msg):
        parts = msg.topic.split("/")
        tail = parts[-1]
        team = parts[2] if len(parts) >= 4 else ""
        vehicle = parts[3] if len(parts) >= 5 else ""
        cls = classes.get(tail)
        if cls is None:
            watch.message(msg.topic, "other", team, vehicle, "", {})
            return
        m = cls()
        try:
            m.ParseFromString(msg.payload)
        except Exception as exc:  # noqa: BLE001 - a bad frame is a card, not a crash
            with watch.lock:
                watch._add({"type": "refused", "kind": "undecodable",
                            "title": "undecodable %s on %s" % (tail, msg.topic),
                            "detail": str(exc), "fields": []})
            return
        body = "course" if tail == "course" else (m.WhichOneof("body") or "")
        decoded = json_format.MessageToDict(m, preserving_proto_field_name=True)
        watch.message(msg.topic, tail, team, vehicle, body, decoded)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="rc-watch-%d" % os.getpid())
    client.on_connect, client.on_disconnect, client.on_message = on_connect, on_disconnect, on_message
    client.reconnect_delay_set(1, 5)
    while True:
        try:
            client.connect(host, port, keepalive=15)
            break
        except OSError:
            watch.mqtt_state(False, "%s:%d" % (host, port))
            time.sleep(2)
    client.loop_forever(retry_first_connection=True)


def run_log_tail(watch: Watch, logs_dir: str) -> None:
    """Follow the test server's newest log from its start, skipping lines stamped before we
    started: history is not news, but a server started alongside us is. The stamps compare
    as text because the watcher runs in the server's own image, so both clocks are UTC."""
    since = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time() - 2))
    path, fh = None, None
    while True:
        files = glob.glob(os.path.join(logs_dir, "robocommand_*.log"))
        newest = max(files, key=os.path.getmtime) if files else None
        if newest and newest != path:
            if fh:
                fh.close()
            path, fh = newest, open(newest, encoding="utf-8", errors="replace")
            with watch.lock:
                watch.server_log = os.path.basename(path)
        line = fh.readline() if fh else ""
        if not line:
            time.sleep(0.3)
            continue
        if line[:4].isdigit() and line[:19] >= since and any(k in line for k in SERVER_KEYS):
            watch.server_line(line.rstrip())


def run_ticker(watch: Watch) -> None:
    while True:
        time.sleep(0.5)
        watch.tick()


def serve(watch: Watch, port: int) -> None:
    routes = {"/api/plan": watch.plan, "/api/step": watch.step,
              "/api/result": watch.result, "/api/note": watch.note}

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body, ctype):
            data = body.encode("utf-8") if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == "/":
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif url.path == "/api/state":
                since = int((parse_qs(url.query).get("since") or ["0"])[0] or 0)
                self._send(200, json.dumps(watch.snapshot(since), default=str), "application/json")
            else:
                self._send(404, "not found", "text/plain")

        def do_POST(self):
            if self.client_address[0] != "127.0.0.1":
                return self._send(403, "tests are driven from the box itself", "text/plain")
            fn = routes.get(urlparse(self.path).path)
            if fn is None:
                return self._send(404, "not found", "text/plain")
            try:
                n = int(self.headers.get("Content-Length") or 0)
                out = fn(json.loads(self.rfile.read(n) or b"{}"))
            except (ValueError, KeyError) as exc:
                return self._send(400, json.dumps({"ok": False, "error": str(exc)}), "application/json")
            self._send(200, json.dumps(out), "application/json")

        def log_message(self, *_args):
            pass

    ThreadingHTTPServer(("0.0.0.0", port), Handler).serve_forever()


def post(path: str, port: int) -> int:
    """Client half: JSON on stdin -> POST to the local watcher. Standard library only."""
    data = sys.stdin.buffer.read() or b"{}"
    req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path), data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as resp:
        print(resp.read().decode("utf-8"))
    return 0


def relay(port: int) -> int:
    """`post` for a driver on another machine: many requests over one ssh session, so a
    test step costs no handshake. A reply line per request line, an error as {"ok": false}."""
    for line in sys.stdin:
        line = line.lstrip("﻿")            # Windows PowerShell 5.1 pipes a BOM
        if not line.strip():
            continue
        try:
            msg = json.loads(line)
            req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, msg["path"]),
                                         data=json.dumps(msg.get("body") or {}).encode("utf-8"),
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=5) as resp:
                out = resp.read().decode("utf-8").strip()
        except (ValueError, KeyError, OSError) as exc:
            out = json.dumps({"ok": False, "error": str(exc)})
        print(out, flush=True)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="rc_watch")
    ap.add_argument("--broker", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--http", type=int, default=8088)
    ap.add_argument("--logs", default="/app/logs", help="the test server's LOG_DIR")
    ap.add_argument("--record", default="/app/watch", help="where events-YYYYMMDD.jsonl goes")
    ap.add_argument("cmd", nargs="*", help="'post /api/step' etc. reads JSON on stdin")
    args = ap.parse_args(argv)
    if args.cmd:
        if args.cmd == ["relay"]:
            return relay(args.http)
        if args.cmd[0] != "post" or len(args.cmd) != 2:
            ap.error("client usage: rc_watch.py post /api/step < step.json, or rc_watch.py relay")
        return post(args.cmd[1], args.http)

    watch = Watch(args.record)
    for target, extra in ((run_mqtt, (args.broker, args.port)), (run_log_tail, (args.logs,)),
                          (run_ticker, ())):
        threading.Thread(target=target, args=(watch,) + extra, daemon=True).start()
    print("rc_watch: http://0.0.0.0:%d  broker %s:%d  logs %s" % (args.http, args.broker,
                                                                  args.port, args.logs), flush=True)
    serve(watch, args.http)
    return 0


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RoboCommand test box</title>
<style>
:root{--bg:#f3f4f7;--card:#fff;--ink:#15171c;--muted:#5b6270;--line:#dde1e8;
 --blue:#1d5bbf;--bluebg:#e7effb;--green:#1b7a3a;--greenbg:#e3f3e8;--amber:#9a5700;--amberbg:#fff1cf;
 --red:#b3261e;--redbg:#fde7e5;--purple:#6a3d9e;--grey:#6b7280;--greybg:#eef0f3}
@media (prefers-color-scheme: dark){:root{--bg:#101217;--card:#191c23;--ink:#e9ebef;--muted:#9aa2b1;
 --line:#2a2f39;--blue:#80b1ff;--bluebg:#16253d;--green:#62d38c;--greenbg:#12301f;--amber:#ffbf57;
 --amberbg:#3a2b0f;--red:#ff7d74;--redbg:#3b1613;--purple:#c5a5f6;--grey:#a1a8b5;--greybg:#22262e}}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.4 system-ui,"Segoe UI",Ubuntu,sans-serif}
header{display:flex;align-items:center;gap:12px;flex-wrap:wrap;padding:10px 16px;border-bottom:1px solid var(--line);background:var(--card)}
h1{font-size:18px;margin:0 8px 0 0}
.chip{border-radius:999px;padding:2px 10px;font-size:13px;background:var(--greybg);color:var(--muted)}
.chip.ok{background:var(--greenbg);color:var(--green)}.chip.bad{background:var(--redbg);color:var(--red)}
#clock{margin-left:auto;font-variant-numeric:tabular-nums;color:var(--muted)}
#now{margin:10px 16px;padding:10px 16px;border-radius:12px;background:var(--card);border:3px solid var(--line)}
#now.running{border-color:var(--amber)}#now.pass{border-color:var(--green)}#now.fail{border-color:var(--red)}
#now .label{font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--muted)}
#now .title{font-size:26px;font-weight:700;margin:0 0 4px}
#now .row{display:flex;gap:10px;margin-top:2px;font-size:17px}#now .row b{min-width:70px;color:var(--muted);font-weight:600}
.pill{display:inline-block;border-radius:8px;padding:2px 10px;font-weight:700;font-size:14px}
.pill.running{background:var(--amberbg);color:var(--amber)}.pill.pass{background:var(--greenbg);color:var(--green)}
.pill.fail{background:var(--redbg);color:var(--red)}.pill.pending,.pill.stopped,.pill.info{background:var(--greybg);color:var(--grey)}
main{display:grid;grid-template-columns:250px minmax(340px,1fr) minmax(400px,1.2fr);gap:12px;padding:0 16px 16px}
@media (max-width:1300px){main{grid-template-columns:minmax(300px,1fr) minmax(340px,1.1fr)}
 #c-tests,#c-mid{grid-column:1}#c-ev{grid-column:2;grid-row:1/span 2}}
@media (max-width:760px){main{grid-template-columns:1fr}#c-ev{grid-column:auto;grid-row:auto}}
#map svg{max-height:36vh}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px 14px;min-width:0}
h2{font-size:13px;letter-spacing:.06em;text-transform:uppercase;color:var(--muted);margin:0 0 8px}
.test{display:flex;gap:8px;align-items:flex-start;padding:6px 0;border-top:1px solid var(--line)}
.test:first-of-type{border-top:0}.test .id{font-weight:700;min-width:34px}.test .note{color:var(--muted);font-size:12.5px}
table{width:100%;border-collapse:collapse;font-size:14px}th,td{text-align:left;padding:4px 6px;border-top:1px solid var(--line);white-space:nowrap}
th{color:var(--muted);font-weight:600;font-size:12px;border-top:0}
td.num{font-variant-numeric:tabular-nums}.stale{color:var(--red);font-weight:700}
svg{width:100%;height:auto;display:block;margin-top:8px}
#events{display:flex;flex-direction:column;gap:8px;max-height:calc(100vh - 220px);overflow:auto}
.ev{border-radius:10px;padding:8px 12px;border-left:6px solid var(--grey);background:var(--greybg)}
.ev .h{display:flex;gap:10px;align-items:baseline}.ev .t{font-weight:700}.ev .when{margin-left:auto;color:var(--muted);font-size:12.5px;font-variant-numeric:tabular-nums}
.ev .f{font-family:ui-monospace,Consolas,monospace;font-size:13px;margin-top:3px;word-break:break-word}
.ev.report{border-left-color:var(--amber);background:var(--amberbg)}.ev.report .t{font-size:20px;color:var(--amber)}
.ev.request{border-left-color:var(--blue);background:var(--bluebg)}.ev.command{border-left-color:var(--green);background:var(--greenbg)}
.ev.refused{border-left-color:var(--red);background:var(--redbg)}.ev.refused .t{color:var(--red);font-size:18px}
.ev.course{border-left-color:var(--purple)}.ev.step{background:var(--card);border:1px solid var(--line);border-left:6px solid var(--ink)}
.ev.result.pass{border-left-color:var(--green);background:var(--greenbg)}.ev.result.fail{border-left-color:var(--red);background:var(--redbg)}
.ev.note{background:var(--card);border:1px dashed var(--line);border-left:6px dashed var(--grey)}
button{font:inherit;border:1px solid var(--line);background:var(--card);color:var(--ink);border-radius:8px;padding:3px 10px;cursor:pointer}
.muted{color:var(--muted)}
</style></head><body>
<header><h1>RoboCommand test box</h1><span id="broker" class="chip">broker …</span>
<span id="slog" class="chip">server log …</span><button id="sound">🔔 sound off</button><button id="fs">⛶ full screen</button><span id="clock"></span></header>
<section id="now"><div class="label">Now testing</div><div class="title" id="n-title">Waiting for the first test</div>
<div class="row"><b>Doing</b><span id="n-doing" class="muted">—</span></div>
<div class="row"><b>Expect</b><span id="n-expect" class="muted">—</span></div>
<div class="row"><b>Status</b><span id="n-status"></span><span id="n-note" class="muted"></span></div></section>
<main>
<div class="card" id="c-tests"><h2>Tests</h2><div id="tests" class="muted">none yet</div></div>
<div id="c-mid">
 <div class="card"><h2>Heartbeats · one live line per vehicle</h2><div id="hbs" class="muted">none yet</div></div>
 <div class="card" style="margin-top:12px"><h2>Course · transit area · declared UAV fence</h2><div id="course" class="muted">no RxCourse yet</div><div id="map"></div></div>
</div>
<div class="card" id="c-ev"><h2>Everything else that reached RoboCommand · newest first</h2><div id="events"></div></div>
</main>
<script>
const $=id=>document.getElementById(id);
const esc=s=>String(s??"").replace(/[&<>"]/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
let since=0, sound=false, ctx=null, shown=0, boot=null;
$("sound").onclick=()=>{sound=!sound; $("sound").textContent=sound?"🔔 sound on":"🔔 sound off"; if(sound&&!ctx) ctx=new AudioContext();};
$("fs").onclick=()=>document.fullscreenElement?document.exitFullscreen():document.documentElement.requestFullscreen();
function beep(){ if(!sound||!ctx) return; const o=ctx.createOscillator(), g=ctx.createGain();
  o.frequency.value=880; g.gain.value=0.15; o.connect(g); g.connect(ctx.destination); o.start(); o.stop(ctx.currentTime+0.25); }
const hhmmss=t=>new Date(t*1000).toLocaleTimeString([], {hour12:false});
function chip(id, text, cls){ const e=$(id); e.textContent=text; e.className="chip "+(cls||""); }
function render(s){
  if(boot!==null&&s.boot!==boot){ location.reload(); return; }
  boot=s.boot; $("clock").textContent=hhmmss(s.now);
  chip("broker", s.broker.connected?"broker "+s.broker.host:"NO BROKER", s.broker.connected?"ok":"bad");
  chip("slog", s.server_log?"server log "+s.server_log:"no server log", s.server_log?"ok":"bad");
  const cur=s.tests.find(t=>t.id===s.current);
  const now=$("now");
  if(cur){ now.className=cur.status; $("n-title").textContent=cur.id+" · "+cur.title;
    $("n-doing").textContent=cur.doing||"—"; $("n-expect").textContent=cur.expect||"—";
    const left=cur.status==="running"?Math.max(0,Math.round(cur.deadline-s.now))+" s left":"";
    $("n-status").innerHTML=`<span class="pill ${cur.status}">${cur.status.toUpperCase()}</span> <span class="muted">${left}</span>`;
    $("n-note").textContent=cur.note||""; }
  $("tests").innerHTML=s.tests.length?s.tests.map(t=>`<div class="test"><span class="id">${esc(t.id)}</span>
    <div><div>${esc(t.title)}</div><div><span class="pill ${t.status}">${t.status}</span> <span class="note">${esc(t.note)}</span></div></div></div>`).join(""):"none yet";
  $("hbs").innerHTML=s.heartbeats.length?"<table><tr><th>vehicle</th><th>Hz</th><th>state</th><th>task</th><th>position</th><th>reg</th><th>age</th></tr>"+
    s.heartbeats.map(h=>`<tr><td><b>${esc(h.team)}/${esc(h.vehicle)}</b></td><td class="num">${h.hz.toFixed(1)}</td>
    <td>${esc(h.state.replace("STATE_",""))}</td><td>${esc(h.task.replace("TASK_",""))}</td>
    <td class="num">${h.lat!=null?h.lat.toFixed(6)+", "+h.lon.toFixed(6):"—"}</td><td>${esc(h.reg||"—")}</td>
    <td class="num ${h.age_s>3?"stale":""}">${h.age_s.toFixed(1)} s</td></tr>`).join("")+"</table>":"none yet — nothing reaches RoboCommand before a declaration";
  if(s.course){ $("course").innerHTML=`<b>${esc(s.course.id)}</b> · ${s.course.corners.length} corners · transit area ${s.course.transit.length?s.course.transit.length+" points":"none"} · pinger ${esc(s.course.pinger)} Hz`+
    (s.fence?` · declared fence ${s.fence.length} pts, ceiling ${s.ceiling??"NOT SENT"} m HAE`:""); drawMap(s); }
  const fresh=s.events.filter(e=>e.id>since);
  if(fresh.length){ since=s.last_id; const box=$("events");
    for(const e of fresh){ if(e.type==="report") beep(); box.insertAdjacentHTML("afterbegin", card(e)); shown++; }
    while(box.children.length>250) box.lastChild.remove(); }
}
function card(e){
  const who=[e.team&&e.vehicle?e.team+"/"+e.vehicle:(e.vehicle||e.team||""), e.seq!=null?"seq "+e.seq:""].filter(Boolean).join(" · ");
  const fields=(e.fields||[]).map(([k,v])=>k?`${esc(k)} = ${esc(v)}`:esc(v)).join("<br>");
  return `<div class="ev ${e.type} ${e.type==="result"?e.kind:""}"><div class="h"><span class="t">${esc(e.title)}</span><span class="muted">${esc(who)}</span><span class="when">${hhmmss(e.at)}</span></div>`+
    (fields?`<div class="f">${fields}</div>`:"")+(e.detail?`<div class="f muted">${esc(e.detail)}</div>`:"")+"</div>";
}
function drawMap(s){
  const all=[...s.course.corners, ...s.course.transit, ...(s.fence||[])];
  for(const h of s.heartbeats) if(h.lat!=null) all.push([h.lat,h.lon]);
  for(const a of s.alerts) if(a.at) all.push(a.at);
  if(!all.length){ $("map").innerHTML=""; return; }
  const lat0=all.reduce((a,p)=>a+p[0],0)/all.length, lon0=all.reduce((a,p)=>a+p[1],0)/all.length;
  const k=Math.PI/180*6371000, xy=p=>[(p[1]-lon0)*k*Math.cos(lat0*Math.PI/180), (p[0]-lat0)*k];
  const P=all.map(xy), xs=P.map(p=>p[0]), ys=P.map(p=>p[1]);
  let pad=8; for(const a of s.alerts) if(a.r) pad=Math.max(pad,a.r+4);
  const minx=Math.min(...xs)-pad, maxx=Math.max(...xs)+pad, miny=Math.min(...ys)-pad, maxy=Math.max(...ys)+pad;
  const W=520, H=Math.max(200, Math.min(420, W*(maxy-miny)/(maxx-minx))), sc=Math.min(W/(maxx-minx), H/(maxy-miny));
  const tx=p=>{const q=xy(p); return [(q[0]-minx)*sc, H-(q[1]-miny)*sc];};
  const poly=(pts,st)=>pts.length?`<polygon points="${pts.map(p=>tx(p).join(",")).join(" ")}" ${st}/>`:"";
  let svg=`<svg viewBox="0 0 ${W} ${H}" role="img" aria-label="course map">`;
  svg+=poly(s.course.corners,'fill="var(--bluebg)" stroke="var(--blue)" stroke-width="2"');
  svg+=poly(s.course.transit,'fill="none" stroke="var(--purple)" stroke-width="2" stroke-dasharray="6 4"');
  if(s.fence) svg+=poly(s.fence,'fill="none" stroke="var(--amber)" stroke-width="3"');
  for(const a of s.alerts){ if(!a.at) continue; const [x,y]=tx(a.at);
    svg+=`<circle cx="${x}" cy="${y}" r="${(a.r||0)*sc}" fill="var(--redbg)" stroke="var(--red)" stroke-width="2" fill-opacity=".6"/><circle cx="${x}" cy="${y}" r="4" fill="var(--red)"/><text x="${x+7}" y="${y-7}" font-size="12" fill="var(--red)">${esc(a.label)}</text>`; }
  for(const h of s.heartbeats){ if(h.lat==null) continue; const [x,y]=tx([h.lat,h.lon]);
    svg+=`<circle cx="${x}" cy="${y}" r="6" fill="var(--ink)"/><text x="${x+9}" y="${y+4}" font-size="13" fill="var(--ink)">${esc(h.vehicle)}</text>`; }
  const m10=10*sc; svg+=`<line x1="10" y1="${H-10}" x2="${10+m10}" y2="${H-10}" stroke="var(--muted)" stroke-width="2"/><text x="10" y="${H-15}" font-size="11" fill="var(--muted)">10 m</text>`;
  svg+=`<text x="${W-10}" y="16" font-size="11" text-anchor="end" fill="var(--muted)">N ↑ · blue course · purple dashed transit · orange declared fence</text></svg>`;
  $("map").innerHTML=svg;
}
async function poll(){ try{ const r=await fetch("/api/state?since="+since); render(await r.json()); }
  catch(e){ chip("broker","PAGE LOST THE WATCHER","bad"); } setTimeout(poll,500); }
poll();
</script></body></html>
"""


if __name__ == "__main__":
    raise SystemExit(main())
