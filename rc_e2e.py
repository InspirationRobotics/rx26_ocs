"""rc_e2e -- OUR OCS against RoboNation's test server on the sim box, narrated on its screen.

rc_box_tests.py proves the box itself: RoboNation's test_client plays both sides, team TEST.
This puts our whole chain in front of it instead, team ASTA:

    USV1 on the boat's own ocs_link.py  --+
    UAV1 on the same link, fence answer --+-- TCP 37564 --> ocs_web (here) -- MQTT --> box
                                                        <-- RxCommand ----- test_client

Every step is announced on the box's page (rc_watch, :8088) before it is driven. A step
about what crossed RoboNation's broker is judged there, by the watcher. A step whose proof
is on OUR side -- the OCS refusing in words, the broker's PUBACK, what reached the boat --
is judged here, from ocs_web's /api/state and the vehicles' own links, and posted to the
page with the evidence. Either way the page says why.

    .venv\\Scripts\\python.exe rc_e2e.py                E1..E9, in order
    .venv\\Scripts\\python.exe rc_e2e.py --hold 300     ...then keep it all up 5 min

Laptop side. Needs key-based ssh to the box (the page takes steps from the box only, so they
go through `rc_watch.py relay` over one ssh session) and 8099, 37564 and 11883 free here.
It starts its own ocs_web on a copy of bridge.toml: run state in a temporary folder
(run/seq.json is never touched), vehicle link on loopback, discovery beacon off, so no real
vehicle can join it. Our side, live: http://127.0.0.1:8099. The call sign is a test value.
"""
from __future__ import annotations

import argparse
import json
import re
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from bench_stage2 import _PKG  # noqa: E402,F401 -- puts the boat's package on sys.path
from crusader_groundstation.ocs_link import OcsLink, fake_report, rfc3339  # noqa: E402
from rc_box_tests import centre_of  # noqa: E402

BOX = "inspiration@192.168.65.2"
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", BOX]
WATCH = "http://192.168.65.2:8088"      # read from here; steps go through the relay
OCS = "http://127.0.0.1:8099"
TCP_PORT = 37564
CONTAINER = "robotx_2026-rc-test-1"
TEAM = "ASTA"
CALLSIGN = "E2EPILOT-0000"
PAUSE_S = 4.0                           # after each verdict, so a person can read it
OURS_S = 90                             # a step judged here: the watcher must not time it out
REVERIFY_S = 60.0                       # how often the radio adapter re-reads Ekko's fence


# ---- the two ends we read --------------------------------------------------------------------

def get(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=5) as resp:
        return json.loads(resp.read().decode("utf-8"))


def ocs() -> dict:
    return get(OCS + "/api/state")


def ocs_post(path: str, body: dict) -> str:
    req = urllib.request.Request(OCS + path, data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))["result"]


def watched() -> dict:
    return get(WATCH + "/api/state?since=%d" % 10 ** 9)        # everything but the events


def wait_for(fn, timeout_s: float):
    """fn() every half second until it is truthy or the time is up; its last answer."""
    end = time.time() + timeout_s
    while True:
        got = fn()
        if got or time.time() >= end:
            return got
        time.sleep(0.5)


def find(obj, key: str):
    """The first value under `key` anywhere in a decoded frame, or None."""
    if isinstance(obj, dict):
        if key in obj:
            return obj[key]
        obj = list(obj.values())
    if isinstance(obj, list):
        for item in obj:
            hit = find(item, key)
            if hit is not None:
                return hit
    return None


# ---- our OCS, isolated -----------------------------------------------------------------------

def set_key(text: str, section: str, key: str, value: str) -> str:
    """`key = value` inside [section] of a TOML text, its comment kept; absent is an error."""
    head = re.search(r"(?m)^\[%s\]\s*$" % re.escape(section), text)
    if head is None:
        raise SystemExit("bridge.toml has no [%s]" % section)
    nxt = re.compile(r"(?m)^\[").search(text, head.end())
    end = nxt.start() if nxt else len(text)
    body, n = re.subn(r'(?m)^(%s\s*=\s*)("[^"]*"|\S+)' % re.escape(key),
                      lambda m: m.group(1) + value, text[head.end():end], count=1)
    if n != 1:
        raise SystemExit("bridge.toml has no %s in [%s]" % (key, section))
    return text[:head.end()] + body + text[end:]


def isolated(text: str, tmp: Path) -> str:
    """bridge.toml with the run state in `tmp`, the vehicle link on loopback, no beacon."""
    for section, key, value in (("tcp", "host", '"127.0.0.1"'),
                                ("discovery", "enabled", "false"),
                                ("paths", "seq_store", '"%s"' % (tmp / "seq.json").as_posix()),
                                ("paths", "wire_log", '"%s"' % (tmp / "wire").as_posix())):
        text = set_key(text, section, key, value)
    return text


def listening(port: int) -> bool:
    with socket.socket() as s:
        s.settimeout(0.5)
        return s.connect_ex(("127.0.0.1", port)) == 0


def start_ocs(tmp: Path) -> subprocess.Popen:
    """ocs_web on the isolated copy, its output in tmp/ocs_web.log, once its page answers."""
    for port, what in ((8099, "an OCS page"), (TCP_PORT, "a vehicle link"),
                       (8098, "fleet_link, whose radio adapter would join as UAV1")):
        if listening(port):
            raise SystemExit("%s is already up on port %d here -- stop it first, or this test "
                             "would drive it" % (what, port))
    config = tmp / "bridge.toml"
    config.write_text(isolated((HERE / "bridge.toml").read_text(encoding="utf-8"), tmp),
                      encoding="utf-8")
    log = tmp / "ocs_web.log"
    proc = subprocess.Popen([sys.executable, "-u", "-m", "ocs_web", "--config", str(config),
                             "--local-broker"], cwd=HERE, stdout=log.open("w", encoding="utf-8"),
                            stderr=subprocess.STDOUT)
    for _ in range(40):
        if proc.poll() is not None:
            break
        try:
            if Path(ocs()["wire"]["path"]).resolve().is_relative_to(tmp.resolve()):
                return proc
            proc.kill()
            raise SystemExit("the OCS answering on 8099 is not the isolated one")
        except OSError:
            time.sleep(0.5)
    proc.kill()
    raise SystemExit("ocs_web did not come up -- see %s" % log)


# ---- the box ---------------------------------------------------------------------------------

class BoxSession:
    """One long-lived ssh session to the box, a line per request on its stdin."""

    def __init__(self, remote: str, stderr=None) -> None:
        self.proc = subprocess.Popen(SSH + [remote], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=stderr,
                                     text=True, encoding="utf-8", bufsize=1)
        self.lock = threading.Lock()

    def send(self, line: str) -> None:
        with self.lock:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()

    def close(self) -> None:
        try:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            self.proc.kill()


class Narrator(BoxSession):
    """rc_watch's /api/plan, /step, /result and /note, through `rc_watch.py relay`."""

    def __init__(self) -> None:
        super().__init__("python3 -u ~/rcwatch/rc_watch.py relay")

    def api(self, path: str, body: dict) -> dict:
        with self.lock:
            self.proc.stdin.write(json.dumps({"path": path, "body": body}) + "\n")
            self.proc.stdin.flush()
            line = self.proc.stdout.readline()
        if not line:
            raise SystemExit("the narration relay on the box closed -- is ~/rcwatch/rc_watch.py "
                             "the one with `relay`?")
        return json.loads(line)


class TestClient(BoxSession):
    """RoboNation's own test_client in their rc-test container: RoboCommand's side."""

    def __init__(self) -> None:
        super().__init__("docker exec -i %s python -u test_client.py" % CONTAINER,
                         subprocess.STDOUT)
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self) -> None:
        for line in self.proc.stdout:
            if "error" in line.lower() or "usage:" in line.lower():
                print("  test_client: " + line.strip())

    def close(self) -> None:
        try:
            self.send("exit")
        except OSError:
            pass
        super().close()


# ---- the vehicles ----------------------------------------------------------------------------

class Vehicle:
    """One vehicle on the boat's own ocs_link.py -- bench_stage3's, but on the course: a 2 Hz
    STATE_AUTO heartbeat, task reports on demand, every frame from the OCS kept as the boat
    sees it, and for an aircraft the radio adapter's half of the fence handshake (answer each
    fence told, re-read it every REVERIFY_S)."""

    def __init__(self, vid: str, vtype: str, where: tuple) -> None:
        self.vid, self.vtype, self.where = vid, vtype, where
        self.received: list = []
        self.log: list = []
        self.fence = None                          # the last uav_geofence told, closed
        self._owe_fence = threading.Event()
        self._confirmed = 0.0
        self._stop = threading.Event()
        self._t0 = time.time()
        self.link = OcsLink("127.0.0.1", TCP_PORT, on_command=self._on_msg,
                            log=self.log.append)
        self.link.start()
        threading.Thread(target=self._run, daemon=True).start()

    def _on_msg(self, msg: dict) -> None:
        self.received.append(msg)
        info = msg.get("ocs_info")
        if self.vtype == "TYPE_UAV" and info and info.get("uav_geofence"):
            self.fence = info["uav_geofence"]
            self._owe_fence.set()

    def _fence_frame(self) -> dict:
        poly = [[p["latitude"], p["longitude"]] for p in self.fence[:-1]]     # OPEN form
        return {"ocs_fence": {"vehicle_id": self.vid, "state": "done", "polygon": poly,
                              "verified": True, "verified_at": rfc3339(),
                              "reason": "written and read back: %d vertices" % len(poly),
                              "lockout": None}}

    def _heartbeat(self) -> dict:
        r = fake_report(self.vid, TEAM, self._t0)
        hb = r["heartbeat"]
        hb["vehicle_type"] = self.vtype
        hb["position"] = {"latitude": self.where[0], "longitude": self.where[1]}
        if self.vtype == "TYPE_UAV":
            hb["flight_phase"] = "FLIGHT_PHASE_GROUNDED"   # a UAV may leave nothing UNKNOWN
        return r

    def report(self, body: str, fields: dict) -> None:
        self.link.send_report({"team_id": TEAM, "vehicle_id": self.vid,
                               "sent_at": rfc3339(), body: fields})

    def _run(self) -> None:
        while not self._stop.is_set():
            if self.fence and time.monotonic() - self._confirmed >= REVERIFY_S:
                self._owe_fence.set()
            if self._owe_fence.is_set() and self.fence:
                self._owe_fence.clear()
                self._confirmed = time.monotonic()
                self.link.publish(self._fence_frame())
            else:
                self.link.publish(self._heartbeat())
            self._stop.wait(0.5)

    def close(self) -> None:
        self._stop.set()
        self.link.stop()


# ---- the steps -------------------------------------------------------------------------------

def verdict(ctx: dict, ok: bool, note: str) -> None:
    """The result of a step judged here, posted to the page with its evidence."""
    ctx["n"].api("/api/result", {"id": ctx["id"], "status": "pass" if ok else "fail",
                                 "note": note})


def act_fence(ctx: dict) -> None:
    lat, lon = ctx["centre"]
    ctx["usv"] = Vehicle("USV1", "TYPE_USV", (lat, lon))
    ctx["uav"] = Vehicle("UAV1", "TYPE_UAV", (lat + 0.00005, lon))

    def ready():
        s = ocs()
        held = (s["fence_confirm"].get("UAV1") or {}).get("matches_course_fence")
        return s if s["course"] and s["uav_geofence"] and held and len(s["active"]) >= 2 else None

    s = wait_for(ready, 30) or ocs()
    course, fence = s["course"] or {}, s["uav_geofence"] or []
    held = s["fence_confirm"].get("UAV1") or {}
    verdict(ctx, bool(course and fence and held.get("matches_course_fence")
                      and len(s["active"]) >= 2),
            "%s: %d corners + uav_transit_area %d points -> fence %d points (%s); UAV1 read it "
            "back %s s ago, matches: %s; reporting: %s"
            % (course.get("course_id", "no course"), len(course.get("corners", [])),
               len(course.get("uav_transit_area", [])), len(fence),
               s["uav_geofence_source"] or s["uav_geofence_reason"], held.get("age_s"),
               held.get("matches_course_fence"), ", ".join(s["active"]) or "nobody"))


def act_declare_blank(ctx: dict) -> None:
    text = ocs_post("/api/declare", {"callsign": ""})
    ctx["n"].api("/api/note", {"text": "Our OCS: " + text})
    if "call sign" not in text:
        verdict(ctx, False, "expected a refusal for the blank call sign, got: " + text)


def act_declare(ctx: dict) -> None:
    text = ocs_post("/api/declare", {"callsign": CALLSIGN})
    ctx["n"].api("/api/note", {"text": "Our OCS: " + text})
    if "cannot declare" in text or "REFUSED" in text:
        verdict(ctx, False, "our OCS refused: " + text)


def act_running(ctx: dict) -> None:
    def running():
        run = ocs()["run"]
        return run if run["state"] == "RUNNING" else None

    run = wait_for(running, 10) or ocs()["run"]
    verdict(ctx, run["state"] == "RUNNING",
            "run state %s, run_id %s, declaration seq %s, declared %s"
            % (run["state"], run["run_id"], run["declaration_seq"], ", ".join(run["declared"])))


def act_docking(ctx: dict) -> None:
    ctx["usv"].report("docking", {"bay_id": 2})


def act_acknowledged(ctx: dict) -> None:
    def acked():
        return next((e for e in ocs()["task_reports"]
                     if e["kind"] == "docking" and e["verdict"] == "acknowledged"), None)

    e = wait_for(acked, 8)
    if e:
        verdict(ctx, True, "%s %s %s: %s" % (e["vehicle"], e["kind"], e["verdict"].upper(),
                                             e["detail"]))
        return
    latest = ocs()["task_reports"][:1]
    verdict(ctx, False, "the card shows %s" % ("%(kind)s %(verdict)s: %(detail)s" % latest[0]
                                              if latest else "no task report at all"))


def act_alert(ctx: dict) -> None:
    lat, lon = ctx["centre"]
    ctx["rc"].send("moving-object-alert --team %s --lat %.7f --lng %.7f --heading 90 --speed 0.5 "
                   "--affected-vehicle-types usv uav --keep-out-radius 7" % (TEAM, lat, lon))


def act_alert_delivered(ctx: dict) -> None:
    def radius(vehicle):
        alert = next((a for a in (find(m, "moving_object_alert") for m in vehicle.received)
                      if a), None)
        return None if alert is None else alert.get("keep_out_radius_m")

    wait_for(lambda: radius(ctx["usv"]) is not None, 8)
    got = {v.vid: radius(v) for v in (ctx["usv"], ctx["uav"])}
    cmd = next((c for c in ocs()["commands"] if c["which"] == "moving_object_alert"), None)
    verdict(ctx, got["USV1"] == 7,
            "the boat's ocs_link.py got keep_out_radius_m=%s (UAV1: %s); our OCS routed it to %s"
            % (got["USV1"], got["UAV1"], ", ".join(cmd["targets"]) if cmd else "nobody"))


TESTS = [
    {"id": "E1", "title": "Our OCS has the course, its transit area, and a fence UAV1 holds",
     "doing": "Our OCS on the laptop takes Course Alpha from this broker, works out the UAV "
              "fence over course + transit area and tells UAV1, which reads it back. USV1 "
              "(the boat's own ocs_link.py) and UAV1 start heartbeating through our OCS.",
     "expect": "Judged on the laptop from our OCS's state, the evidence posted here. On this "
               "page: 'first heartbeat from ASTA/USV1' and '.../UAV1'.",
     "act": act_fence},
    {"id": "E2", "title": "No pilot call sign: our OCS refuses, RoboCommand hears nothing",
     "doing": "Pressing Declare on our OCS with the call-sign box empty.",
     "expect": "Our OCS's refusal, in words, as a note -- and no REQUEST card from ASTA.",
     "absent": [{"type": "request", "team": TEAM}], "timeout_s": 6,
     "act": act_declare_blank},
    {"id": "E3", "title": "With a call sign, RoboCommand accepts the fence and starts the run",
     "doing": "Declaring again with call sign %s: USV1 + UAV1, the course fence, ceiling "
              "40 m HAE." % CALLSIGN,
     "expect": "A blue REQUEST card from ASTA with the call sign and ceiling, the server's "
               "'geofence and ceiling OK', then a green COMMAND run_start for ASTA.",
     "match": [{"type": "request", "kind": "run_declaration", "team": TEAM,
                "contains": CALLSIGN},
               {"type": "server", "kind": "geofence_ok", "contains": TEAM},
               {"type": "command", "kind": "run_start", "team": TEAM}],
     "timeout_s": 20, "act": act_declare},
    {"id": "E4", "title": "Our OCS knows the run started",
     "doing": "Reading our OCS's run state after that RunStart.",
     "expect": "RUNNING, with RoboCommand's run id. Judged on the laptop.",
     "act": act_running},
    {"id": "E5", "title": "Both heartbeats reach RoboCommand at 2 Hz through our OCS",
     "doing": "USV1 and UAV1 keep heartbeating; our OCS re-stamps and publishes each one.",
     "expect": "ASTA/USV1 and ASTA/UAV1 in the Heartbeats table at about 2 Hz. Heartbeats "
               "never become cards.",
     "match": [{"type": "hb", "team": TEAM, "vehicle": "USV1", "min_hz": 1.5},
               {"type": "hb", "team": TEAM, "vehicle": "UAV1", "min_hz": 1.5}],
     "timeout_s": 12},
    {"id": "E6", "title": "A task report from the boat reaches RoboCommand",
     "doing": "USV1 sends one Task 3 DockingReport (bay 2) on the boat's ocs_link.py, among "
              "the heartbeats.",
     "expect": "One big amber TASK REPORT card from ASTA/USV1.",
     "match": [{"type": "report", "kind": "docking", "team": TEAM, "vehicle": "USV1"}],
     "timeout_s": 10, "act": act_docking},
    {"id": "E7", "title": "Our OCS shows that report acknowledged",
     "doing": "Reading the 'Task reports to RoboCommand' card on our OCS.",
     "expect": "The docking report ACKNOWLEDGED: this broker returned its PUBACK (QoS 1). "
               "Judged on the laptop.",
     "act": act_acknowledged},
    {"id": "E8", "title": "RoboCommand sends a MovingObjectAlert with a 7 m keep-out",
     "doing": "RoboNation's test_client, as RoboCommand, alerts ASTA to a moving object at the "
              "course centre with keep_out_radius_m = 7.",
     "expect": "A green COMMAND card for ASTA with keep_out_radius_m = 7.",
     "match": [{"type": "command", "kind": "moving_object_alert", "team": TEAM,
                "contains": "keep_out_radius_m=7"}],
     "timeout_s": 10, "act": act_alert},
    {"id": "E9", "title": "The alert reaches the boat through our OCS, radius intact",
     "doing": "Looking at what our OCS forwarded and what the boat's ocs_link.py received.",
     "expect": "USV1 got moving_object_alert with keep_out_radius_m = 7. Judged on the laptop.",
     "act": act_alert_delivered},
]


def run(test: dict, ctx: dict) -> str:
    ours = "match" not in test and "absent" not in test
    ctx["n"].api("/api/step", {k: test.get(k) for k in ("id", "title", "doing", "expect")} |
                 {"match": test.get("match", []), "absent": test.get("absent", []),
                  "timeout_s": OURS_S if ours else test.get("timeout_s", 15)})
    print("\n%s  %s\n  doing:  %s\n  expect: %s" % (test["id"], test["title"], test["doing"],
                                                     test["expect"]))
    time.sleep(1.5)                     # the announcement lands on the screen first
    ctx["id"] = test["id"]
    if "act" in test:
        test["act"](ctx)
    while True:
        cur = next((t for t in watched()["tests"] if t["id"] == test["id"]), None)
        if cur and cur["status"] != "running":
            print("  -> %s  %s" % (cur["status"].upper(), cur["note"]))
            time.sleep(PAUSE_S)
            return cur["status"]
        time.sleep(0.5)


def main(argv=None) -> int:
    sys.stdout.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description="our OCS against the sim box, narrated on its page "
                                             "-- E1..E9 in order, each needs the ones before")
    ap.add_argument("--hold", type=float, default=0.0, metavar="S",
                    help="keep the OCS and both vehicles up S seconds after the last step")
    args = ap.parse_args(argv)
    box = watched()
    if not box["course"]:
        print("no RxCourse on the box's watcher: is RoboNation's test server up?")
        return 1
    tmp = Path(tempfile.mkdtemp(prefix="rc_e2e_"))
    ctx = {"centre": centre_of(box["course"]["corners"]), "ocs": start_ocs(tmp)}
    try:
        ctx["n"], ctx["rc"] = Narrator(), TestClient()
        ctx["n"].api("/api/plan", {"tests": [{"id": t["id"], "title": t["title"]}
                                             for t in TESTS]})
        ctx["n"].api("/api/note", {"text": "End to end: OUR OCS on the laptop, team %s. USV1 is "
                                           "the boat's own ocs_link.py; UAV1 does Ekko's fence "
                                           "read-back. Our side, live: http://127.0.0.1:8099 on "
                                           "the laptop." % TEAM})
        verdicts = {t["id"]: run(t, ctx) for t in TESTS}
        passed = sum(v == "pass" for v in verdicts.values())
        summary = "End to end: %d/%d passed (%s)" % (passed, len(verdicts), ", ".join(
            "%s %s" % (k, v.upper()) for k, v in verdicts.items()))
        ctx["n"].api("/api/note", {"text": summary,
                                   "level": "pass" if passed == len(verdicts) else "fail"})
        print("\n" + summary)
        if args.hold:
            print("holding %.0f s: our OCS at %s, the box at %s" % (args.hold, OCS, WATCH))
            time.sleep(args.hold)
    finally:
        for key in ("usv", "uav", "rc", "n"):
            if key in ctx:
                ctx[key].close()
        ctx["ocs"].terminate()
        ctx["ocs"].wait(10)
        print("our OCS's log and run state: %s" % tmp)
    return 0 if passed == len(verdicts) else 1


if __name__ == "__main__":
    sys.exit(main())
