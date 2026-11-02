"""rc_box_tests -- narrated RoboCommand tests on the test box, watched on rc_watch's page.

Each test is announced on the page first (Doing / Expect / a countdown), then driven with
RoboNation's own test_client inside their rc-test container, and judged by the watcher
from what really crossed the broker or the test server's log -- so a PASS on the screen
is something RoboCommand's stand-in actually saw, not this script's opinion.

    python3 ~/rcwatch/rc_box_tests.py            all of them, B1..B7
    python3 ~/rcwatch/rc_box_tests.py B2 B4      some

Runs on the box itself (the page takes test steps from 127.0.0.1 only). Standard library
only. Team TEST throughout, so a live OCS (team ASTA) and these tests never answer each
other. The pilot callsign and UAV registration below are test values, not anyone's.
"""
from __future__ import annotations

import json
import math
import subprocess
import sys
import threading
import time
import urllib.request

WATCH = "http://127.0.0.1:8088"
CONTAINER = "robotx_2026-rc-test-1"
TEAM = "TEST"
CALLSIGN = "TESTPILOT-0000"
REGISTRATION = "TEST-0000"
PAUSE_S = 4.0                           # after each verdict, so a person can read it
NO_RUNSTART = [{"type": "command", "kind": "run_start", "team": TEAM}]
DECLARE = "run-declaration --team %s --vehicles USV1 UAV1 --task1 core" % TEAM


def api(path: str, body: dict | None = None) -> dict:
    data = None if body is None else json.dumps(body).encode("utf-8")
    req = urllib.request.Request(WATCH + path, data=data,
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=5) as resp:
        return json.loads(resp.read().decode("utf-8"))


def state() -> dict:
    return api("/api/state?since=%d" % 10 ** 9)        # everything but the events


class Client:
    """One interactive test_client session: one MQTT connection, a command per line."""

    def __init__(self) -> None:
        self.proc = subprocess.Popen(
            ["docker", "exec", "-i", CONTAINER, "python", "-u", "test_client.py"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        self.lock = threading.Lock()
        self.errors = []
        threading.Thread(target=self._drain, daemon=True).start()

    def _drain(self) -> None:
        for line in self.proc.stdout:
            if "error" in line.lower() or "usage:" in line.lower():
                self.errors.append(line.strip())
                print("  test_client: " + line.strip())

    def send(self, line: str) -> None:
        with self.lock:
            self.proc.stdin.write(line + "\n")
            self.proc.stdin.flush()

    def close(self) -> None:
        try:
            self.send("exit")
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except (OSError, subprocess.TimeoutExpired):
            self.proc.kill()


class Heartbeats:
    """USV1 and UAV1 reporting STATE_AUTO at 2 Hz from the course centre, until stopped."""

    def __init__(self, client: Client, centre: tuple) -> None:
        self.client, self.centre = client, centre
        self.registration = ""
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self) -> None:
        lat, lon = self.centre
        while not self.stop.wait(0.5):
            self.client.send("heartbeat --team %s --vehicle USV1 --vehicle-type usv --state auto "
                             "--lat %.7f --lng %.7f" % (TEAM, lat, lon))
            reg = " --registration %s" % self.registration if self.registration else ""
            self.client.send("heartbeat --team %s --vehicle UAV1 --vehicle-type uav --state auto "
                             "--lat %.7f --lng %.7f%s" % (TEAM, lat + 0.00005, lon, reg))


# ---- geometry, for a fence that is deliberately wrong --------------------------------------

def centre_of(points: list) -> tuple:
    ring = points[:-1] if len(points) > 1 and points[0] == points[-1] else points
    return (sum(p[0] for p in ring) / len(ring), sum(p[1] for p in ring) / len(ring))


def pushed_out(points: list, metres: float) -> list:
    """The polygon with every vertex moved `metres` further from its centre, closed."""
    lat0, lon0 = centre_of(points)
    ky = 111320.0
    kx = ky * math.cos(math.radians(lat0))
    out = []
    for lat, lon in points:
        dx, dy = (lon - lon0) * kx, (lat - lat0) * ky
        f = (math.hypot(dx, dy) + metres) / math.hypot(dx, dy)
        out.append((lat0 + dy * f / ky, lon0 + dx * f / kx))
    if out[0] != out[-1]:
        out.append(out[0])
    return out


# ---- the tests -------------------------------------------------------------------------------

def act_course(ctx: dict) -> None:
    course = ctx["course"]
    if course and len(course["transit"]) >= 3:
        api("/api/result", {"id": "B1", "status": "pass",
                            "note": "%s: %d corners, uav_transit_area %d points"
                                    % (course["id"], len(course["corners"]), len(course["transit"]))})
    else:
        api("/api/result", {"id": "B1", "status": "fail",
                            "note": "no RxCourse" if not course else "RxCourse has no uav_transit_area"})


def act_bad_fence(ctx: dict) -> None:
    fence = pushed_out(ctx["course"]["corners"], 15.0)
    ctx["client"].send("%s --geofence-max-altitude-hae 40 --uav-pilot-callsign %s "
                       "--geofence-lats %s --geofence-lngs %s"
                       % (DECLARE, CALLSIGN, " ".join("%.7f" % p[0] for p in fence),
                          " ".join("%.7f" % p[1] for p in fence)))


def act_registration(ctx: dict) -> None:
    ctx["heartbeats"].registration = REGISTRATION


def act_alert(ctx: dict) -> None:
    lat, lon = ctx["centre"]
    ctx["client"].send("moving-object-alert --team %s --lat %.7f --lng %.7f --heading 90 --speed 0.5 "
                       "--affected-vehicle-types usv uav --keep-out-radius 7" % (TEAM, lat, lon))


TESTS = [
    {"id": "B1", "title": "RxCourse carries the UAV transit area",
     "doing": "Reading the course RoboCommand published (retained RxCourse, Course Alpha).",
     "expect": "Course ALPHA with its corners AND a uav_transit_area; on the map the purple "
               "dashed outline is the transit area, the blue one the course.",
     "timeout_s": 5, "act": act_course},
    {"id": "B2", "title": "UAV declaration without a ceiling is refused, silently",
     "doing": "Declaring USV1 + UAV1 with uav_geofence_max_altitude_hae_m = 0 -- what a "
              "declaration without the new field amounts to. Both vehicles report AUTO.",
     "expect": "A blue REQUEST card with ceiling 0, then a red REFUSED card from the server's "
               "log. No RunStart although both are AUTO: RoboCommand never replies.",
     "match": [{"type": "request", "kind": "run_declaration", "team": TEAM},
               {"type": "refused", "contains": TEAM}],
     "absent": NO_RUNSTART, "timeout_s": 10,
     "act": lambda ctx: ctx["client"].send(DECLARE + " --geofence-max-altitude-hae 0")},
    {"id": "B3", "title": "UAV fence outside course + transit area is refused",
     "doing": "Declaring again, ceiling 40 m, but every fence corner pushed 15 m out past "
              "the course.",
     "expect": "A red REFUSED card saying how far the fence leaves the course and transit "
               "area; the orange fence on the map sits outside the blue course. No RunStart.",
     "match": [{"type": "request", "kind": "run_declaration", "team": TEAM},
               {"type": "refused", "contains": "leaves the course and transit area"}],
     "absent": NO_RUNSTART, "timeout_s": 10, "act": act_bad_fence},
    {"id": "B4", "title": "Complete declaration is accepted and the run starts",
     "doing": "Declaring USV1 + UAV1 with a fence over course + transit area, ceiling 40 m "
              "HAE and a pilot callsign. Both vehicles keep reporting AUTO.",
     "expect": "REQUEST card with ceiling 40 and the callsign, 'geofence and ceiling OK', "
               "then a green COMMAND run_start.",
     "match": [{"type": "request", "kind": "run_declaration", "team": TEAM,
                "contains": CALLSIGN},
               {"type": "server", "kind": "geofence_ok", "contains": TEAM},
               {"type": "command", "kind": "run_start", "team": TEAM}],
     "timeout_s": 15,
     "act": lambda ctx: ctx["client"].send(DECLARE + " --geofence-max-altitude-hae 40 "
                                                     "--uav-pilot-callsign " + CALLSIGN)},
    {"id": "B5", "title": "UAV heartbeat carries its registration",
     "doing": "UAV1's heartbeats start carrying uav_registration = %s." % REGISTRATION,
     "expect": "In the Heartbeats table the reg column of TEST/UAV1 changes from a dash to "
               "%s. Heartbeats never become cards." % REGISTRATION,
     "match": [{"type": "hb", "vehicle": "UAV1", "contains": REGISTRATION}],
     "timeout_s": 8, "act": act_registration},
    {"id": "B6", "title": "A task report stands out from the heartbeats",
     "doing": "While both vehicles heartbeat at 2 Hz, USV1 sends one Task 3 DockingReport "
              "(bay 2).",
     "expect": "One big amber TASK REPORT card (a beep if sound is on); the heartbeats only "
               "refresh their line in the table.",
     "match": [{"type": "hb", "vehicle": "USV1", "min_hz": 1.5},
               {"type": "report", "kind": "docking", "team": TEAM}],
     "timeout_s": 10,
     "act": lambda ctx: ctx["client"].send("docking --team %s --vehicle USV1 --bay-id 2" % TEAM)},
    {"id": "B7", "title": "MovingObjectAlert carries its own keep-out radius",
     "doing": "RoboCommand (simulated) sends a Task 4 MovingObjectAlert at the course centre "
              "with keep_out_radius_m = 7.",
     "expect": "A green COMMAND card with keep_out_radius_m = 7 and a 7 m red circle on the "
               "map. Until this change the radius was a fixed 10 m.",
     "match": [{"type": "command", "kind": "moving_object_alert", "team": TEAM,
                "contains": "keep_out_radius_m=7"}],
     "timeout_s": 10, "act": act_alert},
]


def run(test: dict, ctx: dict) -> str:
    api("/api/step", {k: test.get(k) for k in ("id", "title", "doing", "expect")} |
        {"match": test.get("match", []), "absent": test.get("absent", []),
         "timeout_s": test.get("timeout_s", 15)})
    print("\n%s  %s\n  doing:  %s\n  expect: %s" % (test["id"], test["title"], test["doing"],
                                                     test["expect"]))
    time.sleep(1.5)                     # the announcement lands on the screen first
    test["act"](ctx)
    while True:
        cur = next((t for t in state()["tests"] if t["id"] == test["id"]), None)
        if cur and cur["status"] != "running":
            print("  -> %s  %s" % (cur["status"].upper(), cur["note"]))
            time.sleep(PAUSE_S)
            return cur["status"]
        time.sleep(0.5)


def main(argv: list) -> int:
    chosen = [t for t in TESTS if not argv or t["id"] in argv]
    snap = state()
    if not snap["course"]:
        print("no RxCourse on the watcher: is the test server up?")
        return 1
    api("/api/plan", {"tests": [{"id": t["id"], "title": t["title"]} for t in chosen]})
    api("/api/note", {"text": "Box baseline: RoboNation's test_client plays both sides, team %s. "
                              "Our OCS is not involved yet." % TEAM})
    client = Client()
    centre = centre_of(snap["course"]["corners"])
    ctx = {"client": client, "course": snap["course"], "centre": centre}
    ctx["heartbeats"] = Heartbeats(client, centre)
    try:
        verdicts = {t["id"]: run(t, ctx) for t in chosen}
    finally:
        ctx["heartbeats"].stop.set()
        client.close()
    passed = sum(v == "pass" for v in verdicts.values())
    summary = "Box baseline: %d/%d passed (%s)" % (passed, len(verdicts), ", ".join(
        "%s %s" % (k, v.upper()) for k, v in verdicts.items()))
    api("/api/note", {"text": summary, "level": "pass" if passed == len(verdicts) else "fail"})
    print("\n" + summary)
    return 0 if passed == len(verdicts) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
