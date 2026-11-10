"""start_ekko_ocs.py -- one command after a reboot: start the OCS for Ekko.

    .venv\\Scripts\\python.exe start_ekko_ocs.py            (from the OCS folder)
    .venv\\Scripts\\python.exe start_ekko_ocs.py --find     only look for RoboCommand once, and say
                                                          what answered (with the OCS stopped)

For Ekko alone (the 7-8 Nov 2026 assessment). What it does, in order:

 1. Stops any fleet_link / ocs_web already running (OCS.cmd starts fleet_link on COM8 and UDP 14550,
    which is wrong for Ekko).
 2. Starts, each in its own window:
      fleet_link  SYSTEM python, --serial COM28 --wifi-port NONE
      ocs_web     .venv python, --config bridge.toml   (never --bench-auto)

It no longer finds RoboCommand or edits bridge.toml. With [robocommand] host = "auto" the OCS
finds the broker itself, by the course it keeps retained on robocommand/robotx/course, and finds
it again after a move to another network (rx_bridge/rcfind.py); the page's RoboCommand row shows
the search. `--find` runs one round of that search from here and prints every host that answered.
It refuses while ocs_web is running: a probe must not overlap the OCS's own connection, and the
OCS is already searching.
"""
from __future__ import annotations

import re
import socket
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
BRIDGE = HERE / "bridge.toml"
OCS_PAGE_PORT = 8099


def ps(cmd: str) -> str:
    return subprocess.run(["powershell", "-NoProfile", "-Command", cmd],
                          capture_output=True, text=True).stdout.strip()


def ocs_is_running() -> bool:
    try:
        socket.create_connection(("127.0.0.1", OCS_PAGE_PORT), timeout=0.5).close()
        return True
    except OSError:
        return False


def find() -> int:
    """One search round, printed host by host."""
    if ocs_is_running():
        print("ocs_web is running and is already searching: its page's RoboCommand row shows "
              "what it found. Not probing alongside it.")
        return 2
    from rx_bridge import config, rcfind
    cfg = config.load(BRIDGE)
    if not cfg.robocommand.auto:
        print("bridge.toml pins RoboCommand at %s; searching anyway" % cfg.robocommand.host)
    print("looking for a broker with %s on TCP %d ..."
          % (cfg.rc_course_sub(), cfg.robocommand.port))
    rnd = rcfind.find_once(cfg.robocommand.port, cfg.rc_course_sub(), cfg.team_id)
    print("from %s; swept %s; %.1f s" % (", ".join(rnd.addresses) or "no usable address",
                                         ", ".join(rnd.swept) or "nothing", rnd.took_s))
    for t in rnd.tried:
        print("  %-15s %s: %s" % (t["host"], t["why"], t["verdict"]))
    if rnd.found is None:
        print("RoboCommand: not found")
        return 1
    print("RoboCommand: %s:%d, course %s" % (rnd.found.host, rnd.found.port,
                                             rnd.found.course_id or "(no course_id)"))
    return 0


def stop_existing() -> None:
    out = ps("Get-CimInstance Win32_Process -Filter \"Name like 'python%'\" | Where-Object { "
             "$_.CommandLine -like '*fleet.fleet_link*' -or $_.CommandLine -like '*-m ocs_web*' } | "
             "ForEach-Object { $_.ProcessId.ToString() + ' ' + $_.CommandLine; "
             "Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }")
    for line in out.splitlines():
        print("stopped: " + re.sub(r"^(\d+).*?(-m \S+.*)$", r"pid \1  \2", line).strip())
    time.sleep(1.5)


def start(title: str, args: list[str]) -> None:
    subprocess.Popen(["cmd", "/k", "title " + title + " && " + subprocess.list2cmdline(args)],
                     cwd=HERE, creationflags=subprocess.CREATE_NEW_CONSOLE)
    print("started: %s" % title)


def main() -> int:
    if "--find" in sys.argv:
        return find()
    stop_existing()
    start("Fleet link COM28", ["python", "-m", "fleet.fleet_link", "--config", r"fleet\fleet.toml",
                                "--serial", "COM28", "--wifi-port", "NONE"])
    time.sleep(3)
    start("OCS (ocs_web)", [str(HERE / ".venv" / "Scripts" / "python.exe"), "-m", "ocs_web",
                            "--config", "bridge.toml"])
    print("open http://127.0.0.1:8099 -- the RoboCommand row shows the search for the broker")
    return 0


if __name__ == "__main__":
    sys.exit(main())
