"""start_ekko_ocs.py -- one command after a reboot: find RoboCommand, then start the OCS for Ekko.

    .venv\\Scripts\\python.exe start_ekko_ocs.py            (from C:\\Users\\Chris\\OCS)
    .venv\\Scripts\\python.exe start_ekko_ocs.py --find     only find RoboCommand and fix bridge.toml

For Ekko at the 7-8 Nov 2026 assessment. What it does, in order:

 1. Waits for the Ethernet (the RoboCommand-facing port) to have a DHCP address.
 2. Finds the MQTT broker on that network: the gateway, then .2 (the course convention), then every
    address in the /24, by TCP connect to port 1883. More than one answering = it stops and asks.
 3. If bridge.toml's [robocommand] host differs, rewrites that one line (backup in run/) and checks
    the file still loads.
 4. Stops any fleet_link / ocs_web already running (OCS.cmd starts fleet_link on COM8 and UDP 14550,
    which is wrong for Ekko), then starts, each in its own window:
      fleet_link  SYSTEM python, --serial COM28 --wifi-port NONE
      ocs_web     .venv python, --config bridge.toml   (never --bench-auto)
"""
from __future__ import annotations

import ipaddress
import re
import shutil
import socket
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
BRIDGE = HERE / "bridge.toml"
PORT = 1883
ETH = "Ethernet"


def ps(cmd: str) -> str:
    return subprocess.run(["powershell", "-NoProfile", "-Command", cmd],
                          capture_output=True, text=True).stdout.strip()


def ethernet() -> tuple[str, str] | None:
    """(our IPv4, gateway) on the Ethernet port, or None while it has no DHCP address."""
    out = ps("$c = Get-NetIPConfiguration -InterfaceAlias '%s' -ErrorAction SilentlyContinue; "
             "if ((Get-NetAdapter -Name '%s').Status -eq 'Up' -and $c.IPv4Address) "
             "{ $c.IPv4Address.IPAddress + ' ' + ($c.IPv4DefaultGateway.NextHop -join ',') }" % (ETH, ETH))
    parts = out.split()
    if not parts or parts[0].startswith("169.254."):
        return None
    return parts[0], (parts[1] if len(parts) > 1 else "")


def open_1883(host: str, timeout: float = 0.6) -> bool:
    try:
        socket.create_connection((host, PORT), timeout=timeout).close()
        return True
    except OSError:
        return False


def find_broker(ip: str, gw: str) -> list[str]:
    net = ipaddress.ip_network(ip + "/24", strict=False)
    first = [h for h in (gw, str(net.network_address + 2)) if h and h != ip]
    for h in first:
        if open_1883(h, 1.5):
            return [h]
    rest = [str(h) for h in net.hosts() if str(h) not in first and str(h) != ip]
    with ThreadPoolExecutor(64) as ex:
        hits = [h for h, ok in zip(rest, ex.map(open_1883, rest)) if ok]
    return hits


def set_host(host: str) -> None:
    text = BRIDGE.read_bytes().decode("utf-8")      # keep its CRLF line endings exactly
    m = re.search(r"(?ms)^\[robocommand\]\s*$.*?^host\s*=\s*\"([^\"]*)\"", text)
    if not m:
        sys.exit("bridge.toml: no host line under [robocommand] -- not touching it")
    if m.group(1) == host:
        print("bridge.toml already points at %s" % host)
        return
    backup = HERE / "run" / ("bridge.toml.before_host_" + time.strftime("%Y%m%d_%H%M%S"))
    shutil.copy2(BRIDGE, backup)
    start, end = m.span(1)
    BRIDGE.write_bytes((text[:start] + host + text[end:]).encode("utf-8"))
    from rx_bridge import config
    got = config.load(BRIDGE).robocommand.host
    if got != host:
        shutil.copy2(backup, BRIDGE)
        sys.exit("bridge.toml did not load back with %s (got %s); restored the backup" % (host, got))
    print("bridge.toml: RoboCommand %s -> %s (backup %s)" % (m.group(1), host, backup.name))


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


def main() -> None:
    print("waiting for the Ethernet to get a DHCP address ...")
    while (eth := ethernet()) is None:
        time.sleep(2)
    ip, gw = eth
    print("Ethernet %s, gateway %s" % (ip, gw or "none"))
    while True:
        hits = find_broker(ip, gw)
        if len(hits) == 1:
            break
        if len(hits) > 1:
            print("more than one MQTT broker on this network: %s -- using the first" % ", ".join(hits))
            hits = hits[:1]
            break
        print("no MQTT broker on port %d yet; looking again in 5 s" % PORT)
        time.sleep(5)
    print("RoboCommand broker: %s:%d" % (hits[0], PORT))
    set_host(hits[0])
    if "--find" in sys.argv:
        return
    stop_existing()
    start("Fleet link COM28",["python", "-m", "fleet.fleet_link", "--config", r"fleet\fleet.toml",
                                 "--serial", "COM28", "--wifi-port", "NONE"])
    time.sleep(3)
    start("OCS (ocs_web)", [str(HERE / ".venv" / "Scripts" / "python.exe"), "-m", "ocs_web",
                            "--config", "bridge.toml"])
    print("open http://127.0.0.1:8099")


if __name__ == "__main__":
    main()
