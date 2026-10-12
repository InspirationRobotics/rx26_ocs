"""Print the WSL2 VM's eth0 IPv4 address: the HOST for `fleet_link --sim-boat HOST:PORT`.

    python -m fleet.wsl_ip                      (from the OCS directory)
    python -m fleet.wsl_ip --distro Ubuntu-22.04

Stdout is the address and nothing else, so PowerShell can capture it:

    $ip = python -m fleet.wsl_ip

Exit 0 with the address, or exit 1 with the reason on stderr. The address changes every
time the WSL VM restarts, so ask again after a `wsl --shutdown`. Not used by fleet_link
itself; stdlib only. This starts the distro if it is not running.
"""
from __future__ import annotations

import argparse
import ipaddress
import re
import subprocess
import sys
from typing import Callable, Optional, Sequence

DEFAULT_DISTRO = "Ubuntu-22.04"       # the shared distro that hosts the Crusader sim
_INET = re.compile(r"\binet\s+(\d{1,3}(?:\.\d{1,3}){3})/\d+")


class WslIpError(RuntimeError):
    pass


def decode(raw: bytes) -> str:
    """wsl.exe's own messages are UTF-16LE; whatever the Linux command prints is UTF-8."""
    if b"\x00" in raw:
        return raw.decode("utf-16-le", "replace").replace("\x00", "")
    return raw.decode("utf-8", "replace")


def parse_eth0_ipv4(text: str) -> Optional[str]:
    """The first usable address in `ip -4 -o addr show eth0` output, else None."""
    for match in _INET.finditer(text):
        try:
            addr = ipaddress.IPv4Address(match.group(1))
        except ValueError:
            continue
        if not (addr.is_loopback or addr.is_link_local or addr.is_unspecified):
            return str(addr)
    return None


def wsl_eth0_ipv4(distro: str = DEFAULT_DISTRO, *, run: Callable = subprocess.run,
                  timeout_s: float = 60.0) -> str:
    cmd = ["wsl.exe", "-d", distro, "-e", "sh", "-c", "ip -4 -o addr show eth0"]
    try:
        proc = run(cmd, capture_output=True, timeout=timeout_s)
    except FileNotFoundError:
        raise WslIpError("wsl.exe not found: is WSL installed on this PC?")
    except subprocess.TimeoutExpired:
        raise WslIpError("wsl.exe did not answer within %g s" % timeout_s)
    out, err = decode(proc.stdout or b""), decode(proc.stderr or b"")
    if proc.returncode != 0:
        why = next((line.strip() for line in (err + "\n" + out).splitlines() if line.strip()), "no message")
        raise WslIpError("wsl.exe -d %s failed (exit %d): %s" % (distro, proc.returncode, why))
    ip = parse_eth0_ipv4(out)
    if ip is None:
        raise WslIpError("%s has no IPv4 address on eth0 (wsl.exe said: %s)"
                         % (distro, out.strip() or err.strip() or "nothing"))
    return ip


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m fleet.wsl_ip", description=__doc__.split("\n\n")[0])
    ap.add_argument("--distro", default=DEFAULT_DISTRO, help="WSL distro name (default %(default)s)")
    args = ap.parse_args(argv)
    try:
        print(wsl_eth0_ipv4(args.distro))
    except WslIpError as e:
        print("wsl_ip: %s" % e, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
