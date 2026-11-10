"""beacon — tell the vehicles where the OCS is, so nobody types its address.

WHAT THE OCS BROADCASTS. Once per `interval_s` (1 s) the bridge sends one small
UDP datagram to 255.255.255.255, port 37565:

    {"ocs_beacon": {"team": "ASTA", "port": 37564, "v": 1}}

It means "an OCS for team ASTA accepts vehicle links on TCP port 37564, at the
address this datagram came from". There is no address in the payload on purpose:
the vehicle uses the datagram's source IP, which is by construction an address on
a network the vehicle can reach. It is sent once per eligible laptop address, from
a socket bound to that address, so each copy leaves by that address's own
interface and carries that interface's address as its source. Addresses are
listed again every 10 s, because WiFi comes and goes and the laptop has been seen
at 192.168.100.107, 192.168.8.107 and 192.168.1.75.

WHERE IT NEVER BROADCASTS, AND WHY. This laptop is dual-homed and the two networks
are never bridged (OCS/CLAUDE.md): team WiFi on one side, RoboNation's course
network on the other, and vehicles must not be able to reach RoboCommand. A
beacon on the course network would advertise our team code and vehicle port to
RoboNation's side, which is traffic nobody asked that network to carry. So an
address is skipped if it is

  * loopback, or link-local (169.254/16, an interface with no real address),
  * inside any `never_on` subnet -- by default the four course DHCP subnets
    Alpha 192.168.65.0/24 .. Delta 192.168.68.0/24, or
  * in the same /24 as the RoboCommand broker -- the one bridge.toml pins, or the
    one the bridge found by its course topic (rcfind.py) -- or the very address
    the bridge reaches it from. This catches a bench stub on a subnet that is
    not one of the four, and a course that changes its numbering (the Singapore
    screening's was 192.168.69.0/24), or
  * with `[robocommand] host = "auto"`, until RoboCommand has been found: in a
    /24 the search has not swept yet. That network may be the course network.

Skipped addresses are listed with the reason on the operator page. Nothing here
sets an address or touches a route; it only chooses which addresses to speak from.

WHAT THE BOAT DOES WITH IT (crusader_groundstation/ocs_link.py, `ocs_host: auto`).
It listens on UDP 37565 for beacons carrying its own team code. While it has no
connection it dials the source IP of the newest beacon heard in the last 5 s, at
the port the beacon names. With no such beacon it tries each address in
`ocs_fallback_hosts` in turn. Once connected it stays where it is: a later beacon
from somewhere else changes nothing until the connection drops, and then it
chooses again. An explicit `ocs_host: "<ip>"` never listens and behaves exactly as
it always did.

TWO OCS LAPTOPS ON ONE NETWORK. Both beacon. A disconnected boat takes whichever
it heard last when it retries, and keeps that one until the link drops. Nothing
arbitrates between them, and nothing should: the handbook allows the team one
connection to RoboCommand, so the second laptop must not run ocs_web at all (the
bridge's own `DUPLICATE OCS?` warning is the same problem from the other side).

WHEN NO BEACON ARRIVES -- an access point that isolates WiFi clients and drops
broadcasts, an older OCS, discovery switched off -- the boat falls back to its
`ocs_fallback_hosts` list, so the worst case is today's behaviour, not silence.

WINDOWS BROADCAST BEHAVIOUR, measured on this laptop (Windows 11, Python 3.13) with
a Wi-Fi adapter, a USB Ethernet adapter on the course subnet, and a VMware
adapter all holding on-link 255.255.255.255 routes at the same metric: 100 x
1200-byte datagrams sent to 255.255.255.255 from a socket bound to the Wi-Fi
address raised that adapter's SentBroadcastPackets by exactly 100 and the
Ethernet adapter's by 0. So binding to an address does pin a limited broadcast to
that address's interface; no per-subnet directed broadcast is needed, and the
subnet mask is never guessed. That is verified for Windows only, which is where
the OCS runs; do not assume it of another operating system.

Stdlib only, no I/O at import. The pure parts (`build`, `parse`, `eligible`) take
their inputs as arguments so they can be tested without a network.
"""
from __future__ import annotations

import ipaddress
import json
import socket
import threading
import time
from typing import Callable, Iterable

BEACON_KEY = "ocs_beacon"
BEACON_VERSION = 1

#: Paired with the vehicle link's TCP 37564.
DEFAULT_PORT = 37565
DEFAULT_INTERVAL_S = 1.0

#: The boat treats a beacon older than 5 s as gone. Two beacons inside that
#: window is the least that survives one lost datagram, hence the ceiling.
MAX_INTERVAL_S = 2.0

#: How often the laptop's addresses are listed again.
RESCAN_S = 10.0

#: RoboNation's four course networks (handbook: DHCP gives the OCS .65.x on Alpha,
#: .66.x Bravo, .67.x Charlie, .68.x Delta). The boat carries a mirror of this in
#: its ocs_link.py; change one, change both.
COURSE_NETWORKS = ("192.168.65.0/24", "192.168.66.0/24",
                   "192.168.67.0/24", "192.168.68.0/24")
DEFAULT_NEVER_ON = COURSE_NETWORKS

LIMITED_BROADCAST = "255.255.255.255"


# ---- the datagram ------------------------------------------------------------

def is_port(value) -> bool:
    """True for a real port number: an int (not a bool) in 1-65535."""
    return (isinstance(value, int) and not isinstance(value, bool)
            and 1 <= value <= 65535)


def build(team: str, port: int) -> bytes:
    """The beacon for `team`, whose vehicle link listens on TCP `port`."""
    if not isinstance(team, str) or not team.strip():
        raise ValueError("a beacon needs a team code")
    if not is_port(port):
        raise ValueError("beacon port %r is not a TCP port" % (port,))
    body = {BEACON_KEY: {"team": team, "port": port, "v": BEACON_VERSION}}
    return json.dumps(body, separators=(",", ":")).encode("utf-8")


def parse(data: bytes) -> dict | None:
    """{"team", "port", "v"} for a well-formed v1 beacon, else None.

    Strict by design: anything that is not exactly this shape is not ours, and a
    receiver that guessed would dial an address on the strength of a stray packet.
    """
    if not isinstance(data, (bytes, bytearray)) or len(data) > 512:
        return None
    try:
        obj = json.loads(bytes(data).decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None
    inner = obj.get(BEACON_KEY) if isinstance(obj, dict) else None
    if not isinstance(inner, dict) or len(obj) != 1:
        return None
    team, port, ver = inner.get("team"), inner.get("port"), inner.get("v")
    if not isinstance(team, str) or not team:
        return None
    if not is_port(port):
        return None
    if isinstance(ver, bool) or ver != BEACON_VERSION:
        return None
    return {"team": team, "port": port, "v": ver}


# ---- which addresses may speak ----------------------------------------------

def local_ipv4_addresses() -> list[str]:
    """This machine's IPv4 addresses, as the resolver lists them for its own name.

    On the Windows laptop this returned every adapter that holds an address
    (Wi-Fi, USB Ethernet, VMware, Tailscale's link-local) and no loopback; adapters
    with no media or a still-tentative address were absent. It can miss an
    interface whose address Windows does not register under the machine name -- a
    skipped interface only means no beacon there, and the boat's fallback list
    covers it. Raises OSError if the name cannot be resolved at all.
    """
    infos = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET,
                               socket.SOCK_DGRAM)
    out: list[str] = []
    for info in infos:
        addr = info[4][0]
        if addr not in out:
            out.append(addr)
    return out


def _networks(subnets: Iterable[str]) -> list:
    return [ipaddress.ip_network(n, strict=False) for n in subnets]


def slash24(host: str | None) -> ipaddress.IPv4Network | None:
    """The /24 around `host` if it is an IPv4 literal. A hostname is not resolved
    here (a lookup could block the bridge's start); list its subnet in never_on."""
    try:
        return ipaddress.ip_network("%s/24" % ipaddress.IPv4Address(str(host)),
                                    strict=False)
    except ValueError:
        return None


def _why_not(addr: str, never_on: list, broker_net, broker_host,
             via: str | None) -> str | None:
    try:
        ip = ipaddress.IPv4Address(addr)
    except ValueError:
        return "not an IPv4 address"
    if ip.is_loopback:
        return "loopback"
    if ip.is_unspecified:
        return "unspecified"
    if ip.is_link_local:
        return "link-local, no real address"
    for net in never_on:
        if ip in net:
            return "inside never_on %s" % net
    if addr == via:
        return "the address RoboCommand %s is reached from" % broker_host
    if broker_net is not None and ip in broker_net:
        return "same /24 as the RoboCommand broker %s" % broker_host
    return None


def eligible(addresses: Iterable[str],
             never_on: Iterable[str] = DEFAULT_NEVER_ON,
             broker_host: str | None = None,
             via: str | None = None) -> tuple[list[str], list[dict]]:
    """Split `addresses` into (the ones to broadcast from, [{addr, why}] skipped).
    `via` is this laptop's address on its connection to `broker_host`, when known.

    Order is kept and duplicates dropped. Pure: the caller supplies the list.
    """
    nets = _networks(never_on)
    broker_net = slash24(broker_host)
    sending: list[str] = []
    skipped: list[dict] = []
    seen: set[str] = set()
    for raw in addresses:
        addr = str(raw).strip()
        if addr in seen:
            continue
        seen.add(addr)
        why = _why_not(addr, nets, broker_net, broker_host, via)
        if why is None:
            sending.append(addr)
        else:
            skipped.append({"addr": addr, "why": why})
    return sending, skipped


def missing_course_networks(never_on: Iterable[str]) -> list[str]:
    """The course subnets that `never_on` does not cover. A `never_on` that is set
    replaces the default rather than adding to it, so a list that forgets one is
    worth a warning: the broker /24 rule then stands alone for that subnet."""
    nets = _networks(never_on)
    return [c for c in COURSE_NETWORKS
            if not any(ipaddress.ip_network(c).subnet_of(n) for n in nets)]


# ---- the sender --------------------------------------------------------------

class BeaconSender:
    """Broadcasts the beacon from every eligible address until stopped.

    `addresses`, `dest` and `select` are injectable so a test can run the real
    socket path on loopback without touching a network: `select` replaces the
    eligibility filter (and so is the one way to let 127.0.0.1 through -- the
    default filter never does).

    `hold(addr)` returns a reason not to speak from an address the filter passed,
    or None; the bridge's RoboCommand search supplies it (rcfind.py). `set_broker`
    moves the broker rule to a broker found after construction.
    """

    def __init__(self, team: str, *, port: int = DEFAULT_PORT,
                 interval_s: float = DEFAULT_INTERVAL_S,
                 never_on: Iterable[str] = DEFAULT_NEVER_ON,
                 broker_host: str | None = None, log: Callable = print,
                 addresses: Callable[[], list[str]] = local_ipv4_addresses,
                 dest: tuple[str, int] | None = None,
                 select: Callable | None = None,
                 hold: Callable[[str], str | None] | None = None,
                 rescan_s: float = RESCAN_S) -> None:
        self.team = team
        self.port = port
        self.interval_s = interval_s
        self.never_on = tuple(never_on)
        self.log = log
        self.rescan_s = rescan_s
        self._addresses = addresses
        self._dest = dest or (LIMITED_BROADCAST, port)
        self._broker: tuple[str | None, str | None] = (broker_host, None)
        self._select = select or (lambda found: eligible(
            found, self.never_on, *self._broker))
        self._hold = hold

        self._lock = threading.Lock()
        self._socks: dict[str, socket.socket] = {}
        self._skipped: list[dict] = []
        self._sent = 0
        self._errors = 0
        self._last_error = ""
        self._payload = b""
        self._tcp_port = 0
        self._announced: dict = {}            #: channel -> last logged key
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # ---- lifecycle ----------------------------------------------------------

    def start(self, tcp_port: int) -> None:
        """Begin beaconing the vehicle link at `tcp_port` (the BOUND port, so a
        server on port 0 advertises where it really listens)."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._payload = build(self.team, tcp_port)
        self._tcp_port = tcp_port
        for net in missing_course_networks(self.never_on):
            self.log("WARNING: [discovery] never_on does not cover the course "
                     "network %s -- only the broker's /24 stops a beacon there"
                     % net)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ocs-beacon",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2.0)
            self._thread = None
        with self._lock:
            socks, self._socks = list(self._socks.values()), {}
        for s in socks:
            s.close()

    def snapshot(self) -> dict:
        """What the page shows. `sent` counts datagrams (one per address per tick)."""
        with self._lock:
            return {"enabled": True, "why": "", "port": self.port,
                    "tcp_port": self._tcp_port, "interval_s": self.interval_s,
                    "team": self.team, "sending_on": sorted(self._socks),
                    "skipped": [dict(s) for s in self._skipped],
                    "sent": self._sent, "errors": self._errors,
                    "last_error": self._last_error}

    # ---- the thread ---------------------------------------------------------

    def _run(self) -> None:
        next_scan = 0.0
        while not self._stop.is_set():
            try:
                now = time.monotonic()
                if now >= next_scan:
                    self.rescan()
                    next_scan = now + self.rescan_s
                self._send_round()
            except Exception as exc:  # noqa: BLE001 -- the beacon must not die quietly
                self._fail("internal", "beacon: %s: %s" % (type(exc).__name__, exc))
            self._stop.wait(self.interval_s)

    def set_broker(self, host: str | None, via: str | None = None) -> None:
        """RoboCommand's broker is `host`, reached from this laptop's `via`. The
        addresses that rule now skips stop at once, not at the next scan."""
        with self._lock:
            self._broker = (host, via)
        self.rescan()

    def rescan(self) -> None:
        """List this machine's addresses and apply the rules now. Safe from any
        thread: the rules are applied under the lock, so whichever scan finishes
        last uses the newest broker."""
        if self._thread is None or self._stop.is_set():
            return                  # not beaconing: nothing to open, nothing to close
        try:
            found = list(self._addresses())
        except OSError as exc:
            # Keep what is open: a resolver hiccup is not a reason to go silent.
            self._fail("addresses", "cannot list this machine's addresses: %s" % exc)
            return
        opened: list[str] = []
        with self._lock:
            sending, skipped = self._select(found)
            if self._hold is not None:
                held = [(a, self._hold(a)) for a in sending]
                sending = [a for a, why in held if why is None]
                skipped = skipped + [{"addr": a, "why": why} for a, why in held if why]
            for addr in [a for a in self._socks if a not in sending]:
                self._socks.pop(addr).close()
            for addr in sending:
                if addr in self._socks:
                    continue
                try:
                    self._socks[addr] = _open_broadcast_socket(addr)
                    opened.append(addr)
                except OSError as exc:
                    skipped.append({"addr": addr, "why": "bind failed: %s" % exc})
                    self._note_error("bind %s: %s" % (addr, exc))
            self._skipped = skipped
            now_on = sorted(self._socks)
        self._announce("set", (tuple(now_on), tuple(s["addr"] for s in skipped)),
                       "beacon: UDP %d for %s on TCP %d -- from %s; not from %s"
                       % (self._dest[1], self.team, self._tcp_port,
                          ", ".join(now_on) or "nowhere",
                          "; ".join("%s (%s)" % (s["addr"], s["why"])
                                    for s in skipped) or "nothing skipped"))

    def _send_round(self) -> None:
        with self._lock:
            socks = list(self._socks.items())
        for addr, sock in socks:
            try:
                sock.sendto(self._payload, self._dest)
            except OSError as exc:
                # The address has probably gone (WiFi dropped or re-leased). Close
                # it rather than fail every second; the next rescan reopens
                # whatever is really there.
                with self._lock:
                    if self._socks.get(addr) is not sock:
                        continue          # a rescan closed it on purpose meanwhile
                    del self._socks[addr]
                    self._skipped.append({"addr": addr, "why": "send failed: %s" % exc})
                    self._note_error("send from %s: %s" % (addr, exc))
                sock.close()
                self.log("beacon: send from %s failed (%s) -- dropped until the "
                         "next address scan" % (addr, exc))
            else:
                with self._lock:
                    self._sent += 1

    # ---- small helpers ------------------------------------------------------

    def _note_error(self, message: str) -> None:
        """Count an error and keep its text. The caller holds `self._lock`."""
        self._errors += 1
        self._last_error = message

    def _fail(self, channel: str, message: str) -> None:
        with self._lock:
            self._note_error(message)
        self._announce(channel, message, message)

    def _announce(self, channel: str, key, message: str) -> None:
        """Log `message` when `key` differs from the last one logged on `channel`."""
        if self._announced.get(channel) != key:
            self._announced[channel] = key
            self.log(message)


def _open_broadcast_socket(addr: str) -> socket.socket:
    """A UDP socket bound to (addr, ephemeral port) that may send broadcasts."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind((addr, 0))
    except OSError:
        sock.close()
        raise
    return sock


def off_snapshot(port: int, why: str) -> dict:
    """The snapshot shape for a bridge with no sender, so the page has one reader."""
    return {"enabled": False, "why": why, "port": port, "tcp_port": 0,
            "interval_s": 0.0, "team": "", "sending_on": [], "skipped": [],
            "sent": 0, "errors": 0, "last_error": ""}
