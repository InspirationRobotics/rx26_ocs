"""rcfind -- find RoboCommand's MQTT broker by its course topic, not by its address.

WHY. The handbook gives each course a RoboCommand address (3.4.2: Alpha 192.168.65.2 to
Delta 192.168.68.2; the Team Village demo's comes "with the demo connection
instructions"), and at the Singapore screening on 7 Nov 2026 the broker was
192.168.69.3, on none of them. An address in bridge.toml is right on one network and
wrong on the next, and wrong fails quietly: the link just says DOWN. The topic is the same
everywhere. RoboCommand keeps the course retained on robocommand/robotx/course (3.4.9), so
a broker that hands a new subscriber an RxCourse there is RoboCommand, wherever it is.

WHEN. `[robocommand] host = "auto"`, or no host at all. An address there is used as it is,
with no search, exactly as before -- the benches and rc_e2e.py pin theirs. With "auto" the
bridge searches at start, and again once its RoboCommand link has been down REDISCOVER_S
(the laptop moved to another course, or the broker moved); for the first REDISCOVER_S of a
drop paho redials the broker it had, as it always did. A search is a ROUND, run every
ROUND_S until one finds the broker, and at once when this laptop's addresses change (a
cable plugged in, a new DHCP lease).

A ROUND tries each host once, in this order:
  1. where RoboCommand was last found;
  2. each interface's default gateway -- handbook 3.4.3 has the operator "confirm that the
     assigned default gateway is the course RoboCommand address";
  3. .2 of each of this laptop's /24s (the courses' convention), then .3 (the screening
     network's);
  4. every other address in each /24, swept for a TCP answer on the port: the first time
     a network is seen, SWEEP_S later, then twice as long after each sweep up to
     SWEEP_MAX_S -- a laptop left running on a hotel's WiFi should not scan it every
     minute all night -- and afresh when a network that went away comes back.
Each host that accepts a TCP connection is PROBED: a throwaway MQTT client (clean session,
an id of its own, never the bridge's "ocs-<team>") subscribes to the course topic and
waits PROBE_S for a message there that parses as RxCourse. The first host that delivers
one is RoboCommand. A broker with no course on that topic is not recognised -- the page
lists what answered and what it said -- and is found by the first round after RoboCommand
publishes one.

ONE CONNECTION. The handbook asks for one OCS connection (3.4.5). A probe never overlaps
the bridge's: the bridge stops its own client before every round (`pause`) and starts it
again only on what a round found. A probe publishes nothing, keeps no session, and is gone
within CONNECT_S + PROBE_S.

THE BEACON. With "auto" the course network is unknown until RoboCommand is found on it, so
the bridge holds the vehicle beacon (beacon.py) off any address whose /24 no round has
swept yet (`Finder.hold_reason`). Once RoboCommand is found the beacon's own rules apply,
with the found broker's /24 and the address that reached it skipped.

Only /24s are swept: the handbook's networks are /24s. A broker elsewhere on a larger
network is found only as a gateway, a .2 or a .3; pin anything else.
"""
from __future__ import annotations

import ipaddress
import math
import re
import secrets
import socket
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from typing import Callable, Iterable

import paho.mqtt.client as mqtt

from .beacon import local_ipv4_addresses, slash24
from .proto import rx_course_pb2

#: How long a probe waits for the retained course once connected. Their broker hands it
#: over within milliseconds of the SUBACK; the wait is for a broker that is not theirs.
PROBE_S = 3.0
#: A probe's TCP connect timeout.
CONNECT_S = 1.5
#: The TCP check before a probe, and the sweep's. 0.6 s found the screening broker.
TCP_CHECK_S = 0.6
SWEEP_WORKERS = 64

ROUND_S = 5.0
SWEEP_S = 60.0
SWEEP_MAX_S = 600.0
REDISCOVER_S = 30.0
TICK_S = 1.0

#: The first words of the verdict on a host that did not take a TCP connection. The page
#: counts those hosts by the same words rather than listing them.
NO_ANSWER = "no answer"

_IP = r"\d{1,3}(?:\.\d{1,3}){3}"
#: An active default route in Windows `route print -4`: destination and mask 0.0.0.0,
#: gateway, interface address, metric. The persistent-routes table has no interface
#: column, and "On-link" is not an address, so neither matches.
_ROUTE_PRINT = re.compile(r"^\s*0\.0\.0\.0\s+0\.0\.0\.0\s+(%s)\s+(%s)\s+\d+\s*$" % (_IP, _IP),
                          re.M)
#: Linux `ip -4 route show default`: the gateway, and the source address when it names one.
_IP_ROUTE = re.compile(r"^default via (%s)(?:.*?\bsrc (%s))?" % (_IP, _IP), re.M)


# ---- pure ------------------------------------------------------------------------------

def usable(addresses: Iterable) -> list[str]:
    """The IPv4 addresses that can be on a network RoboCommand is on -- not loopback,
    link-local (an interface with no real address), multicast or unspecified -- in order,
    each once."""
    out: list[str] = []
    for raw in addresses:
        try:
            ip = ipaddress.IPv4Address(str(raw).strip())
        except ValueError:
            continue
        if ip.is_loopback or ip.is_link_local or ip.is_multicast or ip.is_unspecified:
            continue
        if str(ip) not in out:
            out.append(str(ip))
    return out


def networks(addresses: Iterable[str]) -> list[ipaddress.IPv4Network]:
    """The /24 around each address, in order, each once."""
    out: list[ipaddress.IPv4Network] = []
    for addr in addresses:
        net = slash24(addr)
        if net is not None and net not in out:
            out.append(net)
    return out


def parse_route_print(text: str) -> list[tuple[str, str]]:
    """[(gateway, the interface address it is reached from)] from `route print -4`."""
    return _ROUTE_PRINT.findall(text)


def parse_ip_route(text: str) -> list[tuple[str, str]]:
    """[(gateway, source address or "")] from `ip -4 route show default`."""
    return _IP_ROUTE.findall(text)


def candidates(last: str | None, gateways: Iterable[tuple[str, str]],
               local: list[str]) -> list[tuple[str, str]]:
    """(host, why) for steps 1-3 of a round, in order: each host once, and never one of
    this laptop's own addresses."""
    out: list[tuple[str, str]] = []
    seen = set(local)

    def add(host, why: str) -> None:
        if host and host not in seen and usable([host]):
            seen.add(host)
            out.append((host, why))

    add(last, "where RoboCommand was last found")
    for gw, via in gateways:
        add(gw, "the default gateway" + (" of " + via if via else ""))
    for n in (2, 3):
        for net in networks(local):
            add(str(net[n]), ".%d of %s" % (n, net))
    return out


# ---- the network -----------------------------------------------------------------------

def default_gateways() -> list[tuple[str, str]]:
    """This machine's default gateways, each with the interface address that reaches it,
    from the routing table. [] when the table cannot be read: the gateways are then simply
    not tried, and the sweep still covers them."""
    if sys.platform == "win32":
        cmd, parse = ["route", "print", "-4"], parse_route_print
    else:
        cmd, parse = ["ip", "-4", "route", "show", "default"], parse_ip_route
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, errors="replace", timeout=5,
                             creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return parse(out or "")


def tcp_open(host: str, port: int, timeout: float = TCP_CHECK_S) -> bool:
    """True if host:port accepts a TCP connection (closed again at once)."""
    try:
        socket.create_connection((host, port), timeout=timeout).close()
        return True
    except OSError:
        return False


def answering(hosts: list[str], port: int, *, timeout: float = TCP_CHECK_S) -> list[str]:
    """The hosts, in their order, that accept a TCP connection on `port`, tried in parallel."""
    if not hosts:
        return []
    with ThreadPoolExecutor(min(SWEEP_WORKERS, len(hosts))) as pool:
        ok = list(pool.map(lambda h: tcp_open(h, port, timeout), hosts))
    return [h for h, up in zip(hosts, ok) if up]


@dataclass(frozen=True)
class Probe:
    """What one host said to a probe."""

    host: str
    is_robocommand: bool
    verdict: str                     #: in words, for the page and the log
    course_id: str | None = None
    via: str | None = None           #: this laptop's address that reached it


def probe(host: str, port: int, topic: str, client_id: str, *,
          connect_s: float = CONNECT_S, wait_s: float = PROBE_S) -> Probe:
    """Whether the MQTT broker at host:port is RoboCommand: subscribe to `topic` and wait
    `wait_s` for a message there that parses as RxCourse. Never raises for anything the
    other end does. No thread: the client is driven by hand and closed before returning."""
    got: dict = {}

    def on_connect(client, _ud, _flags, reason, _props=None) -> None:
        got["connack"] = reason
        if reason == 0:
            client.subscribe(topic, qos=0)

    def on_subscribe(_client, _ud, _mid, reasons, _props=None) -> None:
        got["suback"] = list(reasons)

    def on_message(_client, _ud, msg) -> None:
        if msg.topic != topic or "course" in got:
            return
        course = rx_course_pb2.RxCourse()
        try:
            course.ParseFromString(msg.payload)
        except Exception:  # noqa: BLE001 -- whatever it is, it is not an RxCourse
            got["junk"] = len(msg.payload)
            return
        got["course"] = course

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id,
                         clean_session=True)
    client.connect_timeout = connect_s
    client.on_connect, client.on_subscribe = on_connect, on_subscribe
    client.on_message = on_message
    try:
        client.connect(host, port, keepalive=max(5, math.ceil(connect_s + wait_s) * 2))
    except (OSError, ValueError) as exc:
        return Probe(host, False, "no MQTT connection: %s" % _said(exc))
    via = local_address(client)
    deadline = time.monotonic() + connect_s + wait_s
    closed = False
    try:
        while "course" not in got and time.monotonic() < deadline:
            if client.loop(timeout=0.1) != mqtt.MQTT_ERR_SUCCESS:
                closed = True
                break
            if got.get("connack", 0) != 0:
                break
    finally:
        _close(client)
    if "course" in got:
        cid = got["course"].course_id
        return Probe(host, True, "RoboCommand: course %s on %s" % (cid or "(no course_id)", topic),
                     cid, via)
    if "connack" not in got:
        return Probe(host, False, "accepted TCP %d but %s" % (
            port, "closed it without an MQTT answer" if closed
            else "gave no MQTT answer in %.0f s" % (connect_s + wait_s)), via=via)
    if got["connack"] != 0:
        return Probe(host, False, "an MQTT broker that refused the probe: %s" % got["connack"],
                     via=via)
    if any(getattr(r, "is_failure", False) for r in got.get("suback", [])):
        return Probe(host, False, "an MQTT broker that refused a subscription to %s" % topic,
                     via=via)
    if "junk" in got:
        return Probe(host, False, "an MQTT broker with %d bytes on %s that are not an RxCourse"
                     % (got["junk"], topic), via=via)
    return Probe(host, False, "an MQTT broker with nothing on %s in %.0f s -- not RoboCommand, "
                 "or no course published yet" % (topic, wait_s), via=via)


def _said(exc: Exception) -> str:
    if isinstance(exc, ConnectionRefusedError):
        return "refused"
    if isinstance(exc, (socket.timeout, TimeoutError)):
        return "timed out"
    return str(exc) or type(exc).__name__


def local_address(client) -> str | None:
    """This laptop's address on a paho client's connection, if it has one."""
    try:
        return client.socket().getsockname()[0]
    except (AttributeError, OSError):
        return None


def _close(client) -> None:
    """Disconnect politely when connected, and close the socket whatever happened."""
    try:
        if client.is_connected():
            client.disconnect()
            client.loop(timeout=0.05)       # sends the DISCONNECT; paho closes after it
    except Exception:  # noqa: BLE001 -- closing a throwaway client
        pass
    sock = client.socket()
    if sock is not None:
        try:
            sock.close()
        except OSError:
            pass


# ---- a round ----------------------------------------------------------------------------

def probe_id(team: str) -> str:
    """The client id for one round's probes: never the bridge's own ("ocs-<team>"), so a
    probe cannot take its session, and fresh each round."""
    return "ocs-%s-find-%s" % (team, secrets.token_hex(3))


@dataclass(frozen=True)
class Found:
    host: str
    port: int
    via: str | None                  #: this laptop's address that reached it
    course_id: str
    why: str                         #: which step of the round found it


@dataclass
class Round:
    """One pass over the candidates. `tried` is [{host, why, verdict}] for every host
    tried, the sweep's included only where something answered."""

    at: float                        #: wall clock at the start
    addresses: list[str]             #: this laptop's, searched from
    tried: list[dict] = field(default_factory=list)
    swept: list[str] = field(default_factory=list)
    took_s: float = 0.0
    found: Found | None = None


def search_round(local: list[str], port: int, topic: str, client_id: str, *,
                 last: str | None = None, sweep: Iterable = (),
                 gateways: Callable[[], list] = default_gateways,
                 probe_host: Callable[..., Probe] = probe,
                 answer: Callable[..., list[str]] = answering,
                 stop: threading.Event | None = None) -> Round:
    """Steps 1-3 for `local`, then step 4 over the networks in `sweep`, stopping at the
    first host that is RoboCommand."""
    rnd = Round(at=time.time(), addresses=list(local))
    t0 = time.monotonic()

    def halted() -> bool:
        return stop is not None and stop.is_set()

    def attempt(host: str, why: str) -> bool:
        p = probe_host(host, port, topic, client_id)
        rnd.tried.append({"host": host, "why": why, "verdict": p.verdict})
        if p.is_robocommand:
            rnd.found = Found(host, port, p.via, p.course_id or "", why)
        return p.is_robocommand

    try:
        first = candidates(last, gateways(), list(local))
        up = set(answer([h for h, _ in first], port))
        for host, why in first:
            if halted():
                return rnd
            if host not in up:
                rnd.tried.append({"host": host, "why": why,
                                  "verdict": "%s on TCP %d" % (NO_ANSWER, port)})
            elif attempt(host, why):
                return rnd
        if halted():
            return rnd
        nets = list(sweep)
        skip = set(local) | {h for h, _ in first}
        hosts = [str(h) for n in nets for h in n.hosts() if str(h) not in skip]
        rnd.swept = [str(n) for n in nets]
        for host in answer(hosts, port):
            if halted():
                return rnd
            if attempt(host, "answered on TCP %d in the sweep of %s" % (port, slash24(host))):
                return rnd
        return rnd
    finally:
        rnd.took_s = time.monotonic() - t0


def find_once(port: int, topic: str, team: str) -> Round:
    """One full round from this laptop, every local /24 swept: for a tool run while the
    OCS is stopped (bad_request.py, start_ekko_ocs.py --find)."""
    try:
        local = usable(local_ipv4_addresses())
    except OSError:
        local = []
    return search_round(local, port, topic, probe_id(team), sweep=networks(local))


def answered(tried: Iterable[dict]) -> list[dict]:
    """The hosts of a round that took a TCP connection, in order: what they said is news;
    the rest said only NO_ANSWER."""
    return [t for t in tried if not t["verdict"].startswith(NO_ANSWER)]


def said(tried: Iterable[dict]) -> str:
    """The hosts of a round that answered, and what each said, on one line."""
    return "; ".join("%s (%s): %s" % (t["host"], t["why"], t["verdict"])
                     for t in answered(tried))


# ---- the search thread -------------------------------------------------------------------

class Finder:
    """Runs rounds while the bridge has no RoboCommand link, on a thread of its own.

    The bridge tells it when that link goes up or down (`link`) and supplies three hooks:
    `pause()` runs before every round and must stop the bridge's own client so a probe
    never overlaps it -- returning False if that client connected meanwhile, which skips
    the round; `on_found(found)` gets what a round found; `on_round(round)` gets every
    round. The hooks are called outside this object's lock.

    `addresses`, `search` and `clock` are injectable so tests can drive `tick()` by hand.
    """

    def __init__(self, port: int, topic: str, team: str, *,
                 pause: Callable[[], bool], on_found: Callable[[Found], None],
                 on_round: Callable[[Round], None] = lambda _r: None,
                 log: Callable = print,
                 addresses: Callable[[], list[str]] = local_ipv4_addresses,
                 search: Callable[..., Round] = search_round,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.port, self.topic, self.team = port, topic, team
        self._pause, self._on_found, self._on_round = pause, on_found, on_round
        self.log = log
        self._addresses, self._search, self._clock = addresses, search, clock

        self._lock = threading.Lock()
        self.found: Found | None = None       #: the last broker found
        self.found_at: float | None = None    #: wall clock
        self._up = False
        self._down_since = clock()
        self._addrs: tuple[str, ...] = ()
        self._swept_at: dict[str, float] = {}   #: network -> its last sweep
        self._sweeps: dict[str, int] = {}       #: network -> sweeps since RoboCommand was found
        self._next_round = 0.0
        self._busy = False
        self._rounds = 0
        self._last: Round | None = None
        self._said_key = None
        self._episode: tuple | None = None    #: addresses of the not-found rounds logged
        self._heard: set = set()              #: (host, verdict) already logged for them
        self._halt = threading.Event()
        self._wake = threading.Event()
        self._thread: threading.Thread | None = None

    # ---- the bridge's side --------------------------------------------------------------

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="rc-find", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._halt.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(5.0)
            self._thread = None

    def link(self, up: bool) -> None:
        """The bridge's RoboCommand link came up (True) or went down (False)."""
        with self._lock:
            if up != self._up and not up:
                self._down_since = self._clock()
            self._up = up
        self._wake.set()

    def hold_reason(self, addr: str) -> str | None:
        """Why the beacon must not speak from `addr` yet, or None. Until RoboCommand has been
        found, an address whose /24 no round has swept may be on the course network."""
        net = slash24(addr)
        with self._lock:
            if self.found is not None or net is None or str(net) in self._swept_at:
                return None
        return "%s not yet searched for RoboCommand" % net

    # ---- the thread ---------------------------------------------------------------------

    def _run(self) -> None:
        while not self._halt.is_set():
            try:
                self.tick()
            except Exception as exc:  # noqa: BLE001 -- the search must not die quietly
                self._say(("error", str(exc)), "RoboCommand search: %s: %s"
                          % (type(exc).__name__, exc))
            self._wake.wait(TICK_S)
            self._wake.clear()

    def tick(self) -> Round | None:
        """Run a round if one is due; the round run, or None."""
        now = self._clock()
        try:
            addrs = tuple(usable(self._addresses()))
        except OSError:
            addrs = self._addrs                   # a resolver hiccup: keep the last list
        with self._lock:
            changed, self._addrs = addrs != self._addrs, addrs
            if changed:                           # a network gone is swept afresh if it returns
                here = {str(n) for n in networks(addrs)}
                for net in [n for n in self._swept_at if n not in here]:
                    del self._swept_at[net]
                    self._sweeps.pop(net, None)
            if not self._due(now, changed):
                return None
            self._busy = True
            last = self.found.host if self.found else None
            sweep = [n for n in networks(addrs) if self._sweep_due(str(n), now)]
        try:
            if not self._pause():
                return None
            rnd = self._search(list(addrs), self.port, self.topic, probe_id(self.team),
                               last=last, sweep=sweep, stop=self._halt)
        finally:
            with self._lock:
                self._busy = False
        self._record(rnd)
        return rnd

    def _due(self, now: float, changed: bool) -> bool:
        """Caller holds the lock."""
        if self._up:
            return False
        if self.found is not None and now - self._down_since < REDISCOVER_S:
            return False                          # paho is redialling the broker it had
        return changed or now >= self._next_round

    def _sweep_due(self, net: str, now: float) -> bool:
        """Caller holds the lock."""
        n = self._sweeps.get(net, 0)
        return n == 0 or now - self._swept_at[net] >= min(SWEEP_S * 2 ** (n - 1), SWEEP_MAX_S)

    def _record(self, rnd: Round) -> None:
        if self._halt.is_set():
            return                  # a round that outlived stop(): its hooks must not redial
        now = self._clock()
        with self._lock:
            self._rounds += 1
            self._last = rnd
            for net in rnd.swept:
                self._swept_at[net] = now
                self._sweeps[net] = self._sweeps.get(net, 0) + 1
            if rnd.found is not None:
                self.found, self.found_at = rnd.found, time.time()
                self._down_since = now            # paho's REDISCOVER_S to connect starts now
                self._sweeps.clear()              # a later search starts sweeping afresh
            else:
                self._next_round = now + ROUND_S
        if rnd.found is not None:
            f = rnd.found
            self._said_key, self._episode = None, None
            self.log("RoboCommand found at %s:%d from %s -- course %s on %s (%s; %.1f s)"
                     % (f.host, f.port, f.via or "?", f.course_id or "(no course_id)",
                        self.topic, f.why, rnd.took_s))
            self._on_found(f)
        else:
            self._say_not_found(rnd)
        self._on_round(rnd)

    def _say_not_found(self, rnd: Round) -> None:
        """Log a round that found nothing when it is news: the first of a search, new
        addresses, or a host saying something not logged yet. Every ROUND_S otherwise."""
        heard = answered(rnd.tried)
        pairs = {(t["host"], t["verdict"]) for t in heard}
        if tuple(rnd.addresses) != self._episode:
            self._episode, self._heard = tuple(rnd.addresses), set()
        elif pairs <= self._heard:
            return
        self._heard |= pairs
        self.log("RoboCommand search: not found from %s; %s%s; again every %.0f s"
                 % (", ".join(rnd.addresses) or "no usable address",
                    said(heard) or "nothing answered on TCP %d" % self.port,
                    "; swept " + ", ".join(rnd.swept) if rnd.swept else "", ROUND_S))

    def _say(self, key, message: str) -> None:
        """Log `message` unless the last one logged had the same key."""
        if key != self._said_key:
            self._said_key = key
            self.log(message)

    # ---- the page -----------------------------------------------------------------------

    def snapshot(self) -> dict:
        now = self._clock()
        with self._lock:
            f, last = self.found, self._last
            if self._busy:
                state, again = "searching", None
            elif self._up:
                state, again = "connected", None
            elif f is not None and now - self._down_since < REDISCOVER_S:
                state, again = "dialling", REDISCOVER_S - (now - self._down_since)
            else:
                state, again = "waiting", max(0.0, self._next_round - now)
            return {
                "topic": self.topic, "port": self.port, "state": state,
                "host": f.host if f else None, "via": f.via if f else None,
                "course_id": f.course_id if f else None, "why": f.why if f else None,
                "found_at": self.found_at, "rounds": self._rounds,
                "search_in_s": None if again is None else round(again, 1),
                "searched": sorted(self._swept_at),
                "last_round": None if last is None else {
                    "at": last.at, "took_s": round(last.took_s, 1),
                    "addresses": list(last.addresses), "tried": [dict(t) for t in last.tried],
                    "swept": list(last.swept), "found": last.found is not None},
            }
