"""config — every knob in one TOML file, validated on load.

TOML because tomllib is stdlib from 3.11: the OCS is a laptop that has to work
at a dock, and "pip install a YAML parser" is not a thing you want between you
and a run. Read-only is a feature here -- the bridge never writes its config.

Loading VALIDATES. A team_id typo or a tier spelled "advance" should stop the
process on the trailer with a clear message, not surface as a rejected
declaration thirty seconds before a run window closes.

THE FLEET IS TYPED, and that is why this file changed when the UUV and the UAV
arrived. A vehicle used to be a bare string in `vehicle_ids`, which was fine
while the fleet was one boat. It stops being fine the moment the vehicles
differ in what a *valid* heartbeat looks like -- see `_ALLOW_UNKNOWN_BY_TYPE`
for the single field that made a flat list actively dangerous.
"""
from __future__ import annotations

import ipaddress
import math
import tomllib
from dataclasses import dataclass
from pathlib import Path

from .beacon import (DEFAULT_INTERVAL_S, DEFAULT_NEVER_ON, DEFAULT_PORT,
                     MAX_INTERVAL_S, is_port)
from .geofence import DEFAULT_CONFIRM_MAX_AGE_S, DEFAULT_MARGIN_M

#: TaskTier names from common.proto. TIER_UNKNOWN is deliberately absent: it is
#: the proto3 zero value, never a legal thing to declare.
TIERS = ("TIER_NONE", "TIER_CORE", "TIER_ADVANCED", "TIER_DISRUPTIVE")

#: VehicleType names from rx_common.proto. TYPE_UNKNOWN is absent for the same
#: reason TIER_UNKNOWN is: it is the zero value, never a thing we declare.
VEHICLE_TYPES = ("TYPE_USV", "TYPE_UUV", "TYPE_UAV")

#: The vehicle-link transports a client may announce in its `ocs_hello`. A
#: socket that sends no hello is "wifi": today's boat and Ekko clients dial the
#: team WiFi and say nothing. "radio" is the RFD900 path, spoken by the
#: laptop-side fleet_link adapter. See docs/VEHICLE_LINK.md.
LINKS = ("radio", "wifi")

#: Which enum leaf fields each vehicle type may legitimately leave UNKNOWN.
#:
#: This was one global tuple containing "flight_phase", which was correct while
#: every vehicle was a boat and became a live bug the day a UAV joined. A
#: surface or submerged vehicle has no honest flight phase and the proto offers
#: no "not applicable" value, so UNKNOWN is the truthful encoding. An aircraft
#: has no such excuse: FLIGHT_PHASE_UNKNOWN from a UAV means the reporter
#: forgot the field, and the handbook says heartbeat telemetry for UAVs is
#: relayed to Garuda Robotics to satisfy Singapore's Network Remote ID rules.
#: A globally-permitted UNKNOWN lets that through silently -- the precise
#: failure validate.py exists to prevent, aimed at a regulator rather than at
#: the scoreboard.
_ALLOW_UNKNOWN_BY_TYPE: dict[str, tuple[str, ...]] = {
    "TYPE_USV": ("flight_phase",),
    "TYPE_UUV": ("flight_phase",),
    "TYPE_UAV": (),
}


class ConfigError(ValueError):
    """Raised with a message meant to be read on a trailer, not in a debugger."""


@dataclass(frozen=True)
class Broker:
    host: str
    port: int = 1883
    keepalive: int = 15


@dataclass(frozen=True)
class Vehicle:
    """One vehicle we report on behalf of."""

    id: str
    type: str                       #: one of VEHICLE_TYPES

    #: The vehicle whose radio actually carries these reports, when it is not
    #: this one's own. Graey is submerged and reaches Crusader over a 9600-baud
    #: acoustic modem, so its reports arrive on the team broker published by
    #: the USV. The bridge does not treat a relayed report differently on the
    #: wire -- RoboCommand must see UUV1 attributed to UUV1 either way -- but
    #: the operator needs to know which link a silence is on, and `stale_ms`
    #: below almost always needs raising when this is set.
    relay_via: str | None = None

    #: Per-vehicle staleness threshold, ms. None inherits the global. An
    #: acoustic hop cannot meet the same deadline as WiFi, and warning at the
    #: WiFi threshold twice a second would bury every warning that matters.
    stale_ms: float | None = None

    #: Which link the OCS favours when this vehicle is connected on more than
    #: one. None means no preference. Per report kind: a report on the other
    #: link is dropped as a duplicate while the preferred link has just
    #: delivered the same kind (a kind only one link carries is never dropped,
    #: and a real report beats one the other link synthesizes). Everything the
    #: OCS sends goes down the preferred link when it is up -- see linkpolicy.py.
    prefer: str | None = None

    #: Whether the OCS sends this vehicle `ocs_info` frames (the course, and for
    #: a UAV the declared geofence). Default on; turn it off for a vehicle
    #: client that cannot tolerate a top-level key it does not know.
    send_info: bool = True

    @property
    def allow_unknown(self) -> tuple[str, ...]:
        return _ALLOW_UNKNOWN_BY_TYPE[self.type]

    @property
    def is_uav(self) -> bool:
        return self.type == "TYPE_UAV"


@dataclass(frozen=True)
class Discovery:
    """`[discovery]`: the UDP beacon that tells vehicles where the OCS is.

    OFF unless bridge.toml says `enabled = true`. A config that predates the
    feature -- and every bench and test that writes its own TOML and starts a
    Bridge -- must not begin broadcasting on whatever network the laptop is on,
    advertising a test port to a real vehicle. See beacon.py for what it sends,
    where it never sends, and why.
    """

    enabled: bool = False
    port: int = DEFAULT_PORT
    interval_s: float = DEFAULT_INTERVAL_S
    #: Subnets no beacon is ever sent from. Setting this REPLACES the default
    #: (the four course networks) -- it does not add to it.
    never_on: tuple[str, ...] = DEFAULT_NEVER_ON


@dataclass(frozen=True)
class Config:
    team_id: str
    vehicles: tuple[Vehicle, ...]

    team: Broker            # our broker; the vehicles publish here
    robocommand: Broker     # theirs; the single connection the rules permit

    tiers: tuple[str, str, str, str] = ("TIER_NONE",) * 4

    #: RETIRED. The top-level `uav_geofence` of bridge.toml, parsed and kept only so
    #: nothing that reads the field breaks: the bridge no longer declares it, sends
    #: it to a vehicle or shows it. The UAV geofence is per course now (see
    #: geofence.py) and comes from the two fields below. The key stays in the file
    #: because the Drone repo's tools/scripts/check_config.py still compares it with
    #: uav_params.yaml and fails Ekko's rebuild without it.
    uav_geofence: tuple[tuple[float, float], ...] = ()

    #: `[geofence] boundary_margin_m`: how far inside the course boundary the
    #: derived UAV geofence sits when there is no `[[geofence.course]]` entry. The
    #: declared vertices must be strictly inside the boundary, never on it, where a
    #: float-rounding validation could reject them. Chase's to change.
    geofence_margin_m: float = DEFAULT_MARGIN_M
    #: `[geofence] confirm_max_age_s`: how old an `ocs_fence` read-back may be and
    #: still count as the autopilot holding the fence. The adapter re-verifies every
    #: 60 s.
    geofence_confirm_max_age_s: float = DEFAULT_CONFIRM_MAX_AGE_S
    #: `[[geofence.course]]`: course_id -> the hand-written CLOSED fence for it, as
    #: ((course_id, ((lat, lon), ...)), ...). A tuple of pairs because the config
    #: is frozen; `geofence_table` is the dict.
    geofence_courses: tuple[tuple[str, tuple[tuple[float, float], ...]], ...] = ()

    rate: float = 5.0
    heartbeat_reserve: float = 2.0
    stale_ms: float = 1000.0

    #: Fewest vehicles a RunDeclaration may name. Handbook 3.1 requires the
    #: Communications Proof of Readiness run to be performed with at least
    #: two, so 2 is the real value and anything lower produces rc-test logs
    #: that fail review. It is configurable ONLY so single-vehicle bench
    #: rigs can exercise the rest of the sequence; the bridge warns on every
    #: start when it is below two, because a floor lowered for a bench and
    #: forgotten is exactly how it would be wrong on the day.
    min_declared: int = 2

    #: A report on the non-preferred link is dropped as a duplicate while the
    #: preferred link has delivered the same KIND within this many seconds (or a
    #: real one has beaten a synthesized one: see linkpolicy.py), and a link
    #: that has been silent longer than this stops counting as "fresh" when the
    #: OCS chooses where to send. Generous against a 2 Hz heartbeat (six missed
    #: frames), short enough that a dead radio hands over to WiFi in a couple
    #: of seconds rather than minutes.
    link_prefer_s: float = 3.0

    seq_store: Path = Path("run/seq.json")
    wire_log: Path = Path("run/wire")

    #: Where vehicles dial in. 0.0.0.0 because the OCS is dual-homed and the
    #: team NIC is not the one we bind by name -- see README on why this must
    #: never be 127.0.0.1 (nothing off the laptop could reach it).
    tcp_host: str = "0.0.0.0"
    tcp_port: int = 37564
    tcp_enabled: bool = True

    discovery: Discovery = Discovery()

    # ---- fleet lookups ----------------------------------------------------

    @property
    def vehicle_ids(self) -> tuple[str, ...]:
        """Declaration order, which is config order. Stable on purpose:
        RunDeclaration.vehicle_ids is a repeated field, and diffing this run's
        wire log against the last one is easier when the order does not wander.
        """
        return tuple(v.id for v in self.vehicles)

    def vehicle(self, vehicle_id: str) -> Vehicle | None:
        for v in self.vehicles:
            if v.id == vehicle_id:
                return v
        return None

    def stale_ms_for(self, vehicle_id: str) -> float:
        v = self.vehicle(vehicle_id)
        if v is not None and v.stale_ms is not None:
            return v.stale_ms
        return self.stale_ms

    def has_uav(self) -> bool:
        return any(v.type == "TYPE_UAV" for v in self.vehicles)

    @property
    def geofence_table(self) -> dict[str, tuple[tuple[float, float], ...]]:
        """`[[geofence.course]]` as {course_id: closed polygon}."""
        return dict(self.geofence_courses)

    # ---- topics: one place, so the two namespaces cannot drift ------------

    def team_report_sub(self) -> str:
        return f"team/robotx/{self.team_id}/+/report"

    def team_command_topic(self, vehicle_id: str) -> str:
        """Where the OCS hands an inbound RxCommand to the vehicle it names.

        Mirrors the report topic rather than inventing a second shape, so a
        capture on the team broker reads as one namespace in both directions.
        """
        return f"team/robotx/{self.team_id}/{vehicle_id}/command"

    def rc_report_topic(self, vehicle_id: str) -> str:
        return f"robocommand/robotx/{self.team_id}/{vehicle_id}/report"

    def rc_request_topic(self) -> str:
        return f"robocommand/robotx/{self.team_id}/request"

    def rc_command_sub(self) -> str:
        return f"robocommand/robotx/{self.team_id}/command"

    @staticmethod
    def rc_course_sub() -> str:
        return "robocommand/robotx/course"


def _broker(raw: dict, name: str) -> Broker:
    try:
        section = raw[name]
    except KeyError:
        raise ConfigError(f"[{name}] section missing") from None
    if "host" not in section:
        raise ConfigError(f"[{name}] needs a host")
    return Broker(
        host=str(section["host"]),
        port=int(section.get("port", 1883)),
        keepalive=int(section.get("keepalive", 15)),
    )


def _vehicles(raw: dict) -> tuple[Vehicle, ...]:
    entries = raw.get("vehicle", [])
    if not entries:
        raise ConfigError(
            "no vehicles configured -- declare at least one [[vehicle]] with "
            "an id and a type"
        )

    out: list[Vehicle] = []
    for n, entry in enumerate(entries, start=1):
        if not isinstance(entry, dict):
            raise ConfigError(f"[[vehicle]] #{n} is not a table")

        vid = str(entry.get("id", "")).strip()
        if not vid:
            raise ConfigError(f"[[vehicle]] #{n} needs an id")

        vtype = str(entry.get("type", "")).strip()
        if vtype not in VEHICLE_TYPES:
            raise ConfigError(
                f"vehicle {vid}: type = {vtype!r} is not one of "
                f"{', '.join(VEHICLE_TYPES)}"
            )

        relay = entry.get("relay_via")
        stale = entry.get("stale_ms")

        prefer = entry.get("prefer")
        if prefer is not None and prefer not in LINKS:
            raise ConfigError(
                f"vehicle {vid}: prefer = {prefer!r} is not one of "
                f"{', '.join(LINKS)}"
            )
        send_info = entry.get("send_info", True)
        if not isinstance(send_info, bool):
            raise ConfigError(
                f"vehicle {vid}: send_info = {send_info!r} must be true or false"
            )

        out.append(
            Vehicle(
                id=vid,
                type=vtype,
                relay_via=str(relay) if relay else None,
                stale_ms=float(stale) if stale is not None else None,
                prefer=prefer,
                send_info=send_info,
            )
        )

    ids = [v.id for v in out]
    if len(set(ids)) != len(ids):
        raise ConfigError(f"vehicle ids must be unique, got {tuple(ids)}")

    types = [v.type for v in out]
    if len(set(types)) != len(types):
        # Not illegal in the protocol, but it is not our fleet, and a repeated
        # type is nearly always a copy-pasted block that left two vehicles
        # claiming to be the same craft. Say so at load, not at scoring.
        raise ConfigError(
            f"two vehicles share a type: {tuple(types)} -- check for a "
            f"copy-pasted [[vehicle]] block"
        )

    known = set(ids)
    for v in out:
        if v.relay_via is None:
            continue
        if v.relay_via not in known:
            raise ConfigError(
                f"vehicle {v.id}: relay_via = {v.relay_via!r} is not a "
                f"configured vehicle"
            )
        if v.relay_via == v.id:
            raise ConfigError(f"vehicle {v.id}: relay_via points at itself")

    return tuple(out)


def _discovery(raw: dict) -> Discovery:
    section = raw.get("discovery", {})
    if not isinstance(section, dict):
        raise ConfigError("[discovery] is not a table")

    enabled = section.get("enabled", False)
    if not isinstance(enabled, bool):
        raise ConfigError(f"discovery.enabled = {enabled!r} must be true or false")

    port = section.get("port", DEFAULT_PORT)
    if not is_port(port):
        raise ConfigError(f"discovery.port = {port!r} must be a UDP port, 1-65535")

    interval = section.get("interval_s", DEFAULT_INTERVAL_S)
    if (isinstance(interval, bool) or not isinstance(interval, (int, float))
            or not 0 < interval <= MAX_INTERVAL_S):
        raise ConfigError(
            f"discovery.interval_s = {interval!r} must be above 0 and at most "
            f"{MAX_INTERVAL_S} -- the boat forgets a beacon after 5 s, and two "
            f"inside that window is the least that survives one lost datagram"
        )

    never_on = section.get("never_on", list(DEFAULT_NEVER_ON))
    if not isinstance(never_on, list):
        raise ConfigError("discovery.never_on must be a list of subnets")
    nets: list[str] = []
    for entry in never_on:
        try:
            # A str only: ip_network(65) is happily read as the address 0.0.0.65.
            if not isinstance(entry, str):
                raise ValueError(entry)
            net = ipaddress.ip_network(entry, strict=False)
        except ValueError:
            raise ConfigError(
                f"discovery.never_on entry {entry!r} is not a subnet like "
                f"192.168.65.0/24"
            ) from None
        if net.version != 4:
            raise ConfigError(f"discovery.never_on entry {entry!r} is not IPv4")
        nets.append(str(net))

    return Discovery(enabled=enabled, port=port, interval_s=float(interval),
                     never_on=tuple(nets))


def _number(section: dict, key: str, default: float, where: str) -> float:
    """A positive finite number from `section`, or ConfigError naming where."""
    v = section.get(key, default)
    if (isinstance(v, bool) or not isinstance(v, (int, float))
            or not math.isfinite(v) or not v > 0):
        raise ConfigError(f"{where}.{key} = {v!r} must be a number above 0")
    return float(v)


def _geofence(raw: dict) -> tuple[float, float, tuple]:
    """`[geofence]`: (boundary_margin_m, confirm_max_age_s, the per-course table).

    Every default lives in geofence.py, so a config with no `[geofence]` section
    behaves exactly as the shipped bridge.toml does. A table entry is validated as
    hard as anything else the OCS declares to RoboCommand: it must be a closed
    polygon of at least three distinct vertices, each a real position. Whether it is
    INSIDE the course is not knowable here -- the boundary arrives from RoboCommand
    -- and is checked when the course does (geofence.for_course).
    """
    section = raw.get("geofence", {})
    if not isinstance(section, dict):
        raise ConfigError("[geofence] is not a table")
    margin = _number(section, "boundary_margin_m", DEFAULT_MARGIN_M, "geofence")
    max_age = _number(section, "confirm_max_age_s", DEFAULT_CONFIRM_MAX_AGE_S, "geofence")

    entries = section.get("course", [])
    if not isinstance(entries, list):
        raise ConfigError("geofence.course must be an array of [[geofence.course]] tables")
    table: list[tuple[str, tuple[tuple[float, float], ...]]] = []
    for n, entry in enumerate(entries, start=1):
        where = f"[[geofence.course]] #{n}"
        if not isinstance(entry, dict):
            raise ConfigError(f"{where} is not a table")
        cid = entry.get("course_id")
        if not isinstance(cid, str) or not cid.strip():
            raise ConfigError(f"{where} needs a course_id, e.g. \"ALPHA\"")
        cid = cid.strip()
        where = f"[[geofence.course]] {cid}"
        poly = entry.get("uav_geofence")
        if not isinstance(poly, list):
            raise ConfigError(f"{where} needs uav_geofence = [[lat, lon], ...]")
        pts: list[tuple[float, float]] = []
        for p in poly:
            ok = (isinstance(p, list) and len(p) == 2
                  and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                          and math.isfinite(v) for v in p)
                  and -90.0 <= p[0] <= 90.0 and -180.0 <= p[1] <= 180.0)
            if not ok:
                raise ConfigError(f"{where}: uav_geofence holds {p!r}, not a [lat, lon]")
            pts.append((float(p[0]), float(p[1])))
        if len(pts) < 4 or pts[0] != pts[-1]:
            raise ConfigError(
                f"{where}: uav_geofence must be closed (first point == last) and have "
                f"at least 3 distinct vertices -- RoboCommand rejects fewer than 4 "
                f"points")
        if len(set(pts)) < 3:       # closed: the last point repeats the first
            raise ConfigError(f"{where}: uav_geofence repeats a vertex; it needs at "
                              f"least 3 distinct ones")
        if cid in {c for c, _ in table}:
            raise ConfigError(f"{where}: two [[geofence.course]] entries for {cid}")
        table.append((cid, tuple(pts)))
    return margin, max_age, tuple(table)


def load(path: str | Path) -> Config:
    path = Path(path)
    try:
        raw = tomllib.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise ConfigError(f"no config at {path}") from None
    except tomllib.TOMLDecodeError as exc:
        raise ConfigError(f"{path} is not valid TOML: {exc}") from None

    team_id = str(raw.get("team_id", "")).strip()
    if not team_id:
        raise ConfigError("team_id is required")

    vehicles = _vehicles(raw)

    tiers_raw = raw.get("tiers", {})
    tiers = tuple(str(tiers_raw.get(f"task{n}", "TIER_NONE")) for n in (1, 2, 3, 4))
    for n, tier in enumerate(tiers, start=1):
        if tier not in TIERS:
            raise ConfigError(
                f"tiers.task{n} = {tier!r} is not one of {', '.join(TIERS)}"
            )

    # RETIRED (see Config.uav_geofence): read so the field is still there, never
    # validated and never used. It used to refuse a UAV fleet with no fence; with a
    # fence per course there is legitimately none until a course arrives.
    try:
        fence = tuple((float(a), float(b)) for a, b in raw.get("uav_geofence", []))
    except (TypeError, ValueError):
        raise ConfigError("uav_geofence (retired) is not a list of [lat, lon] pairs") from None
    margin_m, confirm_max_age_s, courses = _geofence(raw)

    limits = raw.get("limits", {})
    paths = raw.get("paths", {})
    tcp = raw.get("tcp", {})

    link_prefer_s = float(limits.get("link_prefer_s", 3.0))
    if not link_prefer_s > 0:
        raise ConfigError(
            f"limits.link_prefer_s = {link_prefer_s} must be greater than zero"
        )

    return Config(
        team_id=team_id,
        vehicles=vehicles,
        team=_broker(raw, "team"),
        robocommand=_broker(raw, "robocommand"),
        tiers=tiers,  # type: ignore[arg-type]
        uav_geofence=fence,
        geofence_margin_m=margin_m,
        geofence_confirm_max_age_s=confirm_max_age_s,
        geofence_courses=courses,
        rate=float(limits.get("rate", 5.0)),
        heartbeat_reserve=float(limits.get("heartbeat_reserve", 2.0)),
        stale_ms=float(limits.get("stale_ms", 1000.0)),
        min_declared=int(limits.get("min_declared", 2)),
        link_prefer_s=link_prefer_s,
        seq_store=Path(paths.get("seq_store", "run/seq.json")),
        wire_log=Path(paths.get("wire_log", "run/wire")),
        tcp_host=str(tcp.get("host", "0.0.0.0")),
        tcp_port=int(tcp.get("port", 37564)),
        tcp_enabled=bool(tcp.get("enabled", True)),
        discovery=_discovery(raw),
    )
