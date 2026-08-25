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

import tomllib
from dataclasses import dataclass
from pathlib import Path

#: TaskTier names from common.proto. TIER_UNKNOWN is deliberately absent: it is
#: the proto3 zero value, never a legal thing to declare.
TIERS = ("TIER_NONE", "TIER_CORE", "TIER_ADVANCED", "TIER_DISRUPTIVE")

#: VehicleType names from rx_common.proto. TYPE_UNKNOWN is absent for the same
#: reason TIER_UNKNOWN is: it is the zero value, never a thing we declare.
VEHICLE_TYPES = ("TYPE_USV", "TYPE_UUV", "TYPE_UAV")

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

    @property
    def allow_unknown(self) -> tuple[str, ...]:
        return _ALLOW_UNKNOWN_BY_TYPE[self.type]


@dataclass(frozen=True)
class Config:
    team_id: str
    vehicles: tuple[Vehicle, ...]

    team: Broker            # our broker; the vehicles publish here
    robocommand: Broker     # theirs; the single connection the rules permit

    tiers: tuple[str, str, str, str] = ("TIER_NONE",) * 4
    uav_geofence: tuple[tuple[float, float], ...] = ()

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

    seq_store: Path = Path("run/seq.json")
    wire_log: Path = Path("run/wire")

    #: Where vehicles dial in. 0.0.0.0 because the OCS is dual-homed and the
    #: team NIC is not the one we bind by name -- see README on why this must
    #: never be 127.0.0.1 (nothing off the laptop could reach it).
    tcp_host: str = "0.0.0.0"
    tcp_port: int = 37564
    tcp_enabled: bool = True

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
        out.append(
            Vehicle(
                id=vid,
                type=vtype,
                relay_via=str(relay) if relay else None,
                stale_ms=float(stale) if stale is not None else None,
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

    fence = tuple((float(a), float(b)) for a, b in raw.get("uav_geofence", []))
    if fence and fence[0] != fence[-1]:
        raise ConfigError(
            "uav_geofence must be closed -- first and last points identical"
        )
    if any(v.type == "TYPE_UAV" for v in vehicles) and not fence:
        # Declaring an aircraft with no geofence is not a thing to discover from
        # a rejected RunDeclaration. Fail here, where the message can name the
        # file that is wrong.
        raise ConfigError(
            "a TYPE_UAV vehicle is configured but uav_geofence is empty -- a "
            "declaration that includes a UAV must carry its geofence"
        )

    limits = raw.get("limits", {})
    paths = raw.get("paths", {})
    tcp = raw.get("tcp", {})

    return Config(
        team_id=team_id,
        vehicles=vehicles,
        team=_broker(raw, "team"),
        robocommand=_broker(raw, "robocommand"),
        tiers=tiers,  # type: ignore[arg-type]
        uav_geofence=fence,
        rate=float(limits.get("rate", 5.0)),
        heartbeat_reserve=float(limits.get("heartbeat_reserve", 2.0)),
        stale_ms=float(limits.get("stale_ms", 1000.0)),
        min_declared=int(limits.get("min_declared", 2)),
        seq_store=Path(paths.get("seq_store", "run/seq.json")),
        wire_log=Path(paths.get("wire_log", "run/wire")),
        tcp_host=str(tcp.get("host", "0.0.0.0")),
        tcp_port=int(tcp.get("port", 37564)),
        tcp_enabled=bool(tcp.get("enabled", True)),
    )
