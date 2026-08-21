"""config — every knob in one TOML file, validated on load.

TOML because tomllib is stdlib from 3.11: the OCS is a laptop that has to work
at a dock, and "pip install a YAML parser" is not a thing you want between you
and a run. Read-only is a feature here -- the bridge never writes its config.

Loading VALIDATES. A team_id typo or a tier spelled "advance" should stop the
process on the trailer with a clear message, not surface as a rejected
declaration thirty seconds before a run window closes.
"""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, field
from pathlib import Path

#: TaskTier names from common.proto. TIER_UNKNOWN is deliberately absent: it is
#: the proto3 zero value, never a legal thing to declare.
TIERS = ("TIER_NONE", "TIER_CORE", "TIER_ADVANCED", "TIER_DISRUPTIVE")


class ConfigError(ValueError):
    """Raised with a message meant to be read on a trailer, not in a debugger."""


@dataclass(frozen=True)
class Broker:
    host: str
    port: int = 1883
    keepalive: int = 15


@dataclass(frozen=True)
class Config:
    team_id: str
    vehicle_ids: tuple[str, ...]

    team: Broker            # our broker; the vehicles publish here
    robocommand: Broker     # theirs; the single connection the rules permit

    tiers: tuple[str, str, str, str] = ("TIER_NONE",) * 4
    uav_geofence: tuple[tuple[float, float], ...] = ()

    rate: float = 5.0
    heartbeat_reserve: float = 2.0
    stale_ms: float = 1000.0

    seq_store: Path = Path("run/seq.json")
    wire_log: Path = Path("run/wire")

    #: Fields allowed to be *_UNKNOWN on the wire. flight_phase has no meaning
    #: for a surface vehicle, and the proto offers no "not applicable" value --
    #: so UNKNOWN is the honest encoding rather than a bug the validator should
    #: reject. current_task is never on this list: the handbook forbids it.
    allow_unknown: tuple[str, ...] = ("flight_phase",)

    # ---- topics: one place, so the two namespaces cannot drift ------------

    def team_report_sub(self) -> str:
        return f"team/robotx/{self.team_id}/+/report"

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

    vehicles = tuple(str(v) for v in raw.get("vehicle_ids", []))
    if not vehicles:
        raise ConfigError("vehicle_ids must list at least one vehicle")
    if len(set(vehicles)) != len(vehicles):
        raise ConfigError(f"vehicle_ids must be unique, got {vehicles}")

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

    limits = raw.get("limits", {})
    paths = raw.get("paths", {})

    return Config(
        team_id=team_id,
        vehicle_ids=vehicles,
        team=_broker(raw, "team"),
        robocommand=_broker(raw, "robocommand"),
        tiers=tiers,  # type: ignore[arg-type]
        uav_geofence=fence,
        rate=float(limits.get("rate", 5.0)),
        heartbeat_reserve=float(limits.get("heartbeat_reserve", 2.0)),
        stale_ms=float(limits.get("stale_ms", 1000.0)),
        seq_store=Path(paths.get("seq_store", "run/seq.json")),
        wire_log=Path(paths.get("wire_log", "run/wire")),
    )
