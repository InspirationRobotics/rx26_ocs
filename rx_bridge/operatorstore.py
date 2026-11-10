"""operatorstore -- the two things RoboCommand needs that no vehicle reports: the UAV
pilot's call sign and each aircraft's registration. The operator types them on the page.

Everything else the OCS sends RoboCommand is relayed as a vehicle reported it. These two
come from nothing on board: Ekko does not know who is flying it, and its heartbeats carry
no registration. So they are typed on the page -- and may be saved as THIS LAPTOP's
defaults, in run/operator.json beside the sequence counters (gitignored like all of run/:
who is flying is not a team-wide fact).

Each value is the first of:
  1. what was typed on the page since the bridge started -- a blank too: a box the
     operator emptied stays empty, never back-filled from a default;
  2. this laptop's saved default (run/operator.json);
  3. bridge.toml's: `[uav] pilot_callsign`, and the aircraft's `registration`.

Saving also remembers the value, newest first, so the page can offer it again: more than
one pilot, more than one aircraft. A blank saved clears that default.

Where they go: the call sign into RunDeclaration.uav_pilot_callsign (blank blocks a
declaration with an aircraft: bridge._uav_rules); the registration onto every heartbeat of
that aircraft, Heartbeat.uav_registration (blank leaves it blank, which RoboCommand accepts
while the team has one UAV registered).
"""
from __future__ import annotations

import json
from pathlib import Path

from .seqstore import write_atomic

#: Longest call sign or registration accepted. Neither has a documented limit: this stops
#: a paste of something else, it is not a rule of RoboCommand's.
MAX_LEN = 64
#: How many saved values of each kind the page offers again.
REMEMBER = 8


class OperatorError(ValueError):
    """Something typed that cannot be used, in words for the page."""


def clean(value, what: str) -> str:
    """`value` as it will be sent: whitespace runs (a tab, a pasted non-breaking space)
    become one space, the ends are trimmed. OperatorError when it cannot be sent."""
    if not isinstance(value, str):
        raise OperatorError("%s must be text, not %r" % (what, value))
    value = " ".join(value.split())
    if len(value) > MAX_LEN:
        raise OperatorError("%s is %d characters; at most %d" % (what, len(value), MAX_LEN))
    if not value.isprintable():
        raise OperatorError("%s has a character in it that cannot be sent" % what)
    return value


def _q(value: str) -> str:
    return repr(value) if value else "blank"


def _remember(values: list[str], value: str) -> list[str]:
    """`value` first, then the rest without it, at most REMEMBER."""
    return ([value] + [v for v in values if v != value])[:REMEMBER] if value else values


class OperatorStore:
    """The call sign and the registrations, with where each came from. Not thread-safe:
    the bridge calls it under its own lock."""

    def __init__(self, path: str | Path, callsign: str = "",
                 registrations: dict[str, str] | None = None, log=print) -> None:
        """callsign       bridge.toml's `[uav] pilot_callsign`
        registrations  every aircraft in bridge.toml -> its `registration` ("" for none);
                       the ids a registration may be typed for"""
        self.path = Path(path)
        self._config_callsign = callsign
        self._config_reg = dict(registrations or {})
        self._typed_callsign: str | None = None
        self._typed_reg: dict[str, str] = {}
        self._saved_callsign: str | None = None
        #: Kept for every id in the file, bridge.toml's aircraft or not, so a save made
        #: under one config does not erase another's.
        self._saved_reg: dict[str, str] = {}
        self._callsigns: list[str] = []
        self._registrations: list[str] = []
        #: Why run/operator.json (or part of it) was not used, for the page; None if it was.
        self.load_error: str | None = None
        self._load(log)

    # ---- the values ----------------------------------------------------------

    @property
    def callsign(self) -> str:
        return self._pick(self._typed_callsign, self._saved_callsign, self._config_callsign)[0]

    def registration(self, vid: str) -> str:
        """`vid`'s registration; "" for none, and for an id that is not an aircraft."""
        return self._reg_field(vid)[0]

    @staticmethod
    def _pick(typed, saved, config) -> tuple[str, str | None]:
        """(value, where from: "typed" | "saved" | "config" | None for nowhere)."""
        if typed is not None:
            return typed, "typed"
        if saved is not None:
            return saved, "saved"
        return config, "config" if config else None

    def _reg_field(self, vid: str) -> tuple[str, str | None]:
        if vid not in self._config_reg:
            return "", None
        return self._pick(self._typed_reg.get(vid), self._saved_reg.get(vid),
                          self._config_reg[vid])

    # ---- typing and saving ---------------------------------------------------

    def set(self, *, callsign: str | None = None, registration: dict | None = None,
            save: bool = False, registration_frozen: str | None = None) -> list[str]:
        """Take what was typed; with `save`, keep it as this laptop's default too. Only
        what is given changes. Everything is checked, and a save written, before anything
        changes: an OperatorError leaves it all as it was. Returns what changed, in words.

        registration_frozen  why no registration may change now (the bridge: a run is
                             on); one given unchanged is let through"""
        sign = None if callsign is None else clean(callsign, "the pilot's call sign")
        regs: dict[str, str] = {}
        if registration is not None:
            if not isinstance(registration, dict):
                raise OperatorError("registration must be {aircraft id: registration}")
            for vid, value in registration.items():
                if vid not in self._config_reg:
                    raise OperatorError("%s is not an aircraft in bridge.toml (aircraft: %s)"
                                        % (vid, ", ".join(self._config_reg) or "none"))
                regs[vid] = clean(value, "%s's registration" % vid)
                if registration_frozen and regs[vid] != self.registration(vid):
                    raise OperatorError(registration_frozen)

        lines = []
        if save and (sign is not None or regs):
            saved_sign = self._saved_callsign if sign is None else (sign or None)
            saved_reg = dict(self._saved_reg)
            for vid, value in regs.items():
                if value:
                    saved_reg[vid] = value
                else:
                    saved_reg.pop(vid, None)
            callsigns = self._callsigns if sign is None else _remember(self._callsigns, sign)
            registrations = self._registrations
            for value in regs.values():
                registrations = _remember(registrations, value)
            try:
                self.path.parent.mkdir(parents=True, exist_ok=True)
                write_atomic(self.path, json.dumps(
                    {"callsign": saved_sign, "registration": saved_reg,
                     "callsigns": callsigns, "registrations": registrations}, indent=2))
            except OSError as exc:
                raise OperatorError("could not save to %s: %s -- nothing changed"
                                    % (self.path, exc)) from None
            for what, before, after in (
                    [("the pilot's call sign", self._saved_callsign, saved_sign)]
                    + [("%s's registration" % vid, self._saved_reg.get(vid), saved_reg.get(vid))
                       for vid in regs]):
                if before != after:
                    lines.append("%s default %s on this laptop (was %s)"
                                 % (what, "saved as %r" % after if after else "cleared",
                                    _q(before or "")))
            self._saved_callsign, self._saved_reg = saved_sign, saved_reg
            self._callsigns, self._registrations = callsigns, registrations
            self.load_error = None

        if sign is not None:
            before = self.callsign
            self._typed_callsign = sign
            if sign != before:
                lines.append("the pilot's call sign is %s (was %s)" % (_q(sign), _q(before)))
        for vid, value in regs.items():
            before = self.registration(vid)
            self._typed_reg[vid] = value
            if value != before:
                lines.append("%s's registration is %s (was %s)" % (vid, _q(value), _q(before)))
        return lines

    # ---- the page ------------------------------------------------------------

    def view(self) -> dict:
        """For the page: each value with where it came from, the saved default and
        bridge.toml's beside it, and what to offer in the boxes' lists."""
        def field(value_from, saved, config):
            value, src = value_from
            return {"value": value, "from": src, "saved": saved, "config": config}

        return {
            "path": str(self.path),
            "load_error": self.load_error,
            "callsign": field(self._pick(self._typed_callsign, self._saved_callsign,
                                         self._config_callsign),
                              self._saved_callsign, self._config_callsign),
            "registration": {vid: field(self._reg_field(vid), self._saved_reg.get(vid), config)
                             for vid, config in self._config_reg.items()},
            "callsigns": _offer(self._callsigns, [self._config_callsign]),
            "registrations": _offer(self._registrations, self._config_reg.values()),
        }

    # ---- the file ------------------------------------------------------------

    def _load(self, log) -> None:
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except (OSError, ValueError) as exc:
            self._unusable("could not be read (%s)" % exc, log)
            return
        if not isinstance(raw, dict):
            self._unusable("is not a JSON object", log)
            return
        bad = []

        def take(value, what):
            try:
                return clean(value, what) or None
            except OperatorError as exc:
                bad.append(str(exc))
                return None

        if raw.get("callsign") is not None:
            self._saved_callsign = take(raw["callsign"], "the saved call sign")
        regs = raw.get("registration") or {}
        if isinstance(regs, dict):
            for vid, value in regs.items():
                value = take(value, "%s's saved registration" % vid)
                if value:
                    self._saved_reg[str(vid)] = value
        else:
            bad.append("`registration` is not an object")
        for key in ("callsigns", "registrations"):
            values = raw.get(key) or []
            kept = [v for v in (take(v, "a remembered value") for v in values
                                if isinstance(values, list)) if v]
            setattr(self, "_" + key, kept[:REMEMBER])
        if bad:
            self._unusable("has values that were left out: " + "; ".join(bad), log)

    def _unusable(self, why: str, log) -> None:
        self.load_error = ("%s %s -- bridge.toml's values apply where it gave none; saving a "
                           "default rewrites it" % (self.path, why))
        log("operator: " + self.load_error)


def _offer(remembered: list[str], config) -> list[str]:
    """The remembered values, then bridge.toml's, each once."""
    out: list[str] = []
    for value in [*remembered, *config]:
        if value and value not in out:
            out.append(value)
    return out
