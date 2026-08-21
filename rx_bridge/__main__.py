"""rx_bridge — run the bridge, and drive it from a prompt.

    python -m rx_bridge --config bridge.toml

There is no web UI yet and deliberately so: the operator actions that exist
today are `declare` and `end`, and a prompt makes both of them exactly as
reviewable as a page would while being three orders of magnitude less code.
When Task 4 arrives -- command-response chains that need a human in under ten
seconds -- that is the moment a page earns its place, not before.

The prompt blocks on stdin; MQTT runs on paho's own threads. Nothing in the
loop below touches shared state without going through Bridge, which holds the
lock.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from .bridge import Bridge
from .config import ConfigError, load

HELP = """commands:
  declare   publish the RunDeclaration (needs the retained RxCourse first)
  status    run state, per-vehicle silence, counters, seq, wire log path
  end       end the run; reports stop
  quit      shut down
"""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="rx_bridge", description=__doc__)
    ap.add_argument("--config", default="bridge.toml", type=Path)
    ap.add_argument(
        "--declare-on-course",
        action="store_true",
        help="declare as soon as the course arrives. Bench only -- at a real "
             "run the declaration is a decision a person makes.",
    )
    args = ap.parse_args(argv)

    try:
        cfg = load(args.config)
    except ConfigError as exc:
        print("config: %s" % exc, file=sys.stderr)
        return 2

    bridge = Bridge(cfg)
    if args.declare_on_course:
        _autodeclare(bridge)

    bridge.start()
    print(HELP)

    try:
        while True:
            try:
                line = input("rx> ").strip().lower()
            except EOFError:
                break
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            elif line in ("d", "declare"):
                print(bridge.declare())
            elif line in ("s", "status"):
                print(bridge.status())
            elif line in ("e", "end"):
                print(bridge.end())
            elif line in ("h", "help", "?"):
                print(HELP)
            else:
                print("unknown: %s" % line)
    except KeyboardInterrupt:
        pass
    finally:
        print("\n" + bridge.status())
        bridge.stop()
    return 0


def _autodeclare(bridge: Bridge) -> None:
    """Wrap on_course so the bench can run the whole sequence unattended."""
    inner = bridge.machine.on_course

    def wrapped(course):
        verdict = inner(course)
        if verdict.ok and bridge.machine.may_declare():
            print("auto-declare: " + bridge.declare())
        return verdict

    bridge.machine.on_course = wrapped  # type: ignore[method-assign]


if __name__ == "__main__":
    raise SystemExit(main())
