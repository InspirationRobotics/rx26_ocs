"""proto — the one place that knows how the generated protobuf is imported.

protoc emits flat, top-level imports (`import common_pb2`) even for files that
live in a package, so `from rx_bridge.gen.robotx import rx_reports_pb2` fails on
the generated code's own internal imports. The usual fix is to put the
generation root on sys.path; the usual mistake is to do it in five modules and
have four of them drift.

So: everything imports its protobuf FROM HERE, and this file is the only thing
that touches sys.path.

Regenerate with ocs/make_protos.sh, which pins the robocommand revision. The
generated tree is COMMITTED -- see that script for why.
"""
from __future__ import annotations

import sys
from pathlib import Path

_GEN = Path(__file__).resolve().parent / "gen"

if not _GEN.is_dir():
    raise ImportError(
        f"no generated protobuf at {_GEN}\n"
        f"Run:  bash ocs/make_protos.sh\n"
        f"(it clones robonation/robocommand at a pinned revision and runs protoc)"
    )

if str(_GEN) not in sys.path:
    sys.path.insert(0, str(_GEN))

import common_pb2                      # noqa: E402
from robotx import rx_commands_pb2     # noqa: E402
from robotx import rx_common_pb2       # noqa: E402
from robotx import rx_course_pb2       # noqa: E402
from robotx import rx_reports_pb2      # noqa: E402
from robotx import rx_requests_pb2     # noqa: E402

__all__ = [
    "common_pb2",
    "rx_commands_pb2",
    "rx_common_pb2",
    "rx_course_pb2",
    "rx_reports_pb2",
    "rx_requests_pb2",
]
