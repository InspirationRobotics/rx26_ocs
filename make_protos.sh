#!/usr/bin/env bash
# make_protos.sh -- generate the RoboCommand protobuf bindings, from a PINNED revision.
#
# WHY THE GENERATED TREE IS COMMITTED. Two reasons, and the second is the one
# that matters at the dock:
#
#   1. protoc then never has to exist on the Jetson, inside the asv container,
#      or on whatever laptop is standing in for the OCS that morning.
#   2. Our wire format cannot shift underneath us. RoboNation can renumber a
#      field in their repo at any time; if we regenerated on every build we
#      would find out by failing a run. Pinning means we adopt their changes
#      deliberately, on a day we chose, with the diff in front of us.
#
# Bump REV on purpose, read the diff, re-run the conformance test, commit.
set -euo pipefail

REV="${REV:-f6457fa47489647a8650c1d3eb74efa84ce4b84a}"
REPO="https://github.com/robonation/robocommand.git"

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
gen="$here/rx_bridge/gen"

# WHICH protoc. The schemas declare edition = "2024", so libprotoc must be new
# enough to know that edition -- roughly 30+. This rules out most distro
# packages: Ubuntu 22.04 ships libprotoc 3.12, which fails with
#   "Edition 2024 is later than the maximum supported edition 2023"
# So prefer the wheel from requirements.txt, which pins a protoc that works, and
# only fall back to a system protoc if there is no venv.
PY_BIN=""
for cand in "$here/.venv/bin/python" "$here/.venv/Scripts/python.exe"; do
    [ -x "$cand" ] && PY_BIN="$cand" && break
done

if [ -n "$PY_BIN" ] && "$PY_BIN" -c "import grpc_tools.protoc" 2>/dev/null; then
    protoc_cmd() { "$PY_BIN" -m grpc_tools.protoc "$@"; }
elif command -v protoc >/dev/null 2>&1; then
    echo "no venv; falling back to system protoc ($(protoc --version))" >&2
    echo "if this fails on 'edition 2024', build the venv from requirements.txt" >&2
    protoc_cmd() { protoc "$@"; }
else
    echo "No protoc. Build the venv first:" >&2
    echo "  python -m venv ocs/.venv" >&2
    echo "  ocs/.venv/bin/python -m pip install -r ocs/requirements.txt" >&2
    exit 1
fi

if [ "$REV" = "main" ]; then
    echo "WARNING: REV=main. Pin a SHA before the competition -- see the header." >&2
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

git clone --quiet --depth 50 "$REPO" "$tmp/robocommand"
git -C "$tmp/robocommand" checkout --quiet "$REV"
sha="$(git -C "$tmp/robocommand" rev-parse HEAD)"

proto_dir="$tmp/robocommand/RobotX_2026/proto"
[ -d "$proto_dir" ] || { echo "no proto/ at $proto_dir -- upstream moved it" >&2; exit 1; }

rm -rf "$gen"
mkdir -p "$gen"
# -I the proto root so generated imports resolve flat; rx_bridge/proto.py puts
# that same root on sys.path so they resolve at runtime too.
( cd "$proto_dir" && protoc_cmd -I . --python_out="$gen" $(find . -name '*.proto') )

# Namespace packages would work, but an explicit __init__.py keeps `robotx`
# importable regardless of how the interpreter was launched.
touch "$gen/__init__.py"
[ -d "$gen/robotx" ] && touch "$gen/robotx/__init__.py"

printf '%s\n' "$sha" > "$gen/REVISION"
echo "generated from robocommand@$sha into $gen"
echo "commit the gen/ tree, including REVISION."
