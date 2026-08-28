#!/bin/bash
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${ROOT}/build"

# First argument = demo binary basename; remaining args pass through.
DEMO="${1:-extropian-sim-optimize}"
shift || true
EXE="${BUILD_DIR}/${DEMO}"

if [ ! -f "$EXE" ]; then
    echo "Not built yet — running build.sh first..."
    cd "$ROOT" && bash build.sh
fi

echo "=== Extropian Simulator: ${DEMO} ==="
cd "$BUILD_DIR"
exec "./${DEMO}" "$@"
