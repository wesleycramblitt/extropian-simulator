#!/bin/bash
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${ROOT}/build"
EXE="${BUILD_DIR}/extropian-simulator-workspace"
if [ ! -f "$EXE" ]; then
    echo "Not built yet — running build.sh first..."
    cd "$ROOT" && bash build.sh
fi
echo "=== Extropian Simulator — Interactive Workspace ==="
cd "$BUILD_DIR"
exec "./extropian-simulator-workspace" "$@"
