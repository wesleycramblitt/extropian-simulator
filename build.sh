#!/bin/bash
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${ROOT}/build"

echo "=== Extropian Simulator Build ==="

# Detect all sibling repos for local development (avoids re-fetching from GitHub)
LOCAL_FLAGS=""
for lib in extropian-core extropian-render extropian-app extropian-physics \
           extropian-geometry extropian-optimization extropian-viz extropian-spatial-ui; do
    local_path="${ROOT}/../${lib}"
    if [ -d "$local_path" ]; then
        echo "  Using local checkout: $lib"
        # FetchContent looks up FETCHCONTENT_SOURCE_DIR_<NAME> with NAME
        # uppercased, so map extropian-foo -> EXD-FOO.
        name=$(echo "$lib" | sed 's/extropian-/exd-/' | tr '[:lower:]' '[:upper:]')
        LOCAL_FLAGS="${LOCAL_FLAGS} -DFETCHCONTENT_SOURCE_DIR_${name}=${local_path}"
    fi
done

echo "  Configuring..."
cmake -S "$ROOT" -B "$BUILD_DIR" -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    $LOCAL_FLAGS \
    "$@"

echo "  Building..."
cmake --build "$BUILD_DIR" --parallel "$(nproc)"

echo "=== Build complete ==="
echo "  Workspace: ${BUILD_DIR}/extropian-simulator-workspace (stubbed UI/UX)"
echo "  Library:   exd::sim (${BUILD_DIR}/libexd-sim.a)"
echo "  Tests:     ${BUILD_DIR}/optimization_test, shape_workshop_test,"
echo "             solver_run_test, engine_run_test, dashboard_feed_test"
echo "  Run:       ${ROOT}/run.sh"
