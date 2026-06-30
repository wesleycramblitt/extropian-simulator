#!/bin/bash
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="${ROOT}/build"

echo "=== Extropian Simulator Build ==="

# Detect all sibling repos for local development (avoids re-fetching from GitHub)
LOCAL_FLAGS=""
for lib in extropian-core extropian-render extropian-app extropian-physics extropian-solver-fluidx3d; do
    local_path="${ROOT}/../${lib}"
    if [ -d "$local_path" ]; then
        echo "  Using local checkout: $lib"
        name=$(echo "$lib" | sed 's/extropian-/exd-/')
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
echo "  Executable: ${BUILD_DIR}/ExtropianSimulator"
