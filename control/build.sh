#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$ROOT/build"
cd "$ROOT/build"
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j"$(nproc 2>/dev/null || echo 2)"
echo "OK: $ROOT/build/chase_fc  $ROOT/build/osd_overlay"
