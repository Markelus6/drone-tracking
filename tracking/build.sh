#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build"

CLEAN=0
for arg in "$@"; do
  if [[ "$arg" == "--clean" ]]; then
    CLEAN=1
  fi
done

if [[ "$CLEAN" -eq 1 ]]; then
  rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -j"$(nproc 2>/dev/null || echo 4)"

echo ""
echo "Built:"
echo "  $BUILD_DIR/nanotrack_fc"
echo "  $BUILD_DIR/lighttrack_fc"
echo "  $BUILD_DIR/multitrack_fc"
echo "  $BUILD_DIR/benchmark_fc"