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
echo "  $BUILD_DIR/libcamera_orchestrator.so  — shared library"
echo "  $BUILD_DIR/orch_daemon                — standalone daemon"
echo ""
echo "Daemon usage:"
echo "  $BUILD_DIR/orch_daemon --add /dev/cam_usb2,640,480,30 --add /dev/cam_usb3,640,480,20"
echo ""
echo "Install:"
echo "  sudo cmake --install $BUILD_DIR"
