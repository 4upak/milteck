#!/bin/sh

set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
VIEWER_BUILD_DIR="$SCRIPT_DIR/viewer/build"
VIEWER_BIN="$VIEWER_BUILD_DIR/drone_viewer"

echo "==> Build simulation"
make -C "$SCRIPT_DIR"

echo "==> Remove old generated files"
rm -f "$SCRIPT_DIR"/simulation.json "$SCRIPT_DIR"/projectile.json \
      "$SCRIPT_DIR"/simulation_*.json "$SCRIPT_DIR"/projectile_*.json

echo "==> Run simulation"
"$SCRIPT_DIR/drone_sim"

echo "==> Configure viewer"
cmake -S "$SCRIPT_DIR/viewer" -B "$VIEWER_BUILD_DIR"

echo "==> Build viewer"
cmake --build "$VIEWER_BUILD_DIR"

if [ "${DRONE_SKIP_VIEWER:-0}" = "1" ]; then
    echo "==> Viewer launch skipped (DRONE_SKIP_VIEWER=1)"
    exit 0
fi

echo "==> Run viewer"
exec "$VIEWER_BIN" "$SCRIPT_DIR"
