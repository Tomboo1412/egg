#!/usr/bin/env bash
set -euo pipefail

WS_PATH="${WS_PATH:-$HOME/egg/egg_ws}"
SETUP="$WS_PATH/devel/setup.bash"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <map_name_without_extension>"
  exit 1
fi
MAP_NAME="$1"

if [[ ! -f "$SETUP" ]]; then
  echo "[ERROR] Not found: $SETUP"
  exit 1
fi

source "$SETUP"
OUT_DIR="$(rospack find robot_slam)/maps"
mkdir -p "$OUT_DIR"

echo "[INFO] Saving map to: $OUT_DIR/${MAP_NAME}.pgm/.yaml"
rosrun map_server map_saver -f "$OUT_DIR/$MAP_NAME"
echo "[OK] Saved map: $OUT_DIR/${MAP_NAME}.pgm and ${MAP_NAME}.yaml"