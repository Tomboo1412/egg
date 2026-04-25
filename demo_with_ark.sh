#!/usr/bin/env bash
set -euo pipefail

# Go to repo root (script directory)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Default workspace path
WS_PATH="${WS_PATH:-$HOME/egg/egg_ws}"
SETUP_BASH="$WS_PATH/devel/setup.bash"

if [[ ! -f "$SETUP_BASH" ]]; then
  echo "[ERROR] Not found: $SETUP_BASH"
  echo "        Did you run: cd $WS_PATH && catkin_make ?"
  exit 1
fi

# Load ARK config from .env (do NOT commit .env)
if [[ -f ".env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source ".env"
  set +a
else
  echo "[WARN] .env not found in repo root: $SCRIPT_DIR/.env"
  echo "       Create it with:"
  echo "       ARK_API_KEY=..."
  echo "       ARK_MODEL=..."
fi

: "${ARK_API_KEY:?ARK_API_KEY is not set. Put it in .env or export it before running.}"
: "${ARK_MODEL:=ep-20260425130756-kvzkk}"

echo "[INFO] ARK_MODEL=$ARK_MODEL"
echo "[INFO] Starting ROS demo..."

### gmapping with abot ###
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e "bash -c \"sleep 3; source '$SETUP_BASH'; roslaunch abot_bringup robot_with_imu.launch; exec bash\"" \
--tab -e "bash -c \"sleep 4; source '$SETUP_BASH'; roslaunch robot_slam navigation.launch; exec bash\"" \
--tab -e "bash -c \"sleep 4; source '$SETUP_BASH'; roslaunch track_tag usb_cam_with_calibration.launch; exec bash\"" \
--tab -e "bash -c \"sleep 4; source '$SETUP_BASH'; roslaunch track_tag ar_track_camera.launch; exec bash\"" \
--tab -e "bash -c \"sleep 4; source '$SETUP_BASH'; roslaunch abot_vlm vlm_node.launch; exec bash\"" \
--tab -e "bash -c \"sleep 4; source '$SETUP_BASH'; roslaunch robot_slam multi_goal.launch; exec bash\"" \
--tab -e "bash -c \"sleep 4; source '$SETUP_BASH'; roslaunch robot_slam view_nav.launch; exec bash\"" \
--tab -e "bash -c \"sleep 4; source '$SETUP_BASH'; rosrun TTS_audio TTS.py; exec bash\"" \
--tab -e "bash -c \"sleep 4; source '$SETUP_BASH'; roslaunch robot_slam GameStart.launch; exec bash\""