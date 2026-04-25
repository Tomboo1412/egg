#!/usr/bin/env bash
set -euo pipefail

WS_PATH="${WS_PATH:-$HOME/egg/egg_ws}"
SETUP="$WS_PATH/devel/setup.bash"

if [[ ! -f "$SETUP" ]]; then
  echo "[ERROR] Not found: $SETUP"
  echo "Run: cd $WS_PATH && catkin_make"
  exit 1
fi

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
  --tab -e "bash -c \"sleep 3; source '$SETUP'; roslaunch abot_bringup robot_with_imu.launch; exec bash\"" \
  --tab -e "bash -c \"sleep 4; source '$SETUP'; roslaunch robot_slam gmapping.launch; exec bash\"" \
  --tab -e "bash -c \"sleep 4; source '$SETUP'; roslaunch robot_slam view_mapping.launch; exec bash\"" \
  --tab -e "bash -c \"sleep 4; source '$SETUP'; rosrun teleop_twist_keyboard teleop_twist_keyboard.py; exec bash\""