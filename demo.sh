#!/usr/bin/env bash
set -euo pipefail

# Default workspace path
WS_PATH="${WS_PATH:-$HOME/egg/egg_ws}"
SETUP_BASH="$WS_PATH/devel/setup.bash"

if [[ ! -f "$SETUP_BASH" ]]; then
  echo "[ERROR] Not found: $SETUP_BASH"
  echo "        Did you run: cd $WS_PATH && catkin_make ?"
  exit 1
fi

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
#--tab -e 'bash -c "sleep 4; source ~/anaconda3/etc/profile.d/conda.sh; conda activate audio; roslaunch TTS_audio tts.launch; exec bash"'
### --tab -e 'bash -c "sleep 4; source ~/new_vision/devel/setup.bash; roslaunch find_object_2d find_object_2d.launch; exec bash"' \