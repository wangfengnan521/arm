#!/usr/bin/env bash
# Read-only X5A eye-to-hand sampling launcher.
# It never starts a controller/bridge and never publishes an arm command.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VENDOR_WS="${VENDOR_WS:-${HOME}/repos/arx/ARX_X5/ROS2/X5_ws}"
WS="${WS:-$(cd -- "${SCRIPT_DIR}/../../.." && pwd)}"
DATA="$WS/src/x5a_handeye/data"
BOARD_CFG="$WS/src/x5a_handeye/config/board.yaml"
SAMPLES="${SAMPLES:-$DATA/manual_samples_readonly.json}"

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$VENDOR_WS/install/setup.bash"
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

mkdir -p "$DATA"

if ! pgrep -x X5Controller >/dev/null 2>&1; then
  echo "ERROR: official X5Controller is not running."
  echo "First run: bash ~/arx/ARX_X5/00-sh/ROS2/04single_arm.sh"
  exit 2
fi

if pgrep -f 'x5a_control_bridge|control_bridge' >/dev/null 2>&1; then
  echo "ERROR: x5a_control_bridge is running and owns an /arm_cmd publisher."
  echo "Stop that bridge before read-only calibration. Keep the official X5Controller."
  exit 3
fi

arm_cmd_publishers="$(ros2 topic info /arm_cmd 2>/dev/null | awk '/Publisher count:/ {print $3}')"
if [[ -n "$arm_cmd_publishers" && "$arm_cmd_publishers" != "0" ]]; then
  echo "[notice] /arm_cmd has $arm_cmd_publishers external publisher(s), normally rqt."
  echo "[notice] this sampler does not use or publish /arm_cmd."
fi

if ! pgrep -f realsense2_camera_node >/dev/null 2>&1; then
  echo "[vision] starting D435i color-only (no robot interface)"
  nohup ros2 run realsense2_camera realsense2_camera_node --ros-args \
    -r __ns:=/camera -r __node:=camera \
    -p serial_no:=_342522073696 \
    -p enable_color:=true -p enable_depth:=false \
    -p enable_infra1:=false -p enable_infra2:=false \
    -p enable_gyro:=false -p enable_accel:=false \
    -p rgb_camera.color_profile:=640x480x15 \
    >"$DATA/realsense_readonly.log" 2>&1 &
  sleep 3
fi

if ! pgrep -f board_detector >/dev/null 2>&1; then
  echo "[vision] starting ChArUco detector (no robot interface)"
  nohup ros2 run x5a_handeye board_detector --ros-args \
    -p board_config:="$BOARD_CFG" \
    -p image_topic:=/camera/camera/color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info \
    >"$DATA/board_readonly.log" 2>&1 &
  sleep 2
fi

echo "[safety] robot inputs are read-only: /arm_status"
echo "[safety] no /arm_cmd or /arx_joy publisher will be created"
echo "[output] $SAMPLES"
echo "Move by hand in the existing gravity mode; stop completely, then press s."

exec ros2 run x5a_handeye readonly_handeye_sampler --ros-args \
  -p sample_json:="$SAMPLES"
