#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${SCRIPT_DIR}"
ROS_SETUP="/opt/ros/humble/setup.bash"

ROBOT_TIMEOUT="${ROBOT_TIMEOUT:-30}"
CAMERA_TIMEOUT="${CAMERA_TIMEOUT:-20}"
VISION_TIMEOUT="${VISION_TIMEOUT:-20}"
OBJECT_TIMEOUT="${OBJECT_TIMEOUT:-60}"
TF_TIMEOUT="${TF_TIMEOUT:-10}"

MODE="real"
case "${1:-}" in
  "") ;;
  --dry-run) MODE="dry-run" ;;
  --vision-only) MODE="vision-only" ;;
  --help|-h)
    cat <<'EOF'
Usage:

./run_x5a_vision_pick.sh
    Run one automatic visual pick-and-place

./run_x5a_vision_pick.sh --dry-run
    Detect and plan only

./run_x5a_vision_pick.sh --vision-only
    Camera + object localization only
EOF
    exit 0
    ;;
  *)
    echo "Unknown option: $1" >&2
    echo "Run $0 --help for usage." >&2
    exit 2
    ;;
esac

RUN_STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="${WS_DIR}/logs/run_${RUN_STAMP}"
mkdir -p "${RUN_DIR}"
ROBOT_LOG="${RUN_DIR}/robot.log"
MOVEIT_LOG="${RUN_DIR}/moveit.log"
CAMERA_LOG="${RUN_DIR}/camera.log"
VISION_LOG="${RUN_DIR}/vision.log"
PICK_LOG="${RUN_DIR}/pick_place.log"
touch "${ROBOT_LOG}" "${MOVEIT_LOG}" "${CAMERA_LOG}" "${VISION_LOG}" "${PICK_LOG}"

ROBOT_PID=""
MOVEIT_PID=""
VISION_PID=""
TASK_PID=""

stop_owned_process() {
  local label="$1"
  local pid="$2"
  [[ -n "${pid}" ]] || return 0
  kill -0 "${pid}" 2>/dev/null || return 0
  echo "Stopping ${label} (PID ${pid})..."
  kill -INT "${pid}" 2>/dev/null || true
  local i
  for i in {1..30}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  stop_owned_process "pick_place" "${TASK_PID}"
  stop_owned_process "vision/camera" "${VISION_PID}"
  stop_owned_process "MoveIt" "${MOVEIT_PID}"
  stop_owned_process "X5Controller" "${ROBOT_PID}"
  exit "${status}"
}

on_signal() {
  echo
  echo "Interrupted; shutting down only processes started by this script."
  exit 130
}

trap cleanup EXIT
trap on_signal INT TERM

fail() {
  local stage="$1"
  shift
  echo
  echo "VISION PICK AND PLACE: FAIL"
  echo
  echo "stage:"
  echo "${stage}"
  echo
  echo "reason:"
  echo "$*"
  echo
  echo "logs: ${RUN_DIR}"
  exit 1
}

discover_vendor_setup() {
  if [[ -n "${X5A_VENDOR_SETUP:-}" && -f "${X5A_VENDOR_SETUP}" ]]; then
    printf '%s\n' "${X5A_VENDOR_SETUP}"
    return 0
  fi
  local candidate
  for candidate in \
    "${HOME}/repos/arx/ARX_X5/ROS2/X5_ws/install/setup.bash" \
    "${HOME}/arx/ARX_X5/ROS2/X5_ws/install/setup.bash" \
    "${WS_DIR}/../ARX_X5/ROS2/X5_ws/install/setup.bash"; do
    if [[ -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  find "${HOME}" -maxdepth 8 -type f \
    -path '*/ARX_X5/ROS2/X5_ws/install/setup.bash' -print -quit 2>/dev/null
}

[[ -f "${ROS_SETUP}" ]] || fail "ENVIRONMENT" "ROS 2 Humble setup not found: ${ROS_SETUP}"
VENDOR_SETUP="$(discover_vendor_setup || true)"

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Workspace has not been built."
  [[ -n "${VENDOR_SETUP}" ]] || fail "BUILD" "ARX vendor workspace not found; set X5A_VENDOR_SETUP."
  echo "Building required packages..."
  (
    set +u
    source "${ROS_SETUP}"
    source "${VENDOR_SETUP}"
    set -u
    cd "${WS_DIR}"
    PATH="/usr/bin:/bin:${PATH}" /usr/bin/colcon build --symlink-install \
      --packages-select X5A x5a_handeye x5a_moveit_official_adapter \
        x5a_moveit_config x5a_vision x5a_pick_place \
      --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
  ) >"${RUN_DIR}/build.log" 2>&1 || {
    tail -n 40 "${RUN_DIR}/build.log" >&2
    fail "BUILD" "colcon build failed."
  }
fi

set +u
source "${ROS_SETUP}"
if [[ -n "${VENDOR_SETUP}" ]]; then
  source "${VENDOR_SETUP}"
fi
source "${WS_DIR}/install/setup.bash"
set -u

if [[ "${MODE}" != "vision-only" ]]; then
  ros2 pkg prefix arx_x5_controller >/dev/null 2>&1 || \
    fail "ENVIRONMENT" "arx_x5_controller not found; set X5A_VENDOR_SETUP."
fi

VISION_CONFIG="$(ros2 pkg prefix --share x5a_vision)/config/vision.yaml"
[[ -f "${VISION_CONFIG}" ]] || fail "ENVIRONMENT" "vision.yaml not found."

mapfile -t CAMERA_TOPICS < <(/usr/bin/python3 - "${VISION_CONFIG}" <<'PY'
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
params = next(iter(cfg.values()))["ros__parameters"]
for key in ("color_topic", "aligned_depth_topic", "camera_info_topic"):
    print(params[key])
PY
)
(( ${#CAMERA_TOPICS[@]} == 3 )) || fail "CONFIG" "Could not read camera topics from vision.yaml."
COLOR_TOPIC="${CAMERA_TOPICS[0]}"
DEPTH_TOPIC="${CAMERA_TOPICS[1]}"
INFO_TOPIC="${CAMERA_TOPICS[2]}"

node_exists() {
  ros2 node list 2>/dev/null | grep -Fxq "$1"
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fxq "$1"
}

action_exists() {
  ros2 action list 2>/dev/null | grep -Fxq "$1"
}

topic_message_once() {
  local topic="$1"
  local output="$2"
  timeout 3s ros2 topic echo "${topic}" --once >"${output}" 2>/dev/null && [[ -s "${output}" ]]
}

wait_until() {
  local timeout_s="$1"
  shift
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if "$@"; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

start_background() {
  local variable="$1"
  local logfile="$2"
  shift 2
  "$@" >"${logfile}" 2>&1 &
  printf -v "${variable}" '%s' "$!"
}

validate_joint_states() {
  /usr/bin/python3 - "$1" <<'PY'
import math, sys, yaml
docs = [d for d in yaml.safe_load_all(open(sys.argv[1], encoding="utf-8")) if d]
if not docs:
    raise SystemExit(1)
msg = docs[0]
names = msg.get("name", [])
pos = msg.get("position", [])
index = {n: i for i, n in enumerate(names)}
required = [f"joint{i}" for i in range(1, 7)]
if any(n not in index for n in required):
    raise SystemExit(2)
if any(index[n] >= len(pos) or not math.isfinite(float(pos[index[n]])) for n in required):
    raise SystemExit(3)
PY
}

check_tf() {
  timeout 4s ros2 run tf2_ros tf2_echo \
    base_link camera_color_optical_frame >"${RUN_DIR}/tf.log" 2>&1 || true
  grep -q 'Translation:' "${RUN_DIR}/tf.log"
}

check_arm_cmd_owner() {
  local info="${RUN_DIR}/arm_cmd_info.txt"
  ros2 topic info /arm_cmd -v >"${info}" 2>&1 || return 1
  grep -q 'Publisher count: 1' "${info}" && \
    grep -q 'Node name: x5a_official_trajectory_adapter' "${info}"
}

echo "========================================"
echo " ARX X5A AUTO VISION PICK"
echo "========================================"
echo "mode: ${MODE}"
echo "logs: ${RUN_DIR}"
echo

if [[ "${MODE}" != "vision-only" ]]; then
  if ! ip -o link show can1 2>/dev/null | grep -q '<[^>]*UP[,>]'; then
    [[ -x "${WS_DIR}/scripts/setup_can_x5a.sh" ]] || \
      fail "CAN" "scripts/setup_can_x5a.sh not found or not executable."
    "${WS_DIR}/scripts/setup_can_x5a.sh" >>"${ROBOT_LOG}" 2>&1 || \
      fail "CAN" "CAN initialization failed; see robot.log."
  fi
  ip -o link show can1 2>/dev/null | grep -q '<[^>]*UP[,>]' || \
    fail "CAN" "can1 is not UP."
  echo "[1/6] CAN              PASS"

  if node_exists /arm && pgrep -x X5Controller >/dev/null 2>&1; then
    echo "Existing X5Controller detected; reusing it." >>"${ROBOT_LOG}"
  elif ! node_exists /arm && ! pgrep -x X5Controller >/dev/null 2>&1; then
    start_background ROBOT_PID "${ROBOT_LOG}" \
      ros2 launch arx_x5_controller open_single_arm.launch.py
  else
    fail "X5A" "Partial/stale X5Controller detected; clean restart required."
  fi
  wait_until "${ROBOT_TIMEOUT}" topic_message_once /arm_status "${RUN_DIR}/arm_status.yaml" || \
    fail "X5A" "No real /arm_status message within ${ROBOT_TIMEOUT} s."
  echo "[2/6] X5A              PASS"

  MOVE_PRESENT=0
  ADAPTER_PRESENT=0
  node_exists /move_group && MOVE_PRESENT=1
  node_exists /x5a_official_trajectory_adapter && ADAPTER_PRESENT=1
  if (( MOVE_PRESENT == 1 && ADAPTER_PRESENT == 1 )); then
    echo "Existing MoveIt/official adapter detected; reusing it." >>"${MOVEIT_LOG}"
  elif (( MOVE_PRESENT == 0 && ADAPTER_PRESENT == 0 )); then
    start_background MOVEIT_PID "${MOVEIT_LOG}" \
      ros2 launch x5a_moveit_config x5a_real_moveit.launch.py \
        start_driver:=false start_adapter:=true use_rviz:=false arm_can_id:=can1
  else
    fail "MOVEIT" "Partial MoveIt stack detected; clean restart required."
  fi
  wait_until "${ROBOT_TIMEOUT}" node_exists /move_group || \
    fail "MOVEIT" "move_group did not become ready."
  wait_until "${ROBOT_TIMEOUT}" action_exists /execute_trajectory || \
    fail "MOVEIT" "/execute_trajectory is unavailable."
  wait_until "${ROBOT_TIMEOUT}" action_exists /x5a_arm_controller/follow_joint_trajectory || \
    fail "MOVEIT" "FollowJointTrajectory action is unavailable."
  wait_until "${ROBOT_TIMEOUT}" action_exists /x5a_gripper_controller/gripper_cmd || \
    fail "MOVEIT" "GripperCommand action is unavailable."
  wait_until "${ROBOT_TIMEOUT}" check_arm_cmd_owner || \
    fail "MOVEIT" "/arm_cmd must have exactly one official adapter publisher."
  wait_until "${ROBOT_TIMEOUT}" topic_message_once /joint_states "${RUN_DIR}/joint_states.yaml" || \
    fail "ROBOT_FEEDBACK" "No /joint_states message received."
  validate_joint_states "${RUN_DIR}/joint_states.yaml" || \
    fail "ROBOT_FEEDBACK" "joint1..joint6 are missing or non-finite."
  echo "[3/6] MoveIt           PASS"
else
  echo "Robot/MoveIt: SKIPPED (vision-only)"
fi

CAMERA_PRESENT=0
VISION_PRESENT=0
TF_PRESENT=0
node_exists /camera/camera && CAMERA_PRESENT=1
node_exists /cube_detector && VISION_PRESENT=1
node_exists /x5a_handeye_tf && TF_PRESENT=1

if (( VISION_PRESENT == 1 || TF_PRESENT == 1 )); then
  if (( VISION_PRESENT != 1 || TF_PRESENT != 1 || CAMERA_PRESENT != 1 )); then
    fail "VISION" "Partial vision stack detected; clean restart required."
  fi
  echo "Existing RealSense/TF/vision stack detected; reusing it." >>"${VISION_LOG}"
  echo "Camera reused from existing stack." >>"${CAMERA_LOG}"
elif (( CAMERA_PRESENT == 1 )); then
  echo "Camera reused; starting hand-eye TF and detector." >>"${CAMERA_LOG}"
  start_background VISION_PID "${VISION_LOG}" \
    ros2 launch x5a_vision vision.launch.py start_camera:=false
else
  echo "Camera is managed by x5a_vision/vision.launch.py; see vision.log." >>"${CAMERA_LOG}"
  start_background VISION_PID "${VISION_LOG}" \
    ros2 launch x5a_vision vision.launch.py start_camera:=true
fi

wait_until "${CAMERA_TIMEOUT}" topic_message_once "${COLOR_TOPIC}" "${RUN_DIR}/color_frame.yaml" || \
  fail "REALSENSE" "No color frame on ${COLOR_TOPIC}."
wait_until "${CAMERA_TIMEOUT}" topic_message_once "${DEPTH_TOPIC}" "${RUN_DIR}/depth_frame.yaml" || \
  fail "REALSENSE" "No aligned depth frame on ${DEPTH_TOPIC}."
wait_until "${CAMERA_TIMEOUT}" topic_message_once "${INFO_TOPIC}" "${RUN_DIR}/camera_info.yaml" || \
  fail "REALSENSE" "No CameraInfo message on ${INFO_TOPIC}."
if [[ "${MODE}" == "vision-only" ]]; then
  echo "[1/3] RealSense        PASS"
else
  echo "[4/6] RealSense        PASS"
fi

wait_until "${TF_TIMEOUT}" check_tf || \
  fail "HAND_EYE_TF" "base_link -> camera_color_optical_frame is unavailable."
if [[ "${MODE}" == "vision-only" ]]; then
  echo "[2/3] Eye-to-Hand TF   PASS"
else
  echo "[5/6] Eye-to-Hand TF   PASS"
fi

wait_until "${VISION_TIMEOUT}" topic_exists /x5a_vision/detection_stable || \
  fail "VISION" "detection_stable topic is unavailable."
wait_until "${VISION_TIMEOUT}" topic_message_once /x5a_vision/detection_stable \
  "${RUN_DIR}/detection_stable.yaml" || \
  fail "VISION" "No detection_stable message received."
if [[ "${MODE}" == "vision-only" ]]; then
  echo "[3/3] Vision           PASS"
else
  echo "[6/6] Vision           PASS"
fi

wait_for_valid_object() {
  local deadline=$((SECONDS + OBJECT_TIMEOUT))
  local stable_file="${RUN_DIR}/stable_current.yaml"
  local pose_file="${RUN_DIR}/object_pose.yaml"
  local point_file="${RUN_DIR}/object_point_camera.yaml"
  while (( SECONDS < deadline )); do
    if topic_message_once /x5a_vision/detection_stable "${stable_file}" && \
       grep -Eq '^data: true$' "${stable_file}" && \
       topic_message_once /x5a_vision/object_pose "${pose_file}" && \
       topic_message_once /x5a_vision/object_point_camera "${point_file}"; then
      if /usr/bin/python3 - "${VISION_CONFIG}" "${pose_file}" "${point_file}" \
          >"${RUN_DIR}/object_xyz.txt" <<'PY'
import math, sys, time, yaml

def first_doc(path):
    docs = [d for d in yaml.safe_load_all(open(path, encoding="utf-8")) if d]
    if not docs:
        raise ValueError("empty message")
    return docs[0]

cfg = first_doc(sys.argv[1])
params = next(iter(cfg.values()))["ros__parameters"]
pose = first_doc(sys.argv[2])
point = first_doc(sys.argv[3])

if pose.get("header", {}).get("frame_id") != params.get("base_frame", "base_link"):
    raise SystemExit(2)
p = pose["pose"]["position"]
x, y, z = (float(p[k]) for k in ("x", "y", "z"))
depth = float(point["point"]["z"])
if not all(math.isfinite(v) for v in (x, y, z, depth)):
    raise SystemExit(3)
w = params["workspace"]
if not (w["x_min"] <= x <= w["x_max"] and
        w["y_min"] <= y <= w["y_max"] and
        w["z_min"] <= z <= w["z_max"]):
    raise SystemExit(4)
d = params["depth"]
if not (d["min_m"] <= depth <= d["max_m"]):
    raise SystemExit(5)
stamp = pose.get("header", {}).get("stamp", {})
t = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9
age = time.time() - t
if t <= 0 or age < -1.0 or age > 3.0:
    raise SystemExit(6)
print(f"{x:.9f} {y:.9f} {z:.9f} {depth:.9f}")
PY
      then
        return 0
      fi
    fi
    sleep 0.25
  done
  return 1
}

echo
echo "Robot: $([[ "${MODE}" == "vision-only" ]] && echo SKIPPED || echo READY)"
echo "MoveIt: $([[ "${MODE}" == "vision-only" ]] && echo SKIPPED || echo READY)"
echo "Camera: READY"
echo "Hand-eye TF: READY"
echo "Vision: READY"
echo
echo "Waiting for cube..."

wait_for_valid_object || \
  fail "OBJECT_DETECTION" "No stable object detected within ${OBJECT_TIMEOUT} s."
read -r OBJ_X OBJ_Y OBJ_Z OBJ_DEPTH <"${RUN_DIR}/object_xyz.txt"

echo
echo "OBJECT DETECTED"
echo
echo "base_link:"
echo "x: ${OBJ_X}"
echo "y: ${OBJ_Y}"
echo "z: ${OBJ_Z}"
echo "depth: ${OBJ_DEPTH}"

if [[ "${MODE}" == "vision-only" ]]; then
  echo
  echo "VISION ONLY: PASS"
  exit 0
fi

PLAN_ONLY=false
[[ "${MODE}" == "dry-run" ]] && PLAN_ONLY=true

echo
echo "Planning..."
start_background TASK_PID "${PICK_LOG}" \
  ros2 launch x5a_pick_place pick_place.launch.py \
    plan_only:="${PLAN_ONLY}" vision_enabled:=true

set +e
wait "${TASK_PID}"
TASK_STATUS=$?
set -e
TASK_PID=""

if [[ "${MODE}" == "dry-run" ]]; then
  if (( TASK_STATUS == 0 )) && grep -q 'VISION PICK AND PLACE PLAN: PASS' "${PICK_LOG}"; then
    echo "PLANNING       PASS"
    echo
    echo "========================================"
    echo " VISION PICK AND PLACE PLAN: PASS"
    echo "========================================"
    exit 0
  fi
  REASON="$(grep -E 'FAIL|failed|exception|ERROR' "${PICK_LOG}" | tail -n 1 || true)"
  fail "PLANNING" "${REASON:-MoveIt planning failed; see pick_place.log.}"
fi

stage_pass() {
  local label="$1"
  local pattern="$2"
  if grep -Eq "${pattern}" "${PICK_LOG}"; then
    printf '%-14s PASS\n' "${label}"
    return 0
  fi
  return 1
}

stage_pass "PRE_GRASP" '\[MOVE_PRE_GRASP_EXEC\].*execution=PASS' || \
  fail "PRE_GRASP" "MoveIt planning or execution failed."
stage_pass "APPROACH" '\[APPROACH_(CART|POSE)_EXEC\].*execution=PASS' || \
  fail "APPROACH" "Approach planning or execution failed."
stage_pass "GRASP" '\[CLOSE_GRIPPER\].*result=PASS' || \
  fail "GRASP" "Gripper close action failed."
stage_pass "LIFT" '\[LIFT_(CART|POSE)_EXEC\].*execution=PASS' || \
  fail "LIFT" "Lift planning or execution failed."
stage_pass "TRANSFER" '\[MOVE_PRE_PLACE_EXEC\].*execution=PASS' || \
  fail "TRANSFER" "Transfer planning or execution failed."
stage_pass "PLACE" '\[DESCEND_(CART|POSE)_EXEC\].*execution=PASS' || \
  fail "PLACE" "Place descent failed."
stage_pass "RETREAT" '\[RETREAT_(CART|POSE)_EXEC\].*execution=PASS' || \
  fail "RETREAT" "Retreat failed."
stage_pass "RETURN_HOME" '\[RETURN_HOME_EXEC\].*execution=PASS' || \
  fail "RETURN_HOME" "MoveIt did not confirm return Home."

if (( TASK_STATUS != 0 )) || ! grep -q 'VISION PICK AND PLACE: PASS' "${PICK_LOG}"; then
  REASON="$(grep -E 'FAIL|failed|exception|ERROR' "${PICK_LOG}" | tail -n 1 || true)"
  fail "TASK" "${REASON:-Pick-and-place task failed; see pick_place.log.}"
fi

echo
echo "PICK AND PLACE COMPLETE"
echo "========================================"
echo " VISION PICK AND PLACE: PASS"
echo "========================================"

