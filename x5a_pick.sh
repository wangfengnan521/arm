#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 red|white|orange" >&2
  exit 2
fi

COLOR="${1,,}"
case "${COLOR}" in
  red|white|orange) ;;
  *)
    echo "Invalid color: ${COLOR}; expected red, white, or orange." >&2
    exit 2
    ;;
esac

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
set -u

exec ros2 action send_goal \
  /x5a_mtc_task_server \
  x5a_task_interfaces/action/PickPlace \
  "{color: '${COLOR}'}" \
  --feedback
