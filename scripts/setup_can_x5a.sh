#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${CAN_INTERFACE:-can1}"
CAN_DEVICE="${CAN_DEVICE:-}"

is_up() {
  ip -o link show "${CAN_INTERFACE}" 2>/dev/null | grep -q '<[^>]*UP[,>]'
}

if ip link show "${CAN_INTERFACE}" >/dev/null 2>&1; then
  if ! is_up; then
    echo "${CAN_INTERFACE} exists but is DOWN; bringing it up without restarting slcand."
    sudo ip link set "${CAN_INTERFACE}" up
  fi
  is_up || { echo "ERROR: ${CAN_INTERFACE} is not UP." >&2; exit 1; }
  ip -details -statistics link show "${CAN_INTERFACE}"
  exit 0
fi

if pgrep -x slcand >/dev/null 2>&1; then
  echo "ERROR: slcand is already running but ${CAN_INTERFACE} does not exist." >&2
  echo "Refusing to kill or duplicate the existing CAN process." >&2
  exit 2
fi

if [[ -z "${CAN_DEVICE}" ]]; then
  if [[ -e /dev/arxcan1 ]]; then
    CAN_DEVICE=/dev/arxcan1
  else
    devices=(/dev/ttyACM*)
    if (( ${#devices[@]} != 1 )); then
      echo "ERROR: expected exactly one /dev/ttyACM* device; set CAN_DEVICE explicitly." >&2
      exit 3
    fi
    CAN_DEVICE="${devices[0]}"
  fi
fi

[[ -e "${CAN_DEVICE}" ]] || { echo "ERROR: CAN device ${CAN_DEVICE} not found." >&2; exit 4; }

echo "Initializing CANable2 ${CAN_DEVICE} -> ${CAN_INTERFACE} at 1 Mbps (slcan s8)."
sudo -v
sudo slcand -o -f -s8 "${CAN_DEVICE}" "${CAN_INTERFACE}"
sudo ip link set "${CAN_INTERFACE}" up

is_up || { echo "ERROR: ${CAN_INTERFACE} initialization failed." >&2; exit 5; }
ip -details -statistics link show "${CAN_INTERFACE}"

