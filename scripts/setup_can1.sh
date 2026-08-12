#!/usr/bin/env bash
set -euo pipefail

CAN_DEVICE="${CAN_DEVICE:-/dev/arxcan1}"
CAN_INTERFACE="${CAN_INTERFACE:-can1}"

if pgrep -af "slcand.*${CAN_INTERFACE}" >/dev/null; then
  echo "ERROR: ${CAN_INTERFACE} already has a slcand process; refusing to start a duplicate." >&2
  exit 2
fi

sudo slcand -o -f -s8 "${CAN_DEVICE}" "${CAN_INTERFACE}"
sudo ip link set "${CAN_INTERFACE}" up
ip -details -statistics link show "${CAN_INTERFACE}"

