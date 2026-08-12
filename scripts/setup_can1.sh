#!/usr/bin/env bash
# Backward-compatible entry point. The CAN initialization logic lives only in
# setup_can_x5a.sh so both manual and one-click startup use the same checks.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/setup_can_x5a.sh" "$@"
