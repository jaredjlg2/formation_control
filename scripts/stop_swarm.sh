#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
LOG_DIR="${REPO_DIR}/logs"
PID_FILE="${LOG_DIR}/sitl_pids.txt"

if [[ -f "${PID_FILE}" ]]; then
  while read -r pid; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done < "${PID_FILE}"
fi

pkill -f 'ardurover|mavproxy.py|sim_vehicle.py' 2>/dev/null || true

rm -f "${PID_FILE}"

echo "Swarm stopped. Any lingering SITL processes have been terminated."
