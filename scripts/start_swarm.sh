#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
LOG_DIR="${REPO_DIR}/logs"

ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
SIM_VEHICLE="${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
ARDUROVER_BIN="${ARDUPILOT_DIR}/build/sitl/bin/ardurover"
QGC_PORT=${QGC_PORT:-14550}

if [[ ! -f "${SIM_VEHICLE}" ]]; then
  echo "sim_vehicle.py not found at ${SIM_VEHICLE}" >&2
  echo "Set ARDUPILOT_DIR to your ArduPilot checkout (default: ~/ardupilot)." >&2
  exit 1
fi

if [[ ! -x "${ARDUROVER_BIN}" ]]; then
  echo "ardurover SITL binary not found at ${ARDUROVER_BIN}." >&2
  echo "Build SITL once (e.g., ./waf configure --board sitl && ./waf rover) and retry." >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"
: > "${LOG_DIR}/sitl_pids.txt"

for idx in 0 1 2; do
  log_file="${LOG_DIR}/sim_vehicle_${idx}.log"

  nohup "${SIM_VEHICLE}" \
    -v Rover \
    -f motorboat \
    -I "${idx}" \
    -N \
    --no-extra-ports \
    --out "udp:127.0.0.1:${QGC_PORT}" \
    > "${log_file}" 2>&1 &

  echo $! >> "${LOG_DIR}/sitl_pids.txt"
  disown || true
  sleep 2

done

cat <<SUMMARY
Swarm started with 3 Rover motorboat SITL instances.

QGC UDP port: ${QGC_PORT}
Instances: 0, 1, 2
SYSIDs: 1, 2, 3 (assigned by sim_vehicle.py using instance index)
Logs: ${LOG_DIR}/sim_vehicle_<I>.log (e.g., ${LOG_DIR}/sim_vehicle_0.log)
PIDs: ${LOG_DIR}/sitl_pids.txt
SUMMARY
