#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
LOG_DIR="${REPO_DIR}/logs"

ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
SIM_VEHICLE="${ARDUPILOT_DIR}/Tools/autotest/sim_vehicle.py"
ARDUROVER_BIN="${ARDUPILOT_DIR}/build/sitl/bin/ardurover"
QGC_PORT=${QGC_PORT:-14550}
CONTROLLER_BASE_PORT=${CONTROLLER_BASE_PORT:-14560}
HEADLESS=${HEADLESS:-0}
MAVPROXY_ARGS=${MAVPROXY_ARGS:-"--daemon --non-interactive"}
SIM_VEHICLE_ARGS=${SIM_VEHICLE_ARGS:-}
BASE_LAT=${BASE_LAT:-47.397742}
BASE_LON=${BASE_LON:-8.545594}
BASE_ALT=${BASE_ALT:-488}
BASE_HEADING=${BASE_HEADING:-0}
START_SPACING_M=${START_SPACING_M:-5}

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

headless_args=()
if [[ "${HEADLESS}" == "1" ]]; then
  headless_args=(--no-console)
fi

extra_args=()
if [[ -n "${SIM_VEHICLE_ARGS}" ]]; then
  read -r -a extra_args <<< "${SIM_VEHICLE_ARGS}"
fi

mavproxy_args=()
if [[ -n "${MAVPROXY_ARGS}" ]]; then
  mavproxy_args=(--mavproxy-args "${MAVPROXY_ARGS}")
fi

for idx in 0 1 2; do
  log_file="${LOG_DIR}/sim_vehicle_${idx}.log"
  sysid=$((idx + 1))
  north_offset_m=0
  east_offset_m=0
  if [[ "${idx}" -eq 1 ]]; then
    east_offset_m="${START_SPACING_M}"
  elif [[ "${idx}" -eq 2 ]]; then
    north_offset_m="${START_SPACING_M}"
  fi
  home_location=$(python3 - <<PY
import math
base_lat = float("${BASE_LAT}")
base_lon = float("${BASE_LON}")
base_alt = float("${BASE_ALT}")
base_heading = float("${BASE_HEADING}")
north = float("${north_offset_m}")
east = float("${east_offset_m}")
lat = base_lat + (north / 111111.0)
lon = base_lon + (east / (111111.0 * math.cos(math.radians(base_lat))))
print(f"{lat},{lon},{base_alt},{base_heading}")
PY
  )

  pushd "${ARDUPILOT_DIR}" >/dev/null
  controller_port=$((CONTROLLER_BASE_PORT + idx))
  nohup "${SIM_VEHICLE}" \
    -v Rover \
    -f motorboat \
    -I "${idx}" \
    -N \
    --sysid "${sysid}" \
    --custom-location "${home_location}" \
    --no-extra-ports \
    --out "udp:127.0.0.1:${QGC_PORT}" \
    --out "udp:127.0.0.1:${controller_port}" \
    "${headless_args[@]}" \
    "${mavproxy_args[@]}" \
    "${extra_args[@]}" \
    > "${log_file}" 2>&1 &
  pid=$!
  popd >/dev/null

  echo "${pid}" >> "${LOG_DIR}/sitl_pids.txt"
  disown "${pid}" || true
  sleep 2

done

cat <<SUMMARY
Swarm started with 3 Rover motorboat SITL instances.

QGC UDP port: ${QGC_PORT}
Controller UDP ports: ${CONTROLLER_BASE_PORT}-$((CONTROLLER_BASE_PORT + 2))
Instances: 0, 1, 2
SYSIDs: 1, 2, 3 (assigned explicitly per instance)
Logs: ${LOG_DIR}/sim_vehicle_<I>.log (e.g., ${LOG_DIR}/sim_vehicle_0.log)
PIDs: ${LOG_DIR}/sitl_pids.txt
SUMMARY
