#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
LOG_DIR="${REPO_DIR}/logs"
CONFIG_FILE="${REPO_DIR}/config/vehicles.yaml"
NUM_VEHICLES=""
WITH_CENTRAL_CONTROLLER=0

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

while [[ $# -gt 0 ]]; do
  case "$1" in
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --num-vehicles)
      NUM_VEHICLES="$2"
      shift 2
      ;;
    --with-central-controller)
      WITH_CENTRAL_CONTROLLER=1
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

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

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Config file not found: ${CONFIG_FILE}" >&2
  exit 1
fi

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

mapfile -t vehicle_lines < <(
  python3 - <<PY
import yaml
from pathlib import Path

config_path = Path("${CONFIG_FILE}")
num_vehicles = "${NUM_VEHICLES}"
with config_path.open() as f:
    data = yaml.safe_load(f)
vehicles = data.get("vehicles", [])
if num_vehicles:
    vehicles = vehicles[: int(num_vehicles)]

for idx, vehicle in enumerate(vehicles):
    sysid = int(vehicle.get("sysid", idx + 1))
    endpoint = vehicle.get("endpoint", "")
    mavlink_port = vehicle.get("mavlink_port")
    port = endpoint.rsplit(":", 1)[-1] if ":" in endpoint else ""
    if not port and mavlink_port:
        port = str(mavlink_port)
    print(f"{sysid}\t{port}")
PY
)

if [[ ${#vehicle_lines[@]} -eq 0 ]]; then
  echo "No vehicles found in config." >&2
  exit 1
fi

vehicle_count=${#vehicle_lines[@]}
grid_columns=5

for idx in $(seq 0 $((vehicle_count - 1))); do
  IFS=$'\t' read -r sysid controller_port <<< "${vehicle_lines[$idx]}"
  log_file="${LOG_DIR}/sim_vehicle_${idx}.log"
  north_offset_m=0
  east_offset_m=0
  if [[ "${idx}" -ne 0 ]]; then
    row=$((idx / grid_columns))
    col=$((idx % grid_columns))
    north_offset_m=$((row * START_SPACING_M))
    east_offset_m=$((col * START_SPACING_M))
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
  if [[ -z "${controller_port}" ]]; then
    controller_port=$((CONTROLLER_BASE_PORT + idx))
  fi
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
Swarm started with ${vehicle_count} Rover motorboat SITL instances.

QGC UDP port: ${QGC_PORT}
Controller UDP ports: configured via ${CONFIG_FILE}
Instances: 0-$((vehicle_count - 1))
SYSIDs: configured via ${CONFIG_FILE}
Logs: ${LOG_DIR}/sim_vehicle_<I>.log (e.g., ${LOG_DIR}/sim_vehicle_0.log)
PIDs: ${LOG_DIR}/sitl_pids.txt
SUMMARY

"${SCRIPT_DIR}/start_companions.sh" --config "${CONFIG_FILE}" ${NUM_VEHICLES:+--num-vehicles "${NUM_VEHICLES}"} &
companions_pid=$!

controller_pid=""
if [[ "${WITH_CENTRAL_CONTROLLER}" == "1" ]]; then
  log_file="${LOG_DIR}/formation_controller.log"
  nohup python3 "${REPO_DIR}/src/formation_controller.py" \
    ${NUM_VEHICLES:+--num-vehicles "${NUM_VEHICLES}"} \
    > "${log_file}" 2>&1 &
  controller_pid=$!
  echo "Started central formation_controller (PID ${controller_pid})"
fi

cleanup() {
  if [[ -n "${controller_pid}" ]] && kill -0 "${controller_pid}" 2>/dev/null; then
    kill "${controller_pid}" 2>/dev/null || true
  fi
  if kill -0 "${companions_pid}" 2>/dev/null; then
    kill "${companions_pid}" 2>/dev/null || true
  fi
  "${SCRIPT_DIR}/stop_swarm.sh"
}

trap cleanup INT TERM

wait "${companions_pid}"
