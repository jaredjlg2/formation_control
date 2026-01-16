#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
LOG_DIR="${REPO_DIR}/logs"
CONFIG_FILE="${REPO_DIR}/config/vehicles.yaml"
NUM_VEHICLES=""

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
    *)
      echo "Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${CONFIG_FILE}" ]]; then
  echo "Config file not found: ${CONFIG_FILE}" >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"

detached_pids=()

cleanup() {
  for pid in "${detached_pids[@]:-}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}

trap cleanup INT TERM

mapfile -t companion_lines < <(
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

swarm_ports = []
for idx, vehicle in enumerate(vehicles):
    swarm_port = vehicle.get("swarm_port", 14600 + idx)
    vehicle["swarm_port"] = swarm_port
    swarm_ports.append(swarm_port)

for idx, vehicle in enumerate(vehicles):
    endpoint = vehicle.get("endpoint", "")
    name = vehicle.get("name", f"vehicle_{idx}")
    sysid = int(vehicle.get("sysid", idx + 1))
    swarm_port = vehicle["swarm_port"]
    peers = ",".join(
        f"127.0.0.1:{port}" for j, port in enumerate(swarm_ports) if j != idx
    )
    print(f"{name}\t{sysid}\t{endpoint}\t{swarm_port}\t{peers}")
PY
)

if [[ ${#companion_lines[@]} -eq 0 ]]; then
  echo "No vehicles found in config." >&2
  exit 1
fi

for line in "${companion_lines[@]}"; do
  IFS=$'\t' read -r name sysid endpoint swarm_port peers <<< "${line}"
  log_file="${LOG_DIR}/companion_${name}.log"
  nohup python3 "${REPO_DIR}/src/companion.py" \
    --vehicle-id "${name}" \
    --mavlink "${endpoint}" \
    --expected-sysid "${sysid}" \
    --swarm-port "${swarm_port}" \
    --peers "${peers}" \
    > "${log_file}" 2>&1 &
  pid=$!
  detached_pids+=("${pid}")
  echo "Started companion ${name} (SYSID ${sysid}) on swarm port ${swarm_port} (PID ${pid})"
  sleep 0.2

done

echo "All companions started. Logs in ${LOG_DIR}/companion_<vehicle>.log"
wait
