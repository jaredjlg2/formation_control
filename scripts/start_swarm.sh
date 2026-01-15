#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
SIM_VEHICLE="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"

if [[ ! -x "$SIM_VEHICLE" ]]; then
  echo "sim_vehicle.py not found at $SIM_VEHICLE"
  echo "Set ARDUPILOT_DIR to your ArduPilot checkout." >&2
  exit 1
fi

mkdir -p logs

for idx in 0 1 2; do
  sysid=$((idx + 1))
  out_port=$((14550 + idx))
  "$SIM_VEHICLE" \
    -v Rover \
    -f motorboat \
    -I "$idx" \
    --sysid "$sysid" \
    --out "udp:127.0.0.1:${out_port}" \
    --out "udp:127.0.0.1:14560" \
    --console \
    --map \
    > "logs/sitl_${sysid}.log" 2>&1 &
  echo $! >> logs/sitl_pids.txt
  sleep 2
  echo "Started SITL SYSID ${sysid} on udp:127.0.0.1:${out_port}"
done

echo "Swarm started. PIDs stored in logs/sitl_pids.txt"
