#!/usr/bin/env bash
set -euo pipefail

ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
ARDUROVER_BIN="$ARDUPILOT_DIR/build/sitl/bin/ardurover"
DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/rover.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/motorboat.parm"

if [[ ! -x "$ARDUROVER_BIN" ]]; then
  echo "ardurover SITL binary not found at $ARDUROVER_BIN" >&2
  echo "Build ArduPilot SITL first (e.g., ./waf configure --board sitl && ./waf rover)." >&2
  echo "You can set ARDUPILOT_DIR to your ArduPilot checkout." >&2
  exit 1
fi

mkdir -p logs
> logs/sitl_pids.txt

for idx in 0 1 2; do
  sysid=$((idx + 1))
  out_port=$((14550 + idx))
  qgc_port=14560

  "$ARDUROVER_BIN" \
    -S \
    --model motorboat \
    --speedup 1 \
    --sysid "$sysid" \
    --defaults "$DEFAULTS" \
    --sim-address=127.0.0.1 \
    --serial0="udp:127.0.0.1:${out_port}" \
    --serial1="udp:127.0.0.1:${qgc_port}" \
    -I"$idx" \
    > "logs/rover_${idx}.log" 2>&1 &
  echo $! >> "logs/sitl_pids.txt"
  sleep 2

  echo "Started SITL SYSID ${sysid} (udp out ${out_port}, qgc ${qgc_port})"
done

echo "Swarm started. PIDs stored in logs/sitl_pids.txt"
