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
  master_port=$((5760 + (10 * idx)))
  sitl_port=$((5501 + (10 * idx)))
  out_port=$((14550 + idx))

  "$ARDUROVER_BIN" \
    -S \
    --model motorboat \
    --speedup 1 \
    --sysid "$sysid" \
    --defaults "$DEFAULTS" \
    --sim-address=127.0.0.1 \
    -I"$idx" \
    > "logs/rover_${idx}.log" 2>&1 &
  echo $! >> "logs/sitl_pids.txt"
  sleep 2

  mavproxy.py \
    --master "tcp:127.0.0.1:${master_port}" \
    --sitl "127.0.0.1:${sitl_port}" \
    --out "udp:127.0.0.1:${out_port}" \
    --out "udp:127.0.0.1:14560" \
    > "logs/mavproxy_${idx}.log" 2>&1 &
  echo $! >> "logs/sitl_pids.txt"
  sleep 2

  echo "Started SITL SYSID ${sysid} on tcp:127.0.0.1:${master_port} (udp out ${out_port})"
done

echo "Swarm started. PIDs stored in logs/sitl_pids.txt"
