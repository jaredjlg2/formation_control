#!/usr/bin/env bash
set -euo pipefail

PID_FILE="logs/sitl_pids.txt"

if [[ ! -f "$PID_FILE" ]]; then
  echo "No PID file found at $PID_FILE; attempting fallback pkill." >&2
  pkill -f "ardurover" 2>/dev/null || true
  pkill -f "mavproxy.py" 2>/dev/null || true
  exit 0
fi

while read -r pid; do
  if [[ -z "$pid" ]]; then
    continue
  fi
  if kill -0 "$pid" 2>/dev/null; then
    echo "Stopping SITL PID $pid"
    kill "$pid" 2>/dev/null || true
  fi
done < "$PID_FILE"

rm -f "$PID_FILE"

pkill -f "ardurover" 2>/dev/null || true
pkill -f "mavproxy.py" 2>/dev/null || true
