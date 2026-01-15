#!/usr/bin/env bash
set -euo pipefail

PID_FILE="logs/sitl_pids.txt"

if [[ ! -f "$PID_FILE" ]]; then
  echo "No PID file found at $PID_FILE" >&2
  exit 1
fi

while read -r pid; do
  if kill -0 "$pid" 2>/dev/null; then
    echo "Stopping SITL PID $pid"
    kill "$pid"
  fi
done < "$PID_FILE"

rm -f "$PID_FILE"
