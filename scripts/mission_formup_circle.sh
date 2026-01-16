#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
CONFIG_FILE="${REPO_DIR}/config/vehicles.yaml"

CENTER_LAT=${CENTER_LAT:-47.397742}
CENTER_LON=${CENTER_LON:-8.545594}
RADIUS_M=${RADIUS_M:-15}

python3 "${REPO_DIR}/src/mission_broadcaster.py" \
  --config "${CONFIG_FILE}" \
  FORM_UP_CIRCLE \
  --center-lat "${CENTER_LAT}" \
  --center-lon "${CENTER_LON}" \
  --radius-m "${RADIUS_M}"
