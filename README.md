# Formation Control (ArduPilot SITL)

Python tools for multi-vehicle formation control using ArduPilot Rover SITL (motorboat)
with MAVLink, visualized in QGroundControl.

ArduPilot is treated as an external dependency installed on your machine (for example at
`~/ardupilot`). This repo only contains the Python controller and helper scripts.

## Prerequisites

- Ubuntu machine with Python 3.10+.
- ArduPilot built for SITL (`Rover`) at `~/ardupilot` or set `ARDUPILOT_DIR` to your path.
- QGroundControl installed locally.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Quick Start

1. Start the swarm:
   ```bash
   bash scripts/start_swarm.sh
   ```
2. Open QGroundControl and add a UDP link listening on port `14550` (server address blank).
3. Wait for ten boats to connect.
4. Run the controller:
   ```bash
   python src/formation_controller.py
   ```
   A small app will open where you can select a swarming behavior (follow leader or
   circle). For circle, optionally set a center lat/lon, radius, and speed.

**What you should see:** ten Rover vehicles appear in QGroundControl. The leader (SYSID 1)
continues its default motion, while followers (SYSIDs 2-10) switch to GUIDED and move to
maintain their formation offsets relative to the leader. For the circle behavior, all
vehicles switch to GUIDED and space themselves evenly around the circle at the requested
speed.

## Details

- `config/vehicles.yaml` holds endpoints, SYSIDs, and formation offsets. By default the
  controller listens on UDP ports `14560`-`14569`.
- `scripts/start_swarm.sh` launches ten motorboat SITL instances via `sim_vehicle.py`
  and forwards MAVLink to QGroundControl on UDP port `14550` (override with `QGC_PORT`).
  It also forwards MAVLink to the controller on UDP ports `14560`-`14569` (override with
  `CONTROLLER_BASE_PORT`).
  It assigns unique SYSIDs (1-10) and staggers start positions around a base location
  (`BASE_LAT`, `BASE_LON`, `BASE_ALT`, `BASE_HEADING`) with spacing in meters controlled
  by `START_SPACING_M`.
  By default it runs in headless mode (`HEADLESS=1`) to avoid opening terminal windows
  and to keep MAVProxy from exiting when no terminal is available. Set `HEADLESS=0` to
  restore MAVProxy/console behavior, or pass extra `sim_vehicle.py` flags with
  `SIM_VEHICLE_ARGS="..."`.
- `scripts/stop_swarm.sh` stops those SITL processes.

## QGroundControl Setup & Scripts

1. In QGroundControl, add a UDP link that listens on port `14550` (leave server addresses
   blank).
2. Start the swarm:
   ```bash
   bash scripts/start_swarm.sh
   ```
3. Stop the swarm:
   ```bash
   bash scripts/stop_swarm.sh
   ```
4. Optional: verify traffic is flowing:
   ```bash
   tcpdump -i lo udp port 14550
   ```

## Notes

- The controller waits for heartbeats and a valid position fix before issuing commands.
- If any vehicle is missing or times out, the controller logs a warning and continues
  retrying.
- Verify the swarm is running with `pgrep -af sim_vehicle.py` (ten processes) and ensure
  QGroundControl connects via UDP on port `14550`.
