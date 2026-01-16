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

## Quick Start (decentralized companions)

1. Start the swarm + companions:
   ```bash
   bash scripts/start_swarm.sh --num-vehicles 3
   ```
2. Open QGroundControl and add a UDP link listening on port `14550` (server address blank).
3. Wait for the vehicles to connect.
4. Broadcast a mission command:
   ```bash
   bash scripts/mission_formup_circle.sh
   ```

**What you should see:** vehicles appear in QGroundControl and move to form a circle around
`CENTER_LAT/CENTER_LON` with the requested radius. Each companion is responsible for its
own vehicle; there is no central controller in the loop.

## How to run (detailed)

### Start the swarm

```bash
bash scripts/start_swarm.sh --num-vehicles 3
```

### Start companions only (optional)

If you want to start companions separately:

```bash
bash scripts/start_companions.sh --num-vehicles 3
```

### Broadcast a mission

Form up around a circle:

```bash
bash scripts/mission_formup_circle.sh
```

Or send the mission manually:

```bash
python3 src/mission_broadcaster.py \
  --config config/vehicles.yaml \
  FORM_UP_CIRCLE \
  --center-lat 47.397742 \
  --center-lon 8.545594 \
  --radius-m 15
```

### Verify via logs

Each companion logs its SYSID match and setpoints:

```bash
tail -f logs/companion_leader.log
```

## Details

- `config/vehicles.yaml` holds endpoints, SYSIDs, and swarm ports. It also defines the
  `mission_broadcast_port` used by the mission broadcaster.
- `scripts/start_swarm.sh` launches motorboat SITL instances via `sim_vehicle.py` and
  forwards MAVLink to QGroundControl on UDP port `14550` (override with `QGC_PORT`).
  It also forwards MAVLink to per-vehicle companions using the endpoints in
  `config/vehicles.yaml`.
  It assigns SYSIDs based on `config/vehicles.yaml` and staggers start positions around
  a base location (`BASE_LAT`, `BASE_LON`, `BASE_ALT`, `BASE_HEADING`) with spacing in
  meters controlled by `START_SPACING_M`.
  By default it runs with console windows enabled (`HEADLESS=0`). Set `HEADLESS=1` to
  avoid opening terminal windows and keep MAVProxy from exiting when no terminal is
  available, or pass extra `sim_vehicle.py` flags with
  `SIM_VEHICLE_ARGS="..."`.
- `scripts/stop_swarm.sh` stops those SITL processes.
- `scripts/start_companions.sh` launches one companion process per vehicle, verifies
  SYSIDs on first heartbeat, listens for mission commands, and opens a lightweight UDP
  swarm channel.

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

## Companion Architecture (decentralized)

Each SITL vehicle has a dedicated companion process:

```
SITL vehicle i --> MAVLink UDP --> companion i --> MAVLink setpoints --> vehicle i
                 MAVLink UDP --> QGroundControl (14550)
```

Companions exchange state over the swarm UDP ports and receive the same mission
command via the mission broadcast port. Each companion:

1. Connects to exactly one MAVLink stream and validates its expected SYSID.
2. Receives mission commands (e.g., `FORM_UP_CIRCLE`).
3. Exchanges peer state over swarm UDP.
4. Computes its own slot based on mission + peers.
5. Sends MAVLink setpoints only to its own vehicle.

## Notes

- The companion waits for heartbeats and a valid position fix before issuing commands.
- The mission broadcast uses a flat-earth approximation around the mission center.
- If any vehicle is missing or times out, companions continue with the active peers.
- Verify the swarm is running with `pgrep -af sim_vehicle.py` and ensure QGroundControl
  connects via UDP on port `14550`.

## Optional: Central controller

The legacy controller remains for comparison and can be started explicitly:

```bash
bash scripts/start_swarm.sh --with-central-controller --num-vehicles 3
```

## Verification checklist

- **Kill test**: start 3 vehicles + 3 companions, broadcast `FORM_UP_CIRCLE`, then confirm
  there is no `formation_controller.py` running and the boats still move.
- **Independence test**: kill one companion process; only that vehicle stops updating
  targets while others continue.
- **Pairing test**: each companion log shows expected SYSID match on the first heartbeat.
