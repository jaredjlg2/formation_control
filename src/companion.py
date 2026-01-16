#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import logging
import math
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple

from pymavlink import mavutil

from mission import (
    MissionCommand,
    MISSION_FORM_UP_CIRCLE,
    MISSION_GOTO_LATLON,
    MISSION_HOLD_POSITION,
)

CONTROL_RATE_HZ = 2.0
MISSION_TIMEOUT_S = 10.0
PEER_TIMEOUT_S = 5.0
MAX_SPEED_MPS = 2.0
SPEED_GAIN = 0.4


@dataclass
class Position:
    lat_deg: float
    lon_deg: float
    alt_m: float
    heading_deg: Optional[float] = None


@dataclass
class PeerState:
    vehicle_id: str
    last_seen: float
    position: Optional[Position]
    status: str


@dataclass
class MissionState:
    command: MissionCommand
    received_time: float


@dataclass
class CompanionConfig:
    vehicle_id: str
    mavlink_endpoint: str
    expected_sysid: int
    swarm_port: int
    mission_port: int
    peers: Tuple[Tuple[str, int], ...]


def parse_args() -> CompanionConfig:
    parser = argparse.ArgumentParser(description="Companion process for a single vehicle.")
    parser.add_argument("--vehicle-id", required=True, help="Vehicle identifier/name")
    parser.add_argument(
        "--mavlink",
        required=True,
        help="MAVLink endpoint (e.g., udp:127.0.0.1:14560 or udpin:0.0.0.0:14560)",
    )
    parser.add_argument("--expected-sysid", required=True, type=int, help="Expected SYSID")
    parser.add_argument("--swarm-port", required=True, type=int, help="UDP port for swarm comms")
    parser.add_argument(
        "--mission-port",
        required=True,
        type=int,
        help="UDP port for mission command broadcast",
    )
    parser.add_argument(
        "--peers",
        default="",
        help="Comma-separated host:port list for peer companions",
    )
    args = parser.parse_args()
    peers = parse_peers(args.peers)
    return CompanionConfig(
        vehicle_id=str(args.vehicle_id),
        mavlink_endpoint=normalize_endpoint(args.mavlink),
        expected_sysid=args.expected_sysid,
        swarm_port=args.swarm_port,
        mission_port=args.mission_port,
        peers=peers,
    )


def normalize_endpoint(endpoint: str) -> str:
    if endpoint.startswith("udp://"):
        return "udp:" + endpoint[len("udp://") :]
    return endpoint


def parse_peers(peers: str) -> Tuple[Tuple[str, int], ...]:
    if not peers:
        return tuple()
    parsed = []
    for entry in peers.split(","):
        entry = entry.strip()
        if not entry:
            continue
        host, port = entry.rsplit(":", 1)
        parsed.append((host, int(port)))
    return tuple(parsed)


def wait_for_heartbeat(connection: mavutil.mavfile, expected_sysid: int) -> None:
    logging.info("Waiting for heartbeat...")
    heartbeat = connection.wait_heartbeat(timeout=30)
    if heartbeat is None:
        raise RuntimeError("Timed out waiting for heartbeat")
    sysid = heartbeat.get_srcSystem()
    if sysid != expected_sysid:
        raise RuntimeError(
            f"SYSID mismatch: expected {expected_sysid}, got {sysid}. "
            "Check config/vehicles.yaml and MAVLink routing."
        )
    logging.info("Heartbeat received from SYSID %s", sysid)


def wait_for_armed(connection: mavutil.mavfile, timeout_s: float) -> bool:
    try:
        connection.motors_armed_wait(timeout_s)
        return True
    except TypeError:
        logging.debug("motors_armed_wait does not accept timeout; falling back")

    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        msg = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                return True
    return False


def set_guided_and_arm(connection: mavutil.mavfile) -> None:
    try:
        connection.set_mode("GUIDED")
    except Exception as exc:
        logging.warning("Failed to set GUIDED mode: %s", exc)
    connection.arducopter_arm()
    if not wait_for_armed(connection, timeout_s=8):
        logging.warning("Timed out waiting for vehicle to arm")


def configure_socket(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except (AttributeError, OSError):
        pass
    sock.bind(("0.0.0.0", port))
    sock.settimeout(0.2)
    return sock


def receive_swarm_messages(
    sock: socket.socket,
    stop_event: threading.Event,
    peer_table: Dict[str, PeerState],
    lock: threading.Lock,
) -> None:
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            message = json.loads(data.decode("utf-8"))
        except json.JSONDecodeError:
            logging.warning("Received non-JSON swarm message from %s", addr)
            continue
        vehicle_id = message.get("vehicle_id")
        if not vehicle_id:
            continue
        position = None
        if "lat" in message and "lon" in message:
            position = Position(
                lat_deg=float(message["lat"]),
                lon_deg=float(message["lon"]),
                alt_m=float(message.get("alt_m", 0.0)),
                heading_deg=message.get("heading_deg"),
            )
        status = str(message.get("status", "unknown"))
        with lock:
            peer_table[vehicle_id] = PeerState(
                vehicle_id=vehicle_id,
                last_seen=time.time(),
                position=position,
                status=status,
            )


def receive_mission_commands(
    sock: socket.socket,
    stop_event: threading.Event,
    mission_state: Dict[str, MissionState],
    lock: threading.Lock,
) -> None:
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue
        except OSError:
            break
        try:
            command = MissionCommand.from_json(data.decode("utf-8"))
        except (ValueError, json.JSONDecodeError) as exc:
            logging.warning("Invalid mission command from %s: %s", addr, exc)
            continue
        with lock:
            mission_state["active"] = MissionState(command=command, received_time=time.time())
        logging.info("Mission received: %s params=%s", command.mission_type, command.params)


def build_status_message(
    vehicle_id: str, position: Optional[Position], status: str
) -> dict:
    payload = {
        "vehicle_id": vehicle_id,
        "timestamp": time.time(),
        "status": status,
    }
    if position:
        payload.update(
            {
                "lat": position.lat_deg,
                "lon": position.lon_deg,
                "alt_m": position.alt_m,
                "heading_deg": position.heading_deg,
            }
        )
    return payload


def send_swarm_message(
    sock: socket.socket,
    peers: Iterable[Tuple[str, int]],
    message: dict,
) -> None:
    if not peers:
        return
    payload = json.dumps(message).encode("utf-8")
    for host, port in peers:
        try:
            sock.sendto(payload, (host, port))
        except OSError as exc:
            logging.warning("Failed to send to %s:%s (%s)", host, port, exc)


def latlon_to_meters_offset(
    lat_origin: float,
    lon_origin: float,
    lat_target: float,
    lon_target: float,
) -> Tuple[float, float]:
    earth_radius = 6378137.0
    d_lat = math.radians(lat_target - lat_origin)
    d_lon = math.radians(lon_target - lon_origin)
    north_m = d_lat * earth_radius
    east_m = d_lon * earth_radius * math.cos(math.radians(lat_origin))
    return north_m, east_m


def meters_to_latlon(
    lat_origin: float,
    lon_origin: float,
    north_m: float,
    east_m: float,
) -> Tuple[float, float]:
    lat = lat_origin + (north_m / 111111.0)
    lon = lon_origin + (east_m / (111111.0 * math.cos(math.radians(lat_origin))))
    return lat, lon


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def compute_slot_target(
    mission: MissionCommand,
    vehicle_id: str,
    peer_table: Dict[str, PeerState],
) -> Tuple[Optional[float], Optional[float]]:
    if mission.mission_type == MISSION_HOLD_POSITION:
        return None, None
    if mission.mission_type == MISSION_GOTO_LATLON:
        return float(mission.params.get("lat")), float(mission.params.get("lon"))
    if mission.mission_type != MISSION_FORM_UP_CIRCLE:
        return None, None

    center_lat = float(mission.params.get("center_lat"))
    center_lon = float(mission.params.get("center_lon"))
    radius_m = float(mission.params.get("radius_m"))

    active_ids = [vehicle_id]
    now = time.time()
    for peer_id, peer in peer_table.items():
        if now - peer.last_seen <= PEER_TIMEOUT_S:
            active_ids.append(peer_id)

    sorted_ids = sorted(set(active_ids))
    if vehicle_id not in sorted_ids:
        return None, None

    index = sorted_ids.index(vehicle_id)
    total = len(sorted_ids)
    angle = 2.0 * math.pi * index / max(total, 1)
    north = radius_m * math.cos(angle)
    east = radius_m * math.sin(angle)
    return meters_to_latlon(center_lat, center_lon, north, east)


def send_position_target(
    connection: mavutil.mavfile,
    target_sysid: int,
    lat: float,
    lon: float,
    velocity_n: float,
    velocity_e: float,
) -> None:
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    time_boot_ms = int(time.monotonic() * 1e3) % (2**32)
    connection.mav.set_position_target_global_int_send(
        time_boot_ms,
        target_sysid,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        type_mask,
        int(lat * 1e7),
        int(lon * 1e7),
        0,
        velocity_n,
        velocity_e,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def main() -> int:
    config = parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
    )

    logging.info(
        "Starting companion for vehicle %s (SYSID %s)",
        config.vehicle_id,
        config.expected_sysid,
    )
    connection = mavutil.mavlink_connection(config.mavlink_endpoint, source_system=255)
    wait_for_heartbeat(connection, config.expected_sysid)

    swarm_sock = configure_socket(config.swarm_port)
    logging.info("Swarm UDP socket bound on %s", config.swarm_port)

    mission_sock = configure_socket(config.mission_port)
    logging.info("Mission UDP socket bound on %s", config.mission_port)

    stop_event = threading.Event()
    peer_table: Dict[str, PeerState] = {}
    peer_lock = threading.Lock()
    mission_state: Dict[str, MissionState] = {}
    mission_lock = threading.Lock()

    receiver = threading.Thread(
        target=receive_swarm_messages,
        args=(swarm_sock, stop_event, peer_table, peer_lock),
        daemon=True,
    )
    receiver.start()

    mission_receiver = threading.Thread(
        target=receive_mission_commands,
        args=(mission_sock, stop_event, mission_state, mission_lock),
        daemon=True,
    )
    mission_receiver.start()

    last_status_log = 0.0
    last_swarm_send = 0.0
    last_control = 0.0
    last_position_time = 0.0
    last_stale_log = 0.0
    position: Optional[Position] = None
    status = "waiting"
    guided_ready = False

    def handle_shutdown(signum, frame):
        logging.info("Shutting down companion (signal %s)", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        while not stop_event.is_set():
            while True:
                message = connection.recv_match(blocking=False)
                if message is None:
                    break
                msg_type = message.get_type()
                if msg_type == "GLOBAL_POSITION_INT":
                    position = Position(
                        lat_deg=message.lat / 1e7,
                        lon_deg=message.lon / 1e7,
                        alt_m=message.alt / 1000.0,
                        heading_deg=message.hdg / 100.0 if message.hdg != 65535 else None,
                    )
                    status = "position_ok"
                    last_position_time = time.time()
                elif msg_type == "HEARTBEAT":
                    status = "heartbeat_ok"

            now = time.time()
            if now - last_status_log >= 5.0:
                logging.info(
                    "Companion status: sysid=%s status=%s position=%s",
                    config.expected_sysid,
                    status,
                    position,
                )
                last_status_log = now

            if now - last_swarm_send >= 0.3:
                swarm_message = build_status_message(config.vehicle_id, position, status)
                send_swarm_message(swarm_sock, config.peers, swarm_message)
                last_swarm_send = now

            if now - last_control >= 1.0 / CONTROL_RATE_HZ:
                last_control = now
                with mission_lock:
                    mission = mission_state.get("active")
                if mission and now - mission.command.timestamp <= MISSION_TIMEOUT_S:
                    if position is None:
                        logging.debug("No position yet; skipping control")
                        continue
                    if not guided_ready:
                        set_guided_and_arm(connection)
                        guided_ready = True

                    with peer_lock:
                        target_lat, target_lon = compute_slot_target(
                            mission.command, config.vehicle_id, peer_table
                        )

                    if mission.command.mission_type == MISSION_HOLD_POSITION:
                        target_lat = position.lat_deg
                        target_lon = position.lon_deg

                    if target_lat is None or target_lon is None:
                        continue

                    north, east = latlon_to_meters_offset(
                        position.lat_deg,
                        position.lon_deg,
                        target_lat,
                        target_lon,
                    )
                    distance = math.hypot(north, east)
                    if distance < 0.5:
                        velocity_n = 0.0
                        velocity_e = 0.0
                    else:
                        speed = clamp(distance * SPEED_GAIN, 0.2, MAX_SPEED_MPS)
                        velocity_n = speed * (north / distance)
                        velocity_e = speed * (east / distance)

                    send_position_target(
                        connection,
                        config.expected_sysid,
                        target_lat,
                        target_lon,
                        velocity_n,
                        velocity_e,
                    )
                    logging.info(
                        "Setpoint: sysid=%s target=(%.7f, %.7f) err=%.1fm v=(%.2f, %.2f)",
                        config.expected_sysid,
                        target_lat,
                        target_lon,
                        distance,
                        velocity_n,
                        velocity_e,
                    )
                elif mission and now - mission.command.timestamp > MISSION_TIMEOUT_S:
                    if now - last_stale_log >= 5.0:
                        logging.warning("Mission stale; withholding setpoints")
                        last_stale_log = now
            if position is None and now - last_position_time > 10.0:
                logging.debug("Waiting for GPS position...")

            time.sleep(0.05)
    finally:
        stop_event.set()
        swarm_sock.close()
        mission_sock.close()
        logging.info("Companion stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
