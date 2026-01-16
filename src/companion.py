#!/usr/bin/env python3
import argparse
import json
import logging
import signal
import socket
import sys
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from pymavlink import mavutil


@dataclass
class Position:
    lat_deg: float
    lon_deg: float
    alt_m: float


@dataclass
class CompanionConfig:
    vehicle_id: str
    mavlink_endpoint: str
    expected_sysid: int
    swarm_port: int
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


def receive_swarm_messages(sock: socket.socket, stop_event: threading.Event) -> None:
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
        logging.info("Swarm message from %s: %s", addr, message)


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
    connection = mavutil.mavlink_connection(config.mavlink_endpoint)
    wait_for_heartbeat(connection, config.expected_sysid)

    swarm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    swarm_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    swarm_sock.bind(("0.0.0.0", config.swarm_port))
    swarm_sock.settimeout(0.2)
    logging.info("Swarm UDP socket bound on %s", config.swarm_port)

    stop_event = threading.Event()
    receiver = threading.Thread(
        target=receive_swarm_messages,
        args=(swarm_sock, stop_event),
        daemon=True,
    )
    receiver.start()

    last_status_log = 0.0
    last_swarm_send = 0.0
    position: Optional[Position] = None
    status = "waiting"

    def handle_shutdown(signum, frame):
        logging.info("Shutting down companion (signal %s)", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        while not stop_event.is_set():
            message = connection.recv_match(blocking=True, timeout=1)
            if message:
                msg_type = message.get_type()
                if msg_type == "GLOBAL_POSITION_INT":
                    position = Position(
                        lat_deg=message.lat / 1e7,
                        lon_deg=message.lon / 1e7,
                        alt_m=message.alt / 1000.0,
                    )
                    status = "position_ok"
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
    finally:
        stop_event.set()
        swarm_sock.close()
        logging.info("Companion stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
