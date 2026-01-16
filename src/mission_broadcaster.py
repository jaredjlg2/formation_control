#!/usr/bin/env python3
"""Broadcast mission commands over UDP."""

from __future__ import annotations

import argparse
import socket
import time
from pathlib import Path
from typing import Dict

import yaml

from mission import MissionCommand, MISSION_FORM_UP_CIRCLE, MISSION_GOTO_LATLON, MISSION_HOLD_POSITION

DEFAULT_MISSION_PORT = 17000
DEFAULT_BROADCAST_HOST = "127.0.0.1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Broadcast mission commands to companions.")
    parser.add_argument(
        "--config",
        default=None,
        help="Optional config/vehicles.yaml path for mission port lookup",
    )
    parser.add_argument(
        "--mission-port",
        type=int,
        default=None,
        help="Mission UDP port (overrides config, default 17000)",
    )
    parser.add_argument(
        "--broadcast-host",
        default=DEFAULT_BROADCAST_HOST,
        help="Broadcast host/IP (default 127.0.0.1)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=3.0,
        help="Seconds to repeat-send the mission (default 3.0)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.2,
        help="Send interval seconds (default 0.2)",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    form_up = subparsers.add_parser(MISSION_FORM_UP_CIRCLE, help="Form up in a circle")
    form_up.add_argument("--center-lat", type=float, required=True)
    form_up.add_argument("--center-lon", type=float, required=True)
    form_up.add_argument("--radius-m", type=float, required=True)

    hold = subparsers.add_parser(MISSION_HOLD_POSITION, help="Hold current position")

    goto = subparsers.add_parser(MISSION_GOTO_LATLON, help="Go to a lat/lon")
    goto.add_argument("--lat", type=float, required=True)
    goto.add_argument("--lon", type=float, required=True)

    return parser.parse_args()


def load_mission_port(config_path: str | None) -> int:
    if not config_path:
        return DEFAULT_MISSION_PORT
    path = Path(config_path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return int(data.get("mission_broadcast_port", DEFAULT_MISSION_PORT))


def build_command(args: argparse.Namespace) -> MissionCommand:
    if args.command == MISSION_FORM_UP_CIRCLE:
        params = {
            "center_lat": args.center_lat,
            "center_lon": args.center_lon,
            "radius_m": args.radius_m,
        }
    elif args.command == MISSION_HOLD_POSITION:
        params = {}
    elif args.command == MISSION_GOTO_LATLON:
        params = {"lat": args.lat, "lon": args.lon}
    else:
        raise ValueError(f"Unsupported mission type: {args.command}")
    return MissionCommand(mission_type=args.command, params=params)


def main() -> int:
    args = parse_args()
    mission_port = args.mission_port or load_mission_port(args.config)
    command = build_command(args)
    payload = command.to_json().encode("utf-8")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if args.broadcast_host.endswith(".255") or args.broadcast_host == "255.255.255.255":
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    deadline = time.time() + max(args.duration, args.interval)
    while time.time() < deadline:
        sock.sendto(payload, (args.broadcast_host, mission_port))
        time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
