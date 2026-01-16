"""Formation controller for three ArduPilot Rover SITL vehicles."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import yaml
from pymavlink import mavutil

LOG = logging.getLogger("formation")


@dataclass(frozen=True)
class VehicleConfig:
    name: str
    sysid: int
    endpoint: str
    offset_n: float
    offset_e: float


@dataclass
class VehicleState:
    config: VehicleConfig
    mav: mavutil.mavfile
    endpoint: str
    last_heartbeat: float = 0.0
    last_global_position: float = 0.0
    lat: float | None = None
    lon: float | None = None
    heading_deg: float | None = None


def load_config(path: Path) -> Dict[str, VehicleConfig]:
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    vehicles = {}
    for entry in raw["vehicles"]:
        vehicles[entry["name"]] = VehicleConfig(
            name=entry["name"],
            sysid=int(entry["sysid"]),
            endpoint=entry["endpoint"],
            offset_n=float(entry["offset"]["north_m"]),
            offset_e=float(entry["offset"]["east_m"]),
        )
    return vehicles


def build_listen_endpoints(endpoint: str) -> list[str]:
    parts = endpoint.split(":")
    if len(parts) < 3:
        return [endpoint]

    port = parts[-1]
    preferred = f"udpin:0.0.0.0:{port}"
    fallback = f"udp:0.0.0.0:{port}"
    endpoints = [preferred, fallback]
    unique_endpoints = []
    for candidate in endpoints:
        if candidate not in unique_endpoints:
            unique_endpoints.append(candidate)
    return unique_endpoints


def connect_vehicle(config: VehicleConfig) -> VehicleState:
    candidates = build_listen_endpoints(config.endpoint)
    last_state = None
    for endpoint in candidates:
        LOG.info("Connecting to %s at %s", config.name, endpoint)
        mav = mavutil.mavlink_connection(
            endpoint,
            source_system=255,
            autoreconnect=True,
            timeout=2,
        )
        state = VehicleState(config=config, mav=mav, endpoint=endpoint)
        last_state = state
        if wait_for_heartbeat(state, timeout_s=8):
            LOG.info("Using %s for %s", endpoint, config.name)
            return state
        LOG.warning("No heartbeat on %s for %s; trying fallback", endpoint, config.name)

    return last_state


def wait_for_heartbeat(state: VehicleState, timeout_s: float) -> bool:
    if state.last_heartbeat > 0:
        return True
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        msg = state.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg:
            state.last_heartbeat = time.monotonic()
            LOG.info("Heartbeat from %s (SYSID %s)", state.config.name, msg.get_srcSystem())
            return True
    LOG.warning(
        "Timed out waiting for heartbeat from %s on %s",
        state.config.name,
        state.endpoint,
    )
    LOG.warning("No UDP packets received; check SITL --out and local bind")
    return False


def wait_for_position(state: VehicleState, timeout_s: float) -> bool:
    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        msg = state.mav.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if msg:
            state.last_global_position = time.monotonic()
            state.lat = msg.lat / 1e7
            state.lon = msg.lon / 1e7
            state.heading_deg = msg.hdg / 100.0 if msg.hdg != 65535 else None
            if state.heading_deg is None:
                continue
            LOG.info("Position fix for %s: lat=%.7f lon=%.7f hdg=%.1f",
                     state.config.name, state.lat, state.lon, state.heading_deg)
            return True
    LOG.warning("Timed out waiting for position fix from %s", state.config.name)
    return False


def set_guided_and_arm(state: VehicleState) -> None:
    LOG.info("Setting %s to GUIDED", state.config.name)
    if not _set_guided_mode(state):
        return

    msg = state.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
    if msg:
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if not armed:
            LOG.info("Arming %s", state.config.name)
            state.mav.arducopter_arm()
            wait_for_armed(state, timeout_s=10)


def wait_for_armed(state: VehicleState, timeout_s: float) -> bool:
    try:
        state.mav.motors_armed_wait(timeout_s)
        return True
    except TypeError:
        LOG.debug("motors_armed_wait does not accept timeout arg; falling back")

    start = time.monotonic()
    while time.monotonic() - start < timeout_s:
        msg = state.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg:
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                return True
    LOG.warning("Timed out waiting for %s to arm", state.config.name)
    return False


def _set_guided_mode(state: VehicleState) -> bool:
    try:
        state.mav.set_mode("GUIDED")
        return True
    except Exception as exc:
        LOG.warning("Failed to set GUIDED via set_mode for %s: %s", state.config.name, exc)

    mode_mapping = None
    if hasattr(state.mav, "mode_mapping"):
        try:
            mode_mapping = state.mav.mode_mapping()
        except Exception as exc:
            LOG.warning(
                "Failed to read mode mapping from vehicle for %s: %s",
                state.config.name,
                exc,
            )

    if not mode_mapping:
        try:
            mode_mapping = mavutil.mode_mapping(state.mav)
        except Exception as exc:
            LOG.warning(
                "Failed to read mode mapping via mavutil for %s: %s",
                state.config.name,
                exc,
            )

    if not mode_mapping or "GUIDED" not in mode_mapping:
        LOG.error("GUIDED mode not available for %s; available: %s",
                  state.config.name, sorted(mode_mapping or []))
        return False

    state.mav.set_mode_apm(mode_mapping["GUIDED"])
    return True


def meters_to_latlon_offset(north_m: float, east_m: float, lat_deg: float) -> Tuple[float, float]:
    earth_radius = 6378137.0
    d_lat = north_m / earth_radius
    d_lon = east_m / (earth_radius * math.cos(math.radians(lat_deg)))
    return math.degrees(d_lat), math.degrees(d_lon)


def rotate_offset(north_m: float, east_m: float, heading_deg: float) -> Tuple[float, float]:
    heading_rad = math.radians(heading_deg)
    cos_h = math.cos(heading_rad)
    sin_h = math.sin(heading_rad)
    rotated_n = north_m * cos_h - east_m * sin_h
    rotated_e = north_m * sin_h + east_m * cos_h
    return rotated_n, rotated_e


def send_position_target(state: VehicleState, lat: float, lon: float) -> None:
    time_boot_ms = int(time.monotonic() * 1e3) % (2**32)
    state.mav.mav.set_position_target_global_int_send(
        time_boot_ms,
        state.config.sysid,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        0b110111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def update_leader_state(state: VehicleState) -> None:
    msg = state.mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg:
        state.last_global_position = time.monotonic()
        state.lat = msg.lat / 1e7
        state.lon = msg.lon / 1e7
        state.heading_deg = msg.hdg / 100.0 if msg.hdg != 65535 else state.heading_deg


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    config_path = Path(__file__).resolve().parents[1] / "config" / "vehicles.yaml"
    vehicles_cfg = load_config(config_path)

    leader_cfg = vehicles_cfg["leader"]
    follower_cfgs = [vehicles_cfg["follower_left"], vehicles_cfg["follower_right"]]

    leader = connect_vehicle(leader_cfg)
    followers = [connect_vehicle(cfg) for cfg in follower_cfgs]

    if not wait_for_heartbeat(leader, timeout_s=30):
        return
    if not wait_for_position(leader, timeout_s=30):
        return

    for follower in followers:
        if not wait_for_heartbeat(follower, timeout_s=30):
            return
        if not wait_for_position(follower, timeout_s=30):
            return
        set_guided_and_arm(follower)

    LOG.info("Starting formation loop")
    send_rate_hz = 2.5
    send_interval = 1.0 / send_rate_hz

    while True:
        update_leader_state(leader)
        if leader.lat is None or leader.lon is None or leader.heading_deg is None:
            LOG.warning("Leader position unavailable; retrying")
            time.sleep(1)
            continue

        now = time.monotonic()
        if now - leader.last_global_position > 5:
            LOG.warning("Leader position timeout; waiting for update")
            time.sleep(1)
            continue

        for follower in followers:
            rotated_n, rotated_e = rotate_offset(
                follower.config.offset_n,
                follower.config.offset_e,
                leader.heading_deg,
            )
            d_lat, d_lon = meters_to_latlon_offset(rotated_n, rotated_e, leader.lat)
            target_lat = leader.lat + d_lat
            target_lon = leader.lon + d_lon
            send_position_target(follower, target_lat, target_lon)
            LOG.info(
                "Target for %s -> lat=%.7f lon=%.7f (offset N=%.1f E=%.1f)",
                follower.config.name,
                target_lat,
                target_lon,
                follower.config.offset_n,
                follower.config.offset_e,
            )

        time.sleep(send_interval)


if __name__ == "__main__":
    main()
