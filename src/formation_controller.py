"""Formation controller for ArduPilot Rover SITL vehicles."""

from __future__ import annotations

import argparse
import logging
import math
import random
import time
from dataclasses import dataclass
from pathlib import Path
from threading import Event, Thread
from typing import Dict, Tuple

import yaml
from pymavlink import mavutil

LOG = logging.getLogger("formation")

BEHAVIOR_FOLLOW_LEADER = "Follow leader"
BEHAVIOR_CIRCLE = "Circle around point"
BEHAVIOR_HORNET_SWARM = "Hornet swarm (random)"
DEFAULT_CIRCLE_RADIUS_M = 15.0
DEFAULT_CIRCLE_SPEED_MPS = 2.0
DEFAULT_SWARM_MIN_SEPARATION_M = 3.0
DEFAULT_SWARM_TURN_RATE_DEG = 18.0
DEFAULT_SWARM_BOUNCE_PAD_M = 1.5


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


@dataclass(frozen=True)
class BehaviorSettings:
    mode: str
    circle_center_lat: float | None
    circle_center_lon: float | None
    circle_radius_m: float
    circle_speed_mps: float
    boat_count: int


def load_config(path: Path) -> Tuple[Dict[str, VehicleConfig], list[str]]:
    with path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)

    vehicles = {}
    vehicle_order = []
    for entry in raw["vehicles"]:
        vehicles[entry["name"]] = VehicleConfig(
            name=entry["name"],
            sysid=int(entry["sysid"]),
            endpoint=entry["endpoint"],
            offset_n=float(entry["offset"]["north_m"]),
            offset_e=float(entry["offset"]["east_m"]),
        )
        vehicle_order.append(entry["name"])
    return vehicles, vehicle_order


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


def send_position_target(
    state: VehicleState,
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
    state.mav.mav.set_position_target_global_int_send(
        time_boot_ms,
        state.config.sysid,
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


def update_leader_state(state: VehicleState) -> None:
    msg = state.mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg:
        state.last_global_position = time.monotonic()
        state.lat = msg.lat / 1e7
        state.lon = msg.lon / 1e7
        state.heading_deg = msg.hdg / 100.0 if msg.hdg != 65535 else state.heading_deg


def update_follower_state(state: VehicleState) -> None:
    msg = state.mav.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
    if msg:
        state.last_global_position = time.monotonic()
        state.lat = msg.lat / 1e7
        state.lon = msg.lon / 1e7


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


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min_value, min(value, max_value))


def normalize_vector(north_m: float, east_m: float) -> Tuple[float, float]:
    magnitude = math.hypot(north_m, east_m)
    if magnitude == 0:
        return 0.0, 0.0
    return north_m / magnitude, east_m / magnitude


def rotate_vector(north_m: float, east_m: float, delta_deg: float) -> Tuple[float, float]:
    delta_rad = math.radians(delta_deg)
    cos_d = math.cos(delta_rad)
    sin_d = math.sin(delta_rad)
    rotated_n = north_m * cos_d - east_m * sin_d
    rotated_e = north_m * sin_d + east_m * cos_d
    return rotated_n, rotated_e


def prompt_behavior_settings(max_boats: int) -> BehaviorSettings | None:
    try:
        import tkinter as tk
        from tkinter import messagebox, ttk
    except ImportError:
        LOG.warning("tkinter not available; defaulting to follow leader")
        return BehaviorSettings(
            mode=BEHAVIOR_FOLLOW_LEADER,
            circle_center_lat=None,
            circle_center_lon=None,
            circle_radius_m=DEFAULT_CIRCLE_RADIUS_M,
            circle_speed_mps=DEFAULT_CIRCLE_SPEED_MPS,
            boat_count=max_boats,
        )

    root = tk.Tk()
    root.title("Swarm Behavior")
    root.resizable(False, False)

    mode_var = tk.StringVar(value=BEHAVIOR_FOLLOW_LEADER)
    center_lat_var = tk.StringVar(value="")
    center_lon_var = tk.StringVar(value="")
    radius_var = tk.StringVar(value=str(DEFAULT_CIRCLE_RADIUS_M))
    speed_var = tk.StringVar(value=str(DEFAULT_CIRCLE_SPEED_MPS))
    boat_count_var = tk.StringVar(value=str(max_boats))

    ttk.Label(root, text="Behavior").grid(row=0, column=0, sticky="w", padx=8, pady=6)
    mode_box = ttk.Combobox(
        root,
        textvariable=mode_var,
        state="readonly",
        values=[BEHAVIOR_FOLLOW_LEADER, BEHAVIOR_CIRCLE, BEHAVIOR_HORNET_SWARM],
        width=28,
    )
    mode_box.grid(row=0, column=1, padx=8, pady=6)

    ttk.Label(root, text="Boats in swarm").grid(row=1, column=0, sticky="w", padx=8, pady=6)
    boats_box = ttk.Combobox(
        root,
        textvariable=boat_count_var,
        state="readonly",
        values=[str(count) for count in range(1, max_boats + 1)],
        width=28,
    )
    boats_box.grid(row=1, column=1, padx=8, pady=6)

    ttk.Label(root, text="Circle center lat (optional)").grid(
        row=2, column=0, sticky="w", padx=8, pady=6
    )
    ttk.Entry(root, textvariable=center_lat_var, width=30).grid(
        row=2, column=1, padx=8, pady=6
    )

    ttk.Label(root, text="Circle center lon (optional)").grid(
        row=3, column=0, sticky="w", padx=8, pady=6
    )
    ttk.Entry(root, textvariable=center_lon_var, width=30).grid(
        row=3, column=1, padx=8, pady=6
    )

    ttk.Label(root, text="Circle radius (m)").grid(row=4, column=0, sticky="w", padx=8, pady=6)
    ttk.Entry(root, textvariable=radius_var, width=30).grid(
        row=4, column=1, padx=8, pady=6
    )

    ttk.Label(root, text="Circle speed (m/s)").grid(row=5, column=0, sticky="w", padx=8, pady=6)
    ttk.Entry(root, textvariable=speed_var, width=30).grid(
        row=5, column=1, padx=8, pady=6
    )

    result: BehaviorSettings | None = None

    def on_start() -> None:
        nonlocal result
        mode = mode_var.get()
        try:
            radius = float(radius_var.get())
            speed = float(speed_var.get())
            boat_count = int(boat_count_var.get())
        except ValueError:
            messagebox.showerror(
                "Invalid input",
                "Radius, speed, and boat count must be numeric values.",
            )
            return

        if radius <= 0 or speed <= 0:
            messagebox.showerror("Invalid input", "Radius and speed must be positive values.")
            return
        if boat_count < 1 or boat_count > max_boats:
            messagebox.showerror(
                "Invalid input",
                f"Boat count must be between 1 and {max_boats}.",
            )
            return

        center_lat = center_lat_var.get().strip()
        center_lon = center_lon_var.get().strip()
        center_lat_val = float(center_lat) if center_lat else None
        center_lon_val = float(center_lon) if center_lon else None

        if (center_lat_val is None) != (center_lon_val is None):
            messagebox.showerror(
                "Invalid input",
                "Provide both center latitude and longitude, or leave both blank.",
            )
            return

        result = BehaviorSettings(
            mode=mode,
            circle_center_lat=center_lat_val,
            circle_center_lon=center_lon_val,
            circle_radius_m=radius,
            circle_speed_mps=speed,
            boat_count=boat_count,
        )
        root.destroy()

    def on_close() -> None:
        root.destroy()

    ttk.Button(root, text="Start", command=on_start).grid(
        row=6, column=0, columnspan=2, pady=(6, 10)
    )
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
    return result


def run_follow_leader(
    leader: VehicleState,
    followers: list[VehicleState],
    stop_event: Event,
) -> None:
    LOG.info("Starting formation loop (follow leader)")
    send_rate_hz = 5.0
    send_interval = 1.0 / send_rate_hz
    catchup_gain = 0.8
    max_catchup_speed = 3.0

    while not stop_event.is_set():
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
            update_follower_state(follower)
            rotated_n, rotated_e = rotate_offset(
                follower.config.offset_n,
                follower.config.offset_e,
                leader.heading_deg,
            )
            d_lat, d_lon = meters_to_latlon_offset(rotated_n, rotated_e, leader.lat)
            target_lat = leader.lat + d_lat
            target_lon = leader.lon + d_lon
            velocity_n = 0.0
            velocity_e = 0.0
            if follower.lat is not None and follower.lon is not None:
                error_n, error_e = latlon_to_meters_offset(
                    follower.lat,
                    follower.lon,
                    target_lat,
                    target_lon,
                )
                velocity_n = clamp(error_n * catchup_gain, -max_catchup_speed, max_catchup_speed)
                velocity_e = clamp(error_e * catchup_gain, -max_catchup_speed, max_catchup_speed)

            send_position_target(follower, target_lat, target_lon, velocity_n, velocity_e)
            LOG.info(
                "Target for %s -> lat=%.7f lon=%.7f (offset N=%.1f E=%.1f vel N=%.2f E=%.2f)",
                follower.config.name,
                target_lat,
                target_lon,
                follower.config.offset_n,
                follower.config.offset_e,
                velocity_n,
                velocity_e,
            )

        time.sleep(send_interval)


def run_circle_formation(
    vehicles: list[VehicleState],
    center_lat: float,
    center_lon: float,
    radius_m: float,
    speed_mps: float,
    stop_event: Event,
) -> None:
    LOG.info(
        "Starting formation loop (circle): center=(%.6f, %.6f) radius=%.1fm speed=%.2fm/s",
        center_lat,
        center_lon,
        radius_m,
        speed_mps,
    )
    send_rate_hz = 5.0
    send_interval = 1.0 / send_rate_hz
    start_time = time.monotonic()
    omega = speed_mps / radius_m
    vehicle_count = len(vehicles)

    while not stop_event.is_set():
        t = time.monotonic() - start_time
        for idx, vehicle in enumerate(vehicles):
            angle = omega * t + (2 * math.pi * idx / vehicle_count)
            offset_n = radius_m * math.cos(angle)
            offset_e = radius_m * math.sin(angle)
            d_lat, d_lon = meters_to_latlon_offset(offset_n, offset_e, center_lat)
            target_lat = center_lat + d_lat
            target_lon = center_lon + d_lon
            velocity_n = -speed_mps * math.sin(angle)
            velocity_e = speed_mps * math.cos(angle)

            send_position_target(vehicle, target_lat, target_lon, velocity_n, velocity_e)
            LOG.info(
                "Circle target for %s -> lat=%.7f lon=%.7f vel N=%.2f E=%.2f",
                vehicle.config.name,
                target_lat,
                target_lon,
                velocity_n,
                velocity_e,
            )

        time.sleep(send_interval)


def run_hornet_swarm(
    vehicles: list[VehicleState],
    center_lat: float,
    center_lon: float,
    radius_m: float,
    speed_mps: float,
    stop_event: Event,
) -> None:
    LOG.info(
        "Starting formation loop (hornet swarm): center=(%.6f, %.6f) radius=%.1fm speed=%.2fm/s",
        center_lat,
        center_lon,
        radius_m,
        speed_mps,
    )
    send_rate_hz = 5.0
    send_interval = 1.0 / send_rate_hz
    min_separation = DEFAULT_SWARM_MIN_SEPARATION_M
    bounce_pad = DEFAULT_SWARM_BOUNCE_PAD_M

    velocities: Dict[str, Tuple[float, float]] = {}
    for vehicle in vehicles:
        heading_deg = random.uniform(0, 360)
        v_n = speed_mps * math.cos(math.radians(heading_deg))
        v_e = speed_mps * math.sin(math.radians(heading_deg))
        velocities[vehicle.config.name] = (v_n, v_e)

    while not stop_event.is_set():
        positions: Dict[str, Tuple[float, float]] = {}
        for vehicle in vehicles:
            update_follower_state(vehicle)
            if vehicle.lat is None or vehicle.lon is None:
                continue
            offset_n, offset_e = latlon_to_meters_offset(
                center_lat,
                center_lon,
                vehicle.lat,
                vehicle.lon,
            )
            positions[vehicle.config.name] = (offset_n, offset_e)

        for vehicle in vehicles:
            if vehicle.config.name not in positions:
                continue

            offset_n, offset_e = positions[vehicle.config.name]
            v_n, v_e = velocities.get(vehicle.config.name, (0.0, 0.0))

            if v_n == 0.0 and v_e == 0.0:
                heading_deg = random.uniform(0, 360)
                v_n = speed_mps * math.cos(math.radians(heading_deg))
                v_e = speed_mps * math.sin(math.radians(heading_deg))

            turn_delta = random.uniform(-DEFAULT_SWARM_TURN_RATE_DEG, DEFAULT_SWARM_TURN_RATE_DEG)
            v_n, v_e = rotate_vector(v_n, v_e, turn_delta)

            distance_from_center = math.hypot(offset_n, offset_e)
            if distance_from_center >= radius_m:
                normal_n, normal_e = normalize_vector(offset_n, offset_e)
                dot_out = v_n * normal_n + v_e * normal_e
                if dot_out > 0:
                    v_n -= 2 * dot_out * normal_n
                    v_e -= 2 * dot_out * normal_e
            elif distance_from_center >= radius_m - bounce_pad:
                normal_n, normal_e = normalize_vector(offset_n, offset_e)
                v_n -= normal_n * speed_mps * 0.3
                v_e -= normal_e * speed_mps * 0.3

            repel_n = 0.0
            repel_e = 0.0
            for other_name, (other_n, other_e) in positions.items():
                if other_name == vehicle.config.name:
                    continue
                delta_n = offset_n - other_n
                delta_e = offset_e - other_e
                separation = math.hypot(delta_n, delta_e)
                if separation == 0 or separation >= min_separation:
                    continue
                strength = (min_separation - separation) / min_separation
                dir_n, dir_e = normalize_vector(delta_n, delta_e)
                repel_n += dir_n * strength * speed_mps
                repel_e += dir_e * strength * speed_mps

            v_n += repel_n
            v_e += repel_e
            dir_n, dir_e = normalize_vector(v_n, v_e)
            if dir_n == 0.0 and dir_e == 0.0:
                heading_deg = random.uniform(0, 360)
                dir_n = math.cos(math.radians(heading_deg))
                dir_e = math.sin(math.radians(heading_deg))
            v_n = dir_n * speed_mps
            v_e = dir_e * speed_mps
            velocities[vehicle.config.name] = (v_n, v_e)

            target_offset_n = offset_n + v_n * send_interval
            target_offset_e = offset_e + v_e * send_interval
            target_distance = math.hypot(target_offset_n, target_offset_e)
            if target_distance > radius_m:
                scale = (radius_m - 0.5) / target_distance
                target_offset_n *= scale
                target_offset_e *= scale

            d_lat, d_lon = meters_to_latlon_offset(target_offset_n, target_offset_e, center_lat)
            target_lat = center_lat + d_lat
            target_lon = center_lon + d_lon

            send_position_target(vehicle, target_lat, target_lon, v_n, v_e)
            LOG.info(
                "Hornet target for %s -> lat=%.7f lon=%.7f vel N=%.2f E=%.2f",
                vehicle.config.name,
                target_lat,
                target_lon,
                v_n,
                v_e,
            )

        time.sleep(send_interval)


def run_behavior_control(
    leader: VehicleState,
    followers: list[VehicleState],
    max_boats: int,
) -> None:
    try:
        import tkinter as tk
        from tkinter import messagebox, ttk
    except ImportError:
        LOG.warning("tkinter not available; running default follow leader")
        stop_event = Event()
        run_follow_leader(leader, followers, stop_event)
        return

    root = tk.Tk()
    root.title("Swarm Behavior")
    root.resizable(False, False)

    mode_var = tk.StringVar(value=BEHAVIOR_FOLLOW_LEADER)
    center_lat_var = tk.StringVar(value="")
    center_lon_var = tk.StringVar(value="")
    radius_var = tk.StringVar(value=str(DEFAULT_CIRCLE_RADIUS_M))
    speed_var = tk.StringVar(value=str(DEFAULT_CIRCLE_SPEED_MPS))
    boat_count_var = tk.StringVar(value=str(max_boats))

    ttk.Label(root, text="Behavior").grid(row=0, column=0, sticky="w", padx=8, pady=6)
    ttk.Combobox(
        root,
        textvariable=mode_var,
        state="readonly",
        values=[BEHAVIOR_FOLLOW_LEADER, BEHAVIOR_CIRCLE, BEHAVIOR_HORNET_SWARM],
        width=28,
    ).grid(row=0, column=1, padx=8, pady=6)

    ttk.Label(root, text="Boats in swarm").grid(row=1, column=0, sticky="w", padx=8, pady=6)
    ttk.Combobox(
        root,
        textvariable=boat_count_var,
        state="readonly",
        values=[str(count) for count in range(1, max_boats + 1)],
        width=28,
    ).grid(row=1, column=1, padx=8, pady=6)

    ttk.Label(root, text="Circle center lat (optional)").grid(
        row=2, column=0, sticky="w", padx=8, pady=6
    )
    ttk.Entry(root, textvariable=center_lat_var, width=30).grid(
        row=2, column=1, padx=8, pady=6
    )

    ttk.Label(root, text="Circle center lon (optional)").grid(
        row=3, column=0, sticky="w", padx=8, pady=6
    )
    ttk.Entry(root, textvariable=center_lon_var, width=30).grid(
        row=3, column=1, padx=8, pady=6
    )

    ttk.Label(root, text="Circle radius (m)").grid(row=4, column=0, sticky="w", padx=8, pady=6)
    ttk.Entry(root, textvariable=radius_var, width=30).grid(
        row=4, column=1, padx=8, pady=6
    )

    ttk.Label(root, text="Circle speed (m/s)").grid(row=5, column=0, sticky="w", padx=8, pady=6)
    ttk.Entry(root, textvariable=speed_var, width=30).grid(
        row=5, column=1, padx=8, pady=6
    )

    control_state = {"thread": None, "stop_event": None}
    all_vehicles = [leader, *followers]

    def stop_running() -> None:
        if control_state["stop_event"] is not None:
            control_state["stop_event"].set()
        if control_state["thread"] is not None:
            control_state["thread"].join(timeout=5)
        control_state["thread"] = None
        control_state["stop_event"] = None

    def on_start() -> None:
        mode = mode_var.get()
        try:
            radius = float(radius_var.get())
            speed = float(speed_var.get())
            boat_count = int(boat_count_var.get())
        except ValueError:
            messagebox.showerror(
                "Invalid input",
                "Radius, speed, and boat count must be numeric values.",
            )
            return

        if radius <= 0 or speed <= 0:
            messagebox.showerror("Invalid input", "Radius and speed must be positive values.")
            return
        if boat_count < 1 or boat_count > max_boats:
            messagebox.showerror(
                "Invalid input",
                f"Boat count must be between 1 and {max_boats}.",
            )
            return

        center_lat = center_lat_var.get().strip()
        center_lon = center_lon_var.get().strip()
        center_lat_val = float(center_lat) if center_lat else None
        center_lon_val = float(center_lon) if center_lon else None

        if (center_lat_val is None) != (center_lon_val is None):
            messagebox.showerror(
                "Invalid input",
                "Provide both center latitude and longitude, or leave both blank.",
            )
            return

        stop_running()
        selected_vehicles = all_vehicles[:boat_count]
        stop_event = Event()
        control_state["stop_event"] = stop_event

        if mode == BEHAVIOR_CIRCLE:
            circle_center_lat = center_lat_val or leader.lat
            circle_center_lon = center_lon_val or leader.lon
            if circle_center_lat is None or circle_center_lon is None:
                messagebox.showerror(
                    "Invalid input",
                    "Unable to resolve circle center; ensure leader has a position fix.",
                )
                return
            for vehicle in selected_vehicles:
                set_guided_and_arm(vehicle)
            thread = Thread(
                target=run_circle_formation,
                args=(
                    selected_vehicles,
                    circle_center_lat,
                    circle_center_lon,
                    radius,
                    speed,
                    stop_event,
                ),
                daemon=True,
            )
        elif mode == BEHAVIOR_HORNET_SWARM:
            swarm_center_lat = center_lat_val or leader.lat
            swarm_center_lon = center_lon_val or leader.lon
            if swarm_center_lat is None or swarm_center_lon is None:
                messagebox.showerror(
                    "Invalid input",
                    "Unable to resolve swarm center; ensure leader has a position fix.",
                )
                return
            for vehicle in selected_vehicles:
                set_guided_and_arm(vehicle)
            thread = Thread(
                target=run_hornet_swarm,
                args=(
                    selected_vehicles,
                    swarm_center_lat,
                    swarm_center_lon,
                    radius,
                    speed,
                    stop_event,
                ),
                daemon=True,
            )
        else:
            followers_subset = selected_vehicles[1:]
            for follower in followers_subset:
                set_guided_and_arm(follower)
            thread = Thread(
                target=run_follow_leader,
                args=(selected_vehicles[0], followers_subset, stop_event),
                daemon=True,
            )

        control_state["thread"] = thread
        thread.start()

    def on_stop() -> None:
        stop_running()

    def on_close() -> None:
        stop_running()
        root.destroy()

    ttk.Button(root, text="Start", command=on_start).grid(
        row=6, column=0, padx=8, pady=(6, 10), sticky="ew"
    )
    ttk.Button(root, text="Stop", command=on_stop).grid(
        row=6, column=1, padx=8, pady=(6, 10), sticky="ew"
    )
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Formation controller")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "vehicles.yaml",
        help="Path to vehicles.yaml config",
    )
    parser.add_argument(
        "--num-vehicles",
        type=int,
        default=None,
        help="Number of vehicles to control (including leader)",
    )
    return parser.parse_args()


def select_vehicle_configs(
    vehicles_cfg: Dict[str, VehicleConfig],
    vehicle_order: list[str],
    num_vehicles: int | None,
) -> tuple[VehicleConfig, list[VehicleConfig]]:
    if "leader" not in vehicles_cfg:
        raise ValueError("Leader vehicle missing from config")

    leader_cfg = vehicles_cfg["leader"]
    ordered_names = [name for name in vehicle_order if name in vehicles_cfg]
    if "leader" not in ordered_names:
        ordered_names.insert(0, "leader")

    if num_vehicles is not None:
        if num_vehicles < 1:
            raise ValueError("--num-vehicles must be >= 1")
        selected_names = ordered_names[:num_vehicles]
        if "leader" not in selected_names:
            selected_names.insert(0, "leader")
            if len(selected_names) > num_vehicles:
                selected_names = selected_names[:num_vehicles]
    else:
        selected_names = ordered_names

    follower_names = [
        name for name in selected_names if name != "leader" and name in vehicles_cfg
    ]
    follower_cfgs = [vehicles_cfg[name] for name in follower_names]
    return leader_cfg, follower_cfgs


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    args = parse_args()
    vehicles_cfg, vehicle_order = load_config(args.config)
    try:
        leader_cfg, follower_cfgs = select_vehicle_configs(
            vehicles_cfg, vehicle_order, args.num_vehicles
        )
    except ValueError as exc:
        LOG.error("%s; exiting.", exc)
        return

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

    run_behavior_control(leader, followers, max_boats=len([leader, *followers]))


if __name__ == "__main__":
    main()
