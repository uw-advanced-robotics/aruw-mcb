#!/usr/bin/env python3
"""Compare current vacuum ballistics against the spherical drag model."""

from __future__ import annotations

import math

from spherical_projectile_aim_model import (
    ProjectileModel,
    ProjectileState,
    Vec3,
    add_scaled,
    derivative,
    solve_moving_target,
)


G = 9.80665
LAUNCH_SPEED = 30.0
MAX_DRAG_PITCH_CORRECTION_DEG = 8.0
MAX_DRAG_YAW_CORRECTION_DEG = 10.0
MIN_DRAG_TOF_RATIO = 0.8
MAX_DRAG_TOF_RATIO = 1.5
EMBEDDED_DT = 0.01
EMBEDDED_MAX_TIME = 1.2
EMBEDDED_LOCAL_SEARCH_RAD = math.radians(25.0)
EMBEDDED_BISECTION_ITERS = 8
EMBEDDED_PROJECTION_ITERS = 2


def vacuum_static(target: Vec3, launch_speed: float, pitch_axis_offset: float = 0.0):
    horizontal = math.hypot(target.x, target.y) + pitch_axis_offset
    v2 = launch_speed * launch_speed
    sqrt_term = v2 * v2 - G * (G * horizontal * horizontal + 2.0 * target.z * v2)
    if horizontal <= 0.0 or launch_speed <= 0.0 or sqrt_term < 0.0:
        return None

    pitch = -math.atan2(v2 - math.sqrt(sqrt_term), G * horizontal)
    if abs(pitch) < 1e-2:
        vertical_sqrt_term = launch_speed * launch_speed - 2.0 * G * target.z
        if vertical_sqrt_term < 0.0:
            return None
        time_of_flight = (launch_speed - math.sqrt(vertical_sqrt_term)) / G
    else:
        time_of_flight = horizontal / (launch_speed * math.cos(pitch))

    return pitch, math.atan2(target.y, target.x), time_of_flight, target.length()


def vacuum_moving(
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    launch_speed: float,
    pitch_axis_offset: float = 0.0,
    iterations: int = 3,
):
    projected = position
    solution = None
    for _ in range(iterations):
        solution = vacuum_static(projected, launch_speed, pitch_axis_offset)
        if solution is None:
            return None
        projected = position.project(velocity, acceleration, solution[2])
    return solution[0], math.atan2(projected.y, projected.x), solution[2], projected.length()


def angle_diff_deg(lhs: float, rhs: float) -> float:
    return math.degrees(math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs)))


def drag_is_safe(vacuum, drag) -> bool:
    drag_pitch = drag.pitch_rad if hasattr(drag, "pitch_rad") else drag[0]
    drag_yaw = drag.yaw_rad if hasattr(drag, "yaw_rad") else drag[1]
    drag_tof = drag.tof_s if hasattr(drag, "tof_s") else drag[2]
    tof_ratio = drag_tof / vacuum[2] if vacuum[2] > 0.0 else float("inf")
    return (
        abs(math.degrees(drag_pitch - vacuum[0])) < MAX_DRAG_PITCH_CORRECTION_DEG
        and abs(angle_diff_deg(drag_yaw, vacuum[1])) < MAX_DRAG_YAW_CORRECTION_DEG
        and MIN_DRAG_TOF_RATIO < tof_ratio < MAX_DRAG_TOF_RATIO
    )


def rk4_fixed_drag_step(state: ProjectileState, dt: float, drag_scale: float) -> ProjectileState:
    def fixed_derivative(s: ProjectileState) -> ProjectileState:
        speed = math.hypot(s.vx, s.vz)
        return ProjectileState(s.vx, s.vz, -drag_scale * speed * s.vx, -G - drag_scale * speed * s.vz)

    k1 = fixed_derivative(state)
    k2 = fixed_derivative(add_scaled(state, k1, dt * 0.5))
    k3 = fixed_derivative(add_scaled(state, k2, dt * 0.5))
    k4 = fixed_derivative(add_scaled(state, k3, dt))
    return ProjectileState(
        state.x + dt * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x) / 6.0,
        state.z + dt * (k1.z + 2.0 * k2.z + 2.0 * k3.z + k4.z) / 6.0,
        state.vx + dt * (k1.vx + 2.0 * k2.vx + 2.0 * k3.vx + k4.vx) / 6.0,
        state.vz + dt * (k1.vz + 2.0 * k2.vz + 2.0 * k3.vz + k4.vz) / 6.0,
    )


def embedded_simulate_to_range(horizontal: float, z: float, speed: float, pitch: float, model: ProjectileModel):
    state = ProjectileState(0.0, 0.0, speed * math.cos(pitch), -speed * math.sin(pitch))
    if horizontal <= 0.0 or state.vx <= 0.0:
        return None
    drag_scale = model.drag_scale(speed)
    previous_time = 0.0
    t = EMBEDDED_DT
    while t <= EMBEDDED_MAX_TIME:
        previous = state
        state = rk4_fixed_drag_step(state, EMBEDDED_DT, drag_scale)
        if state.x >= horizontal:
            dx = state.x - previous.x
            alpha = 1.0 if dx <= 0.0 else (horizontal - previous.x) / dx
            hit_z = previous.z + alpha * (state.z - previous.z)
            return hit_z - z, previous_time + alpha * EMBEDDED_DT
        if state.vx <= 0.0:
            return None
        previous_time = t
        t += EMBEDDED_DT
    return None


def embedded_static(target: Vec3, speed: float, model: ProjectileModel):
    vacuum = vacuum_static(target, speed)
    if vacuum is None:
        return None
    horizontal = math.hypot(target.x, target.y)
    low = max(math.radians(-80.0), vacuum[0] - EMBEDDED_LOCAL_SEARCH_RAD)
    high = min(math.radians(45.0), vacuum[0] + EMBEDDED_LOCAL_SEARCH_RAD)
    low_eval = embedded_simulate_to_range(horizontal, target.z, speed, low, model)
    high_eval = embedded_simulate_to_range(horizontal, target.z, speed, high, model)
    if low_eval is None or high_eval is None or low_eval[0] * high_eval[0] > 0.0:
        return None
    low_error = low_eval[0]
    root = high_eval
    for _ in range(EMBEDDED_BISECTION_ITERS):
        mid = 0.5 * (low + high)
        mid_eval = embedded_simulate_to_range(horizontal, target.z, speed, mid, model)
        if mid_eval is None:
            high = mid
            continue
        root = mid_eval
        if low_error * mid_eval[0] <= 0.0:
            high = mid
        else:
            low = mid
            low_error = mid_eval[0]
    pitch = 0.5 * (low + high)
    return pitch, math.atan2(target.y, target.x), root[1], target.length()


def embedded_moving(position: Vec3, velocity: Vec3, acceleration: Vec3, speed: float, model: ProjectileModel):
    projected = position
    solution = None
    for _ in range(EMBEDDED_PROJECTION_ITERS + 1):
        solution = embedded_static(projected, speed, model)
        if solution is None:
            return None
        projected = position.project(velocity, acceleration, solution[2])
    return solution[0], math.atan2(projected.y, projected.x), solution[2], projected.length()


def compare_case(name: str, position: Vec3, velocity: Vec3, acceleration: Vec3) -> None:
    vacuum = vacuum_moving(position, velocity, acceleration, LAUNCH_SPEED)
    model = ProjectileModel()
    drag = solve_moving_target(position, velocity, acceleration, LAUNCH_SPEED, model)
    embedded = embedded_moving(position, velocity, acceleration, LAUNCH_SPEED, model)

    if vacuum is None or drag is None or embedded is None:
        print(f"{name:18} vacuum={vacuum is not None} drag={drag is not None} embedded={embedded is not None}")
        return

    safe = drag_is_safe(vacuum, drag)
    embedded_safe = drag_is_safe(vacuum, embedded)
    selected_pitch = drag.pitch_rad if safe else vacuum[0]
    selected_yaw = drag.yaw_rad if safe else vacuum[1]
    selected_tof = drag.tof_s if safe else vacuum[2]

    print(
        f"{name:18} "
        f"vac(p={math.degrees(vacuum[0]):7.3f}, y={math.degrees(vacuum[1]):7.3f}, "
        f"t={vacuum[2]:6.4f})  "
        f"drag(p={math.degrees(drag.pitch_rad):7.3f}, y={math.degrees(drag.yaw_rad):7.3f}, "
        f"t={drag.tof_s:6.4f})  "
        f"d(p={math.degrees(drag.pitch_rad - vacuum[0]):6.3f}, "
        f"y={angle_diff_deg(drag.yaw_rad, vacuum[1]):6.3f}, "
        f"t={drag.tof_s / vacuum[2]:5.3f})  "
        f"emb(p={math.degrees(embedded[0]):7.3f}, y={math.degrees(embedded[1]):7.3f}, "
        f"t={embedded[2]:6.4f}, {'safe' if embedded_safe else 'reject':6})  "
        f"{'drag' if safe else 'vacuum':6} -> "
        f"p={math.degrees(selected_pitch):7.3f}, "
        f"y={math.degrees(selected_yaw):7.3f}, t={selected_tof:6.4f}"
    )


def main() -> None:
    cases = [
        ("static 2m", Vec3(2.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("static 3m", Vec3(3.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("static 4m", Vec3(4.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("static 5m", Vec3(5.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("static 6m", Vec3(6.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("static 7m", Vec3(7.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("static 8m", Vec3(8.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("high target", Vec3(5.0, 0.0, 0.5), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("low target", Vec3(5.0, 0.0, -0.5), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("lateral moving", Vec3(6.0, 1.0, 0.2), Vec3(0.0, 1.5, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("closing moving", Vec3(7.0, -1.0, 0.2), Vec3(-2.0, 0.5, 0.0), Vec3(0.0, 0.0, 0.0)),
        ("rising target", Vec3(5.0, 1.0, 0.0), Vec3(0.0, 0.0, 1.0), Vec3(0.0, 0.0, 0.0)),
    ]

    for case in cases:
        compare_case(*case)


if __name__ == "__main__":
    main()
