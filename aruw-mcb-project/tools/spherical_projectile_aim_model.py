#!/usr/bin/env python3
"""Headless spherical projectile drag model used by the GUI and local checks."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple


G = 9.81


@dataclass
class Vec3:
    x: float
    y: float
    z: float

    def project(self, velocity: "Vec3", acceleration: "Vec3", dt: float) -> "Vec3":
        return Vec3(
            self.x + velocity.x * dt + 0.5 * acceleration.x * dt * dt,
            self.y + velocity.y * dt + 0.5 * acceleration.y * dt * dt,
            self.z + velocity.z * dt + 0.5 * acceleration.z * dt * dt,
        )

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)


@dataclass
class ProjectileModel:
    diameter_m: float = 0.017
    mass_kg: float = 0.0032
    air_density_kg_m3: float = 1.225
    dynamic_viscosity_pa_s: float = 1.81e-5

    def area_m2(self) -> float:
        radius = self.diameter_m / 2.0
        return math.pi * radius * radius

    def cd(self, speed_mps: float) -> float:
        if speed_mps <= 0.0 or self.dynamic_viscosity_pa_s <= 0.0:
            return 0.0
        re = self.air_density_kg_m3 * speed_mps * self.diameter_m / self.dynamic_viscosity_pa_s
        if re <= 0.0:
            return 0.0
        return (
            24.0 / re
            + (2.6 * (re / 5.0)) / (1.0 + (re / 5.0) ** 1.52)
            + (0.411 * (re / 263000.0) ** -7.94) / (1.0 + (re / 263000.0) ** -8.0)
            + re**0.8 / 461000.0
        )

    def drag_scale(self, speed_mps: float) -> float:
        return 0.5 * self.air_density_kg_m3 * self.cd(speed_mps) * self.area_m2() / self.mass_kg


@dataclass
class ProjectileState:
    x: float
    z: float
    vx: float
    vz: float


@dataclass
class AimSolution:
    pitch_rad: float
    yaw_rad: float
    tof_s: float
    distance_m: float
    vertical_error_m: float
    projected_target: Vec3


def derivative(state: ProjectileState, model: ProjectileModel) -> ProjectileState:
    speed = math.hypot(state.vx, state.vz)
    drag = model.drag_scale(speed)
    return ProjectileState(
        state.vx,
        state.vz,
        -drag * speed * state.vx,
        -G - drag * speed * state.vz,
    )


def add_scaled(state: ProjectileState, delta: ProjectileState, scale: float) -> ProjectileState:
    return ProjectileState(
        state.x + delta.x * scale,
        state.z + delta.z * scale,
        state.vx + delta.vx * scale,
        state.vz + delta.vz * scale,
    )


def rk4_step(state: ProjectileState, dt: float, model: ProjectileModel) -> ProjectileState:
    k1 = derivative(state, model)
    k2 = derivative(add_scaled(state, k1, dt * 0.5), model)
    k3 = derivative(add_scaled(state, k2, dt * 0.5), model)
    k4 = derivative(add_scaled(state, k3, dt), model)
    return ProjectileState(
        state.x + dt * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x) / 6.0,
        state.z + dt * (k1.z + 2.0 * k2.z + 2.0 * k3.z + k4.z) / 6.0,
        state.vx + dt * (k1.vx + 2.0 * k2.vx + 2.0 * k3.vx + k4.vx) / 6.0,
        state.vz + dt * (k1.vz + 2.0 * k2.vz + 2.0 * k3.vz + k4.vz) / 6.0,
    )


def simulate_to_range(
    horizontal_distance_m: float,
    target_height_m: float,
    launch_speed_mps: float,
    pitch_rad: float,
    model: ProjectileModel,
    dt: float = 0.002,
    max_time: float = 2.0,
) -> Optional[Tuple[float, float]]:
    states = trajectory_to_range(
        horizontal_distance_m,
        launch_speed_mps,
        pitch_rad,
        model,
        dt,
        max_time,
    )
    if len(states) < 2:
        return None
    previous = states[-2]
    current = states[-1]
    dx = current.x - previous.x
    alpha = 1.0 if dx <= 0.0 else (horizontal_distance_m - previous.x) / dx
    z = previous.z + alpha * (current.z - previous.z)
    t = (len(states) - 2 + alpha) * dt
    return z - target_height_m, t


def trajectory_to_range(
    horizontal_distance_m: float,
    launch_speed_mps: float,
    pitch_rad: float,
    model: ProjectileModel,
    dt: float = 0.002,
    max_time: float = 2.0,
) -> list[ProjectileState]:
    state = ProjectileState(
        0.0,
        0.0,
        launch_speed_mps * math.cos(pitch_rad),
        -launch_speed_mps * math.sin(pitch_rad),
    )
    if horizontal_distance_m <= 0.0 or state.vx <= 0.0:
        return []
    states = [state]
    t = 0.0
    while t < max_time:
        state = rk4_step(state, dt, model)
        states.append(state)
        if state.x >= horizontal_distance_m or state.vx <= 0.0:
            break
        t += dt
    return states


def solve_static_target(
    target: Vec3,
    launch_speed_mps: float,
    model: ProjectileModel,
    pitch_axis_offset_m: float = 0.0,
    min_pitch_rad: float = -math.radians(80.0),
    max_pitch_rad: float = math.radians(45.0),
) -> Optional[AimSolution]:
    horizontal = math.hypot(target.x, target.y) + pitch_axis_offset_m
    if horizontal <= 0.0 or launch_speed_mps <= 0.0:
        return None

    samples = []
    for i in range(49):
        pitch = min_pitch_rad + (max_pitch_rad - min_pitch_rad) * i / 48.0
        result = simulate_to_range(horizontal, target.z, launch_speed_mps, pitch, model)
        if result is not None and math.isfinite(result[0]):
            samples.append((pitch, result[0]))

    brackets = []
    for (p0, e0), (p1, e1) in zip(samples, samples[1:]):
        if e0 == 0.0 or e0 * e1 <= 0.0:
            brackets.append((p0, p1))

    if not brackets:
        return None

    best = None
    for low, high in brackets:
        low_eval = simulate_to_range(horizontal, target.z, launch_speed_mps, low, model)
        if low_eval is None:
            continue
        low_error = low_eval[0]
        root_eval = low_eval
        for _ in range(18):
            mid = 0.5 * (low + high)
            mid_eval = simulate_to_range(horizontal, target.z, launch_speed_mps, mid, model)
            if mid_eval is None:
                high = mid
                continue
            root_eval = mid_eval
            if low_error * mid_eval[0] <= 0.0:
                high = mid
            else:
                low = mid
                low_error = mid_eval[0]
        pitch = 0.5 * (low + high)
        candidate = AimSolution(
            pitch,
            math.atan2(target.y, target.x),
            root_eval[1],
            target.length(),
            root_eval[0],
            target,
        )
        if best is None or abs(candidate.pitch_rad) < abs(best.pitch_rad):
            best = candidate
    return best


def solve_moving_target(
    position: Vec3,
    velocity: Vec3,
    acceleration: Vec3,
    launch_speed_mps: float,
    model: ProjectileModel,
    pitch_axis_offset_m: float = 0.0,
    iterations: int = 3,
) -> Optional[AimSolution]:
    projected = position
    solution = None
    for _ in range(iterations + 1):
        solution = solve_static_target(projected, launch_speed_mps, model, pitch_axis_offset_m)
        if solution is None:
            return None
        projected = position.project(velocity, acceleration, solution.tof_s)
    solution.yaw_rad = math.atan2(projected.y, projected.x)
    solution.distance_m = projected.length()
    solution.projected_target = projected
    return solution
