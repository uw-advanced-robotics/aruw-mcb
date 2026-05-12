#!/usr/bin/env python3
"""Headless smoke checks for the spherical projectile drag solver."""

from __future__ import annotations

import math

from spherical_projectile_aim_model import ProjectileModel, Vec3, solve_moving_target


def print_case(name: str, position: Vec3, velocity: Vec3, launch_speed: float) -> None:
    solution = solve_moving_target(
        position,
        velocity,
        Vec3(0.0, 0.0, 0.0),
        launch_speed,
        ProjectileModel(),
    )
    if solution is None:
        print(f"{name}: no solution")
        return
    print(
        f"{name}: pitch={math.degrees(solution.pitch_rad):.3f} deg, "
        f"yaw={math.degrees(solution.yaw_rad):.3f} deg, "
        f"tof={solution.tof_s:.4f} s, distance={solution.distance_m:.3f} m"
    )


def main() -> None:
    print_case("static 4m", Vec3(4.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), 30.0)
    print_case("moving 6m", Vec3(6.0, 1.0, 0.4), Vec3(1.0, -0.5, 0.0), 30.0)
    print_case("crossing target", Vec3(7.0, -2.0, 0.2), Vec3(-0.5, 1.5, 0.0), 30.0)


if __name__ == "__main__":
    main()
