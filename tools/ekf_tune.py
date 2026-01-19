#!/usr/bin/env python3
# EKF tuner for wheel_ekf_odometry based on 5m beyblading samples.

import csv
import glob
import math
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np


DT = 0.002
MIN_DT = 0.0005

# Standard chassis constants (used in wheel_ekf_odometry.cpp).
WIDTH_BETWEEN_WHEELS_X = 0.33
WIDTH_BETWEEN_WHEELS_Y = 0.33
WHEELBASE_HYPOTENUSE = 2.0 / (WIDTH_BETWEEN_WHEELS_X + WIDTH_BETWEEN_WHEELS_Y)
ROTATION_RADIUS = 1.0 / WHEELBASE_HYPOTENUSE

WHEEL_RADIUS_SCALE_BASE = 1.031644

# Reference tuning from chassis_kf_odometry.hpp (used as a soft prior).
KF_REF_R_WHEEL = 1.0
KF_REF_R_ACCEL = 1.2

STATE_DIM = 8
MEAS_DIM = 8


@dataclass
class Dataset:
    name: str
    times: np.ndarray
    z: np.ndarray  # shape (N, 8)
    target_x: float


@dataclass
class Params:
    wheel_radius_scale: float
    q_pos: float
    q_vel: float
    q_yaw: float
    q_yaw_rate: float
    q_acc: float
    r_wheel: float
    r_acc: float
    r_gyro: float
    r_yaw: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def read_dataset(path: str, target_x: float) -> Dataset:
    with open(path, "r") as f:
        reader = csv.reader(f)
        header = next(reader)
        header_map = {name: idx for idx, name in enumerate(header)}

        z_indices = [
            header_map[f"(odometrySubsystem).z[{i}]"] for i in range(8)
        ]
        time_index = header_map["Time"]

        times = []
        zs = []
        for row in reader:
            if not row:
                continue
            times.append(float(row[time_index]))
            zs.append([float(row[i]) for i in z_indices])

    return Dataset(
        name=os.path.basename(path),
        times=np.array(times, dtype=float),
        z=np.array(zs, dtype=float),
        target_x=target_x,
    )


def build_q(params: Params) -> np.ndarray:
    q = np.zeros((STATE_DIM, STATE_DIM), dtype=float)
    q[0, 0] = params.q_pos
    q[1, 1] = params.q_pos
    q[2, 2] = params.q_vel
    q[3, 3] = params.q_vel
    q[4, 4] = params.q_yaw
    q[5, 5] = params.q_yaw_rate
    q[6, 6] = params.q_acc
    q[7, 7] = params.q_acc
    return q


def build_r(params: Params, slip_scale: float) -> np.ndarray:
    r = np.zeros((MEAS_DIM, MEAS_DIM), dtype=float)
    wheel_var = params.r_wheel * slip_scale
    for i in range(4):
        r[i, i] = wheel_var
    r[4, 4] = params.r_acc
    r[5, 5] = params.r_acc
    r[6, 6] = params.r_gyro
    r[7, 7] = params.r_yaw
    return r


def f_state(x: np.ndarray, dt: float) -> np.ndarray:
    pos_x, pos_y, vel_x, vel_y, yaw, yaw_rate, acc_x, acc_y = x
    return np.array(
        [
            pos_x + vel_x * dt + 0.5 * acc_x * dt * dt,
            pos_y + vel_y * dt + 0.5 * acc_y * dt * dt,
            vel_x + acc_x * dt,
            vel_y + acc_y * dt,
            yaw + yaw_rate * dt,
            yaw_rate,
            acc_x,
            acc_y,
        ],
        dtype=float,
    )


def F_jacobian(dt: float) -> np.ndarray:
    f = np.zeros((STATE_DIM, STATE_DIM), dtype=float)
    f[0, 0] = 1.0
    f[0, 2] = dt
    f[0, 6] = 0.5 * dt * dt
    f[1, 1] = 1.0
    f[1, 3] = dt
    f[1, 7] = 0.5 * dt * dt
    f[2, 2] = 1.0
    f[2, 6] = dt
    f[3, 3] = 1.0
    f[3, 7] = dt
    f[4, 4] = 1.0
    f[4, 5] = dt
    f[5, 5] = 1.0
    f[6, 6] = 1.0
    f[7, 7] = 1.0
    return f


def h_obs(x: np.ndarray, rotation_radius: float) -> np.ndarray:
    vel_x, vel_y, yaw, yaw_rate = x[2], x[3], x[4], x[5]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    vel_x_chassis = cos_yaw * vel_x + sin_yaw * vel_y
    vel_y_chassis = -sin_yaw * vel_x + cos_yaw * vel_y
    wheel = np.zeros(4, dtype=float)
    wheel[0] = vel_x_chassis - vel_y_chassis - rotation_radius * yaw_rate
    wheel[1] = -vel_x_chassis - vel_y_chassis - rotation_radius * yaw_rate
    wheel[2] = vel_x_chassis + vel_y_chassis - rotation_radius * yaw_rate
    wheel[3] = -vel_x_chassis + vel_y_chassis - rotation_radius * yaw_rate
    return np.array(
        [wheel[0], wheel[1], wheel[2], wheel[3], x[6], x[7], yaw_rate, yaw], dtype=float
    )


def H_jacobian(x: np.ndarray, rotation_radius: float) -> np.ndarray:
    H = np.zeros((MEAS_DIM, STATE_DIM), dtype=float)
    vel_x = x[2]
    vel_y = x[3]
    yaw = x[4]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    d_vx_chassis_d_vx = cos_yaw
    d_vx_chassis_d_vy = sin_yaw
    d_vy_chassis_d_vx = -sin_yaw
    d_vy_chassis_d_vy = cos_yaw
    d_vx_chassis_d_yaw = -sin_yaw * vel_x + cos_yaw * vel_y
    d_vy_chassis_d_yaw = -cos_yaw * vel_x - sin_yaw * vel_y

    d_wheel0_d_vx = d_vx_chassis_d_vx - d_vy_chassis_d_vx
    d_wheel0_d_vy = d_vx_chassis_d_vy - d_vy_chassis_d_vy
    d_wheel0_d_yaw = d_vx_chassis_d_yaw - d_vy_chassis_d_yaw

    d_wheel1_d_vx = -d_vx_chassis_d_vx - d_vy_chassis_d_vx
    d_wheel1_d_vy = -d_vx_chassis_d_vy - d_vy_chassis_d_vy
    d_wheel1_d_yaw = -d_vx_chassis_d_yaw - d_vy_chassis_d_yaw

    d_wheel2_d_vx = d_vx_chassis_d_vx + d_vy_chassis_d_vx
    d_wheel2_d_vy = d_vx_chassis_d_vy + d_vy_chassis_d_vy
    d_wheel2_d_yaw = d_vx_chassis_d_yaw + d_vy_chassis_d_yaw

    d_wheel3_d_vx = -d_vx_chassis_d_vx + d_vy_chassis_d_vx
    d_wheel3_d_vy = -d_vx_chassis_d_vy + d_vy_chassis_d_vy
    d_wheel3_d_yaw = -d_vx_chassis_d_yaw + d_vy_chassis_d_yaw

    H[0, 2] = d_wheel0_d_vx
    H[0, 3] = d_wheel0_d_vy
    H[0, 4] = d_wheel0_d_yaw
    H[0, 5] = -rotation_radius

    H[1, 2] = d_wheel1_d_vx
    H[1, 3] = d_wheel1_d_vy
    H[1, 4] = d_wheel1_d_yaw
    H[1, 5] = -rotation_radius

    H[2, 2] = d_wheel2_d_vx
    H[2, 3] = d_wheel2_d_vy
    H[2, 4] = d_wheel2_d_yaw
    H[2, 5] = -rotation_radius

    H[3, 2] = d_wheel3_d_vx
    H[3, 3] = d_wheel3_d_vy
    H[3, 4] = d_wheel3_d_yaw
    H[3, 5] = -rotation_radius

    H[4, 6] = 1.0
    H[5, 7] = 1.0
    H[6, 5] = 1.0
    H[7, 4] = 1.0
    return H


def run_ekf(
    dataset: Dataset,
    params: Params,
    rotation_radius: float,
    stride: int = 1,
    wheel_remap: Tuple[int, int, int, int] = (0, 1, 2, 3),
) -> np.ndarray:
    x = np.zeros(STATE_DIM, dtype=float)
    P = np.diag([0.25, 0.25, 0.25, 0.25, 0.0305, 1.0, 1.0, 1.0]).astype(float)
    Q = build_q(params)

    prev_time = None
    prev_wheel = None

    for i in range(0, len(dataset.times), stride):
        t = dataset.times[i]
        if prev_time is None:
            dt = DT
        else:
            dt = max(t - prev_time, MIN_DT)
        prev_time = t

        z = dataset.z[i].copy()
        wheel = z[0:4][list(wheel_remap)] * params.wheel_radius_scale
        z[0:4] = wheel

        if prev_wheel is None or dt <= 1e-5:
            slip_scale = 1.0
        else:
            wheel_accel = np.abs((z[0:4] - prev_wheel) / dt)
            wheel_accel_indicator = float(np.mean(wheel_accel))
            imu_accel_mag = float(np.linalg.norm(z[4:6]))
            slip_indicator = max(0.0, wheel_accel_indicator - imu_accel_mag)
            slip_scale = 1.0 + slip_indicator
        prev_wheel = z[0:4].copy()

        # Predict
        x = f_state(x, dt)
        F = F_jacobian(dt)
        P = F @ P @ F.T + Q

        # Wrap yaw measurement to avoid discontinuities.
        z_pred = h_obs(x, rotation_radius)
        yaw_residual = math.atan2(
            math.sin(z[7] - z_pred[7]),
            math.cos(z[7] - z_pred[7]),
        )
        z[7] = z_pred[7] + yaw_residual

        H = H_jacobian(x, rotation_radius)
        R = build_r(params, slip_scale)
        S = H @ P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return np.array([math.nan, math.nan])
        K = P @ H.T @ S_inv
        y = z - z_pred
        x = x + K @ y
        P = (np.eye(STATE_DIM) - K @ H) @ P

        x[4] = normalize_angle(x[4])

    return x


def objective(
    params: Params,
    datasets: List[Dataset],
    rotation_radius: float,
    wheel_remap: Tuple[int, int, int, int],
) -> float:
    errors = []
    for dataset in datasets:
        x = run_ekf(dataset, params, rotation_radius, stride=3, wheel_remap=wheel_remap)
        if not np.isfinite(x[0]):
            return 1e9
        errors.append((abs(x[0]) - dataset.target_x) ** 2)
    tracking_loss = float(np.mean(errors))

    # Soft prior toward chassis KF measurement scales to avoid pathological R.
    prior = 0.0
    prior += (math.log10(params.r_wheel / KF_REF_R_WHEEL)) ** 2
    prior += (math.log10(params.r_acc / KF_REF_R_ACCEL)) ** 2

    return tracking_loss + 0.02 * prior


def random_search(
    base: Params,
    datasets: List[Dataset],
    rotation_radius: float,
    wheel_remap: Tuple[int, int, int, int],
    iterations: int = 250,
) -> Params:
    rng = np.random.default_rng(0)
    best = base
    best_score = objective(best, datasets, rotation_radius, wheel_remap)

    log_base = np.log10(
        [
            base.q_pos,
            base.q_vel,
            base.q_yaw,
            base.q_yaw_rate,
            base.q_acc,
            base.r_wheel,
            base.r_acc,
            base.r_gyro,
            base.r_yaw,
        ]
    )

    for _ in range(iterations):
        jitter = rng.normal(0.0, 0.5, size=log_base.shape)
        candidate_logs = log_base + jitter
        candidate = Params(
            wheel_radius_scale=float(
                np.clip(base.wheel_radius_scale * (10 ** rng.normal(0.0, 0.08)), 0.7, 1.8)
            ),
            q_pos=float(10 ** candidate_logs[0]),
            q_vel=float(10 ** candidate_logs[1]),
            q_yaw=float(10 ** candidate_logs[2]),
            q_yaw_rate=float(10 ** candidate_logs[3]),
            q_acc=float(10 ** candidate_logs[4]),
            r_wheel=float(10 ** candidate_logs[5]),
            r_acc=float(10 ** candidate_logs[6]),
            r_gyro=float(10 ** candidate_logs[7]),
            r_yaw=float(10 ** candidate_logs[8]),
        )
        score = objective(candidate, datasets, rotation_radius, wheel_remap)
        if score < best_score:
            best = candidate
            best_score = score

    # Local refinement around the current best.
    log_best = np.log10(
        [
            best.q_pos,
            best.q_vel,
            best.q_yaw,
            best.q_yaw_rate,
            best.q_acc,
            best.r_wheel,
            best.r_acc,
            best.r_gyro,
            best.r_yaw,
        ]
    )
    for _ in range(max(50, iterations // 2)):
        jitter = rng.normal(0.0, 0.2, size=log_best.shape)
        candidate_logs = log_best + jitter
        candidate = Params(
            wheel_radius_scale=float(
                np.clip(best.wheel_radius_scale * (10 ** rng.normal(0.0, 0.04)), 0.7, 1.8)
            ),
            q_pos=float(10 ** candidate_logs[0]),
            q_vel=float(10 ** candidate_logs[1]),
            q_yaw=float(10 ** candidate_logs[2]),
            q_yaw_rate=float(10 ** candidate_logs[3]),
            q_acc=float(10 ** candidate_logs[4]),
            r_wheel=float(10 ** candidate_logs[5]),
            r_acc=float(10 ** candidate_logs[6]),
            r_gyro=float(10 ** candidate_logs[7]),
            r_yaw=float(10 ** candidate_logs[8]),
        )
        score = objective(candidate, datasets, rotation_radius, wheel_remap)
        if score < best_score:
            best = candidate
            best_score = score

    return best


def print_params(params: Params) -> None:
    print("WHEEL_RADIUS_SCALE:", f"{params.wheel_radius_scale:.6f}")
    print("Q (diag):")
    print(f"  pos_x    : {params.q_pos:.6g}")
    print(f"  pos_y    : {params.q_pos:.6g}")
    print(f"  vel_x    : {params.q_vel:.6g}")
    print(f"  vel_y    : {params.q_vel:.6g}")
    print(f"  yaw      : {params.q_yaw:.6g}")
    print(f"  yaw_rate : {params.q_yaw_rate:.6g}")
    print(f"  acc_x    : {params.q_acc:.6g}")
    print(f"  acc_y    : {params.q_acc:.6g}")
    print("R (diag):")
    print(f"  wheel0 : {params.r_wheel:.6g}")
    print(f"  wheel1 : {params.r_wheel:.6g}")
    print(f"  wheel2 : {params.r_wheel:.6g}")
    print(f"  wheel3 : {params.r_wheel:.6g}")
    print(f"  acc_x  : {params.r_acc:.6g}")
    print(f"  acc_y  : {params.r_acc:.6g}")
    print(f"  gyro_z : {params.r_gyro:.6g}")
    print(f"  yaw    : {params.r_yaw:.6g}")


def dataset_targets(paths: List[str]) -> List[Dataset]:
    datasets: List[Dataset] = []
    for path in paths:
        name = os.path.basename(path)
        if "RETURN_TO_START" in name or "GOTOANDREUTRN" in name:
            target_x = 0.0
        else:
            target_x = 5.0
        datasets.append(read_dataset(path, target_x=target_x))
    return datasets


def find_best_remap(
    dataset: Dataset,
    params: Params,
    rotation_radius: float,
) -> Tuple[int, int, int, int]:
    from itertools import permutations

    best = (0, 1, 2, 3)
    best_error = float("inf")
    for perm in permutations(range(4)):
        x = run_ekf(dataset, params, rotation_radius, stride=4, wheel_remap=perm)
        if not np.isfinite(x[0]):
            continue
        err = abs(abs(x[0]) - dataset.target_x)
        if err < best_error:
            best_error = err
            best = perm
    return best


def main() -> None:
    data_paths = []
    data_paths.extend(glob.glob("tools/FIVEMETER_NOTBEYBLADING_NOBUMPS*.csv"))
    data_paths.extend(glob.glob("tools/BUMPS_NOTBEYBLADING.csv"))
    data_paths = sorted(set(data_paths))
    if not data_paths:
        raise SystemExit("No non-beyblading datasets found.")

    datasets = dataset_targets(data_paths)

    base = Params(
        wheel_radius_scale=WHEEL_RADIUS_SCALE_BASE,
        q_pos=5.44086e-09,
        q_vel=2.47185e-05,
        q_yaw=3.91168e-08,
        q_yaw_rate=2.93711e-05,
        q_acc=8.2437e-03,
        r_wheel=9.66511e-03,
        r_acc=9.66825e-02,
        r_gyro=3.69454e-04,
        r_yaw=2.04033e-07,
    )

    rotation_radius = ROTATION_RADIUS

    print(f"Tuning on {len(datasets)} dataset(s):")
    for dataset in datasets:
        print(f"  - {dataset.name} (target x={dataset.target_x:.2f} m)")

    remap = find_best_remap(datasets[0], base, rotation_radius)
    print(f"Wheel remap: {remap}")

    tuned = random_search(base, datasets, rotation_radius, remap)
    print("\nBest tuned parameters:")
    print_params(tuned)

    for dataset in datasets:
        x_final = run_ekf(dataset, tuned, rotation_radius, wheel_remap=remap)
        print(
            f"\n{dataset.name}: final x={x_final[0]:.3f} m, y={x_final[1]:.3f} m,"
            f" abs error={abs(x_final[0]) - dataset.target_x:.3f} m"
        )


if __name__ == "__main__":
    main()
