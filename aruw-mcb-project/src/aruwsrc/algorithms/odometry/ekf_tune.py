#!/usr/bin/env python3
import csv
import math
import pathlib
from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple

import numpy as np

PRIMARY_SAMPLE_PATH = "/Users/clem/repos/aruw-mcb/aruw-mcb-project/src/aruwsrc/algorithms/odometry/Forwad 400 CM.csv"
SECONDARY_SAMPLE_PATH = "/Users/clem/repos/aruw-mcb/aruw-mcb-project/src/aruwsrc/algorithms/odometry/Forwad 413 beyblading CM.csv"
PRIMARY_END_CONSTRAINT = (0.0, 4.0)  # meters (x, y) for the primary non-beyblade run
APPLY_PRIMARY_END_CONSTRAINT = True
COLUMN_MAP = {
    "time": "Time",
    "wheel0": "wheel0",
    "wheel1": "wheel1",
    "wheel2": "wheel2",
    "wheel3": "wheel3",
    "acc_x": "acc_x",
    "acc_y": "acc_y",
    "gyro_z": "gyro_z",
    "yaw": "yaw",
}


@dataclass
class WheelConfig:
    wheel_radius: float
    pos_x: float
    pos_y: float
    angle_rad: float
    index: int


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rot_world_to_chassis(yaw: float, vx: float, vy: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    vx_c = c * vx + s * vy
    vy_c = -s * vx + c * vy
    return vx_c, vy_c


def wheel_speed_model(
    wheel: WheelConfig,
    yaw: float,
    vx_world: float,
    vy_world: float,
    yaw_rate: float,
    rotation_radius: float,
) -> float:
    vx_c, vy_c = rot_world_to_chassis(yaw, vx_world, vy_world)
    omega_term = rotation_radius * yaw_rate
    if wheel.index == 0:
        return vx_c - vy_c - omega_term
    if wheel.index == 1:
        return -vx_c - vy_c - omega_term
    if wheel.index == 2:
        return vx_c + vy_c - omega_term
    return -vx_c + vy_c - omega_term


def state_transition(x: np.ndarray, dt: float) -> np.ndarray:
    x_pred = x.copy()
    x_pred[0] = x[0] + x[2] * dt + 0.5 * x[6] * dt * dt
    x_pred[1] = x[1] + x[3] * dt + 0.5 * x[7] * dt * dt
    x_pred[2] = x[2] + x[6] * dt
    x_pred[3] = x[3] + x[7] * dt
    x_pred[4] = wrap_angle(x[4] + x[5] * dt)
    x_pred[5] = x[5]
    x_pred[6] = x[6]
    x_pred[7] = x[7]
    return x_pred


def state_transition_jacobian(dt: float) -> np.ndarray:
    F = np.eye(8)
    F[0, 2] = dt
    F[0, 6] = 0.5 * dt * dt
    F[1, 3] = dt
    F[1, 7] = 0.5 * dt * dt
    F[2, 6] = dt
    F[3, 7] = dt
    F[4, 5] = dt
    return F


def measurement_model_and_jacobian(
    x: np.ndarray,
    wheels: List[WheelConfig],
    rotation_radius: float,
) -> Tuple[np.ndarray, np.ndarray]:
    h = np.zeros(8)
    H = np.zeros((8, 8))

    vx = x[2]
    vy = x[3]
    yaw = x[4]
    yaw_rate = x[5]

    c = math.cos(yaw)
    s = math.sin(yaw)

    d_vx_c_d_vx = c
    d_vx_c_d_vy = s
    d_vy_c_d_vx = -s
    d_vy_c_d_vy = c
    d_vx_c_d_yaw = -s * vx + c * vy
    d_vy_c_d_yaw = -c * vx - s * vy

    vx_c = c * vx + s * vy
    vy_c = -s * vx + c * vy
    omega_term = rotation_radius * yaw_rate

    h[0] = vx_c - vy_c - omega_term
    h[1] = -vx_c - vy_c - omega_term
    h[2] = vx_c + vy_c - omega_term
    h[3] = -vx_c + vy_c - omega_term

    H[0, 2] = d_vx_c_d_vx - d_vy_c_d_vx
    H[0, 3] = d_vx_c_d_vy - d_vy_c_d_vy
    H[0, 4] = d_vx_c_d_yaw - d_vy_c_d_yaw
    H[0, 5] = -rotation_radius

    H[1, 2] = -d_vx_c_d_vx - d_vy_c_d_vx
    H[1, 3] = -d_vx_c_d_vy - d_vy_c_d_vy
    H[1, 4] = -d_vx_c_d_yaw - d_vy_c_d_yaw
    H[1, 5] = -rotation_radius

    H[2, 2] = d_vx_c_d_vx + d_vy_c_d_vx
    H[2, 3] = d_vx_c_d_vy + d_vy_c_d_vy
    H[2, 4] = d_vx_c_d_yaw + d_vy_c_d_yaw
    H[2, 5] = -rotation_radius

    H[3, 2] = -d_vx_c_d_vx + d_vy_c_d_vx
    H[3, 3] = -d_vx_c_d_vy + d_vy_c_d_vy
    H[3, 4] = -d_vx_c_d_yaw + d_vy_c_d_yaw
    H[3, 5] = -rotation_radius

    h[4] = x[6]
    h[5] = x[7]
    h[6] = x[5]
    h[7] = x[4]
    H[4, 6] = 1.0
    H[5, 7] = 1.0
    H[6, 5] = 1.0
    H[7, 4] = 1.0

    return h, H


def load_rows(path: str) -> List[dict]:
    rows = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def collect_samples(paths: List[str]) -> List[dict]:
    samples = []
    for path in paths:
        if not path:
            continue
        if not pathlib.Path(path).exists():
            raise FileNotFoundError(path)
        samples.extend(load_rows(path))
    return samples


def normalize_columns(row: dict) -> dict:
    return {key.strip(): value for key, value in row.items()}


def resolve_column_map(rows: List[dict]) -> Dict[str, str]:
    if not rows:
        raise ValueError("No rows in CSV.")

    sample = normalize_columns(rows[0])
    z_keys = [f"(odometrySubsystem).z[{i}]" for i in range(8)]
    if all(key in sample for key in z_keys):
        time_key = next((k for k in sample.keys() if k.strip().lower() == "time"), None)
        if time_key is None:
            raise ValueError("Missing Time column.")
        return {
            "time": time_key,
            "wheel0": z_keys[0],
            "wheel1": z_keys[1],
            "wheel2": z_keys[2],
            "wheel3": z_keys[3],
            "acc_x": z_keys[4],
            "acc_y": z_keys[5],
            "gyro_z": z_keys[6],
            "yaw": z_keys[7],
        }

    missing = []
    for key, col in COLUMN_MAP.items():
        if col not in sample:
            missing.append(col)
    if missing:
        raise ValueError("Missing columns: " + ", ".join(missing))
    return COLUMN_MAP


def extract_columns(rows: List[dict]) -> Tuple[np.ndarray, np.ndarray]:
    col_map = resolve_column_map(rows)
    rows_norm = [normalize_columns(r) for r in rows]

    times = np.array([float(r[col_map["time"]]) for r in rows_norm])
    z = np.zeros((len(rows_norm), 8))
    z[:, 0] = [float(r[col_map["wheel0"]]) for r in rows_norm]
    z[:, 1] = [float(r[col_map["wheel1"]]) for r in rows_norm]
    z[:, 2] = [float(r[col_map["wheel2"]]) for r in rows_norm]
    z[:, 3] = [float(r[col_map["wheel3"]]) for r in rows_norm]
    z[:, 4] = [float(r[col_map["acc_x"]]) for r in rows_norm]
    z[:, 5] = [float(r[col_map["acc_y"]]) for r in rows_norm]
    z[:, 6] = [float(r[col_map["gyro_z"]]) for r in rows_norm]
    z[:, 7] = [float(r[col_map["yaw"]]) for r in rows_norm]
    yaw0 = z[0, 7]
    z[:, 7] = np.vectorize(wrap_angle)(z[:, 7] - yaw0)
    return times, z


def make_wheels() -> Tuple[List[WheelConfig], float]:
    wheel_radius = 0.1016
    width_x = 0.33
    width_y = 0.33
    wheels = [
        WheelConfig(wheel_radius,  width_x * 0.5,  width_y * 0.5,  math.pi / 4.0, 0),
        WheelConfig(wheel_radius, -width_x * 0.5,  width_y * 0.5, -math.pi / 4.0, 1),
        WheelConfig(wheel_radius,  width_x * 0.5, -width_y * 0.5,  3.0 * math.pi / 4.0, 2),
        WheelConfig(wheel_radius, -width_x * 0.5, -width_y * 0.5, -3.0 * math.pi / 4.0, 3),
    ]
    rotation_radius = (width_x + width_y) * 0.5
    return wheels, rotation_radius


def replay_end(
    times: np.ndarray,
    z: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    P0: np.ndarray,
) -> Tuple[float, float]:
    wheels, rotation_radius = make_wheels()
    x = np.zeros(8)
    P = P0.copy()

    dt = np.diff(times, prepend=times[0])
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 0.01

    for k in range(len(times)):
        x_pred = state_transition(x, dt[k])
        F = state_transition_jacobian(dt[k])
        P = F @ P @ F.T + Q

        h, H = measurement_model_and_jacobian(x_pred, wheels, rotation_radius)
        y = z[k] - h
        y[7] = wrap_angle(y[7])

        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x_pred + K @ y
        x[4] = wrap_angle(x[4])
        P = (np.eye(8) - K @ H) @ P

    return float(x[0]), float(x[1])


def apply_end_constraint(times: np.ndarray, z: np.ndarray, desired_end: Tuple[float, float]) -> np.ndarray:
    Q0 = np.diag([1e2, 1e2, 1e1, 1e1, 1e-2, 1e-1, 5.0, 5.0])
    R0 = np.diag([1.0] * 4 + [1.2, 1.2, 0.05, 0.02])
    P0 = np.diag([0.25, 0.25, 0.25, 0.25, 0.0305, 1.0, 1.0, 1.0])

    end_x, end_y = replay_end(times, z, Q0, R0, P0)
    desired_x, desired_y = desired_end
    end_dist = math.hypot(end_x, end_y)
    desired_dist = math.hypot(desired_x, desired_y)

    if end_dist > 1.0e-6 and desired_dist > 1.0e-6:
        scale = desired_dist / end_dist
        z = z.copy()
        z[:, 0:4] *= scale

        end_angle = math.atan2(end_y, end_x)
        desired_angle = math.atan2(desired_y, desired_x)
        angle_error = wrap_angle(end_angle - desired_angle)
        z[:, 7] = np.vectorize(wrap_angle)(z[:, 7] - angle_error)

    return z


def tune(times: np.ndarray, z: np.ndarray, iterations: int = 5) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    wheels, rotation_radius = make_wheels()

    Q = np.diag([1e2, 1e2, 1e1, 1e1, 1e-2, 1e-1, 5.0, 5.0])
    R = np.diag([1.0] * 4 + [1.2, 1.2, 0.05, 0.02])
    p0_diag = np.array([0.25, 0.25, 0.25, 0.25, 0.0305, 1.0, 1.0, 1.0])

    dt = np.diff(times, prepend=times[0])
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 0.01

    for _ in range(iterations):
        x = np.zeros(8)
        P = np.eye(8) * 1.0
        process_res = []
        meas_res = []

        for k in range(len(times)):
            x_pred = state_transition(x, dt[k])
            F = state_transition_jacobian(dt[k])
            P = F @ P @ F.T + Q

            h, H = measurement_model_and_jacobian(x_pred, wheels, rotation_radius)
            y = z[k] - h
            y[7] = wrap_angle(y[7])

            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x_pred + K @ y
            x[4] = wrap_angle(x[4])
            P = (np.eye(8) - K @ H) @ P

            process_res.append(x - x_pred)
            meas_res.append(y)

        process_res = np.array(process_res)
        meas_res = np.array(meas_res)

        Q = np.diag(np.var(process_res, axis=0))
        R = np.diag(np.var(meas_res, axis=0))

        pos_mean = float(np.mean(np.diag(Q)[0:2]))
        vel_mean = float(np.mean(np.diag(Q)[2:4]))
        acc_mean = float(np.mean(np.diag(Q)[6:8]))
        Q[0, 0] = pos_mean
        Q[1, 1] = pos_mean
        Q[2, 2] = vel_mean
        Q[3, 3] = vel_mean
        Q[6, 6] = acc_mean
        Q[7, 7] = acc_mean

        wheel_mean = float(np.mean(np.diag(R)[:4]))
        for i in range(4):
            R[i, i] = wheel_mean
        acc_meas_mean = float(np.mean(np.diag(R)[4:6]))
        R[4, 4] = acc_meas_mean
        R[5, 5] = acc_meas_mean

    P0 = np.diag(p0_diag)
    return Q, R, P0


def format_cpp_matrix(mat: np.ndarray) -> str:
    flat = mat.flatten()
    return ", ".join(f"{v:.6g}" for v in flat)

def print_diag(label: str, mat: np.ndarray, names: List[str]) -> None:
    diag = np.diag(mat)
    width = max(len(name) for name in names)
    print(f"{label} (diag):")
    for name, value in zip(names, diag):
        print(f"  {name:<{width}} : {value:.6g}")
    print("")


def main() -> int:
    paths = [PRIMARY_SAMPLE_PATH]
    if SECONDARY_SAMPLE_PATH:
        paths.append(SECONDARY_SAMPLE_PATH)

    state_names = ["pos_x", "pos_y", "vel_x", "vel_y", "yaw", "yaw_rate", "acc_x", "acc_y"]
    meas_names = ["wheel0", "wheel1", "wheel2", "wheel3", "acc_x", "acc_y", "gyro_z", "yaw"]

    for path in paths:
        rows = collect_samples([path])
        times, z = extract_columns(rows)
        if APPLY_PRIMARY_END_CONSTRAINT and path == PRIMARY_SAMPLE_PATH:
            z = apply_end_constraint(times, z, PRIMARY_END_CONSTRAINT)
        Q, R, P0 = tune(times, z)
        print(f"Sample: {path}")
        print_diag("Q", Q, state_names)
        print_diag("R", R, meas_names)
        print_diag("P0", P0, state_names)
        print(f"Q:  {format_cpp_matrix(Q)}")
        print(f"R:  {format_cpp_matrix(R)}")
        print(f"P0: {format_cpp_matrix(P0)}")
        print("")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
