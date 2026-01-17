#!/usr/bin/env python3
import argparse
import importlib.util
import os
import pathlib

import numpy as np

os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplconfig")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_module(path: pathlib.Path):
    spec = importlib.util.spec_from_file_location("ekf_tune", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def apply_primary_constraint(ekf_module, sample_path: pathlib.Path, times, z):
    if not getattr(ekf_module, "APPLY_PRIMARY_END_CONSTRAINT", False):
        return z
    primary = pathlib.Path(getattr(ekf_module, "PRIMARY_SAMPLE_PATH", ""))
    if primary and sample_path.resolve() == primary.resolve():
        desired = getattr(ekf_module, "PRIMARY_END_CONSTRAINT", None)
        if desired is not None and hasattr(ekf_module, "apply_end_constraint"):
            return ekf_module.apply_end_constraint(times, z, desired)
    return z


def run_replay(ekf_module, sample_path: pathlib.Path, out_dir: pathlib.Path) -> pathlib.Path:
    rows = ekf_module.collect_samples([str(sample_path)])
    times, z = ekf_module.extract_columns(rows)
    z = apply_primary_constraint(ekf_module, sample_path, times, z)
    Q, R, P0 = ekf_module.tune(times, z)

    x = np.zeros(8)
    P = P0.copy()
    positions = []

    dt = np.diff(times, prepend=times[0])
    dt[dt <= 0] = np.median(dt[dt > 0]) if np.any(dt > 0) else 0.01
    wheels, rotation_radius = ekf_module.make_wheels()

    for k in range(len(times)):
        x_pred = ekf_module.state_transition(x, dt[k])
        F = ekf_module.state_transition_jacobian(dt[k])
        P = F @ P @ F.T + Q

        h, H = ekf_module.measurement_model_and_jacobian(x_pred, wheels, rotation_radius)
        y = z[k] - h
        y[7] = ekf_module.wrap_angle(y[7])

        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        x = x_pred + K @ y
        x[4] = ekf_module.wrap_angle(x[4])
        P = (np.eye(8) - K @ H) @ P

        positions.append((x[0], x[1]))

    positions = np.array(positions)
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(positions[:, 0], positions[:, 1], label="ekf")
    ax.set_aspect("equal", "box")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(sample_path.name)
    ax.grid(True, alpha=0.3)
    ax.legend()

    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"ekf_path_{sample_path.stem}.png"
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def main() -> int:
    repo_root = pathlib.Path(__file__).resolve().parents[1]
    ekf_path = repo_root / "aruw-mcb-project/src/aruwsrc/algorithms/odometry/ekf_tune.py"
    ekf_module = load_module(ekf_path)

    parser = argparse.ArgumentParser(description="Replay EKF odometry from z[] logs.")
    parser.add_argument(
        "samples",
        nargs="*",
        help="CSV paths. Defaults to the sample paths in ekf_tune.py.",
    )
    parser.add_argument(
        "--out-dir",
        default=str(repo_root / "tools"),
        help="Directory to write plot images.",
    )
    args = parser.parse_args()

    if args.samples:
        sample_paths = [pathlib.Path(p) for p in args.samples]
    else:
        sample_paths = [
            pathlib.Path(ekf_module.PRIMARY_SAMPLE_PATH),
        ]
        if ekf_module.SECONDARY_SAMPLE_PATH:
            sample_paths.append(pathlib.Path(ekf_module.SECONDARY_SAMPLE_PATH))

    out_dir = pathlib.Path(args.out_dir)
    for sample in sample_paths:
        out_path = run_replay(ekf_module, sample, out_dir)
        print(f"Wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
