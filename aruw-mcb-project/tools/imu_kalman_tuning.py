#!/usr/bin/env python3
"""Estimate Kalman Q/R diagonals from single-IMU CSV logs.

Expected signals: accG (m/s^2 or g) and gyroRadPerSec.
This script assumes the fused IMU uses A = I (state is directly measured),
so Q can be approximated from consecutive sample differences, and R from
measurement variance.
"""

import csv
import math
import os
import re
import sys
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

AXIS_KEYS = ("ax", "ay", "az", "gx", "gy", "gz")
ACC_ORDER = ("ax", "ay", "az")
GYRO_ORDER = ("gx", "gy", "gz")

HEADER_ALIASES = {
    "ax": {"ax", "accx", "acc_x", "accgx", "accg_x", "accg[x]", "accg_x_g"},
    "ay": {"ay", "accy", "acc_y", "accgy", "accg_y", "accg[y]", "accg_y_g"},
    "az": {"az", "accz", "acc_z", "accgz", "accg_z", "accg[z]", "accg_z_g"},
    "gx": {"gx", "gyrox", "gyro_x", "gyro_rads_per_sec_x", "gyrolx"},
    "gy": {"gy", "gyroy", "gyro_y", "gyro_rads_per_sec_y", "gyroly"},
    "gz": {"gz", "gyroz", "gyro_z", "gyro_rads_per_sec_z", "gyrolz"},
}

SOURCE_HINTS = {
    "mpu": "mpu6500",
    "fused": "fusedimu",
}

ACC_HINT = "accg"
GYRO_HINT = "gyroradpersec"

INDEX_TO_AXIS = {0: "x", 1: "y", 2: "z"}


def _normalize_header(name: str) -> str:
    return "".join(ch for ch in name.strip().lower() if ch.isalnum() or ch in {"_", "[", "]"})


def _detect_columns(fieldnames: Sequence[str]) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    normalized = {_normalize_header(name): name for name in fieldnames}
    for key, aliases in HEADER_ALIASES.items():
        for alias in aliases:
            if alias in normalized:
                mapping[key] = normalized[alias]
                break
    return mapping


def _detect_columns_from_dump(fieldnames: Sequence[str], source: str) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    source_key = SOURCE_HINTS.get(source, "")
    for name in fieldnames:
        norm = _normalize_header(name)
        if source_key and source_key not in norm:
            continue
        axis_index = None
        match = re.search(r"\\[(\\d+)\\]", name)
        if match:
            axis_index = int(match.group(1))
        if axis_index is None or axis_index not in INDEX_TO_AXIS:
            continue
        axis_letter = INDEX_TO_AXIS[axis_index]
        if ACC_HINT in norm:
            mapping[f"a{axis_letter}"] = name
        elif GYRO_HINT in norm:
            mapping[f"g{axis_letter}"] = name
    return mapping


def _read_rows(path: str) -> Tuple[List[List[float]], List[str]]:
    with open(path, "r", newline="") as handle:
        sniffer = csv.Sniffer()
        sample = handle.read(4096)
        handle.seek(0)
        has_header = sniffer.has_header(sample)
        reader = csv.reader(handle)
        if not has_header:
            rows = [row for row in reader if row]
            return rows, []

        dict_reader = csv.DictReader(handle)
        rows = [row for row in dict_reader if row]
        return rows, dict_reader.fieldnames or []


def _parse_numeric_rows(
    raw_rows: Iterable,
    fieldnames: Sequence[str],
    columns: Optional[Sequence[str]],
) -> List[List[float]]:
    if not fieldnames:
        parsed: List[List[float]] = []
        for row in raw_rows:
            if len(row) < 6:
                continue
            try:
                values = [float(x) for x in row[:6]]
            except ValueError:
                continue
            parsed.append(values)
        return parsed

    if columns:
        if len(columns) != 6:
            raise ValueError("--columns must specify exactly 6 column names")
        mapping = dict(zip(AXIS_KEYS, columns))
    else:
        mapping = _detect_columns_from_dump(fieldnames, "mpu")
        if len(mapping) != 6:
            mapping = _detect_columns_from_dump(fieldnames, "fused")
        if len(mapping) != 6:
            mapping = _detect_columns(fieldnames)
        if len(mapping) != 6:
            raise ValueError(
                "Could not auto-detect all 6 columns. Provide --columns ax,ay,az,gx,gy,gz."
            )

    parsed = []
    for row in raw_rows:
        values: List[float] = []
        try:
            for key in AXIS_KEYS:
                values.append(float(row[mapping[key]]))
        except (KeyError, ValueError, TypeError):
            continue
        parsed.append(values)
    return parsed


def _mean(values: Sequence[float]) -> float:
    return sum(values) / len(values) if values else float("nan")


def _variance(values: Sequence[float]) -> float:
    if len(values) < 2:
        return float("nan")
    mean_val = _mean(values)
    return sum((v - mean_val) ** 2 for v in values) / (len(values) - 1)


def _diff_variance(values: Sequence[float]) -> float:
    if len(values) < 2:
        return float("nan")
    diffs = [values[i] - values[i - 1] for i in range(1, len(values))]
    return _variance(diffs)


def _format_matrix(diagonal: Sequence[float]) -> str:
    rows = []
    for value in diagonal:
        rows.append("[{: .6e}]".format(value))
    return "\n".join(rows)


def _format_cpp_diagonal(diagonal: Sequence[float], matrix_name: str) -> str:
    lines = []
    for idx, value in enumerate(diagonal):
        lines.append("    {name}({i}, {i}) = {val:.6e}f;".format(name=matrix_name, i=idx, val=value))
    return "\n".join(lines)


def main() -> int:
    csv_path = "Standstill_calibrated_Mpu_vs_fused.csv"
    source = "mpu"
    columns = [
        "((((((aruwsrc::motor_tester::DriversSingleton::drivers).mpu6500).imuData).accG).coordinates_).data)._M_elems[0]",
        "((((((aruwsrc::motor_tester::DriversSingleton::drivers).mpu6500).imuData).accG).coordinates_).data)._M_elems[1]",
        "((((((aruwsrc::motor_tester::DriversSingleton::drivers).mpu6500).imuData).accG).coordinates_).data)._M_elems[2]",
        "((((((aruwsrc::motor_tester::DriversSingleton::drivers).mpu6500).imuData).gyroRadPerSec).coordinates_).data)._M_elems[0]",
        "((((((aruwsrc::motor_tester::DriversSingleton::drivers).mpu6500).imuData).gyroRadPerSec).coordinates_).data)._M_elems[1]",
        "((((((aruwsrc::motor_tester::DriversSingleton::drivers).mpu6500).imuData).gyroRadPerSec).coordinates_).data)._M_elems[2]",
    ]

    if not os.path.exists(csv_path):
        print("Calibrated sample not found: {}".format(csv_path), file=sys.stderr)
        return 2

    raw_rows, fieldnames = _read_rows(csv_path)

    if fieldnames and columns:
        missing = [name for name in columns if name not in fieldnames]
        if missing:
            print("Missing expected columns:", file=sys.stderr)
            for name in missing:
                print("  {}".format(name), file=sys.stderr)
            return 2
    try:
        rows = _parse_numeric_rows(raw_rows, fieldnames, columns)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if not rows:
        print("No valid rows found.", file=sys.stderr)
        return 2

    series = list(zip(*rows))
    means = [_mean(values) for values in series]
    variances = [_variance(values) for values in series]
    diff_variances = [_diff_variance(values) for values in series]

    print("Samples: {}".format(len(rows)))
    print("Source: {}".format(source))
    print("Means (ax, ay, az, gx, gy, gz):")
    print("  " + ", ".join("{: .6e}".format(v) for v in means))
    print("\nR diagonal (measurement variance):")
    print(_format_matrix(variances))
    print("\nQ diagonal (process variance from successive diffs):")
    print(_format_matrix(diff_variances))
    print("\nSuggested accel variance (avg ax/ay/az): {: .6e}".format(_mean(variances[:3])))
    print("Suggested gyro variance (avg gx/gy/gz): {: .6e}".format(_mean(variances[3:])))

    print("\nC++ snippet (Q diagonal):")
    print(_format_cpp_diagonal(diff_variances, "q"))
    print("\nC++ snippet (R diagonal):")
    print(_format_cpp_diagonal(variances, "r"))

    if any(math.isnan(v) for v in variances + diff_variances):
        print("\nWarning: insufficient samples for some variances.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
