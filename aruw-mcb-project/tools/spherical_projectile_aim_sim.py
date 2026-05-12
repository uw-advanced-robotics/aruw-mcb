#!/usr/bin/env python3
"""GUI simulator for the spherical projectile drag solver."""

from __future__ import annotations

import math
import os
import sys
import tempfile
import tkinter as tk
from tkinter import ttk

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "aruw_mcb_matplotlib"))
sys.path.insert(0, os.path.dirname(__file__))

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from spherical_projectile_aim_model import (
    ProjectileModel,
    Vec3,
    solve_moving_target,
    trajectory_to_range,
)


class SphericalProjectileAimSim(tk.Tk):
    FIELD_DEFAULTS = (
        ("Target x (m)", "x", 4.0),
        ("Target y (m)", "y", 0.0),
        ("Target z (m)", "z", 0.0),
        ("Target vx (m/s)", "vx", 0.0),
        ("Target vy (m/s)", "vy", 0.0),
        ("Target vz (m/s)", "vz", 0.0),
        ("Target ax (m/s^2)", "ax", 0.0),
        ("Target ay (m/s^2)", "ay", 0.0),
        ("Target az (m/s^2)", "az", 0.0),
        ("Launch speed (m/s)", "launch_speed", 30.0),
        ("Pitch axis offset (m)", "pitch_axis_offset", 0.0),
        ("Projectile diameter (m)", "diameter", 0.017),
        ("Projectile mass (kg)", "mass", 0.0032),
        ("Air density (kg/m^3)", "rho", 1.225),
        ("Dynamic viscosity (Pa*s)", "mu", 1.81e-5),
    )

    def __init__(self) -> None:
        super().__init__()
        self.title("Spherical Projectile Aim Simulator")
        self.geometry("1200x780")

        self.fields: dict[str, tk.StringVar] = {}
        self._pending_update: str | None = None

        self._build_controls()
        self._build_plots()
        self.recompute()

    def _build_controls(self) -> None:
        controls = ttk.Frame(self, padding=8)
        controls.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(controls, text="Inputs", font=("", 14, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 8)
        )

        for row, (label, key, default) in enumerate(self.FIELD_DEFAULTS, start=1):
            ttk.Label(controls, text=label).grid(row=row, column=0, sticky="w", pady=2)
            var = tk.StringVar(value=f"{default:g}")
            var.trace_add("write", lambda *_: self.schedule_update())
            entry = ttk.Entry(controls, textvariable=var, width=14)
            entry.grid(row=row, column=1, sticky="ew", padx=(8, 0), pady=2)
            self.fields[key] = var

        controls.columnconfigure(1, weight=1)

        ttk.Separator(controls).grid(
            row=len(self.FIELD_DEFAULTS) + 1,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=12,
        )
        ttk.Label(controls, text="Solution", font=("", 14, "bold")).grid(
            row=len(self.FIELD_DEFAULTS) + 2,
            column=0,
            columnspan=2,
            sticky="w",
            pady=(0, 8),
        )

        self.status = tk.StringVar(value="")
        ttk.Label(controls, textvariable=self.status, justify=tk.LEFT, wraplength=280).grid(
            row=len(self.FIELD_DEFAULTS) + 3,
            column=0,
            columnspan=2,
            sticky="nw",
        )

    def _build_plots(self) -> None:
        plot_frame = ttk.Frame(self, padding=8)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.figure = Figure(figsize=(8, 6), dpi=100)
        self.side_axis = self.figure.add_subplot(211)
        self.top_axis = self.figure.add_subplot(212)
        self.figure.tight_layout(pad=3.0)

        self.canvas = FigureCanvasTkAgg(self.figure, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def schedule_update(self) -> None:
        if self._pending_update is not None:
            self.after_cancel(self._pending_update)
        self._pending_update = self.after(120, self.recompute)

    def read_float(self, key: str) -> float:
        return float(self.fields[key].get())

    def recompute(self) -> None:
        self._pending_update = None
        try:
            position = Vec3(self.read_float("x"), self.read_float("y"), self.read_float("z"))
            velocity = Vec3(self.read_float("vx"), self.read_float("vy"), self.read_float("vz"))
            acceleration = Vec3(self.read_float("ax"), self.read_float("ay"), self.read_float("az"))
            launch_speed = self.read_float("launch_speed")
            pitch_axis_offset = self.read_float("pitch_axis_offset")
            model = ProjectileModel(
                self.read_float("diameter"),
                self.read_float("mass"),
                self.read_float("rho"),
                self.read_float("mu"),
            )
        except ValueError:
            self.status.set("Invalid numeric input.")
            return

        solution = solve_moving_target(
            position,
            velocity,
            acceleration,
            launch_speed,
            model,
            pitch_axis_offset,
        )

        self.side_axis.clear()
        self.top_axis.clear()
        self.side_axis.set_title("Side view")
        self.side_axis.set_xlabel("Horizontal distance (m)")
        self.side_axis.set_ylabel("Height (m)")
        self.top_axis.set_title("Top view")
        self.top_axis.set_xlabel("X (m)")
        self.top_axis.set_ylabel("Y (m)")
        self.side_axis.grid(True, alpha=0.3)
        self.top_axis.grid(True, alpha=0.3)

        if solution is None:
            self.status.set("No dragged projectile solution.")
            self.canvas.draw_idle()
            return

        target = solution.projected_target
        horizontal = math.hypot(target.x, target.y) + pitch_axis_offset
        trajectory = trajectory_to_range(horizontal, launch_speed, solution.pitch_rad, model)

        if trajectory:
            xs = [state.x for state in trajectory]
            zs = [state.z for state in trajectory]
            self.side_axis.plot(xs, zs, label="projectile")
            self.side_axis.scatter([horizontal], [target.z], label="projected target", zorder=3)

        yaw = solution.yaw_rad
        top_x = [0.0, math.cos(yaw) * horizontal]
        top_y = [0.0, math.sin(yaw) * horizontal]
        self.top_axis.plot(top_x, top_y, label="shot bearing")
        self.top_axis.scatter([target.x], [target.y], label="projected target", zorder=3)

        self.side_axis.legend(loc="best")
        self.top_axis.legend(loc="best")
        self.side_axis.relim()
        self.side_axis.autoscale_view()
        self.top_axis.axis("equal")
        self.top_axis.relim()
        self.top_axis.autoscale_view()

        self.status.set(
            f"pitch: {math.degrees(solution.pitch_rad):.3f} deg\n"
            f"yaw: {math.degrees(solution.yaw_rad):.3f} deg\n"
            f"time of flight: {solution.tof_s:.4f} s\n"
            f"distance: {solution.distance_m:.3f} m\n"
            f"projected target: ({target.x:.3f}, {target.y:.3f}, {target.z:.3f}) m\n"
            f"residual vertical error: {solution.vertical_error_m:.6f} m"
        )
        self.canvas.draw_idle()


def main() -> None:
    SphericalProjectileAimSim().mainloop()


if __name__ == "__main__":
    main()
