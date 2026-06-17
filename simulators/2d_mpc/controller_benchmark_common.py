#!/usr/bin/env python3
"""Shared 2D path-following benchmark utilities for simple controllers."""

import argparse
import csv
import json
import math
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "amcaf-matplotlib"),
)

import matplotlib.pyplot as plt
import numpy as np


WHEEL_BASE_M = 0.32
MAX_STEER_RAD = math.radians(32.0)
MAX_ACCEL_MPS2 = 2.0
MIN_SPEED_MPS = 0.0
MAX_SPEED_MPS = 4.0
DEFAULT_DT_S = 0.05
DEFAULT_DURATION_S = 55.0
TRACK_CHOICES = ("oval", "figure8", "chicane", "hairpin")


@dataclass
class VehicleState:
    """Planar bicycle-model state."""

    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float


@dataclass
class ControlCommand:
    """Controller output for one simulation step."""

    steer_rad: float
    accel_mps2: float
    target_speed_mps: float
    cbf_active: bool = False


class PID:
    """Small PID controller with derivative-on-error."""

    def __init__(self, kp, ki, kd, output_limit=None, integral_limit=10.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_error = None

    def step(self, error, dt):
        """Return the PID output for one sample."""
        self.integral += error * dt
        self.integral = clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )
        if self.previous_error is None or dt <= 0.0:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.output_limit is not None:
            output = clamp(output, -self.output_limit, self.output_limit)
        return output


class ReferenceTrack:
    """Closed reference path with nearest-point and lookahead helpers."""

    def __init__(self, points, target_speed_mps, name):
        self.name = name
        self.points = np.asarray(points, dtype=float)
        self.target_speed_mps = np.asarray(target_speed_mps, dtype=float)
        self.segment_lengths = np.linalg.norm(
            np.roll(self.points, -1, axis=0) - self.points,
            axis=1,
        )
        self.s_m = np.concatenate(([0.0], np.cumsum(self.segment_lengths[:-1])))
        self.length_m = float(np.sum(self.segment_lengths))
        self.yaws = self._compute_yaws()

    @classmethod
    def oval(cls, sample_count=900):
        """Build the shared benchmark oval used by every controller."""
        theta = np.linspace(0.0, 2.0 * math.pi, sample_count, endpoint=False)
        x = 24.0 * np.cos(theta) + 4.0 * np.sin(2.0 * theta)
        y = 13.0 * np.sin(theta)
        return cls.from_points("oval", np.column_stack((x, y)))

    @classmethod
    def figure8(cls, sample_count=900):
        """Build a crossing figure-eight trajectory."""
        theta = np.linspace(0.0, 2.0 * math.pi, sample_count, endpoint=False)
        x = 24.0 * np.sin(theta)
        y = 13.0 * np.sin(theta) * np.cos(theta)
        return cls.from_points("figure8", np.column_stack((x, y)))

    @classmethod
    def chicane(cls, sample_count=900):
        """Build a closed trajectory with repeated left-right transitions."""
        theta = np.linspace(0.0, 2.0 * math.pi, sample_count, endpoint=False)
        x = 30.0 * np.cos(theta)
        y = 7.0 * np.sin(theta) + 5.0 * np.sin(3.0 * theta)
        return cls.from_points("chicane", np.column_stack((x, y)))

    @classmethod
    def hairpin(cls, sample_count=900):
        """Build a loop with one tight hairpin-like end."""
        theta = np.linspace(0.0, 2.0 * math.pi, sample_count, endpoint=False)
        radius = 16.0 + 9.0 * np.cos(theta)
        x = radius * np.cos(theta)
        y = 12.0 * np.sin(theta)
        return cls.from_points("hairpin", np.column_stack((x, y)))

    @classmethod
    def from_name(cls, name):
        """Build a named trajectory."""
        builders = {
            "oval": cls.oval,
            "figure8": cls.figure8,
            "chicane": cls.chicane,
            "hairpin": cls.hairpin,
        }
        return builders[name]()

    @classmethod
    def from_points(cls, name, points):
        """Build a track and assign lower speed to tighter sections."""
        target_speed = target_speed_from_points(points)
        return cls(points, target_speed, name)

    def _compute_yaws(self):
        delta = np.roll(self.points, -1, axis=0) - np.roll(
            self.points,
            1,
            axis=0,
        )
        return np.arctan2(delta[:, 1], delta[:, 0])

    def nearest_index(self, x_m, y_m):
        """Return nearest reference-point index."""
        distances = np.square(self.points[:, 0] - x_m) + np.square(
            self.points[:, 1] - y_m
        )
        return int(np.argmin(distances))

    def lookahead_index(self, nearest_index, lookahead_m):
        """Return an index roughly lookahead_m forward on the track."""
        distance = 0.0
        index = nearest_index
        while distance < lookahead_m:
            distance += self.segment_lengths[index]
            index = (index + 1) % len(self.points)
        return index

    def signed_cross_track_error(self, state, index):
        """Return signed lateral error in the path tangent frame."""
        dx = state.x_m - self.points[index, 0]
        dy = state.y_m - self.points[index, 1]
        yaw = self.yaws[index]
        return -math.sin(yaw) * dx + math.cos(yaw) * dy

    def heading_error(self, state, index):
        """Return vehicle yaw minus path tangent yaw."""
        return wrap_angle(state.yaw_rad - self.yaws[index])

    def progress_m(self, index):
        """Return arc-length progress for a reference index."""
        return float(self.s_m[index])


class BenchmarkMetrics:
    """Collect comparable controller performance metrics."""

    def __init__(self):
        self.rows = []
        self.cbf_interventions = 0
        self.laps_completed = 0
        self.previous_progress_m = 0.0

    def update(self, row, track_length_m):
        """Append one row and update aggregate event counts."""
        if row["cbf_active"]:
            self.cbf_interventions += 1
        progress_m = row["progress_m"]
        if (
            self.previous_progress_m > 0.8 * track_length_m
            and progress_m < 0.2 * track_length_m
        ):
            self.laps_completed += 1
        self.previous_progress_m = progress_m
        self.rows.append(row)

    def summary(self):
        """Return aggregate metrics useful for controller comparison."""
        if not self.rows:
            return {}
        cte = np.asarray([row["cte_m"] for row in self.rows])
        heading = np.asarray([row["heading_error_rad"] for row in self.rows])
        speed_error = np.asarray([row["speed_error_mps"] for row in self.rows])
        steer = np.asarray([row["steer_rad"] for row in self.rows])
        accel = np.asarray([row["accel_mps2"] for row in self.rows])
        clearance = np.asarray([row["obstacle_clearance_m"] for row in self.rows])
        collision_samples = int(np.sum(clearance < 0.0))
        if len(self.rows) > 1:
            dt = self.rows[1]["time_s"] - self.rows[0]["time_s"]
        else:
            dt = 0.0
        return {
            "samples": len(self.rows),
            "duration_s": self.rows[-1]["time_s"],
            "laps_completed": self.laps_completed,
            "mean_abs_cte_m": float(np.mean(np.abs(cte))),
            "rmse_cte_m": float(np.sqrt(np.mean(np.square(cte)))),
            "max_abs_cte_m": float(np.max(np.abs(cte))),
            "mean_abs_heading_error_deg": float(
                math.degrees(np.mean(np.abs(heading)))
            ),
            "mean_abs_speed_error_mps": float(np.mean(np.abs(speed_error))),
            "steering_effort_rad_s": float(np.sum(np.abs(steer)) * dt),
            "accel_effort_mps": float(np.sum(np.abs(accel)) * dt),
            "min_obstacle_clearance_m": float(np.min(clearance)),
            "collision_samples": collision_samples,
            "cbf_interventions": int(self.cbf_interventions),
        }


class BaseTrackController:
    """Shared steering PID and constant-speed controller behavior."""

    name = "pid"

    def __init__(self, track, args):
        self.track = track
        self.args = args
        self.steering_pid = PID(
            args.kp_cte,
            args.ki_cte,
            args.kd_cte,
            output_limit=MAX_STEER_RAD,
            integral_limit=args.integral_limit,
        )

    def command(self, state, dt, nearest_index, obstacle):
        """Return a command for one state sample."""
        cte = self.track.signed_cross_track_error(state, nearest_index)
        heading_error = self.track.heading_error(state, nearest_index)
        steer = -self.steering_pid.step(cte, dt) - self.args.kp_heading * heading_error
        steer = clamp(steer, -MAX_STEER_RAD, MAX_STEER_RAD)
        target_speed = self.target_speed(state, nearest_index, obstacle)
        accel = self.accel_command(state, target_speed, dt)
        return ControlCommand(steer, accel, target_speed)

    def target_speed(self, _state, _nearest_index, _obstacle):
        """Return target speed for the current sample."""
        return self.args.constant_speed_mps

    def accel_command(self, state, target_speed, _dt):
        """Default PID-only behavior uses a fixed acceleration to reach speed."""
        speed_error = target_speed - state.speed_mps
        return clamp(self.args.speed_kp * speed_error, -MAX_ACCEL_MPS2, MAX_ACCEL_MPS2)


class PIDVelocityController(BaseTrackController):
    """Path PID plus velocity control using reference speed."""

    name = "pid_velocity"

    def __init__(self, track, args):
        super().__init__(track, args)
        self.speed_pid = PID(
            args.speed_kp,
            args.speed_ki,
            args.speed_kd,
            output_limit=MAX_ACCEL_MPS2,
            integral_limit=4.0,
        )

    def target_speed(self, _state, nearest_index, _obstacle):
        return float(self.track.target_speed_mps[nearest_index])

    def accel_command(self, state, target_speed, dt):
        return self.speed_pid.step(target_speed - state.speed_mps, dt)


class PIDVelocityCBFController(PIDVelocityController):
    """Path PID plus velocity control with a simple safety filter."""

    name = "pid_velocity_cbf"

    def target_speed(self, state, nearest_index, obstacle):
        desired = super().target_speed(state, nearest_index, obstacle)
        clearance = obstacle.clearance(state)
        if clearance <= self.args.cbf_min_clearance_m:
            return 0.0
        if clearance >= self.args.cbf_slowdown_distance_m:
            return desired
        ratio = (
            (clearance - self.args.cbf_min_clearance_m)
            / (self.args.cbf_slowdown_distance_m - self.args.cbf_min_clearance_m)
        )
        return desired * clamp(ratio, 0.0, 1.0)

    def command(self, state, dt, nearest_index, obstacle):
        command = super().command(state, dt, nearest_index, obstacle)
        clearance = obstacle.clearance(state)
        command.cbf_active = clearance < self.args.cbf_slowdown_distance_m
        if clearance < self.args.cbf_min_clearance_m:
            command.accel_mps2 = min(command.accel_mps2, -MAX_ACCEL_MPS2)
        return command


class StaticObstacle:
    """Circular obstacle used for CBF benchmark metrics."""

    def __init__(self, x_m, y_m, radius_m):
        self.x_m = x_m
        self.y_m = y_m
        self.radius_m = radius_m

    def clearance(self, state):
        """Return distance from vehicle point to obstacle boundary."""
        return math.hypot(state.x_m - self.x_m, state.y_m - self.y_m) - self.radius_m


def simulate(controller_cls, args):
    """Run a benchmark simulation and return summary, rows, and plot data."""
    track = ReferenceTrack.from_name(args.track)
    obstacle_x, obstacle_y = obstacle_position(args, track)
    obstacle = StaticObstacle(
        obstacle_x,
        obstacle_y,
        args.obstacle_radius_m,
    )
    state = VehicleState(
        x_m=track.points[0, 0],
        y_m=track.points[0, 1],
        yaw_rad=track.yaws[0],
        speed_mps=args.initial_speed_mps,
    )
    controller = controller_cls(track, args)
    metrics = BenchmarkMetrics()
    history = {"x": [], "y": []}

    steps = int(args.duration_s / args.dt_s)
    for step in range(steps):
        nearest = track.nearest_index(state.x_m, state.y_m)
        command = controller.command(state, args.dt_s, nearest, obstacle)
        update_bicycle_model(state, command, args.dt_s)

        cte = track.signed_cross_track_error(state, nearest)
        heading_error = track.heading_error(state, nearest)
        speed_ref = command.target_speed_mps
        row = {
            "time_s": step * args.dt_s,
            "x_m": state.x_m,
            "y_m": state.y_m,
            "yaw_rad": state.yaw_rad,
            "speed_mps": state.speed_mps,
            "target_speed_mps": speed_ref,
            "speed_error_mps": speed_ref - state.speed_mps,
            "cte_m": cte,
            "heading_error_rad": heading_error,
            "steer_rad": command.steer_rad,
            "accel_mps2": command.accel_mps2,
            "obstacle_clearance_m": obstacle.clearance(state),
            "cbf_active": command.cbf_active,
            "progress_m": track.progress_m(nearest),
        }
        metrics.update(row, track.length_m)
        history["x"].append(state.x_m)
        history["y"].append(state.y_m)

    summary = metrics.summary()
    summary["track"] = track.name
    return summary, metrics.rows, history, track, obstacle


def update_bicycle_model(state, command, dt):
    """Integrate a kinematic bicycle model one step."""
    state.x_m += state.speed_mps * math.cos(state.yaw_rad) * dt
    state.y_m += state.speed_mps * math.sin(state.yaw_rad) * dt
    state.yaw_rad = wrap_angle(
        state.yaw_rad
        + state.speed_mps / WHEEL_BASE_M * math.tan(command.steer_rad) * dt
    )
    state.speed_mps = clamp(
        state.speed_mps + command.accel_mps2 * dt,
        MIN_SPEED_MPS,
        MAX_SPEED_MPS,
    )


def target_speed_from_points(points):
    """Create a reference-speed profile from local path curvature."""
    points = np.asarray(points, dtype=float)
    forward = np.roll(points, -1, axis=0) - points
    segment_lengths = np.linalg.norm(forward, axis=1)
    yaw = np.unwrap(np.arctan2(forward[:, 1], forward[:, 0]))
    dyaw = np.abs(np.gradient(yaw))
    curvature_hint = dyaw / np.maximum(segment_lengths, 1e-6)
    if float(np.max(curvature_hint)) > 0.0:
        curvature_hint = curvature_hint / np.max(curvature_hint)
    target_speed = 3.0 - 1.5 * np.sqrt(curvature_hint)
    return np.clip(target_speed, 1.2, 3.0)


def obstacle_position(args, track):
    """Return user-provided or track-specific obstacle coordinates."""
    if args.obstacle_x_m is not None and args.obstacle_y_m is not None:
        return args.obstacle_x_m, args.obstacle_y_m
    fractions = {
        "oval": 0.25,
        "figure8": 0.18,
        "chicane": 0.12,
        "hairpin": 0.50,
    }
    index = int(fractions[track.name] * len(track.points)) % len(track.points)
    return float(track.points[index, 0]), float(track.points[index, 1])


def write_outputs(name, summary, rows, history, track, obstacle, args):
    """Write metrics JSON, samples CSV, and optional plot."""
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_stem = f"{name}_{track.name}"
    summary_path = output_dir / f"{output_stem}_metrics.json"
    csv_path = output_dir / f"{output_stem}_samples.csv"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    if args.plot:
        plot_path = output_dir / f"{output_stem}_trajectory.png"
        plot_trajectory(plot_path, history, track, obstacle, name)
    return summary_path, csv_path


def plot_trajectory(path, history, track, obstacle, name):
    """Save a simple path and trajectory plot."""
    figure, axes = plt.subplots(figsize=(9, 6))
    axes.plot(track.points[:, 0], track.points[:, 1], "k--", label="reference")
    axes.plot(history["x"], history["y"], label=name)
    obstacle_patch = plt.Circle(
        (obstacle.x_m, obstacle.y_m),
        obstacle.radius_m,
        color="tab:red",
        alpha=0.25,
        label="obstacle",
    )
    axes.add_patch(obstacle_patch)
    axes.set_aspect("equal")
    axes.grid(True)
    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.set_title(f"{name} on {track.name}")
    axes.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def common_arg_parser(description):
    """Create a shared CLI parser for benchmark scripts."""
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument("--track", choices=TRACK_CHOICES, default="oval")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--dt-s", type=float, default=DEFAULT_DT_S)
    parser.add_argument("--initial-speed-mps", type=float, default=1.2)
    parser.add_argument("--constant-speed-mps", type=float, default=2.0)
    parser.add_argument("--kp-cte", type=float, default=0.55)
    parser.add_argument("--ki-cte", type=float, default=0.0)
    parser.add_argument("--kd-cte", type=float, default=0.08)
    parser.add_argument("--kp-heading", type=float, default=1.4)
    parser.add_argument("--integral-limit", type=float, default=8.0)
    parser.add_argument("--speed-kp", type=float, default=1.2)
    parser.add_argument("--speed-ki", type=float, default=0.05)
    parser.add_argument("--speed-kd", type=float, default=0.02)
    parser.add_argument("--cbf-min-clearance-m", type=float, default=2.0)
    parser.add_argument("--cbf-slowdown-distance-m", type=float, default=8.0)
    parser.add_argument("--obstacle-x-m", type=float)
    parser.add_argument("--obstacle-y-m", type=float)
    parser.add_argument("--obstacle-radius-m", type=float, default=1.6)
    parser.add_argument("--output-dir", default="simulators/2d_mpc/results")
    parser.add_argument("--plot", action="store_true")
    return parser


def run_cli(controller_cls, description, args=None):
    """Run one controller script and print comparable metrics."""
    parser = common_arg_parser(description)
    parsed = parser.parse_args(args)
    summary, rows, history, track, obstacle = simulate(controller_cls, parsed)
    summary_path, csv_path = write_outputs(
        controller_cls.name,
        summary,
        rows,
        history,
        track,
        obstacle,
        parsed,
    )
    print(json.dumps(summary, indent=2))
    print(f"metrics: {summary_path}")
    print(f"samples: {csv_path}")


def clamp(value, lower, upper):
    """Clamp value to inclusive bounds."""
    return max(lower, min(value, upper))


def wrap_angle(angle):
    """Wrap angle to -pi..pi."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
