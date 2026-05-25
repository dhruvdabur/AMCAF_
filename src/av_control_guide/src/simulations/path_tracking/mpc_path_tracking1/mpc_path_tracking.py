"""
mpc_path_tracking.py

Path tracking simulation using the do-mpc based MPC controller.

Visualises:
  - The cubic-spline reference course
  - The four-wheels vehicle animated along the track
  - The MPC predicted (optimal) trajectory at each step.

"""

import sys
import math
from pathlib import Path

import numpy as np

# ── Path setup (identical pattern to mppi_path_tracking.py) ───────────────
components_root = Path(__file__).resolve().parents[3] / "components"
sys.path.append(str(components_root / "visualization"))
sys.path.append(str(components_root / "state"))
sys.path.append(str(components_root / "vehicle"))
sys.path.append(str(components_root / "obstacle"))
sys.path.append(str(components_root / "course" / "cubic_spline_course"))
sys.path.append(str(components_root / "control" / "mpc"))

from global_xy_visualizer import GlobalXYVisualizer
from path_tracking_metrics import PathTrackingMetrics
from min_max import MinMax
from time_parameters import TimeParameters
from vehicle_specification import VehicleSpecification
from state import State
from four_wheels_vehicle import FourWheelsVehicle
from obstacle import Obstacle  # type: ignore[reportMissingImports]
from obstacle_list import ObstacleList  # type: ignore[reportMissingImports]
from cubic_spline_course import CubicSplineCourse
from mpc_controller import MPCController              # ← do-mpc controller

show_plot = True
save_gif = False


def build_dynamic_obstacles(course, num_obstacles=6):
    obstacles = ObstacleList()

    start_x, start_y = course.x_array[0], course.y_array[0]
    end_x, end_y = course.x_array[-1], course.y_array[-1]
    path_yaw = math.atan2(start_y - end_y, start_x - end_x)
    path_length = math.hypot(start_x - end_x, start_y - end_y)
    path_step = path_length / max(num_obstacles, 1)

    lateral_dx = -math.sin(path_yaw)
    lateral_dy = math.cos(path_yaw)

    for index in range(num_obstacles):
        distance_from_end = min(index * path_step * 0.7, path_length)
        progress = 0.0 if path_length == 0.0 else distance_from_end / path_length

        base_x = end_x + (start_x - end_x) * progress
        base_y = end_y + (start_y - end_y) * progress
        lateral_offset = (-1.0) ** index * (1.5 + 0.3 * index)

        obstacles.add_obstacle(
            Obstacle(
                State(
                    x_m=base_x + lateral_dx * lateral_offset,
                    y_m=base_y + lateral_dy * lateral_offset,
                    yaw_rad=path_yaw,
                    speed_mps=0.8 + 0.15 * index,
                ),
                yaw_rate_rps=0.0,
                length_m=2.2,
                width_m=1.0,
            )
        )

    return obstacles


def main():
    # ── Visualiser window  ────────────────────────────────────────────────
    # Same course extents and time span as the MPPI script so results are
    # directly comparable side-by-side.
    x_lim, y_lim = MinMax(-5, 215), MinMax(-45, 45)
    gif_path = (
        str(Path(__file__).absolute().parent / "mpc_path_tracking.gif")
        if save_gif
        else None
    )

    vis = GlobalXYVisualizer(
        x_lim, y_lim, TimeParameters(span_sec=90), gif_name=gif_path
    )

    # ── Reference course  ─────────────────────────────────────────────────
    course = CubicSplineCourse(
        [0.0, 8.0, 16.0, 24.0, 33.0, 42.0, 51.0,
         60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0,
         132.0, 144.0, 156.0, 168.0, 180.0, 192.0, 205.0],
        [0.0, 12.0, -10.0, 18.0, -22.0, 20.0, -16.0,
         -28.0, 8.0, 30.0, -6.0, -32.0, 16.0, 34.0,
         -18.0, -36.0, 6.0, 28.0, -24.0, -8.0, 10.0],
        20,
    )
    vis.add_object(course)

    dynamic_obstacles = build_dynamic_obstacles(course, num_obstacles=6)
    vis.add_object(dynamic_obstacles)

    # ── Vehicle spec & initial state  ─────────────────────────────────────
    spec  = VehicleSpecification(area_size=20.0)
    state = State(color=spec.color)

    # ── MPC controller  ───────────────────────────────────────────────────
    # Parameters are chosen to be comparable to the MPPI configuration:
    #   delta_t and horizon give a similar prediction window (≈ 2 s).
    #   Stage / terminal cost weights mirror the MPPI weights exactly so
    #   both controllers optimise the same objective shape.
    #
    # MPC-specific knobs not present in MPPI:
    #   control_cost_weight    – penalises large raw inputs (do-mpc mterm)
    #   smoothness_cost_weight – penalises delta_u changes (do-mpc rterm)
    #   v_min / v_max          – hard speed bounds enforced by IPOPT
    #   ipopt_max_iter         – solver iteration budget
    mpc = MPCController(
        spec,
        course,
        color                  = "g",      # green dashed predicted trajectory
        delta_t                = 0.1,      # same as MPPI
        horizon_step_T         = 20,       # ≈ 2 s look-ahead  (MPPI uses 22)
        stage_cost_weight      = [50.0, 50.0, 1.0, 20.0],   # [x, y, yaw, v]
        terminal_cost_weight   = [50.0, 50.0, 1.0, 20.0],   # same as MPPI
        control_cost_weight    = [1.0, 0.5],                # [steer, accel]
        smoothness_cost_weight = [5.0, 2.0],                # [delta_steer, delta_accel]
        max_steer_abs          = 0.523,    # rad  (~30°) – same as MPPI
        max_accel_abs          = 2.0,      # m/s²        – same as MPPI
        v_min                  = 0.0,      # m/s  (hard bound, MPC only)
        v_max                  = 15.0,     # m/s  (hard bound, MPC only)
        ipopt_max_iter         = 100,      # IPOPT iteration limit
        ipopt_print_level      = 0,        # silent solver output
        visualize_optimal_traj = True,     # draw the predicted trajectory
    )

    # ── Vehicle  ──────────────────────────────────────────────────────────
    start_x, start_y = course.x_array[0], course.y_array[0]
    start_yaw = course.yaw_array[0]
    state = State(x_m=start_x, y_m=start_y, yaw_rad=start_yaw, color=spec.color)

    vehicle = FourWheelsVehicle(state, spec, controller=mpc)
    vis.add_object(vehicle)
    vis.add_object(PathTrackingMetrics(vehicle, course))

    # ── Run  ──────────────────────────────────────────────────────────────
    if not show_plot:
        vis.not_show_plot()

    vis.draw()


if __name__ == "__main__":
    main()
