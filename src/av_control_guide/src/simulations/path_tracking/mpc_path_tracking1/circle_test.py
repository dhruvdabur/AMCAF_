import sys
from pathlib import Path
import math
import numpy as np

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
from obstacle import Obstacle
from obstacle_list import ObstacleList
from cubic_spline_course import CubicSplineCourse
from mpc_controller1 import MPCController1 

save_gif = False


def build_dynamic_obstacles():
    obstacles = ObstacleList()

    obstacles.add_obstacle(
        Obstacle(
            State(x_m=-24.0, y_m=-8.0, yaw_rad=0.0, speed_mps=1.3),
            yaw_rate_rps=0.0,
            length_m=2.5,
            width_m=1.1,
        )
    )
    obstacles.add_obstacle(
        Obstacle(
            State(x_m=7.0, y_m=25.0, yaw_rad=-math.pi / 2.0, speed_mps=1.0),
            yaw_rate_rps=0.05,
            length_m=2.0,
            width_m=1.0,
        )
    )
    obstacles.add_obstacle(
        Obstacle(
            State(x_m=26.0, y_m=-18.0, yaw_rad=math.pi, speed_mps=0.8),
            yaw_rate_rps=-0.04,
            length_m=2.0,
            width_m=1.0,
        )
    )

    return obstacles


def main():
    # Visualization setup
    x_lim, y_lim = MinMax(-30, 30), MinMax(-30, 30)
    gif_path = (
        str(Path(__file__).absolute().parent / "mpc_circle_test.gif")
        if save_gif
        else None
    )

    vis = GlobalXYVisualizer(
        x_lim, y_lim, TimeParameters(span_sec=40), gif_name=gif_path
    )

    # Generate one nearly closed lap. endpoint=False avoids making the
    # final waypoint identical to the start, which would trigger stop-at-goal.
    radius = 20.0
    angles = np.linspace(0, 2 * math.pi, 30, endpoint=False)
    x_course = (radius * np.cos(angles)).tolist()
    y_course = (radius * np.sin(angles)).tolist()

    course = CubicSplineCourse(x_course, y_course, 10.0)  # 10 m/s speed
    vis.add_object(course)

    dynamic_obstacles = build_dynamic_obstacles()
    vis.add_object(dynamic_obstacles)

    # Initial state at the start of the circle (R, 0)
    spec  = VehicleSpecification(area_size=30.0)
    state = State(x_m=radius, y_m=0.0, yaw_rad=math.pi/2.0, color=spec.color)

    # Setup the new Ackermann MPC controller
    mpc = MPCController1(
        spec,
        course,
        color="b", 
        delta_t=0.1,
        horizon_step_T=20,
        max_steer_abs=0.523,
        max_accel_abs=2.0,
        v_min=0.0,
        v_max=15.0,
        closed_loop=True,
    )

    vehicle = FourWheelsVehicle(state, spec, controller=mpc)
    vis.add_object(vehicle)
    vis.add_object(PathTrackingMetrics(vehicle, course))

    # vis.not_show_plot()  # uncomment to run headlessly
    vis.draw()

if __name__ == "__main__":
    main()
