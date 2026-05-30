"""
mpc_path_tracking.py

City traffic simulation using an MPC controller with obstacle avoidance.
The scene contains a curving arterial road, crossing streets, moving traffic,
crosswalks, traffic lights, buildings, and an ego vehicle tracking the right
lane while considering nearby traffic as nonlinear MPC safety constraints.
"""

import argparse
import math
import sys
import warnings
from pathlib import Path

warnings.filterwarnings("ignore", message="Pandas requires version .*")

import numpy as np
import do_mpc
import casadi as ca
import matplotlib.animation as anm
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.widgets import Button

# Reuse the algorithm modules kept separate from the runnable simulator.
components_root = (
    Path(__file__).resolve().parents[2]
    / "src"
    / "av_control_guide"
    / "src"
    / "components"
)
sys.path.append(str(components_root / "visualization"))
sys.path.append(str(components_root / "state"))
sys.path.append(str(components_root / "vehicle"))
sys.path.append(str(components_root / "obstacle"))
sys.path.append(str(components_root / "course" / "cubic_spline_course"))
sys.path.append(str(components_root / "control" / "mpc"))
sys.path.append(str(components_root / "common"))

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
from angle_lib import pi_to_pi


TRACK_SCALE = 1.35
NUM_OBSTACLES_TRACKED = 4
OBSTACLE_RADIUS = 3.0
DEFAULT_TRAFFIC_COUNT = 12
MIN_TRAFFIC_COUNT = 4
MAX_TRAFFIC_COUNT = 32
LANE_WIDTH_M = 3.8
LANE_OFFSET_MAIN_M = -1.5 * LANE_WIDTH_M
INNER_LANE_OFFSET_M = -0.5 * LANE_WIDTH_M
MAIN_ROAD_WIDTH_M = 4.0 * LANE_WIDTH_M + 4.0
SECONDARY_ROAD_WIDTH_M = 4.0 * LANE_WIDTH_M + 3.0
LANE_CHANGE_PROB_PER_SEC = 0.08
LANE_CHANGE_RATE_MPS = 1.3
DEFAULT_CONTROL_TOPIC = "/mpc/ackermann_command"


class ROSCommandPublisher:
    """Publish MPC controls only when real-vehicle output is requested."""

    def __init__(self, topic):
        try:
            import rclpy
            from rc_msgs.msg import AckermannCommand
        except ImportError as exc:
            raise RuntimeError(
                "ROS control output needs sourced ROS 2 and built rc_msgs"
            ) from exc

        self._rclpy = rclpy
        self._message_type = AckermannCommand
        self._owns_context = not rclpy.ok()
        if self._owns_context:
            rclpy.init(args=[])
        self._node = rclpy.create_node("mpc_control_publisher")
        self._publisher = self._node.create_publisher(
            AckermannCommand, topic, 10
        )
        print(f"Publishing MPC control commands on {topic}.")

    def publish(self, controller):
        """Publish the control generated for the current simulation step."""
        command = self._message_type()
        command.steering_angle_rad = controller.get_target_steer_rad()
        command.acceleration_mps2 = controller.get_target_accel_mps2()
        command.target_speed_mps = controller.get_reference_speed_mps()
        self._publisher.publish(command)

    def close(self):
        """Destroy ROS resources created for optional control output."""
        self._node.destroy_node()
        if self._owns_context:
            self._rclpy.try_shutdown()


class LaneCourse:
    """Offset-lane wrapper around a centerline course."""

    def __init__(self, base_course, offset):
        self.base_course = base_course
        self.offset = offset

    def point_x_m(self, idx):
        idx = int(idx)
        x = self.base_course.point_x_m(idx)
        yaw = self.base_course.point_yaw_rad(idx)
        return x - math.sin(yaw) * self.offset

    def point_y_m(self, idx):
        idx = int(idx)
        y = self.base_course.point_y_m(idx)
        yaw = self.base_course.point_yaw_rad(idx)
        return y + math.cos(yaw) * self.offset

    def point_yaw_rad(self, idx):
        return self.base_course.point_yaw_rad(int(idx))

    def point_speed_mps(self, idx):
        return self.base_course.point_speed_mps(int(idx))

    def point_curvature(self, idx):
        return self.base_course.point_curvature(int(idx))

    def max_speed_mps(self):
        return self.base_course.max_speed_mps()

    def length(self):
        return self.base_course.length()

    def search_nearest_point_index(self, state):
        vehicle_x = state.get_x_m()
        vehicle_y = state.get_y_m()
        best_idx = 0
        best_dist_sq = float("inf")
        for idx in range(self.length()):
            dx = vehicle_x - self.point_x_m(idx)
            dy = vehicle_y - self.point_y_m(idx)
            dist_sq = dx * dx + dy * dy
            if dist_sq < best_dist_sq:
                best_idx = idx
                best_dist_sq = dist_sq
        return best_idx

    def calculate_distance_from_point(self, state, point_index):
        dx = state.get_x_m() - self.point_x_m(point_index)
        dy = state.get_y_m() - self.point_y_m(point_index)
        return math.hypot(dx, dy)

    def calculate_speed_difference_mps(self, state, point_index):
        return self.point_speed_mps(point_index) - state.get_speed_mps()

    def calculate_angle_difference_rad(self, state, point_index):
        dx = self.point_x_m(point_index) - state.get_x_m()
        dy = self.point_y_m(point_index) - state.get_y_m()
        return math.atan2(dy, dx) - state.get_yaw_rad()

    def calculate_lonlat_error(self, state, point_index):
        error_x = state.get_x_m() - self.point_x_m(point_index)
        error_y = state.get_y_m() - self.point_y_m(point_index)
        current_yaw = state.get_yaw_rad()
        error_yaw = pi_to_pi(current_yaw - self.point_yaw_rad(point_index))
        error_lon = math.cos(current_yaw) * error_x + math.sin(current_yaw) * error_y
        error_lat = -math.sin(current_yaw) * error_x + math.cos(current_yaw) * error_y
        return error_lon, error_lat, error_yaw


class TrafficMPCController(MPCController1):
    """MPC path tracker with soft circular point-mass obstacle constraints."""

    def __init__(self, *args, **kwargs):
        self._last_state = [0.0, 0.0, 0.0, 0.0]
        self.obstacles = None
        self.closest_obstacle_distance_m = float("inf")
        self.closest_obstacle_clearance_m = float("inf")
        self.obstacle_constraints_enabled = NUM_OBSTACLES_TRACKED > 0
        super().__init__(*args, **kwargs)

    def _build_model(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            model = do_mpc.model.Model("discrete")

        model.set_variable("_x", "px")
        model.set_variable("_x", "py")
        model.set_variable("_x", "yaw")
        model.set_variable("_x", "vel")
        model.set_variable("_u", "steer")
        model.set_variable("_u", "accel")
        model.set_variable("_tvp", "ref_x")
        model.set_variable("_tvp", "ref_y")
        model.set_variable("_tvp", "ref_yaw")
        model.set_variable("_tvp", "ref_vel")

        for i in range(NUM_OBSTACLES_TRACKED):
            model.set_variable("_tvp", f"obst_{i}_x")
            model.set_variable("_tvp", f"obst_{i}_y")

        yaw_err = ca.atan2(
            ca.sin(model.x["yaw"] - model.tvp["ref_yaw"]),
            ca.cos(model.x["yaw"] - model.tvp["ref_yaw"]),
        )
        model.set_expression("yaw_err", yaw_err)

        L = self.WHEEL_BASE_M
        dt = self.delta_t
        model.set_rhs("px", model.x["px"] + model.x["vel"] * ca.cos(model.x["yaw"]) * dt)
        model.set_rhs("py", model.x["py"] + model.x["vel"] * ca.sin(model.x["yaw"]) * dt)
        model.set_rhs("yaw", model.x["yaw"] + model.x["vel"] / L * ca.tan(model.u["steer"]) * dt)
        model.set_rhs("vel", model.x["vel"] + model.u["accel"] * dt)
        model.setup()
        return model

    def _build_mpc(self, model):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            mpc = do_mpc.controller.MPC(model)

        mpc.set_param(
            n_horizon=self.T,
            t_step=self.delta_t,
            n_robust=0,
            store_full_solution=True,
            nlpsol_opts=self._ipopt_opts,
        )

        tvp_template = mpc.get_tvp_template()

        def tvp_fun(t_now):
            ego_x, ego_y = self._last_state[0], self._last_state[1]
            sorted_obst = []
            if self.obstacles:
                for obst in self.obstacles.get_list():
                    dx = obst.state.get_x_m() - ego_x
                    dy = obst.state.get_y_m() - ego_y
                    dist = math.hypot(dx, dy)
                    vx = obst.state.get_speed_mps() * math.cos(obst.state.get_yaw_rad())
                    vy = obst.state.get_speed_mps() * math.sin(obst.state.get_yaw_rad())
                    sorted_obst.append((dist, obst.state.get_x_m(), obst.state.get_y_m(), vx, vy))
                sorted_obst.sort()

            for k in range(self.T + 1):
                tvp_template["_tvp", k, "ref_x"] = float(self._current_ref[0, k])
                tvp_template["_tvp", k, "ref_y"] = float(self._current_ref[1, k])
                tvp_template["_tvp", k, "ref_yaw"] = float(self._current_ref[2, k])
                tvp_template["_tvp", k, "ref_vel"] = float(self._current_ref[3, k])
                for i in range(NUM_OBSTACLES_TRACKED):
                    if i < len(sorted_obst):
                        _, ox, oy, ovx, ovy = sorted_obst[i]
                        tvp_template["_tvp", k, f"obst_{i}_x"] = ox + k * self.delta_t * ovx
                        tvp_template["_tvp", k, f"obst_{i}_y"] = oy + k * self.delta_t * ovy
                    else:
                        tvp_template["_tvp", k, f"obst_{i}_x"] = 10000.0
                        tvp_template["_tvp", k, f"obst_{i}_y"] = 10000.0
            return tvp_template

        mpc.set_tvp_fun(tvp_fun)

        sw = self._sw
        tw = self._tw
        x = model.x
        tvp = model.tvp

        lterm = (
            sw[0] * (x["px"] - tvp["ref_x"]) ** 2
            + sw[1] * (x["py"] - tvp["ref_y"]) ** 2
            + sw[2] * model.aux["yaw_err"] ** 2
            + sw[3] * (x["vel"] - tvp["ref_vel"]) ** 2
        )
        mterm = (
            tw[0] * (x["px"] - tvp["ref_x"]) ** 2
            + tw[1] * (x["py"] - tvp["ref_y"]) ** 2
            + tw[2] * model.aux["yaw_err"] ** 2
            + tw[3] * (x["vel"] - tvp["ref_vel"]) ** 2
        )
        mpc.set_objective(lterm=lterm, mterm=mterm)
        mpc.set_rterm(steer=float(self._cw[0] + self._smw[0]), accel=float(self._cw[1] + self._smw[1]))

        for i in range(NUM_OBSTACLES_TRACKED):
            dist_sq = (x["px"] - tvp[f"obst_{i}_x"]) ** 2 + (x["py"] - tvp[f"obst_{i}_y"]) ** 2
            mpc.set_nl_cons(
                f"obst_{i}",
                OBSTACLE_RADIUS ** 2 - dist_sq,
                ub=0.0,
                soft_constraint=True,
                penalty_term_cons=100.0,
            )

        mpc.bounds["lower", "_u", "steer"] = -self.max_steer
        mpc.bounds["upper", "_u", "steer"] = self.max_steer
        mpc.bounds["lower", "_u", "accel"] = -self.max_accel
        mpc.bounds["upper", "_u", "accel"] = self.max_accel
        mpc.bounds["lower", "_x", "vel"] = self.v_min
        mpc.bounds["upper", "_x", "vel"] = self.v_max
        mpc.setup()
        return mpc

    def update(self, state, time_s):
        self._last_state = [state.get_x_m(), state.get_y_m(), state.get_yaw_rad(), state.get_speed_mps()]
        self._update_obstacle_diagnostics(state)
        super().update(state, time_s)

    def get_reference_speed_mps(self):
        """Return the non-reversing route speed requested by the MPC."""
        return max(0.0, float(self._current_ref[3, 0]))

    def set_obstacles(self, obstacles):
        self.obstacles = obstacles

    def _update_obstacle_diagnostics(self, state):
        if not self.obstacles:
            self.closest_obstacle_distance_m = float("inf")
            self.closest_obstacle_clearance_m = float("inf")
            return
        distances = [
            math.hypot(obst.state.get_x_m() - state.get_x_m(), obst.state.get_y_m() - state.get_y_m())
            for obst in self.obstacles.get_list()
        ]
        self.closest_obstacle_distance_m = min(distances) if distances else float("inf")
        self.closest_obstacle_clearance_m = self.closest_obstacle_distance_m - OBSTACLE_RADIUS


class TrafficVehicle:
    def __init__(self, course, start_idx, speed, color="blue", offset=0.0, direction=1, lane_offsets=None):
        self.course = course
        self.idx = float(start_idx)
        self.target_speed = speed
        self.speed = speed
        self.color = color
        self.offset = offset
        self.target_offset = offset
        self.lane_offsets = lane_offsets or [offset]
        self.direction = direction
        self._rng = np.random.default_rng(
            int((self.idx + 1000.0 * abs(offset) + 100.0 * abs(speed)) % 2**32)
        )

        x, y, yaw = self._get_state_at_idx(int(self.idx))
        self.state = State(
            x_m=x,
            y_m=y,
            yaw_rad=yaw if direction == 1 else pi_to_pi(yaw + math.pi),
            speed_mps=speed,
            color=color,
        )
        self.obstacle = Obstacle(self.state, length_m=2.4, width_m=1.1)

    def _get_state_at_idx(self, idx):
        idx = int(idx) % self.course.length()
        base_x = self.course.point_x_m(idx)
        base_y = self.course.point_y_m(idx)
        base_yaw = self.course.point_yaw_rad(idx)
        off_x = base_x - math.sin(base_yaw) * self.offset
        off_y = base_y + math.cos(base_yaw) * self.offset
        return off_x, off_y, base_yaw

    def update(self, dt, signals):
        self._maybe_change_lane(dt)
        self._update_lane_offset(dt)
        x, y, _ = self._get_state_at_idx(int(self.idx))
        near_red = any(signal.is_red and math.hypot(x - signal.x, y - signal.y) < signal.stop_radius for signal in signals)
        desired_speed = 0.0 if near_red else self.target_speed
        accel = 3.0 if desired_speed > self.speed else 5.0
        step_speed = accel * dt
        if self.speed < desired_speed:
            self.speed = min(desired_speed, self.speed + step_speed)
        else:
            self.speed = max(desired_speed, self.speed - step_speed)

        step = (self.speed * dt / 0.1) * self.direction
        self.idx = (self.idx + step) % self.course.length()
        target_x, target_y, target_yaw = self._get_state_at_idx(int(self.idx))
        self.state.x_m = target_x
        self.state.y_m = target_y
        self.state.yaw_rad = target_yaw if self.direction == 1 else pi_to_pi(target_yaw + math.pi)
        self.state.speed_mps = self.speed

    def _maybe_change_lane(self, dt):
        if len(self.lane_offsets) < 2:
            return
        if abs(self.offset - self.target_offset) > 0.1:
            return
        if self._rng.random() < LANE_CHANGE_PROB_PER_SEC * dt:
            choices = [lane for lane in self.lane_offsets if abs(lane - self.target_offset) > 0.1]
            self.target_offset = float(self._rng.choice(choices))

    def _update_lane_offset(self, dt):
        delta = self.target_offset - self.offset
        max_step = LANE_CHANGE_RATE_MPS * dt
        if abs(delta) <= max_step:
            self.offset = self.target_offset
        else:
            self.offset += math.copysign(max_step, delta)

    def draw(self, axes, elems):
        self.obstacle.draw(axes, elems)
        x, y = self.state.get_x_m(), self.state.get_y_m()
        marker, = axes.plot(x, y, marker="o", markersize=3.5, color=self.color, zorder=8)
        elems.append(marker)


class TrafficSignal:
    def __init__(self, x, y, phase_offset=0.0, period=12.0):
        self.x = x
        self.y = y
        self.phase_offset = phase_offset
        self.period = period
        self.time_s = 0.0
        self.stop_radius = 10.0
        self.is_red = True

    def update(self, dt):
        self.time_s += dt
        phase = (self.time_s + self.phase_offset) % self.period
        self.is_red = phase > self.period * 0.5

    def draw(self, axes, elems):
        color = "red" if self.is_red else "limegreen"
        pole = Rectangle((self.x - 0.5, self.y - 0.5), 1.0, 1.0, color="black", zorder=12)
        lamp = Circle((self.x, self.y), 1.1, color=color, ec="black", lw=0.8, zorder=13)
        axes.add_patch(pole)
        axes.add_patch(lamp)
        elems.extend([pole, lamp])


class CityTrafficEnvironment:
    def __init__(self, traffic_count=DEFAULT_TRAFFIC_COUNT):
        self.traffic_count = int(np.clip(traffic_count, MIN_TRAFFIC_COUNT, MAX_TRAFFIC_COUNT))
        s = TRACK_SCALE
        self.main_course = CubicSplineCourse(
            [s * x for x in [0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300,
                             300, 260, 220, 180, 140, 100, 60, 20, 0, 0]],
            [s * y for y in [0, 10, -5, 15, -10, 5, 0, -15, 10, -5, 0,
                             -48, -58, -48, -58, -48, -58, -48, -58, -48, 0]],
            40,
            road_width_m=MAIN_ROAD_WIDTH_M,
        )
        self.cross1_course = CubicSplineCourse(
            [s * 80, s * 80], [s * -88, s * 70], 30, road_width_m=SECONDARY_ROAD_WIDTH_M
        )
        self.cross2_course = CubicSplineCourse(
            [s * 200, s * 200], [s * -88, s * 70], 30, road_width_m=SECONDARY_ROAD_WIDTH_M
        )
        self.north_course = CubicSplineCourse(
            [0, s * 300], [s * 48, s * 48], 40, road_width_m=SECONDARY_ROAD_WIDTH_M
        )
        self.south_course = CubicSplineCourse(
            [0, s * 300], [s * -82, s * -82], 40, road_width_m=SECONDARY_ROAD_WIDTH_M
        )
        self.courses = [self.main_course, self.cross1_course, self.cross2_course, self.north_course, self.south_course]

        self.signals = [
            TrafficSignal(s * 80 - 7, 7, phase_offset=0.0),
            TrafficSignal(s * 80 + 7, -7, phase_offset=6.0),
            TrafficSignal(s * 200 - 7, 7, phase_offset=4.0),
            TrafficSignal(s * 200 + 7, -7, phase_offset=10.0),
        ]

        self.buildings = []
        self.traffic_vehicles = []
        for course, start_idx, speed, color, offset, direction in self._traffic_specs(self.traffic_count):
            self.traffic_vehicles.append(
                TrafficVehicle(
                    course,
                    start_idx,
                    speed,
                    color,
                    offset,
                    direction,
                    self._lane_offsets_for_direction(direction),
                )
            )

        self.obstacle_list = ObstacleList()
        for vehicle in self.traffic_vehicles:
            self.obstacle_list.add_obstacle(vehicle.obstacle)

    def _lane_offsets_for_direction(self, direction):
        if direction >= 0:
            return [-1.5 * LANE_WIDTH_M, -0.5 * LANE_WIDTH_M]
        return [0.5 * LANE_WIDTH_M, 1.5 * LANE_WIDTH_M]

    def _traffic_specs(self, traffic_count):
        def idx(course, fraction):
            return int((fraction % 1.0) * course.length())

        main_specs = [
            (self.main_course, idx(self.main_course, 0.02), 2.3, "dodgerblue", -1.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.08), 2.8, "royalblue", -0.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.16), 3.1, "deepskyblue", -1.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.24), 3.4, "cornflowerblue", -0.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.32), 3.8, "steelblue", -1.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.40), 4.1, "mediumblue", -0.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.50), 3.2, "dodgerblue", -1.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.62), 4.0, "lightskyblue", -0.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.74), 3.5, "royalblue", -1.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.88), 4.2, "navy", -0.5 * LANE_WIDTH_M, 1),
            (self.main_course, idx(self.main_course, 0.05), 3.1, "cyan", 1.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.13), 3.4, "darkturquoise", 0.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.21), 4.0, "teal", 1.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.29), 2.7, "paleturquoise", 0.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.37), 3.0, "cadetblue", 1.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.47), 3.6, "darkcyan", 0.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.59), 4.5, "cyan", 1.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.71), 4.1, "lightseagreen", 0.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.83), 3.7, "mediumturquoise", 1.5 * LANE_WIDTH_M, -1),
            (self.main_course, idx(self.main_course, 0.95), 4.6, "cyan", 0.5 * LANE_WIDTH_M, -1),
        ]
        side_specs = []
        side_fractions = [0.12, 0.35, 0.58, 0.81]
        for i, fraction in enumerate(side_fractions):
            side_specs.extend([
                (self.cross1_course, idx(self.cross1_course, fraction), 4.0, "tomato", -1.5 * LANE_WIDTH_M, 1),
                (self.cross2_course, idx(self.cross2_course, fraction + 0.11), 4.0, "limegreen", 1.5 * LANE_WIDTH_M, -1),
                (self.north_course, idx(self.north_course, fraction + 0.05), 5.8, "purple", -1.5 * LANE_WIDTH_M, 1),
                (self.south_course, idx(self.south_course, fraction + 0.17), 5.8, "sienna", 1.5 * LANE_WIDTH_M, -1),
            ])
        distributed_specs = []
        main_idx = 0
        side_idx = 0
        while len(distributed_specs) < traffic_count and (
            main_idx < len(main_specs) or side_idx < len(side_specs)
        ):
            for _ in range(2):
                if main_idx < len(main_specs) and len(distributed_specs) < traffic_count:
                    distributed_specs.append(main_specs[main_idx])
                    main_idx += 1
            if side_idx < len(side_specs) and len(distributed_specs) < traffic_count:
                distributed_specs.append(side_specs[side_idx])
                side_idx += 1
        return distributed_specs

    def _build_city_blocks(self):
        s = TRACK_SCALE
        blocks = []
        x_positions = [18, 52, 110, 150, 235, 275]
        y_positions = [-82, -34, 26, 70]
        palette = ["#b8c0cc", "#d0b49f", "#a6b7a0", "#c9c0d3", "#d8c99b", "#b5c7d3"]
        for ix, x in enumerate(x_positions):
            for iy, y in enumerate(y_positions):
                width = s * (18 + (ix % 3) * 4)
                height = s * (16 + (iy % 2) * 5)
                block_x = s * x
                block_y = s * y
                if self._overlaps_intersection(block_x, block_y, width, height):
                    continue
                blocks.append((block_x, block_y, width, height, palette[(ix + iy) % len(palette)]))
        return blocks

    def _overlaps_intersection(self, block_x, block_y, width, height):
        for center_x in [TRACK_SCALE * 80, TRACK_SCALE * 200]:
            clear_x = center_x - 42.0
            clear_y = -42.0
            clear_w = 84.0
            clear_h = 84.0
            overlaps_x = block_x < clear_x + clear_w and block_x + width > clear_x
            overlaps_y = block_y < clear_y + clear_h and block_y + height > clear_y
            if overlaps_x and overlaps_y:
                return True
        return False

    def update(self, dt):
        for signal in self.signals:
            signal.update(dt)
        for vehicle in self.traffic_vehicles:
            vehicle.update(dt, self.signals)

    def draw(self, axes, elems):
        self._draw_background(axes, elems)
        for course in self.courses:
            course.draw(axes, elems)
        self._draw_city_details(axes, elems)
        for signal in self.signals:
            signal.draw(axes, elems)
        for vehicle in self.traffic_vehicles:
            vehicle.draw(axes, elems)

    def _draw_background(self, axes, elems):
        bg = Rectangle((-20, -120), TRACK_SCALE * 330 + 40, 240, color="#edf1e7", zorder=-20)
        axes.add_patch(bg)
        elems.append(bg)
        for x, y, width, height, color in self.buildings:
            shadow = Rectangle((x + 1.8, y - 1.8), width, height, color="#8b8f94", alpha=0.25, zorder=-8)
            building = Rectangle((x, y), width, height, color=color, ec="#6f7478", lw=0.9, zorder=-7)
            axes.add_patch(shadow)
            axes.add_patch(building)
            elems.extend([shadow, building])

    def _draw_city_details(self, axes, elems):
        s = TRACK_SCALE
        for course in self.courses:
            self._draw_lane_markings(axes, elems, course)

        for x in [s * 80, s * 200]:
            plaza = Rectangle((x - 22, -22), 44, 44, color="#4d4d4d", alpha=0.85, zorder=1)
            axes.add_patch(plaza)
            elems.append(plaza)
            for offset in [-17, -13, -9, 9, 13, 17]:
                stripe_h = Rectangle((x - 19, offset), 38, 1.1, color="white", alpha=0.85, zorder=9)
                stripe_v = Rectangle((x + offset, -19), 1.1, 38, color="white", alpha=0.85, zorder=9)
                axes.add_patch(stripe_h)
                axes.add_patch(stripe_v)
                elems.extend([stripe_h, stripe_v])

        for y in [s * 48, s * -82]:
            for x in np.arange(10, s * 300, 28):
                dash, = axes.plot([x, x + 10], [y, y], color="white", lw=1.0, ls="-", alpha=0.8, zorder=5)
                elems.append(dash)

    def _draw_lane_markings(self, axes, elems, course):
        xs = np.asarray(course.x_array)
        ys = np.asarray(course.y_array)
        yaws = np.asarray(course.yaw_array)

        for offset, color, linewidth, linestyle in [
            (0.0, "#ffd500", 1.6, "-"),
            (-LANE_WIDTH_M, "white", 1.1, "--"),
            (LANE_WIDTH_M, "white", 1.1, "--"),
        ]:
            line_x = xs - np.sin(yaws) * offset
            line_y = ys + np.cos(yaws) * offset
            (line,) = axes.plot(
                line_x,
                line_y,
                color=color,
                linewidth=linewidth,
                linestyle=linestyle,
                alpha=0.9,
                zorder=6,
            )
            elems.append(line)


class TrafficScenarioOverlay:
    def __init__(self, controller, gui=None):
        self.controller = controller
        self.gui = gui

    def draw(self, axes, elems):
        clearance = self.controller.closest_obstacle_clearance_m
        clearance_text = "inf" if not np.isfinite(clearance) else f"{clearance:.1f} m"
        mode = "Follow" if not self.gui or self.gui.follow_vehicle else "City"
        speed_label = "1x" if not self.gui else f"{self.gui.playback_speed}x"
        traffic_count = "?" if not self.gui else str(self.gui.traffic_count)
        text = (
            "City traffic MPC\n"
            f"Obstacle constraints: {'on' if self.controller.obstacle_constraints_enabled else 'off'}\n"
            f"Nearest clearance: {clearance_text}\n"
            f"Camera: {mode}  Playback: {speed_label}\n"
            f"Vehicles: {traffic_count}"
        )
        elems.append(
            axes.text(
                0.72,
                0.96,
                text,
                transform=axes.transAxes,
                va="top",
                ha="left",
                fontsize=10,
                color="black",
                bbox={"facecolor": "white", "alpha": 0.85, "edgecolor": "0.7"},
                zorder=30,
            )
        )


def build_city_scene(gui=None):
    traffic_count = DEFAULT_TRAFFIC_COUNT if gui is None else gui.traffic_count
    env = CityTrafficEnvironment(traffic_count=traffic_count)
    spec = VehicleSpecification(area_size=34.0)
    ego_course = LaneCourse(env.main_course, offset=LANE_OFFSET_MAIN_M)

    mpc = TrafficMPCController(
        spec,
        ego_course,
        color="g",
        delta_t=0.1,
        horizon_step_T=20,
        stage_cost_weight=[80.0, 80.0, 5.0, 20.0],
        terminal_cost_weight=[80.0, 80.0, 5.0, 20.0],
        control_cost_weight=[1.0, 0.5],
        smoothness_cost_weight=[5.0, 2.0],
        max_steer_abs=0.523,
        max_accel_abs=2.0,
        v_min=0.0,
        v_max=12.0,
        visualize_optimal_traj=True,
        closed_loop=True,
    )
    mpc.set_obstacles(env.obstacle_list)

    start_state = State(
        x_m=ego_course.point_x_m(0),
        y_m=ego_course.point_y_m(0),
        yaw_rad=ego_course.point_yaw_rad(0),
        speed_mps=8.0,
        color=spec.color,
    )
    vehicle = FourWheelsVehicle(start_state, spec, controller=mpc, show_zoom=True)
    objects = [
        env,
        vehicle,
        PathTrackingMetrics(vehicle, ego_course),
        TrafficScenarioOverlay(mpc, gui),
    ]
    return objects, vehicle, mpc


class CityTrafficSimulationGUI:
    def __init__(self, x_lim, y_lim, time_params, command_publisher=None):
        self.x_lim = x_lim
        self.y_lim = y_lim
        self.time_params = time_params
        self.objects = []
        self.elems = []
        self.vehicle = None
        self.controller = None
        self.frame_index = 0
        self.sim_time_s = 0.0
        self.paused = False
        self.follow_vehicle = True
        self.playback_speed = 1
        self.traffic_count = DEFAULT_TRAFFIC_COUNT
        self.command_publisher = command_publisher

        self.figure = plt.figure(figsize=(10, 8))
        self.axes = self.figure.add_subplot(111)
        self.figure.subplots_adjust(bottom=0.13)
        self._setup_axes()
        self._setup_buttons()
        self.reset_scene()
        self._sync_button_labels()

    def _setup_axes(self):
        self.axes.set_aspect("equal")
        self.axes.set_xlabel("X [m]", fontsize=14)
        self.axes.set_ylabel("Y [m]", fontsize=14)
        self.axes.grid(True, color="white", alpha=0.25, linewidth=0.8)

    def _setup_buttons(self):
        button_specs = [
            ("Restart", 0.03, self._on_restart),
            ("Pause", 0.18, self._on_pause),
            ("Camera", 0.33, self._on_camera),
            ("Speed", 0.48, self._on_speed),
            ("Traffic -", 0.63, self._on_traffic_down),
            ("Traffic +", 0.78, self._on_traffic_up),
        ]
        self.buttons = []
        for label, x0, callback in button_specs:
            button_axes = self.figure.add_axes([x0, 0.035, 0.12, 0.045])
            button = Button(button_axes, label, hovercolor="#d9e8ff")
            button.on_clicked(callback)
            self.buttons.append(button)

    def _sync_button_labels(self):
        self.buttons[1].label.set_text("Resume" if self.paused else "Pause")
        self.buttons[2].label.set_text("City" if self.follow_vehicle else "Follow")
        self.buttons[3].label.set_text(f"{self.playback_speed}x")
        self.buttons[4].label.set_text("Traffic -")
        self.buttons[5].label.set_text("Traffic +")

    def reset_scene(self):
        self._clear_artists()
        self.objects, self.vehicle, self.controller = build_city_scene(gui=self)
        self.vehicle.show_zoom = self.follow_vehicle
        self.frame_index = 0
        self.sim_time_s = 0.0
        self._draw_frame()

    def _clear_artists(self):
        while self.elems:
            artist = self.elems.pop()
            try:
                artist.remove()
            except Exception:
                pass

    def _on_restart(self, _event):
        self.paused = False
        self.reset_scene()
        self._sync_button_labels()
        self.figure.canvas.draw_idle()

    def _on_pause(self, _event):
        self.paused = not self.paused
        self._sync_button_labels()
        self.figure.canvas.draw_idle()

    def _on_camera(self, _event):
        self.follow_vehicle = not self.follow_vehicle
        if self.vehicle:
            self.vehicle.show_zoom = self.follow_vehicle
        self._draw_frame()
        self._sync_button_labels()
        self.figure.canvas.draw_idle()

    def _on_speed(self, _event):
        self.playback_speed = {1: 2, 2: 3, 3: 1}[self.playback_speed]
        self._sync_button_labels()
        self.figure.canvas.draw_idle()

    def _on_traffic_down(self, _event):
        self.traffic_count = max(MIN_TRAFFIC_COUNT, self.traffic_count - 4)
        self.reset_scene()
        self._sync_button_labels()
        self.figure.canvas.draw_idle()

    def _on_traffic_up(self, _event):
        self.traffic_count = min(MAX_TRAFFIC_COUNT, self.traffic_count + 4)
        self.reset_scene()
        self._sync_button_labels()
        self.figure.canvas.draw_idle()

    def _draw_frame(self):
        self._clear_artists()
        pause_label = "paused" if self.paused else "running"
        self.axes.set_title(f"City traffic MPC - Time {self.sim_time_s:.1f}s ({pause_label})", fontsize=14)

        for obj in self.objects:
            obj.draw(self.axes, self.elems)

        if not self.follow_vehicle:
            self.axes.set_xlim(self.x_lim.min_value(), self.x_lim.max_value())
            self.axes.set_ylim(self.y_lim.min_value(), self.y_lim.max_value())
        self._setup_axes()

    def update(self, _frame):
        if not self.paused:
            dt = self.time_params.get_interval_sec() * self.playback_speed
            for obj in self.objects:
                if hasattr(obj, "update"):
                    obj.update(dt)
            if self.command_publisher:
                self.command_publisher.publish(self.controller)
            self.sim_time_s += dt
            self.frame_index += self.playback_speed
        self._draw_frame()
        return self.elems

    def draw(self):
        self.anime = anm.FuncAnimation(
            self.figure,
            self.update,
            frames=self.time_params.get_frame_num(),
            interval=self.time_params.get_interval_msec(),
            repeat=False,
        )
        plt.show()


def parse_args():
    """Read optional ROS command publication settings."""
    parser = argparse.ArgumentParser(
        description="Run the city traffic MPC simulation."
    )
    parser.add_argument(
        "--publish-control",
        action="store_true",
        help="Publish MPC steer/acceleration output for hardware actuation.",
    )
    parser.add_argument(
        "--control-topic",
        default=DEFAULT_CONTROL_TOPIC,
        help="ROS topic used with --publish-control.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    x_lim = MinMax(-15, TRACK_SCALE * 300 + 15)
    y_lim = MinMax(TRACK_SCALE * -102, TRACK_SCALE * 92)
    command_publisher = (
        ROSCommandPublisher(args.control_topic) if args.publish_control else None
    )
    app = CityTrafficSimulationGUI(
        x_lim,
        y_lim,
        TimeParameters(span_sec=90),
        command_publisher=command_publisher,
    )

    try:
        app.draw()
    finally:
        if command_publisher:
            command_publisher.close()


if __name__ == "__main__":
    main()
