"""Reusable controller core for the closed-ellipse ArUco follower."""

UPDATE_RATE_HZ = 50.0

from dataclasses import dataclass
import heapq
import math
import time

import numpy as np

from ..common import bounded
from ..common import wrap_angle
from ..controllers import Car
from ..controllers import EllipseCBFQPConfig
from ..controllers import EllipseCBFQPSafetyFilter
from ..controllers import PID_CBF
from ..controllers import PID_VELOCITY
from ..controllers import PID_VELOCITY_CBF
from ..controllers import PID_VELOCITY_CBF_QP_ELLIPSE
from ..controllers import PID_VELOCITY_DCLF_DCBF
from ..controllers import MPC_CBF
from ..controllers import MPCController
from ..controllers import MPCConfig
from ..controllers import DCLF_DCBFConfig
from ..controllers import DCLF_DCBFSafetyFilter
from ..controllers import DCLF_DCBFPointObstacle
from ..controllers import PIDController
from ..controllers import PIDVelocityCBFController
from ..controllers import PointObstacle
from ..controllers import VirtualLidar
from ..metrics import FollowerMetrics
from ..metrics import unique_metrics_path
from ..road import make_ellipse_road_scene
from ..road import obstacle_clearance_px
from . import gap_planner


@dataclass
class EllipseControlResult:
    """Controller output for one marker/pose update."""

    center: np.ndarray
    heading: float
    target: np.ndarray
    tangent: np.ndarray
    lateral_error_px: float
    heading_error_rad: float
    nearest_index: int
    roll_pwm: int
    throttle_pwm: int
    safety_clearance_px: float
    target_lap_reached: bool


class EllipseStaticController:
    """ROS-free controller state for the closed elliptical road follower."""

    def __init__(self, config):
        self.config = config
        self.track_points = None
        self.road_boundaries = None
        self.road_tangents = None
        self.road_normals = None
        self.road_half_width_px = 0.0
        self.latest_free_space_target = None
        self.latest_free_space_interval = None
        self.latest_free_space_lateral_target_px = 0.0
        self.latest_free_space_blocked_intervals = []
        self.latest_free_space_intervals = []
        self.smoothed_free_space_lateral_target_px = None
        self.static_obstacles = []
        self.road_boundary_obstacles = []
        self.latest_nearest_index = None
        self.latest_target_index = None
        self.latest_target_center = None
        self.latest_target_tangent = None
        self.latest_lateral_error_px = 0.0
        self.latest_heading_error_rad = 0.0
        self.latest_safety_clearances = {}
        self.marker_trail = []
        self.last_image_size = None
        self.last_command_time = None
        self.last_detection_time = None
        self.last_marker_heading = 0.0
        self.latest_vehicle_center = None
        self.aruco_marker_size_px = None
        self.throttle = config.neutral_throttle_pwm
        self.roll = config.center_steering_pwm
        self.nominal_throttle = config.neutral_throttle_pwm
        self.nominal_roll = config.center_steering_pwm
        self.last_marker_seen = False
        self.stop_requested = False
        self.completion_reason = 'running'
        self.pid = PIDController(
            config.steering_kp_px,
            config.steering_ki_px,
            config.steering_kd_px,
            config.integral_limit_px_s,
        )
        self.heading_kp = config.heading_kp
        self.virtual_lidar = VirtualLidar()
        self.velocity_controller = PIDVelocityCBFController(
            PIDController(
                config.velocity_kp_pwm,
                config.velocity_ki_pwm,
                config.velocity_kd_pwm,
                config.velocity_integral_limit,
            ),
            self.virtual_lidar,
        )
        self.cbf_qp_ellipse_filter = EllipseCBFQPSafetyFilter(
            EllipseCBFQPConfig(
                a_ell=config.cbf_a_ell,
                b_ell=config.cbf_b_ell,
                wheelbase=config.qp_wheelbase_px,
                gamma1=config.cbf_gamma1,
                gamma2=config.cbf_gamma2,
                gamma3=config.cbf_gamma3,
                min_accel=config.qp_min_accel,
                max_accel=config.qp_max_accel,
                min_delta=config.qp_min_delta,
                max_delta=config.qp_max_delta,
                solver=config.qp_solver,
                slack_weight=getattr(config, 'qp_slack_weight', 0.0),
            )
        )
        self.dclf_dcbf_filter = DCLF_DCBFSafetyFilter(
            DCLF_DCBFConfig(
                a_ell=config.cbf_a_ell,
                b_ell=config.cbf_b_ell,
                wheelbase=config.qp_wheelbase_px,
                gamma1=config.cbf_gamma1,
                gamma2=config.cbf_gamma2,
                gamma3=config.cbf_gamma3,
                min_accel=config.qp_min_accel,
                max_accel=config.qp_max_accel,
                min_delta=config.qp_min_delta,
                max_delta=config.qp_max_delta,
                solver=config.qp_solver,
                slack_weight=getattr(config, 'qp_slack_weight', 0.0),
                clf_alpha=getattr(config, 'clf_alpha', 0.1),
                clf_slack_weight=getattr(config, 'clf_slack_weight', 500.0),
            )
        )
        self.mpc_controller = MPCController(
            MPCConfig(
                wheelbase=config.qp_wheelbase_px,
                delta_t=1.0 / UPDATE_RATE_HZ,
                max_steer=config.qp_max_delta,
                min_steer=config.qp_min_delta,
                max_accel=config.qp_max_accel,
                min_accel=config.qp_min_accel,
                v_min=0.0,
                v_max=2000.0,
            )
        )
        self.target_track_speed_pps = config.target_track_speed_pps
        self.effective_target_track_speed_pps = config.target_track_speed_pps
        self.cbf_stop_error_px = config.cbf_stop_error_px
        self.last_progress_index = None
        self.last_progress_time = None
        self.last_vehicle_center = None
        self.last_vehicle_time = None
        self.raw_track_speed_pps = 0.0
        self.track_speed_pps = 0.0
        self.speed_error_pps = 0.0
        self.velocity_delta_pwm = 0.0
        self.cbf_scale = 1.0
        self.cbf_active = False
        self.ftg_stuck_repulsion_active = False
        self.ftg_stuck_repulsion_roll_bias = 0.0
        self.ftg_stuck_repulsion_throttle_pwm = config.neutral_throttle_pwm
        self.cbf_qp_accel = 0.0
        self.cbf_qp_delta = 0.0
        self.cbf_qp_status = 'unused'
        self.cbf_qp_h = 0.0
        self.cbf_qp_h_dot = 0.0
        self.cbf_qp_h_ddot = 0.0
        self.cbf_qp_lhs_a = 0.0
        self.cbf_qp_lhs_delta = 0.0
        self.cbf_qp_rhs = 0.0
        self.cbf_qp_solve_time_ms = 0.0
        self.cbf_qp_slack = 0.0
        self.cbf_qp_brake_gate_active = False
        self.cbf_qp_obstacle_x = None
        self.cbf_qp_obstacle_y = None
        self.latest_lidar_points = []
        self.nearest_static_clearance_px = float('inf')
        self.metrics = FollowerMetrics()
        self.metrics_saved = False
        self.metrics_file_path = unique_metrics_path(config.metrics_file)
        self.lap_limit_enabled = config.enable_lap_limit
        self.target_laps = max(0, config.target_laps)
        self.last_pid_debug_print_time = 0.0

    @staticmethod
    def is_road_boundary_obstacle(obstacle):
        """Return whether an obstacle record represents a track boundary wall."""
        return str(obstacle.get('kind', '')).startswith('road_boundary_wall')

    def reset_cbf_qp_debug(self):
        """Clear CBF-QP math diagnostics when the QP path is inactive."""
        self.cbf_qp_h = 0.0
        self.cbf_qp_h_dot = 0.0
        self.cbf_qp_h_ddot = 0.0
        self.cbf_qp_lhs_a = 0.0
        self.cbf_qp_lhs_delta = 0.0
        self.cbf_qp_rhs = 0.0
        self.cbf_qp_brake_gate_active = False
        self.cbf_qp_obstacle_x = None
        self.cbf_qp_obstacle_y = None

    def set_aruco_marker_size_px(self, marker_size_px):
        """Update marker pixel scale from the detected 10 cm ArUco marker."""
        if marker_size_px is None or marker_size_px <= 1e-6:
            return
        self.aruco_marker_size_px = float(marker_size_px)

    def cbf_ellipse_axes_px(self):
        """Return CBF ellipse semi-axes in image pixels."""
        marker_size_px = self.cbf_ellipse_marker_scale_px()
        return (
            float(self.config.cbf_a_ell) * marker_size_px,
            float(self.config.cbf_b_ell) * marker_size_px,
        )

    def cbf_ellipse_marker_scale_px(self):
        """Return the marker pixel scale used by CBF-QP and visualization."""
        if self.aruco_marker_size_px is not None:
            return float(self.aruco_marker_size_px)
        return float(getattr(self.config, 'virtual_marker_size_px', 1.0))

    def cbf_ellipse_axes_cm(self):
        """Return CBF ellipse semi-axes in centimeters."""
        marker_cm = float(self.config.aruco_marker_size_cm)
        return (
            float(self.config.cbf_a_ell) * marker_cm,
            float(self.config.cbf_b_ell) * marker_cm,
        )

    def process_detection(self, center, heading, image_size=None, now=None):
        """Update controller state from one marker/pose detection."""
        if image_size is not None and self.last_image_size != image_size:
            width, height = image_size
            self.build_track_scene(width, height)
        if self.track_points is None:
            raise RuntimeError('track scene must be built before control updates')

        if now is None:
            now = time.monotonic()
        self.last_marker_heading = heading
        self.latest_vehicle_center = np.asarray(center, dtype=np.float32)
        target, tangent, error, nearest_index = self.track_error(center, heading)
        heading_error = wrap_angle(math.atan2(tangent[1], tangent[0]) - heading)
        self.latest_lateral_error_px = error
        self.latest_heading_error_rad = heading_error
        self.latest_target_center = target
        self.latest_target_tangent = tangent
        self.latest_nearest_index = nearest_index
        self.update_marker_trail(center)
        self.roll = self.steering_to_pwm(error, heading_error, now)
        self.nominal_roll = self.roll
        self.throttle = self.throttle_to_pwm(
            center,
            error,
            heading_error,
            nearest_index,
            now,
        )
        speed_error = self.speed_error_pps
        safety_clearance = self.safety_clearance_px(
            center,
            error,
            heading_error,
        )
        self.metrics.update(
            now,
            error,
            heading_error,
            speed_error,
            self.roll,
            self.throttle,
            safety_clearance,
            self.cbf_active,
            nearest_index,
            len(self.track_points),
            self.config,
        )
        reached = self.target_lap_reached()
        if not reached:
            self.last_command_time = now
            self.last_detection_time = now
            self.last_marker_seen = True
        return EllipseControlResult(
            center=center,
            heading=heading,
            target=target,
            tangent=tangent,
            lateral_error_px=error,
            heading_error_rad=heading_error,
            nearest_index=nearest_index,
            roll_pwm=self.roll,
            throttle_pwm=self.throttle,
            safety_clearance_px=safety_clearance,
            target_lap_reached=reached,
        )

    def mark_marker_lost(self):
        """Neutralize controller state when the marker/pose source is lost."""
        if self.last_marker_seen:
            self.metrics.marker_lost()
        self.throttle = self.config.neutral_throttle_pwm
        self.roll = self.config.center_steering_pwm
        self.nominal_throttle = self.throttle
        self.nominal_roll = self.roll
        self.last_command_time = None
        self.last_detection_time = None
        self.last_marker_seen = False
        self.pid.reset()
        self.velocity_controller.reset()
        self.mpc_controller.reset()
        self.last_progress_index = None
        self.last_progress_time = None
        self.last_vehicle_center = None
        self.last_vehicle_time = None
        self.raw_track_speed_pps = 0.0
        self.track_speed_pps = 0.0
        self.speed_error_pps = 0.0
        self.velocity_delta_pwm = 0.0
        self.cbf_scale = 0.0
        self.cbf_active = False
        self.cbf_qp_accel = 0.0
        self.cbf_qp_delta = 0.0
        self.cbf_qp_status = 'reset'
        self.reset_cbf_qp_debug()
        self.cbf_qp_solve_time_ms = 0.0
        self.cbf_qp_slack = 0.0
        self.latest_lidar_points = []

    def request_stop_state(self, reason):
        """Record a requested stop and reset command-producing state."""
        if self.stop_requested:
            return
        self.stop_requested = True
        self.completion_reason = reason
        self.throttle = self.config.neutral_throttle_pwm
        self.roll = self.config.center_steering_pwm
        self.nominal_throttle = self.throttle
        self.nominal_roll = self.roll
        self.last_command_time = None
        self.pid.reset()
        self.velocity_controller.reset()
        self.mpc_controller.reset()
        self.latest_lidar_points = []
        self.cbf_qp_accel = 0.0
        self.cbf_qp_delta = 0.0
        self.cbf_qp_status = 'reset'
        self.reset_cbf_qp_debug()
        self.cbf_qp_solve_time_ms = 0.0
        self.cbf_qp_slack = 0.0

    def command_is_fresh(self, now=None):
        """Return whether a recent marker-derived command is available."""
        if now is None:
            now = time.monotonic()
        return (
            self.last_command_time is not None
            and now - self.last_command_time <= self.config.command_timeout
        )

    def current_tuning_values(self, values=None):
        """Return latest controller tuning values for saving."""
        if values is None:
            values = {
                'steering_kp_px': self.config.steering_kp_px,
                'steering_ki_px': self.config.steering_ki_px,
                'steering_kd_px': self.config.steering_kd_px,
                'heading_kp': self.heading_kp,
                'aruco_parallax_factor': self.config.aruco_parallax_factor,
                'aruco_offset_x_px': getattr(self.config, 'aruco_offset_x_px', 0.0),
                'aruco_offset_y_px': getattr(self.config, 'aruco_offset_y_px', 0.0),
                'forward_pwm': self.config.forward_pwm,
                'target_track_speed_pps': self.target_track_speed_pps,
                'track_speed_filter_alpha': self.config.track_speed_filter_alpha,
                'track_speed_slew_rate_pps2': self.config.track_speed_slew_rate_pps2,
                'ftg_stuck_speed_pps': getattr(self.config, 'ftg_stuck_speed_pps', 0.0),
                'ftg_stuck_forward_pwm': getattr(self.config, 'ftg_stuck_forward_pwm', 0.0),
                'ftg_stuck_steering_gain_pwm': getattr(
                    self.config,
                    'ftg_stuck_steering_gain_pwm',
                    0.0,
                ),
                'ftg_stuck_max_steering_bias_pwm': getattr(
                    self.config,
                    'ftg_stuck_max_steering_bias_pwm',
                    0.0,
                ),
                'ftg_fov_deg': getattr(self.config, 'ftg_fov_deg', 240.0),
                'ftg_max_range_px': getattr(self.config, 'ftg_max_range_px', 500.0),
                'ftg_bubble_radius_px': getattr(self.config, 'ftg_bubble_radius_px', 0.0),
                'velocity_kp_pwm': self.config.velocity_kp_pwm,
                'velocity_ki_pwm': self.config.velocity_ki_pwm,
                'velocity_kd_pwm': self.config.velocity_kd_pwm,
                'cbf_a_ell': self.config.cbf_a_ell,
                'cbf_b_ell': self.config.cbf_b_ell,
                'cbf_gamma1': self.config.cbf_gamma1,
                'cbf_gamma2': self.config.cbf_gamma2,
                'cbf_gamma3': self.config.cbf_gamma3,
                'qp_wheelbase_px': self.config.qp_wheelbase_px,
                'qp_min_accel': self.config.qp_min_accel,
                'qp_max_accel': self.config.qp_max_accel,
                'qp_min_delta': self.config.qp_min_delta,
                'qp_max_delta': self.config.qp_max_delta,
                'qp_solver': self.config.qp_solver,
                'qp_slack_weight': self.config.qp_slack_weight,
                'qp_max_obstacles': getattr(self.config, 'qp_max_obstacles', 4),
                'lap_limit_enabled': self.lap_limit_enabled,
                'target_laps': self.target_laps,
            }
        values.update(
            {
                'controller_mode': self.config.controller_mode,
                'track_shape': self.config.track_shape,
                'aruco_parallax_factor': self.config.aruco_parallax_factor,
                'aruco_offset_x_px': getattr(self.config, 'aruco_offset_x_px', 0.0),
                'aruco_offset_y_px': getattr(self.config, 'aruco_offset_y_px', 0.0),
                'heading_kp': self.heading_kp,
                'min_forward_pwm': self.config.min_forward_pwm,
                'max_forward_pwm': self.config.max_forward_pwm,
                'target_track_speed_pps': self.target_track_speed_pps,
                'track_speed_filter_alpha': self.config.track_speed_filter_alpha,
                'track_speed_slew_rate_pps2': self.config.track_speed_slew_rate_pps2,
                'ftg_stuck_speed_pps': getattr(self.config, 'ftg_stuck_speed_pps', 0.0),
                'ftg_stuck_forward_pwm': getattr(self.config, 'ftg_stuck_forward_pwm', 0.0),
                'ftg_stuck_steering_gain_pwm': getattr(
                    self.config,
                    'ftg_stuck_steering_gain_pwm',
                    0.0,
                ),
                'ftg_stuck_max_steering_bias_pwm': getattr(
                    self.config,
                    'ftg_stuck_max_steering_bias_pwm',
                    0.0,
                ),
                'ftg_fov_deg': getattr(self.config, 'ftg_fov_deg', 240.0),
                'ftg_max_range_px': getattr(self.config, 'ftg_max_range_px', 500.0),
                'ftg_bubble_radius_px': getattr(self.config, 'ftg_bubble_radius_px', 0.0),
                'velocity_kp_pwm': self.config.velocity_kp_pwm,
                'velocity_ki_pwm': self.config.velocity_ki_pwm,
                'velocity_kd_pwm': self.config.velocity_kd_pwm,
                'cbf_a_ell': self.config.cbf_a_ell,
                'cbf_b_ell': self.config.cbf_b_ell,
                'cbf_gamma1': self.config.cbf_gamma1,
                'cbf_gamma2': self.config.cbf_gamma2,
                'cbf_gamma3': self.config.cbf_gamma3,
                'qp_wheelbase_px': self.config.qp_wheelbase_px,
                'qp_min_accel': self.config.qp_min_accel,
                'qp_max_accel': self.config.qp_max_accel,
                'qp_min_delta': self.config.qp_min_delta,
                'qp_max_delta': self.config.qp_max_delta,
                'qp_solver': self.config.qp_solver,
                'qp_slack_weight': self.config.qp_slack_weight,
                'qp_max_obstacles': getattr(self.config, 'qp_max_obstacles', 4),
                'lap_limit_enabled': self.lap_limit_enabled,
                'target_laps': self.target_laps,
                'road_half_width_px': self.config.road_half_width_px,
                'obstacle_margin_px': self.config.obstacle_margin_px,
                'completion_reason': self.completion_reason,
            }
        )
        return values

    def refresh_cbf_qp_config(self):
        """Refresh the QP filter config after CLI/tuning changes."""
        self.cbf_qp_ellipse_filter.config = EllipseCBFQPConfig(
            a_ell=self.config.cbf_a_ell,
            b_ell=self.config.cbf_b_ell,
            wheelbase=self.config.qp_wheelbase_px,
            gamma1=self.config.cbf_gamma1,
            gamma2=self.config.cbf_gamma2,
            gamma3=self.config.cbf_gamma3,
            min_accel=self.config.qp_min_accel,
            max_accel=self.config.qp_max_accel,
            min_delta=self.config.qp_min_delta,
            max_delta=self.config.qp_max_delta,
            solver=self.config.qp_solver,
            slack_weight=getattr(self.config, 'qp_slack_weight', 0.0),
        )
        self.dclf_dcbf_filter.config = DCLF_DCBFConfig(
            a_ell=self.config.cbf_a_ell,
            b_ell=self.config.cbf_b_ell,
            wheelbase=self.config.qp_wheelbase_px,
            gamma1=self.config.cbf_gamma1,
            gamma2=self.config.cbf_gamma2,
            gamma3=self.config.cbf_gamma3,
            min_accel=self.config.qp_min_accel,
            max_accel=self.config.qp_max_accel,
            min_delta=self.config.qp_min_delta,
            max_delta=self.config.qp_max_delta,
            solver=self.config.qp_solver,
            slack_weight=getattr(self.config, 'qp_slack_weight', 0.0),
            clf_alpha=getattr(self.config, 'clf_alpha', 0.1),
            clf_slack_weight=getattr(self.config, 'clf_slack_weight', 500.0),
        )

    def build_track_scene(self, width, height):
        """Create the laneless elliptical road scene for the current image size."""
        self.last_image_size = (width, height)
        scene = make_ellipse_road_scene(width, height, self.config)
        self.road_boundaries = scene['boundaries']
        self.road_tangents = scene['tangents']
        self.road_normals = scene['normals']
        self.road_half_width_px = scene['road_half_width_px']
        self.road_boundary_obstacles = [
            obstacle for obstacle in scene['obstacles']
            if self.is_road_boundary_obstacle(obstacle)
        ]
        self.static_obstacles = [
            obstacle for obstacle in scene['obstacles']
            if not self.is_road_boundary_obstacle(obstacle)
        ]
        if getattr(self.config, 'include_road_boundary_walls', False):
            self.static_obstacles.extend(self.road_boundary_obstacles)
        self.latest_free_space_target = None
        self.latest_free_space_interval = None
        self.latest_free_space_lateral_target_px = 0.0
        self.latest_free_space_blocked_intervals = []
        self.latest_free_space_intervals = []
        self.smoothed_free_space_lateral_target_px = None
        self.track_points = scene['centerline']
        self.mpc_controller.update_track(self.track_points)

    def road_scene_enabled(self):
        """Return whether the current track uses road-scene planning."""
        return True

    def closed_road_scene_enabled(self):
        """Return whether the road scene should wrap around as a loop."""
        return True

    def track_error(self, center, heading):
        """Find a lookahead target and signed image-space steering error."""
        self.track_points = self.select_free_space_path(center)
        distances = np.linalg.norm(self.track_points - center, axis=1)
        nearest_index = int(np.argmin(distances))
        target_index = (
            nearest_index + self.config.lookahead_points
        ) % len(self.track_points)
        self.latest_nearest_index = nearest_index
        self.latest_target_index = target_index
        target = self.free_space_target(target_index)
        next_index = (target_index + 1) % len(self.track_points)
        previous_index = (target_index - 1) % len(self.track_points)
        next_target = self.track_points[next_index]
        previous_target = self.track_points[previous_index]
        tangent = next_target - previous_target
        tangent_norm = np.linalg.norm(tangent)
        if tangent_norm > 0.0:
            tangent = tangent / tangent_norm
        vehicle_right = np.array(
            [-math.sin(heading), math.cos(heading)],
            dtype=np.float32,
        )
        error = float(np.dot(target - center, vehicle_right))
        return target, tangent, error, nearest_index

    def select_free_space_path(self, _center):
        """Use the road centerline as the progress path for laneless roads."""
        return self.track_points

    def free_space_target(self, path_index):
        """Return the selected obstacle-free road interval target."""
        if getattr(self.config, 'traffic_scenario', '') == 'head_on':
            dynamic_obs = getattr(self, 'dynamic_obstacles', [])
            if dynamic_obs:
                obstacle_center = dynamic_obs[0].get('center')
                if obstacle_center is not None:
                    target = np.asarray(obstacle_center, dtype=np.float32)
                    self.latest_free_space_target = target
                    if getattr(self.config, 'gap_planner_mode', 'stable_free_space') == 'follow_the_gap_advanced':
                        heading = getattr(self, 'last_marker_heading', 0.0)
                        if heading is None:
                            heading = 0.0
                        origin = self.latest_vehicle_center
                        if origin is None:
                            origin = np.asarray(self.track_points[path_index], dtype=np.float32)
                        delta_vec = target - origin
                        target_dist = float(np.linalg.norm(delta_vec))
                        target_angle = 0.0
                        if target_dist > 0.0:
                            target_angle = float(math.atan2(delta_vec[1], delta_vec[0]) - heading)
                        self.latest_ftg_debug = {
                            'target': target,
                            'target_angle': target_angle,
                            'target_dist': target_dist,
                            'nearest_point': target,
                            'nearest_angle': target_angle,
                            'nearest_dist': target_dist,
                            'bubble_radius': float(getattr(self.config, 'obstacle_margin_px', 42.0)),
                            'max_range_cap': 500.0,
                            'safe_ranges': np.array([target_dist], dtype=np.float32),
                            'angles': np.array([target_angle], dtype=np.float32),
                        }
                    return target

        if getattr(self.config, 'gap_planner_mode', 'stable_free_space') == 'follow_the_gap_advanced':
            from . import follow_the_gap_advanced
            return follow_the_gap_advanced.select_advanced_free_space_target(self, path_index)

        return gap_planner.free_space_target(self, path_index)

    def gap_planner_obstacles(self):
        """Return obstacles considered by the lateral gap planner."""
        return self.static_obstacles

    def steering_to_pwm(self, lateral_error_px, heading_error_rad, now):
        """Convert image-space path error into bounded steering PWM."""
        steering_delta = self.pid.step(lateral_error_px, now)
        heading_term = self.heading_kp * heading_error_rad
        steering_delta += heading_term
        roll = self.config.center_steering_pwm - steering_delta
        bounded_roll = bounded(
            roll,
            self.config.right_pwm,
            self.config.left_pwm,
        )
        if getattr(self.config, 'debug_visuals', False):
            debug_interval = max(
                0.0,
                float(getattr(self.config, 'debug_print_interval_s', 1.0)),
            )
            if now - self.last_pid_debug_print_time >= debug_interval:
                self.last_pid_debug_print_time = now
                d_error = 0.0 if self.pid.previous_error is None or self.pid.previous_time is None or now == self.pid.previous_time else (lateral_error_px - self.pid.previous_error) / (now - self.pid.previous_time)
                print(f"[PID_STEER_DEBUG] cte={lateral_error_px:.1f}px | head_err={heading_error_rad:.3f}rad | p_term={self.pid.kp*lateral_error_px:.2f} | i_term={self.pid.ki*self.pid.integral:.2f} | d_term={self.pid.kd*d_error:.2f} | heading_term={heading_term:.2f} | roll_pwm={bounded_roll}")
        if (
            (roll > self.config.left_pwm and lateral_error_px < 0.0)
            or (roll < self.config.right_pwm and lateral_error_px > 0.0)
        ):
            self.pid.decay_integral(0.5)
        return round(bounded_roll)

    def throttle_to_pwm(
        self,
        center,
        lateral_error_px,
        heading_error_rad,
        nearest_index,
        now,
    ):
        """Return drive PWM for the selected controller mode."""
        self.update_track_speed(nearest_index, now)
        self.effective_target_track_speed_pps = self.target_track_speed_pps
        self.speed_error_pps = 0.0
        self.velocity_delta_pwm = 0.0

        if self.config.controller_mode == PID_VELOCITY:
            throttle, speed_delta, speed_error = (
                self.velocity_controller.velocity_throttle(
                    self.target_track_speed_pps,
                    self.track_speed_pps,
                    self.config.forward_pwm,
                    self.config.min_forward_pwm,
                    self.config.max_forward_pwm,
                    now,
                )
            )
            self.velocity_delta_pwm = speed_delta
            self.speed_error_pps = speed_error
            self.cbf_scale = 1.0
            self.cbf_active = False
            self.cbf_qp_status = 'unused'
            self.reset_cbf_qp_debug()
            self.cbf_qp_solve_time_ms = 0.0
            self.cbf_qp_slack = 0.0
            self.nominal_throttle = throttle
            return self.command_throttle_pwm(throttle)

        if self.config.controller_mode == PID_VELOCITY_CBF:
            return self.pid_velocity_cbf_throttle(
                center,
                lateral_error_px,
                heading_error_rad,
                now,
            )

        if self.config.controller_mode == PID_CBF:
            return self.pid_cbf_throttle(
                center,
                lateral_error_px,
                heading_error_rad,
                now,
            )

        if self.config.controller_mode in (PID_VELOCITY_CBF_QP_ELLIPSE, PID_VELOCITY_DCLF_DCBF):
            target_speed = self.target_track_speed_pps
            target_speed *= self.cbf_velocity_scale(
                center,
                lateral_error_px,
                heading_error_rad,
                now,
            )
            self.effective_target_track_speed_pps = target_speed
            nominal_throttle, speed_delta, speed_error = (
                self.velocity_controller.velocity_throttle(
                    target_speed,
                    self.track_speed_pps,
                    self.config.forward_pwm,
                    self.config.min_forward_pwm,
                    self.config.max_forward_pwm,
                    now,
                )
            )
            self.velocity_delta_pwm = speed_delta
            self.speed_error_pps = speed_error
            cbf_scale_before_qp = self.cbf_scale
            throttle, roll = self.apply_cbf_qp(
                center,
                nominal_throttle,
                self.roll,
            )
            self.cbf_scale = min(cbf_scale_before_qp, self.cbf_scale)
            self.cbf_active = self.cbf_active or self.cbf_scale < 0.999
            throttle, roll = self.apply_ftg_stuck_repulsion(throttle, roll)
            self.roll = round(roll)
            return self.command_throttle_pwm(throttle)

        if self.config.controller_mode == MPC_CBF:
            spacing = 1.0
            if self.track_points is not None and len(self.track_points) > 1:
                spacing = float(np.linalg.norm(self.track_points[1] - self.track_points[0]))

            target_speed = self.target_track_speed_pps
            target_speed *= self.cbf_velocity_scale(
                center,
                lateral_error_px,
                heading_error_rad,
                now,
            )
            self.effective_target_track_speed_pps = target_speed
            target_speed_px = target_speed * spacing
            current_speed_px = float(self.track_speed_pps) * spacing

            px = float(center[0])
            py = float(center[1])
            yaw = float(self.lidar_heading_rad())

            accel, steer = self.mpc_controller.step(
                px=px,
                py=py,
                yaw=yaw,
                vel=current_speed_px,
                target_speed=target_speed_px,
            )

            nominal_throttle = self.qp_accel_to_throttle_pwm(accel)
            nominal_roll = self.qp_delta_to_roll_pwm(steer)

            self.speed_error_pps = target_speed - self.track_speed_pps

            cbf_scale_before_qp = self.cbf_scale
            throttle, roll = self.apply_cbf_qp(
                center,
                nominal_throttle,
                nominal_roll,
            )
            self.cbf_scale = min(cbf_scale_before_qp, self.cbf_scale)
            self.cbf_active = self.cbf_active or self.cbf_scale < 0.999
            throttle, roll = self.apply_ftg_stuck_repulsion(throttle, roll)
            self.roll = round(roll)
            return self.command_throttle_pwm(throttle)

        self.cbf_scale = 1.0
        self.cbf_active = False
        self.cbf_qp_status = 'unused'
        self.reset_cbf_qp_debug()
        self.cbf_qp_solve_time_ms = 0.0
        self.cbf_qp_slack = 0.0
        self.nominal_throttle = self.config.forward_pwm
        return self.command_throttle_pwm(self.config.forward_pwm)

    def command_throttle_pwm(self, throttle_pwm):
        """Return either stopped throttle or a minimum actuating forward PWM."""
        throttle_pwm = float(throttle_pwm)
        neutral = float(self.config.neutral_throttle_pwm)
        if throttle_pwm < neutral:
            return int(round(neutral))
        return int(round(throttle_pwm))

    def apply_ftg_stuck_repulsion(self, throttle_pwm, roll_pwm):
        """Apply a small FTG-directed nudge when safety filtering leaves the bot stuck."""
        self.ftg_stuck_repulsion_active = False
        self.ftg_stuck_repulsion_roll_bias = 0.0
        self.ftg_stuck_repulsion_throttle_pwm = self.config.neutral_throttle_pwm

        if getattr(self.config, 'gap_planner_mode', '') != 'follow_the_gap_advanced':
            return throttle_pwm, roll_pwm
        if getattr(self.config, 'traffic_scenario', '') == 'head_on':
            return throttle_pwm, roll_pwm

        nudge_pwm = float(getattr(self.config, 'ftg_stuck_forward_pwm', 0.0))
        if nudge_pwm <= 0.0:
            return throttle_pwm, roll_pwm

        stuck_speed = max(0.0, float(getattr(self.config, 'ftg_stuck_speed_pps', 0.0)))
        if abs(float(self.track_speed_pps)) > stuck_speed:
            return throttle_pwm, roll_pwm

        neutral = float(self.config.neutral_throttle_pwm)
        if float(throttle_pwm) > neutral + max(1.0, 0.25 * nudge_pwm):
            return throttle_pwm, roll_pwm

        ftg_debug = getattr(self, 'latest_ftg_debug', None)
        if not ftg_debug:
            return throttle_pwm, roll_pwm

        target_dist = float(ftg_debug.get('target_dist', 0.0) or 0.0)
        if (
            target_dist <= 0.0
            or (
                ftg_debug.get('scaled_target') is None
                and ftg_debug.get('target') is None
            )
        ):
            return throttle_pwm, roll_pwm

        target_angle = float(ftg_debug.get('target_angle', 0.0) or 0.0)
        steering_gain = float(getattr(self.config, 'ftg_stuck_steering_gain_pwm', 0.0))
        max_bias = max(0.0, float(getattr(self.config, 'ftg_stuck_max_steering_bias_pwm', 0.0)))
        roll_bias = bounded(
            steering_gain * math.sin(target_angle),
            -max_bias,
            max_bias,
        )
        nudged_roll = bounded(
            float(roll_pwm) - roll_bias,
            self.config.right_pwm,
            self.config.left_pwm,
        )
        nudged_throttle = max(float(throttle_pwm), neutral + nudge_pwm)

        self.ftg_stuck_repulsion_active = True
        self.ftg_stuck_repulsion_roll_bias = roll_bias
        self.ftg_stuck_repulsion_throttle_pwm = nudged_throttle
        self.cbf_active = True
        if self.cbf_qp_status not in ('unused', 'unavailable'):
            self.cbf_qp_status = f'{self.cbf_qp_status}+ftg_unstuck'
        return nudged_throttle, nudged_roll

    def pid_cbf_throttle(
        self,
        center,
        lateral_error_px,
        heading_error_rad,
        now,
    ):
        """Run fixed forward throttle with derivative-CBF scaling."""
        self.speed_error_pps = self.target_track_speed_pps - self.track_speed_pps
        result = self.velocity_controller.apply_cbf(
            self.config.forward_pwm,
            self.config.neutral_throttle_pwm,
            center,
            self.last_marker_heading,
            lateral_error_px,
            heading_error_rad,
            self.last_image_size,
            self.static_obstacles,
            self.config.cbf_slow_error_px,
            self.cbf_stop_error_px,
            self.config.cbf_stop_heading_rad,
            self.config.cbf_edge_margin_px,
            self.config.obstacle_margin_px,
            self.config.cbf_h_px,
            self.config.cbf_alpha,
            now,
        )
        self.cbf_scale = result.scale
        self.cbf_active = result.active
        self.cbf_qp_status = 'unused'
        self.reset_cbf_qp_debug()
        self.cbf_qp_solve_time_ms = 0.0
        self.cbf_qp_slack = 0.0
        self.nearest_static_clearance_px = result.lidar_clearance_px
        self.latest_lidar_points = (
            self.velocity_controller.virtual_lidar.latest_points
        )
        self.nominal_throttle = self.config.forward_pwm
        throttle, roll = self.apply_ftg_stuck_repulsion(result.throttle_pwm, self.roll)
        self.roll = round(roll)
        return self.command_throttle_pwm(throttle)

    def pid_velocity_cbf_throttle(
        self,
        center,
        lateral_error_px,
        heading_error_rad,
        now,
    ):
        """Run velocity PID with derivative-CBF throttle scaling."""
        throttle, speed_delta, speed_error = (
            self.velocity_controller.velocity_throttle(
                self.target_track_speed_pps,
                self.track_speed_pps,
                self.config.forward_pwm,
                self.config.min_forward_pwm,
                self.config.max_forward_pwm,
                now,
            )
        )
        self.velocity_delta_pwm = speed_delta
        self.speed_error_pps = speed_error
        result = self.velocity_controller.apply_cbf(
            throttle,
            self.config.neutral_throttle_pwm,
            center,
            self.last_marker_heading,
            lateral_error_px,
            heading_error_rad,
            self.last_image_size,
            self.static_obstacles,
            self.config.cbf_slow_error_px,
            self.cbf_stop_error_px,
            self.config.cbf_stop_heading_rad,
            self.config.cbf_edge_margin_px,
            self.config.obstacle_margin_px,
            self.config.cbf_h_px,
            self.config.cbf_alpha,
            now,
        )
        self.cbf_scale = result.scale
        self.cbf_active = result.active
        self.cbf_qp_status = 'unused'
        self.reset_cbf_qp_debug()
        self.cbf_qp_solve_time_ms = 0.0
        self.cbf_qp_slack = 0.0
        self.nearest_static_clearance_px = result.lidar_clearance_px
        self.latest_lidar_points = (
            self.velocity_controller.virtual_lidar.latest_points
        )
        self.nominal_throttle = throttle
        self.velocity_controller.apply_throttle_anti_windup(
            throttle,
            result.throttle_pwm,
            speed_error,
        )
        throttle, roll = self.apply_ftg_stuck_repulsion(result.throttle_pwm, self.roll)
        self.roll = round(roll)
        return self.command_throttle_pwm(throttle)

    def cbf_velocity_scale(
        self,
        center,
        lateral_error_px,
        heading_error_rad,
        now,
    ):
        """Return a derivative-CBF scale for the target velocity."""
        result = self.velocity_controller.apply_cbf(
            self.config.max_forward_pwm,
            self.config.neutral_throttle_pwm,
            center,
            self.last_marker_heading,
            lateral_error_px,
            heading_error_rad,
            self.last_image_size,
            self.static_obstacles,
            self.config.cbf_slow_error_px,
            self.cbf_stop_error_px,
            self.config.cbf_stop_heading_rad,
            self.config.cbf_edge_margin_px,
            self.config.obstacle_margin_px,
            self.config.cbf_h_px,
            self.config.cbf_alpha,
            now,
        )
        self.cbf_scale = result.scale
        self.cbf_active = result.active
        self.nearest_static_clearance_px = result.lidar_clearance_px
        self.latest_lidar_points = (
            self.velocity_controller.virtual_lidar.latest_points
        )
        return result.scale

    def apply_cbf_qp(self, center, throttle_pwm, roll_pwm):
        """Filter nominal throttle and steering through the ellipse CBF-QP."""
        spacing = 1.0
        if self.track_points is not None and len(self.track_points) > 1:
            spacing = float(np.linalg.norm(self.track_points[1] - self.track_points[0]))

        car = Car(
            x=float(center[0]),
            y=float(center[1]),
            psi=float(self.lidar_heading_rad()),
            v=max(0.0, float(self.track_speed_pps) * spacing),
            v_cmd=max(0.0, float(self.track_speed_pps) * spacing),
        )
        lidar_points = self.virtual_lidar.scan(
            center,
            self.lidar_heading_rad(),
            self.static_obstacles,
        )
        self.latest_lidar_points = self.virtual_lidar.latest_points
        self.nearest_static_clearance_px = self.virtual_lidar.nearest_clearance_px(
            self.config.cbf_h_px
        )
        obstacles = []
        max_obstacles = max(1, int(getattr(self.config, 'qp_max_obstacles', 4)))
        closest_points = heapq.nsmallest(
            max_obstacles,
            lidar_points,
            key=lambda point: point.distance_px,
        )
        is_dclf = (self.config.controller_mode == PID_VELOCITY_DCLF_DCBF)
        
        # 1. Add closest lidar points
        for point in closest_points:
            obstacle_velocity = self.lidar_point_obstacle_velocity(point)
            if is_dclf:
                obstacles.append(DCLF_DCBFPointObstacle(
                    x=float(point.x_px),
                    y=float(point.y_px),
                    vx=obstacle_velocity[0],
                    vy=obstacle_velocity[1],
                ))
            else:
                obstacles.append(PointObstacle(
                    x=float(point.x_px),
                    y=float(point.y_px),
                    vx=obstacle_velocity[0],
                    vy=obstacle_velocity[1],
                ))
                
        # 2. Add road boundary wall obstacles if enabled
        if getattr(self.config, 'include_road_boundary_walls', False) and self.track_points is not None and self.road_normals is not None:
            idx = self.latest_nearest_index
            if idx is None:
                center_np = np.array([car.x, car.y], dtype=np.float32)
                dists = np.linalg.norm(self.track_points - center_np, axis=1)
                idx = int(np.argmin(dists))
            
            closed = 'ellipse' in str(self.__class__.__name__).lower()
            
            # Add boundary points at offsets (0, 8) for current and lookahead projection (4 points total)
            for offset in (0, 8):
                i = idx + offset
                if closed:
                    i = i % len(self.track_points)
                else:
                    i = max(0, min(i, len(self.track_points) - 1))
                
                pt = self.track_points[i]
                normal = self.road_normals[i]
                left_pt = pt - normal * self.road_half_width_px
                right_pt = pt + normal * self.road_half_width_px
                
                if is_dclf:
                    obstacles.append(DCLF_DCBFPointObstacle(
                        x=float(left_pt[0]),
                        y=float(left_pt[1]),
                        vx=0.0,
                        vy=0.0
                    ))
                    obstacles.append(DCLF_DCBFPointObstacle(
                        x=float(right_pt[0]),
                        y=float(right_pt[1]),
                        vx=0.0,
                        vy=0.0
                    ))
                else:
                    obstacles.append(PointObstacle(
                        x=float(left_pt[0]),
                        y=float(left_pt[1]),
                        vx=0.0,
                        vy=0.0
                    ))
                    obstacles.append(PointObstacle(
                        x=float(right_pt[0]),
                        y=float(right_pt[1]),
                        vx=0.0,
                        vy=0.0
                    ))

        # 3. If no obstacles to filter, return nominal controls
        if not obstacles:
            self.nominal_throttle = self.config.forward_pwm
            self.nominal_roll = roll_pwm
            self.cbf_qp_status = 'no_x_obs'
            self.cbf_qp_accel = 0.0
            self.cbf_qp_delta = self.roll_pwm_to_qp_delta(roll_pwm)
            self.reset_cbf_qp_debug()
            self.cbf_qp_solve_time_ms = 0.0
            self.cbf_qp_slack = 0.0
            self.cbf_scale = 1.0
            self.cbf_active = False
            self.velocity_controller.apply_throttle_anti_windup(
                throttle_pwm,
                self.config.forward_pwm,
                self.speed_error_pps,
            )
            return float(self.config.forward_pwm), roll_pwm
        accel_ref = self.throttle_pwm_to_qp_accel(throttle_pwm)
        delta_ref = self.roll_pwm_to_qp_delta(roll_pwm)
        try:
            a_ell_px, b_ell_px = self.cbf_ellipse_axes_px()
            if is_dclf:
                self.dclf_dcbf_filter.config.a_ell = a_ell_px
                self.dclf_dcbf_filter.config.b_ell = b_ell_px
                accel, delta = self.dclf_dcbf_filter.solve(
                    car,
                    obstacles,
                    accel_ref,
                    delta_ref,
                )
                active_filter = self.dclf_dcbf_filter
            else:
                self.cbf_qp_ellipse_filter.config.a_ell = a_ell_px
                self.cbf_qp_ellipse_filter.config.b_ell = b_ell_px
                accel, delta = self.cbf_qp_ellipse_filter.solve(
                    car,
                    obstacles,
                    accel_ref,
                    delta_ref,
                )
                active_filter = self.cbf_qp_ellipse_filter

            self.cbf_qp_status = active_filter.last_status or 'unknown'
            self.cbf_qp_h = active_filter.last_h
            self.cbf_qp_h_dot = active_filter.last_h_dot
            self.cbf_qp_h_ddot = active_filter.last_h_ddot
            self.cbf_qp_lhs_a = active_filter.last_lhs_a_coeff
            self.cbf_qp_lhs_delta = active_filter.last_lhs_delta_coeff
            self.cbf_qp_rhs = active_filter.last_rhs
            self.cbf_qp_solve_time_ms = active_filter.last_solve_time_ms
            self.cbf_qp_slack = active_filter.last_slack
            self.cbf_qp_brake_gate_active = getattr(active_filter, 'last_brake_gate_active', False)
            self.cbf_qp_obstacle_x = active_filter.last_obstacle_x
            self.cbf_qp_obstacle_y = active_filter.last_obstacle_y
        except RuntimeError as exc:
            self.cbf_qp_status = 'unavailable'
            raise RuntimeError(
                f'controller-mode {self.config.controller_mode} requires cvxpy and a '
                f'working QP solver ({self.config.qp_solver})'
            ) from exc

        safe_throttle = self.qp_accel_to_throttle_pwm(accel)
        safe_roll = self.qp_delta_to_roll_pwm(delta)
        self.nominal_throttle = throttle_pwm
        self.nominal_roll = roll_pwm
        self.cbf_qp_accel = accel
        self.cbf_qp_delta = delta
        front_scale = self.front_obstacle_velocity_scale(center)
        safe_throttle = self.apply_throttle_scale(safe_throttle, front_scale)
        throttle_changed = abs(safe_throttle - throttle_pwm) > 1.0
        steering_changed = abs(safe_roll - roll_pwm) > 1.0
        self.cbf_active = throttle_changed or steering_changed or front_scale < 0.999
        self.velocity_controller.apply_throttle_anti_windup(
            throttle_pwm,
            safe_throttle,
            self.speed_error_pps,
        )
        if steering_changed:
            self.pid.decay_integral(0.7)
        if throttle_pwm > self.config.neutral_throttle_pwm:
            self.cbf_scale = bounded(
                (safe_throttle - self.config.neutral_throttle_pwm)
                / max(1.0, throttle_pwm - self.config.neutral_throttle_pwm),
                0.0,
                1.0,
            )
        else:
            self.cbf_scale = 1.0
        return safe_throttle, safe_roll

    def lidar_point_obstacle_velocity(self, point):
        """Return the source obstacle velocity for a lidar surface point."""
        best_obstacle = None
        best_distance = float('inf')
        point_xy = np.array([float(point.x_px), float(point.y_px)], dtype=np.float64)
        for obstacle in self.static_obstacles:
            polygon = obstacle.get('polygon')
            if polygon is None:
                continue
            distance = self.point_polygon_distance_px(point_xy, polygon)
            if distance < best_distance:
                best_distance = distance
                best_obstacle = obstacle

        if best_obstacle is None or best_distance > 2.0:
            return 0.0, 0.0
        return (
            float(best_obstacle.get('vx', 0.0)),
            float(best_obstacle.get('vy', 0.0)),
        )

    @staticmethod
    def point_polygon_distance_px(point_xy, polygon):
        """Return the shortest distance between a point and polygon boundary."""
        points = np.asarray(polygon, dtype=np.float64)
        if len(points) == 0:
            return float('inf')
        best = float('inf')
        for index, start in enumerate(points):
            end = points[(index + 1) % len(points)]
            segment = end - start
            length_sq = float(np.dot(segment, segment))
            if length_sq <= 1e-9:
                closest = start
            else:
                ratio = float(np.dot(point_xy - start, segment) / length_sq)
                ratio = bounded(ratio, 0.0, 1.0)
                closest = start + ratio * segment
            best = min(best, float(np.linalg.norm(point_xy - closest)))
        return best

    def front_obstacle_velocity_scale(self, center, lidar_points=None):
        """Return a forward-obstacle throttle scale for lidar-detected obstacles."""
        if lidar_points is None:
            lidar_points = self.latest_lidar_points
        if not lidar_points:
            return 1.0
        a_ell_px, b_ell_px = self.cbf_ellipse_axes_px()
        forward = np.array(
            [math.cos(self.last_marker_heading), math.sin(self.last_marker_heading)],
            dtype=np.float32,
        )
        right = np.array(
            [-math.sin(self.last_marker_heading), math.cos(self.last_marker_heading)],
            dtype=np.float32,
        )
        scale = 1.0
        for point in lidar_points:
            obstacle_point = np.array([point.x_px, point.y_px], dtype=np.float32)
            delta = obstacle_point - center
            ahead = float(np.dot(delta, forward))
            lateral = abs(float(np.dot(delta, right)))
            lateral_limit = b_ell_px
            if ahead <= 0.0 or lateral > lateral_limit:
                continue
            slow_distance = max(1.0, a_ell_px)
            scale = min(scale, bounded(ahead / slow_distance, 0.0, 1.0))
        return scale

    def apply_throttle_scale(self, throttle_pwm, scale):
        """Scale forward throttle around neutral without commanding reverse."""
        if throttle_pwm <= self.config.neutral_throttle_pwm:
            return throttle_pwm
        return self.config.neutral_throttle_pwm + (
            throttle_pwm - self.config.neutral_throttle_pwm
        ) * bounded(scale, 0.0, 1.0)

    def throttle_pwm_to_qp_accel(self, throttle_pwm):
        """Map drive PWM into the QP acceleration interval."""
        if throttle_pwm <= self.config.neutral_throttle_pwm:
            return self.config.qp_min_accel
        if throttle_pwm >= self.config.min_forward_pwm:
            span = max(1.0, self.config.max_forward_pwm - self.config.min_forward_pwm)
            ratio = bounded(
                (throttle_pwm - self.config.min_forward_pwm) / span,
                0.0,
                1.0,
            )
            return ratio * self.config.qp_max_accel
        else:
            span = max(1.0, self.config.min_forward_pwm - self.config.neutral_throttle_pwm)
            ratio = bounded(
                (throttle_pwm - self.config.neutral_throttle_pwm) / span,
                0.0,
                1.0,
            )
            return self.config.qp_min_accel + ratio * (0.0 - self.config.qp_min_accel)

    def qp_accel_to_throttle_pwm(self, accel):
        """Map QP acceleration back to bounded forward PWM."""
        if accel > 0.0:
            ratio = bounded(accel / max(1e-6, self.config.qp_max_accel), 0.0, 1.0)
            return bounded(
                self.config.min_forward_pwm
                + ratio * (self.config.max_forward_pwm - self.config.min_forward_pwm),
                self.config.min_forward_pwm,
                self.config.max_forward_pwm,
            )
        min_accel = min(-1e-6, self.config.qp_min_accel)
        ratio = bounded(1.0 - (accel / min_accel), 0.0, 1.0)
        return bounded(
            self.config.neutral_throttle_pwm
            + ratio * (self.config.min_forward_pwm - self.config.neutral_throttle_pwm),
            self.config.neutral_throttle_pwm,
            self.config.min_forward_pwm,
        )

    def roll_pwm_to_qp_delta(self, roll_pwm):
        """Map steering PWM into QP steering angle limits."""
        center = self.config.center_steering_pwm
        if roll_pwm <= center:
            span = max(1.0, center - self.config.right_pwm)
            ratio = bounded((center - roll_pwm) / span, 0.0, 1.0)
            return ratio * self.config.qp_max_delta
        span = max(1.0, self.config.left_pwm - center)
        ratio = bounded((roll_pwm - center) / span, 0.0, 1.0)
        return ratio * self.config.qp_min_delta

    def qp_delta_to_roll_pwm(self, delta):
        """Map QP steering angle back to steering PWM."""
        center = self.config.center_steering_pwm
        if delta >= 0.0:
            ratio = bounded(delta / max(1e-6, self.config.qp_max_delta), 0.0, 1.0)
            return bounded(
                center - ratio * (center - self.config.right_pwm),
                self.config.right_pwm,
                self.config.left_pwm,
            )
        ratio = bounded(delta / min(-1e-6, self.config.qp_min_delta), 0.0, 1.0)
        return bounded(
            center + ratio * (self.config.left_pwm - center),
            self.config.right_pwm,
            self.config.left_pwm,
        )

    def update_lidar_feedback(self, center):
        """Refresh virtual lidar points for preview and safety reporting."""
        if not self.road_scene_enabled():
            self.latest_lidar_points = []
            self.nearest_static_clearance_px = float('inf')
            return
        self.virtual_lidar.scan(
            center,
            self.lidar_heading_rad(),
            self.static_obstacles,
        )
        self.latest_lidar_points = self.virtual_lidar.latest_points
        self.nearest_static_clearance_px = self.virtual_lidar.nearest_clearance_px(
            self.config.cbf_h_px
        )

    def lidar_heading_rad(self):
        """Return marker heading plus the configured virtual lidar frame offset."""
        heading = getattr(self, 'last_marker_heading', 0.0)
        if heading is None:
            heading = 0.0
        offset = float(getattr(self.config, 'lidar_heading_offset_rad', 0.0))
        return wrap_angle(float(heading) + offset)

    def update_track_speed(self, nearest_index, now):
        """Estimate progress speed in track-points per second."""
        if self.last_progress_index is None or self.last_progress_time is None:
            self.last_progress_index = nearest_index
            self.last_progress_time = now
            self.last_vehicle_center = None
            self.last_vehicle_time = None
            self.raw_track_speed_pps = 0.0
            self.track_speed_pps = 0.0
            return
        dt = now - self.last_progress_time
        if dt <= 0.0:
            return

        center = self.latest_vehicle_center
        spacing = 1.0
        if self.track_points is not None and len(self.track_points) > 1:
            spacing = float(np.linalg.norm(self.track_points[1] - self.track_points[0]))
        spacing = max(1e-6, spacing)

        if (
            self.last_vehicle_center is None
            or self.last_vehicle_time is None
            or center is None
            or (now - self.last_vehicle_time) > 0.2
        ):
            # Fallback to index-based delta on first frame or after tracking loss / time gap
            if self.closed_road_scene_enabled() or not self.road_scene_enabled():
                point_count = len(self.track_points)
                forward_delta = (
                    nearest_index - self.last_progress_index
                ) % point_count
                reverse_delta = forward_delta - point_count
                if abs(forward_delta) < abs(reverse_delta):
                    delta = float(forward_delta)
                else:
                    delta = float(reverse_delta)
            else:
                delta = float(nearest_index - self.last_progress_index)
        else:
            displacement = center - self.last_vehicle_center
            tangent = self.road_tangents[nearest_index]
            delta_px = float(np.dot(displacement, tangent))
            delta = delta_px / spacing

        # Update tracking state for next frame
        if center is not None:
            self.last_vehicle_center = center.copy()
            self.last_vehicle_time = now
        else:
            self.last_vehicle_center = None
            self.last_vehicle_time = None

        self.raw_track_speed_pps = delta / dt
        alpha = bounded(
            float(getattr(self.config, 'track_speed_filter_alpha', 1.0)),
            0.0,
            1.0,
        )
        filtered_speed = (
            alpha * self.raw_track_speed_pps
            + (1.0 - alpha) * self.track_speed_pps
        )
        slew_rate = float(getattr(self.config, 'track_speed_slew_rate_pps2', 0.0))
        if slew_rate > 0.0:
            max_step = slew_rate * dt
            filtered_speed = bounded(
                filtered_speed,
                self.track_speed_pps - max_step,
                self.track_speed_pps + max_step,
            )
        self.track_speed_pps = filtered_speed
        self.last_progress_index = nearest_index
        self.last_progress_time = now

    def cbf_safety_scale(self, center, lateral_error_px, heading_error_rad):
        """Scale throttle down as image-space safety limits are approached."""
        self.update_lidar_feedback(center)
        return 1.0

    def target_lap_reached(self):
        """Return whether the configured lap limit has been completed."""
        return (
            self.lap_limit_enabled
            and self.target_laps > 0
            and self.metrics.laps_completed >= self.target_laps
        )

    def safety_clearance_px(self, center, lateral_error_px, heading_error_rad):
        """Return image-space safety clearance."""
        width, height = self.last_image_size
        edge_distance = min(
            center[0],
            center[1],
            width - center[0],
            height - center[1],
        )
        edge_clearance = edge_distance - self.config.cbf_edge_margin_px
        error_clearance = self.cbf_stop_error_px - abs(lateral_error_px)
        heading_px = (
            self.config.cbf_stop_heading_rad - abs(heading_error_rad)
        ) * self.cbf_stop_error_px / self.config.cbf_stop_heading_rad
        static_clearance = float('inf')
        if self.road_scene_enabled():
            self.update_lidar_feedback(center)
            static_clearance = self.nearest_static_clearance_px
        safety_clearance = min(
            edge_clearance,
            error_clearance,
            heading_px,
            static_clearance,
        )
        self.latest_safety_clearances = {
            'edge': edge_clearance,
            'lateral': error_clearance,
            'heading': heading_px,
            'obstacle': static_clearance,
            'min': safety_clearance,
        }
        return safety_clearance

    def update_marker_trail(self, center):
        """Append a marker center to the bounded debug trail."""
        if self.config.debug_trail_length <= 0:
            self.marker_trail = []
            return
        self.marker_trail.append(np.array(center, dtype=np.float32))
        excess = len(self.marker_trail) - self.config.debug_trail_length
        if excess > 0:
            del self.marker_trail[:excess]

    def static_obstacle_clearance_px(self, point):
        """Return clearance from a point to the nearest virtual road obstacle."""
        if not self.static_obstacles:
            return float('inf')
        clearances = [
            obstacle_clearance_px(point, obstacle)
            for obstacle in self.static_obstacles
        ]
        return min(clearances) - self.config.obstacle_margin_px

    def apply_tuning_values(self, values):
        """Apply tuning values to current config and controller gains."""
        self.config.steering_kp_px = values.get(
            'steering_kp_px',
            self.config.steering_kp_px,
        )
        self.config.steering_ki_px = values.get(
            'steering_ki_px',
            self.config.steering_ki_px,
        )
        self.config.steering_kd_px = values.get(
            'steering_kd_px',
            self.config.steering_kd_px,
        )
        self.heading_kp = values.get('heading_kp', self.heading_kp)
        self.config.heading_kp = self.heading_kp
        self.config.aruco_offset_x_px = values.get(
            'aruco_offset_x_px',
            getattr(self.config, 'aruco_offset_x_px', 0.0),
        )
        self.config.aruco_offset_y_px = values.get(
            'aruco_offset_y_px',
            getattr(self.config, 'aruco_offset_y_px', 0.0),
        )
        self.config.forward_pwm = values.get('forward_pwm', self.config.forward_pwm)
        if 'lookahead_points' in values:
            self.config.lookahead_points = max(1, int(values['lookahead_points']))
        self.target_track_speed_pps = values.get(
            'target_track_speed_pps',
            self.target_track_speed_pps,
        )
        self.config.track_speed_filter_alpha = values.get(
            'track_speed_filter_alpha',
            self.config.track_speed_filter_alpha,
        )
        self.config.track_speed_slew_rate_pps2 = values.get(
            'track_speed_slew_rate_pps2',
            self.config.track_speed_slew_rate_pps2,
        )
        self.config.road_half_width_px = values.get(
            'road_half_width_px',
            self.config.road_half_width_px,
        )
        self.config.obstacle_margin_px = values.get(
            'obstacle_margin_px',
            self.config.obstacle_margin_px,
        )
        self.config.lidar_heading_offset_rad = values.get(
            'lidar_heading_offset_rad',
            getattr(self.config, 'lidar_heading_offset_rad', 0.0),
        )
        for key in (
            'preview_max_fps',
            'telemetry_max_fps',
            'debug_print_interval_s',
        ):
            if key in values and hasattr(self.config, key):
                setattr(self.config, key, values[key])
        for key in (
            'ftg_stuck_speed_pps',
            'ftg_stuck_forward_pwm',
            'ftg_stuck_steering_gain_pwm',
            'ftg_stuck_max_steering_bias_pwm',
            'ftg_fov_deg',
            'ftg_max_range_px',
            'ftg_bubble_radius_px',
        ):
            if key in values and hasattr(self.config, key):
                setattr(self.config, key, values[key])
        self.virtual_lidar.front_view_rad = math.radians(
            max(0.0, min(360.0, float(getattr(self.config, 'ftg_fov_deg', 240.0))))
        )
        self.virtual_lidar.max_range_px = float(
            getattr(self.config, 'ftg_max_range_px', self.virtual_lidar.max_range_px)
        )
        self.config.velocity_kp_pwm = values.get(
            'velocity_kp_pwm',
            self.config.velocity_kp_pwm,
        )
        self.config.velocity_ki_pwm = values.get(
            'velocity_ki_pwm',
            self.config.velocity_ki_pwm,
        )
        self.config.velocity_kd_pwm = values.get(
            'velocity_kd_pwm',
            self.config.velocity_kd_pwm,
        )
        self.config.cbf_a_ell = values.get('cbf_a_ell', self.config.cbf_a_ell)
        self.config.cbf_b_ell = values.get('cbf_b_ell', self.config.cbf_b_ell)
        self.config.cbf_gamma1 = values.get('cbf_gamma1', self.config.cbf_gamma1)
        self.config.cbf_gamma2 = values.get('cbf_gamma2', self.config.cbf_gamma2)
        self.config.cbf_gamma3 = values.get('cbf_gamma3', self.config.cbf_gamma3)
        self.config.qp_wheelbase_px = values.get(
            'qp_wheelbase_px',
            self.config.qp_wheelbase_px,
        )
        self.config.qp_min_accel = values.get(
            'qp_min_accel',
            self.config.qp_min_accel,
        )
        self.config.qp_max_accel = values.get(
            'qp_max_accel',
            self.config.qp_max_accel,
        )
        self.config.qp_min_delta = values.get(
            'qp_min_delta',
            self.config.qp_min_delta,
        )
        self.config.qp_max_delta = values.get(
            'qp_max_delta',
            self.config.qp_max_delta,
        )
        self.config.qp_solver = values.get('qp_solver', self.config.qp_solver)
        self.config.qp_slack_weight = values.get(
            'qp_slack_weight',
            self.config.qp_slack_weight,
        )
        if 'clf_alpha' in values:
            self.config.clf_alpha = float(values['clf_alpha'])
        if 'clf_slack_weight' in values:
            self.config.clf_slack_weight = float(values['clf_slack_weight'])
        if 'qp_max_obstacles' in values and hasattr(self.config, 'qp_max_obstacles'):
            self.config.qp_max_obstacles = max(1, int(values['qp_max_obstacles']))
        self.refresh_cbf_qp_config()
        self.lap_limit_enabled = values.get(
            'lap_limit_enabled',
            self.lap_limit_enabled,
        )
        self.target_laps = max(0, int(values.get('target_laps', self.target_laps)))
        self.pid.update_gains(
            self.config.steering_kp_px,
            self.config.steering_ki_px,
            self.config.steering_kd_px,
            self.config.integral_limit_px_s,
        )
        self.velocity_controller.update_velocity_gains(
            self.config.velocity_kp_pwm,
            self.config.velocity_ki_pwm,
            self.config.velocity_kd_pwm,
            self.config.velocity_integral_limit,
        )
