"""Reusable controller core for the closed-ellipse ArUco follower."""

from dataclasses import dataclass
import math
import time

import numpy as np

from ..common import bounded
from ..common import wrap_angle
from ..controllers import CBFQPConfig
from ..controllers import CBFQPSafetyFilter
from ..controllers import PID_CBF
from ..controllers import PID_VELOCITY
from ..controllers import PID_VELOCITY_CBF
from ..controllers import PID_VELOCITY_CBF_QP
from ..controllers import PIDController
from ..controllers import PIDVelocityCBFController
from ..controllers import PointObstacle
from ..controllers import VehicleState
from ..controllers import VirtualLidar
from ..metrics import FollowerMetrics
from ..metrics import unique_metrics_path
from ..road import free_lateral_intervals
from ..road import make_ellipse_road_scene
from ..road import obstacle_clearance_px


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
        self.static_obstacles = []
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
        self.throttle = config.neutral_throttle_pwm
        self.roll = config.center_steering_pwm
        self.last_marker_seen = False
        self.stop_requested = False
        self.completion_reason = 'running'
        self.pid = PIDController(
            config.steering_kp_px,
            config.steering_ki_px,
            config.steering_kd_px,
            config.integral_limit_px_s,
        )
        self.velocity_pid = PIDController(
            config.velocity_kp_pwm,
            config.velocity_ki_pwm,
            config.velocity_kd_pwm,
            config.velocity_integral_limit,
        )
        self.virtual_lidar = VirtualLidar()
        self.pid_velocity_cbf = PIDVelocityCBFController(
            self.velocity_pid,
            self.virtual_lidar,
        )
        self.cbf_qp_filter = CBFQPSafetyFilter(
            CBFQPConfig(
                r_safe=config.cbf_r_safe,
                wheelbase=config.qp_wheelbase_px,
                gamma1=config.cbf_gamma1,
                gamma2=config.cbf_gamma2,
                gamma3=config.cbf_gamma3,
                min_accel=config.qp_min_accel,
                max_accel=config.qp_max_accel,
                min_delta=config.qp_min_delta,
                max_delta=config.qp_max_delta,
                solver=config.qp_solver,
            )
        )
        self.heading_kp = config.heading_kp
        self.target_track_speed_pps = config.target_track_speed_pps
        self.cbf_stop_error_px = config.cbf_stop_error_px
        self.last_progress_index = None
        self.last_progress_time = None
        self.track_speed_pps = 0.0
        self.speed_error_pps = 0.0
        self.velocity_delta_pwm = 0.0
        self.cbf_scale = 1.0
        self.cbf_active = False
        self.cbf_qp_accel = 0.0
        self.cbf_qp_delta = 0.0
        self.cbf_qp_status = 'unused'
        self.latest_lidar_points = []
        self.nearest_static_clearance_px = float('inf')
        self.metrics = FollowerMetrics()
        self.metrics_saved = False
        self.metrics_file_path = unique_metrics_path(config.metrics_file)
        self.lap_limit_enabled = config.enable_lap_limit
        self.target_laps = max(0, config.target_laps)

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
        target, tangent, error, nearest_index = self.track_error(center, heading)
        heading_error = wrap_angle(math.atan2(tangent[1], tangent[0]) - heading)
        self.latest_lateral_error_px = error
        self.latest_heading_error_rad = heading_error
        self.latest_target_center = target
        self.latest_target_tangent = tangent
        self.latest_nearest_index = nearest_index
        self.update_marker_trail(center)
        self.roll = self.steering_to_pwm(error, heading_error, now)
        self.throttle = self.throttle_to_pwm(
            center,
            error,
            heading_error,
            nearest_index,
            now,
        )
        speed_error = self.target_track_speed_pps - self.track_speed_pps
        self.speed_error_pps = speed_error
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
        self.last_command_time = None
        self.last_detection_time = None
        self.last_marker_seen = False
        self.pid.reset()
        self.pid_velocity_cbf.reset()
        self.last_progress_index = None
        self.last_progress_time = None
        self.track_speed_pps = 0.0
        self.speed_error_pps = 0.0
        self.velocity_delta_pwm = 0.0
        self.cbf_scale = 0.0
        self.cbf_active = False
        self.cbf_qp_accel = 0.0
        self.cbf_qp_delta = 0.0
        self.cbf_qp_status = 'reset'
        self.latest_lidar_points = []

    def request_stop_state(self, reason):
        """Record a requested stop and reset command-producing state."""
        if self.stop_requested:
            return
        self.stop_requested = True
        self.completion_reason = reason
        self.throttle = self.config.neutral_throttle_pwm
        self.roll = self.config.center_steering_pwm
        self.last_command_time = None
        self.pid.reset()
        self.pid_velocity_cbf.reset()
        self.latest_lidar_points = []
        self.cbf_qp_accel = 0.0
        self.cbf_qp_delta = 0.0
        self.cbf_qp_status = 'reset'

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
                'forward_pwm': self.config.forward_pwm,
                'target_track_speed_pps': self.target_track_speed_pps,
                'velocity_kp_pwm': self.config.velocity_kp_pwm,
                'cbf_r_safe': self.config.cbf_r_safe,
                'cbf_gamma1': self.config.cbf_gamma1,
                'cbf_gamma2': self.config.cbf_gamma2,
                'cbf_gamma3': self.config.cbf_gamma3,
                'lap_limit_enabled': self.lap_limit_enabled,
                'target_laps': self.target_laps,
            }
        values.update(
            {
                'controller_mode': self.config.controller_mode,
                'track_shape': self.config.track_shape,
                'min_forward_pwm': self.config.min_forward_pwm,
                'max_forward_pwm': self.config.max_forward_pwm,
                'velocity_ki_pwm': self.config.velocity_ki_pwm,
                'velocity_kd_pwm': self.config.velocity_kd_pwm,
                'cbf_r_safe': self.config.cbf_r_safe,
                'cbf_gamma1': self.config.cbf_gamma1,
                'cbf_gamma2': self.config.cbf_gamma2,
                'cbf_gamma3': self.config.cbf_gamma3,
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
        self.cbf_qp_filter.config = CBFQPConfig(
            r_safe=self.config.cbf_r_safe,
            wheelbase=self.config.qp_wheelbase_px,
            gamma1=self.config.cbf_gamma1,
            gamma2=self.config.cbf_gamma2,
            gamma3=self.config.cbf_gamma3,
            min_accel=self.config.qp_min_accel,
            max_accel=self.config.qp_max_accel,
            min_delta=self.config.qp_min_delta,
            max_delta=self.config.qp_max_delta,
            solver=self.config.qp_solver,
        )

    def build_track_scene(self, width, height):
        """Create the laneless elliptical road scene for the current image size."""
        self.last_image_size = (width, height)
        scene = make_ellipse_road_scene(width, height, self.config)
        self.road_boundaries = scene['boundaries']
        self.road_tangents = scene['tangents']
        self.road_normals = scene['normals']
        self.road_half_width_px = scene['road_half_width_px']
        self.static_obstacles = scene['obstacles']
        self.latest_free_space_target = None
        self.latest_free_space_interval = None
        self.track_points = scene['centerline']

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
        """Return the center of the widest obstacle-free road interval."""
        if (
            self.road_normals is None
            or self.road_tangents is None
            or self.road_half_width_px <= 0.0
        ):
            return self.track_points[path_index]

        center = self.track_points[path_index]
        tangent = self.road_tangents[path_index]
        normal = self.road_normals[path_index]
        road_margin = max(4.0, self.config.obstacle_margin_px * 0.35)
        lower = -self.road_half_width_px + road_margin
        upper = self.road_half_width_px - road_margin
        blocked = []
        lookahead_length = max(
            self.config.road_half_width_px,
            self.config.lookahead_points * 4.0,
        )

        for obstacle in self.static_obstacles:
            delta = obstacle['center'] - center
            along = float(np.dot(delta, tangent))
            if abs(along) > lookahead_length + obstacle['half_length']:
                continue
            lateral = float(np.dot(delta, normal))
            half_width = obstacle['half_width'] + self.config.obstacle_margin_px
            blocked_lower = max(lower, lateral - half_width)
            blocked_upper = min(upper, lateral + half_width)
            if blocked_lower < blocked_upper:
                blocked.append((blocked_lower, blocked_upper))

        free_intervals = free_lateral_intervals(lower, upper, blocked)
        if free_intervals:
            best_lower, best_upper = max(
                free_intervals,
                key=lambda interval: interval[1] - interval[0],
            )
            lateral_target = 0.5 * (best_lower + best_upper)
            self.latest_free_space_interval = (best_lower, best_upper)
        else:
            lateral_target = 0.0
            self.latest_free_space_interval = None

        self.latest_free_space_target = center + normal * lateral_target
        return self.latest_free_space_target

    def steering_to_pwm(self, lateral_error_px, heading_error_rad, now):
        """Convert image-space path error into bounded steering PWM."""
        steering_delta = self.pid.step(lateral_error_px, now)
        steering_delta += self.heading_kp * heading_error_rad
        roll = self.config.center_steering_pwm - steering_delta
        return round(
            bounded(
                roll,
                self.config.right_pwm,
                self.config.left_pwm,
            )
        )

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
        throttle = self.config.forward_pwm
        self.speed_error_pps = self.target_track_speed_pps - self.track_speed_pps
        self.velocity_delta_pwm = 0.0

        if self.config.controller_mode in (
            PID_VELOCITY,
            PID_VELOCITY_CBF,
            PID_VELOCITY_CBF_QP,
        ):
            throttle, speed_delta, speed_error = (
                self.pid_velocity_cbf.velocity_throttle(
                    self.target_track_speed_pps,
                    self.track_speed_pps,
                    self.config.forward_pwm,
                    self.config.min_forward_pwm,
                    self.config.max_forward_pwm,
                    now,
                )
            )
            self.speed_error_pps = speed_error
            self.velocity_delta_pwm = speed_delta

        if self.config.controller_mode == PID_VELOCITY:
            self.cbf_scale = 1.0
            self.cbf_active = False
            self.cbf_qp_status = 'unused'
            return round(throttle)

        if self.config.controller_mode in (PID_CBF, PID_VELOCITY_CBF):
            result = self.pid_velocity_cbf.apply_cbf(
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
            self.nearest_static_clearance_px = result.lidar_clearance_px
            self.latest_safety_clearances = {
                'edge': result.edge_clearance_px,
                'lateral': result.error_clearance_px,
                'heading': result.heading_clearance_px,
                'obstacle': result.lidar_clearance_px,
                'min': result.safety_clearance_px,
            }
            self.latest_lidar_points = self.virtual_lidar.latest_points
            self.cbf_qp_status = 'unused'
            return round(result.throttle_pwm)

        if self.config.controller_mode != PID_VELOCITY_CBF_QP:
            self.cbf_scale = 1.0
            self.cbf_active = False
            self.cbf_qp_status = 'unused'
            return self.config.forward_pwm

        throttle, roll = self.apply_cbf_qp(center, throttle, self.roll)
        self.roll = round(roll)
        self.cbf_qp_status = self.cbf_qp_filter.last_status or 'unknown'

        return round(throttle)

    def apply_cbf_qp(self, center, throttle_pwm, roll_pwm):
        """Filter nominal throttle and steering through cbf_qp.py."""
        car = VehicleState(
            x=float(center[0]),
            y=float(center[1]),
            v=max(0.0, float(self.track_speed_pps)),
            psi=float(self.last_marker_heading),
        )
        obstacles = [
            PointObstacle(
                x=float(obstacle['center'][0]),
                y=float(obstacle['center'][1]),
            )
            for obstacle in self.static_obstacles
        ]
        accel_ref = self.throttle_pwm_to_qp_accel(throttle_pwm)
        delta_ref = self.roll_pwm_to_qp_delta(roll_pwm)
        try:
            accel, delta = self.cbf_qp_filter.solve(
                car,
                obstacles,
                accel_ref,
                delta_ref,
            )
        except RuntimeError as exc:
            self.cbf_qp_status = 'unavailable'
            raise RuntimeError(
                'controller-mode pid_velocity_cbf_qp requires cvxpy and a '
                f'working QP solver ({self.config.qp_solver})'
            ) from exc

        safe_throttle = self.qp_accel_to_throttle_pwm(accel)
        safe_roll = self.qp_delta_to_roll_pwm(delta)
        self.cbf_qp_accel = accel
        self.cbf_qp_delta = delta
        self.cbf_qp_status = self.cbf_qp_filter.last_status or 'unknown'
        throttle_changed = abs(safe_throttle - throttle_pwm) > 1.0
        steering_changed = abs(safe_roll - roll_pwm) > 1.0
        self.cbf_active = throttle_changed or steering_changed
        if throttle_pwm > self.config.neutral_throttle_pwm:
            self.cbf_scale = bounded(
                (safe_throttle - self.config.neutral_throttle_pwm)
                / max(1.0, throttle_pwm - self.config.neutral_throttle_pwm),
                0.0,
                1.0,
            )
        else:
            self.cbf_scale = 1.0
        self.nearest_static_clearance_px = self.qp_obstacle_clearance_px(center)
        return safe_throttle, safe_roll

    def throttle_pwm_to_qp_accel(self, throttle_pwm):
        """Map drive PWM into the QP acceleration interval."""
        if throttle_pwm <= self.config.neutral_throttle_pwm:
            return self.config.qp_min_accel
        span = max(
            1.0,
            self.config.max_forward_pwm - self.config.neutral_throttle_pwm,
        )
        ratio = bounded(
            (throttle_pwm - self.config.neutral_throttle_pwm) / span,
            0.0,
            1.0,
        )
        return ratio * self.config.qp_max_accel

    def qp_accel_to_throttle_pwm(self, accel):
        """Map QP acceleration back to bounded forward PWM."""
        if accel <= 0.0:
            return float(self.config.neutral_throttle_pwm)
        ratio = bounded(accel / max(1e-6, self.config.qp_max_accel), 0.0, 1.0)
        return bounded(
            self.config.neutral_throttle_pwm
            + ratio
            * (self.config.max_forward_pwm - self.config.neutral_throttle_pwm),
            self.config.neutral_throttle_pwm,
            self.config.max_forward_pwm,
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

    def qp_obstacle_clearance_px(self, center):
        """Return nearest QP circular-obstacle clearance."""
        if not self.static_obstacles:
            return float('inf')
        clearances = [
            float(np.linalg.norm(obstacle['center'] - center))
            - self.config.cbf_r_safe
            for obstacle in self.static_obstacles
        ]
        return min(clearances)

    def update_lidar_feedback(self, center):
        """Refresh virtual lidar points for preview and safety reporting."""
        if not self.road_scene_enabled():
            self.latest_lidar_points = []
            self.nearest_static_clearance_px = float('inf')
            return
        self.virtual_lidar.scan(
            center,
            self.last_marker_heading,
            self.static_obstacles,
        )
        self.latest_lidar_points = self.virtual_lidar.latest_points
        self.nearest_static_clearance_px = self.virtual_lidar.nearest_clearance_px(
            self.config.cbf_h_px
        )

    def update_track_speed(self, nearest_index, now):
        """Estimate progress speed in track-points per second."""
        if self.last_progress_index is None or self.last_progress_time is None:
            self.last_progress_index = nearest_index
            self.last_progress_time = now
            self.track_speed_pps = 0.0
            return
        dt = now - self.last_progress_time
        if dt <= 0.0:
            return
        if self.closed_road_scene_enabled() or not self.road_scene_enabled():
            point_count = len(self.track_points)
            forward_delta = (
                nearest_index - self.last_progress_index
            ) % point_count
            reverse_delta = forward_delta - point_count
            if abs(forward_delta) < abs(reverse_delta):
                delta = forward_delta
            else:
                delta = reverse_delta
        else:
            delta = nearest_index - self.last_progress_index
        self.track_speed_pps = delta / dt
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
        self.config.steering_kp_px = values['steering_kp_px']
        self.config.steering_ki_px = values['steering_ki_px']
        self.config.steering_kd_px = values['steering_kd_px']
        self.heading_kp = values['heading_kp']
        self.config.forward_pwm = values['forward_pwm']
        self.target_track_speed_pps = values['target_track_speed_pps']
        self.config.velocity_kp_pwm = values['velocity_kp_pwm']
        self.config.cbf_r_safe = values['cbf_r_safe']
        self.config.cbf_gamma1 = values['cbf_gamma1']
        self.config.cbf_gamma2 = values['cbf_gamma2']
        self.config.cbf_gamma3 = values['cbf_gamma3']
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
        self.pid_velocity_cbf.update_velocity_gains(
            self.config.velocity_kp_pwm,
            self.config.velocity_ki_pwm,
            self.config.velocity_kd_pwm,
            self.config.velocity_integral_limit,
        )
