"""ROS node for the straight-road ArUco hil follower."""

import json
import math
from pathlib import Path
import threading
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from .config import ARMING_SERVICE
from .config import NEUTRAL_VALUE
from .config import parse_args
from .config import print_config
from .config import SETTLE_DURATION
from .config import UPDATE_RATE_HZ
from .config import validate_config
from hardware.hil.control import StraightStaticController
from .road import make_laneless_static_obstacle
from .runtime import make_sensor_qos
from .tuning import StraightStaticTuning
from .ui import draw_label
from .ui import draw_vector
from .ui import noop
from .ui import put_status
from .ui import resize_for_preview
from .ui import set_overlay_text_style


PREVIEW_WINDOW = 'Aruco Track Follower'


class ArucoTrackFollower(Node):
    """ROS transport wrapper around the reusable straight follower controller."""

    def __init__(self, config):
        super().__init__('aruco_track_follower')
        # ponytail: Reset/clear current active log file on launch
        import os
        tuning_file = str(getattr(config, 'tuning_file', 'tuning.json') or 'tuning.json')
        is_stable = 'stable' in tuning_file.lower()
        log_name = 'telemetry_stable.csv' if is_stable else 'telemetry_unstable.csv'
        log_path = os.path.join('/home/dhruv/amcaf/simulation', log_name)
        try:
            if os.path.exists(log_path):
                os.remove(log_path)
        except Exception:
            pass
        cv2.setUseOptimized(True)
        self.config = config
        if config.virtual_vehicle_test:
            zoom_out_factor = 0.55
            config.road_half_width_px = config.road_half_width_px * zoom_out_factor
            config.obstacle_margin_px = config.obstacle_margin_px * zoom_out_factor
            config.target_track_speed_pps = config.target_track_speed_pps * zoom_out_factor
            config.virtual_start_speed_pps = config.virtual_start_speed_pps * zoom_out_factor
            if getattr(config, 'ftg_bubble_radius_px', 0.0) > 0.0:
                config.ftg_bubble_radius_px = config.ftg_bubble_radius_px * zoom_out_factor
            config.ftg_max_range_px = getattr(config, 'ftg_max_range_px', 500.0) * zoom_out_factor
        self.bridge = CvBridge()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        self.command_callback_group = ReentrantCallbackGroup()
        self.command_pub = self.create_publisher(
            RCMessage,
            config.command_topic,
            10,
        )
        if not config.virtual_vehicle_test:
            self.image_sub = self.create_subscription(
                Image,
                config.image_topic,
                self.image_callback,
                make_sensor_qos(),
                callback_group=self.control_callback_group,
            )
        else:
            self.image_sub = None
        self.arming_client = self.create_client(CommandBool, ARMING_SERVICE)
        self.aruco_dictionary = self.load_aruco_dictionary(config.marker_dict)
        self.aruco_parameters = self.make_detector_parameters()
        self.detector = self.make_detector()
        self.controller = self.make_controller(config)
        if config.virtual_vehicle_test and hasattr(self.controller, 'virtual_lidar'):
            self.controller.virtual_lidar.max_range_px = 500.0 * 0.55
        self.telemetry_publishers = self.create_tuning_telemetry_publishers()
        self.tuning = StraightStaticTuning(
            config,
            self.controller,
            self.get_logger(),
            enabled=not config.no_pid_panel,
        )
        self.latest_preview_frame = None
        self.pending_preview = None
        self.preview_lock = threading.Lock()
        self.preview_window_ready = False
        self.preview_window_size_initialized = False
        self.preview_text_controls_ready = False
        self.last_preview_schedule_time = 0.0
        self.last_telemetry_publish_time = 0.0
        self.output_enabled = False
        self.armed = False
        self.last_debug_print_time = 0.0
        self.virtual_vehicle_state = None
        self.virtual_last_time = None
        self.virtual_road_tile_index = 0
        self.random_static_obstacles = []
        self.last_pose_time = None
        self.last_lateral_error = None
        self.last_ftg_target_angle = None
        self.last_ftg_target_time = None
        self.detected_random_obstacles = []
        self.flash_timer = 0
        if config.load_tuning:
            self.tuning.load_tuning_file()
            self.controller.refresh_cbf_qp_config()
        if self.tuning.enabled:
            self.tuning.create_pid_panel()
        if config.virtual_vehicle_test:
            self.reset_virtual_vehicle()
            self.create_timer(
                1.0 / UPDATE_RATE_HZ,
                self.virtual_vehicle_tick,
                callback_group=self.control_callback_group,
            )
        self.create_timer(
            1.0 / UPDATE_RATE_HZ,
            self.publish_command,
            callback_group=self.command_callback_group,
        )

    def make_controller(self, config):
        """Create the ROS-free controller core for this node."""
        return StraightStaticController(config)

    def create_tuning_telemetry_publishers(self):
        """Create live numeric topics for PlotJuggler tuning."""
        if self.config.no_telemetry:
            return {}
        prefix = self.config.telemetry_prefix.rstrip('/')
        names = (
            'marker_seen',
            'marker_x_px',
            'marker_y_px',
            'lateral_error_px',
            'free_space_lateral_target_px',
            'heading_error_rad',
            'nominal_roll_pwm',
            'roll_pwm',
            'nominal_throttle_pwm',
            'throttle_pwm',
            'raw_track_speed_pps',
            'track_speed_pps',
            'target_speed_pps',
            'speed_error_pps',
            'safety_clearance_px',
            'cbf_scale',
            'cbf_active',
            'nearest_static_clearance_px',
            'cbf_qp_accel',
            'cbf_qp_delta',
            'cbf_qp_h',
            'cbf_qp_h_dot',
            'cbf_qp_h_ddot',
            'cbf_qp_lhs_a',
            'cbf_qp_lhs_delta',
            'cbf_qp_rhs',
            'cbf_qp_solve_time_ms',
            'cbf_qp_slack',
            'pose_position_error_px',
            'pose_heading_error_rad',
            'pose_update_frequency_hz',
            'pose_confidence_level',
            'control_smoothness',
            'actuator_steer_saturated',
            'actuator_throttle_saturated',
            'safety_violated',
            'safety_override_magnitude',
            'ftg_solve_time_ms',
            'ftg_no_gap_found',
            'ftg_target_angle_rate_rad_s',
        )
        return {
            name: self.create_publisher(Float64, f'{prefix}/{name}', 10)
            for name in names
        }

    def __getattr__(self, name):
        """Expose controller state to preview helpers and legacy callers."""
        controller = self.__dict__.get('controller')
        if controller is not None and hasattr(controller, name):
            return getattr(controller, name)
        raise AttributeError(name)

    def load_aruco_dictionary(self, dictionary_name):
        """Load an OpenCV ArUco dictionary by name."""
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError(
                'cv2.aruco is unavailable; install opencv-contrib-python or '
                'the ROS/OpenCV package that includes ArUco.'
            )
        if not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(f'unknown ArUco dictionary: {dictionary_name}')
        return cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name)
        )

    def make_detector(self):
        """Create an ArUco detector compatible with current OpenCV versions."""
        if hasattr(cv2.aruco, 'ArucoDetector'):
            return cv2.aruco.ArucoDetector(
                self.aruco_dictionary,
                self.aruco_parameters,
            )
        return None

    def make_detector_parameters(self):
        """Create ArUco detector parameters across OpenCV API variants."""
        if hasattr(cv2.aruco, 'DetectorParameters'):
            params = cv2.aruco.DetectorParameters()
        else:
            params = cv2.aruco.DetectorParameters_create()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 53
        params.adaptiveThreshWinSizeStep = 10
        params.adaptiveThreshConstant = 7
        if hasattr(cv2.aruco, 'CORNER_REFINE_SUBPIX'):
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        else:
            params.cornerRefinementMethod = 1
        params.cornerRefinementWinSize = 5
        params.cornerRefinementMaxIterations = 50
        params.cornerRefinementMinAccuracy = 0.03
        params.minMarkerPerimeterRate = 0.02
        params.minCornerDistanceRate = 0.03
        return params

    def image_callback(self, image_msg):
        """Process one camera frame and update the latest RC command."""
        if self.config.virtual_vehicle_test:
            return
        if self.message_is_stale(image_msg):
            return
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        detection = self.detect_marker(frame)
        if detection is None:
            self.stop_immediately()
            self.publish_tuning_telemetry(marker_seen=False)
            if self.config.preview:
                self.schedule_preview(frame, None, None, None, None)
            return

        center, heading, corners = detection
        self.update_marker_scale(corners)
        self.update_pid_from_panel()
        result = self.controller.process_detection(
            center,
            heading,
            image_size=(width, height),
            now=time.monotonic(),
        )
        self.publish_tuning_telemetry(result, marker_seen=True)
        if result.target_lap_reached:
            self.request_stop(
                f'target laps completed ({self.metrics.laps_completed}/'
                f'{self.target_laps})'
            )
            return

        if self.output_enabled and not self.config.dry_run:
            command = self.drive_message()
            self.command_pub.publish(command)

        if self.config.preview:
            self.schedule_preview(frame, corners, center, result.target, result.tangent)

    def publish_tuning_telemetry(self, result=None, marker_seen=False):
        """Publish one sample of numeric telemetry for PlotJuggler."""
        if not self.telemetry_publishers:
            return
        nan = float('nan')
        now = time.monotonic()
        max_fps = float(getattr(self.config, 'telemetry_max_fps', 10.0))
        if max_fps > 0.0:
            min_period = 1.0 / max_fps
            if now - self.last_telemetry_publish_time < min_period:
                return
            self.last_telemetry_publish_time = now
        
        # Calculate update rate for pose
        pose_update_freq = nan
        if self.last_pose_time is not None:
            dt_pose = now - self.last_pose_time
            pose_update_freq = 1.0 / max(1e-6, dt_pose)
        self.last_pose_time = now

        # Get actuator saturation flags
        roll_pwm_val = float(result.roll_pwm) if result is not None else float(self.roll)
        throttle_pwm_val = float(result.throttle_pwm) if result is not None else float(self.throttle)
        
        # steering limits are left 1850, right 1150
        steer_saturated = 1.0 if (roll_pwm_val >= 1845.0 or roll_pwm_val <= 1155.0) else 0.0
        # throttle max is max_forward_pwm (typically 1590), min is neutral (1500)
        throttle_saturated = 1.0 if (throttle_pwm_val >= (self.config.max_forward_pwm - 2.0) or throttle_pwm_val <= 1502.0) else 0.0

        # Safety override magnitude
        nom_throttle = float(self.nominal_throttle)
        safety_override = float(max(0.0, nom_throttle - throttle_pwm_val))

        # Safety violated (h < 0)
        violated = 1.0 if (float(self.cbf_qp_h) < 0.0) else 0.0

        # Pose Estimation Accuracy (if virtual vehicle test is running)
        pose_pos_err = nan
        pose_heading_err = nan
        if self.config.virtual_vehicle_test and self.virtual_vehicle_state is not None and result is not None:
            actual_pos = np.array([self.virtual_vehicle_state['x'], self.virtual_vehicle_state['y']], dtype=np.float32)
            estimated_pos = np.array(result.center, dtype=np.float32)
            pose_pos_err = float(np.linalg.norm(actual_pos - estimated_pos))
            
            # Heading error comparison
            actual_heading = float(self.virtual_vehicle_state['heading'])
            estimated_heading = float(getattr(self, 'last_marker_heading', 0.0) or 0.0)
            diff = (actual_heading - estimated_heading + math.pi) % (2.0 * math.pi) - math.pi
            pose_heading_err = float(abs(diff))

        # Control smoothness (rate of change of CTE)
        smoothness = 0.0
        if result is not None:
            if self.last_lateral_error is not None:
                smoothness = float(abs(result.lateral_error_px - self.last_lateral_error))
            self.last_lateral_error = result.lateral_error_px

        # FTG planner statistics
        ftg_solve = nan
        no_gap = 1.0
        target_angle_rate = nan
        
        ftg_debug = getattr(self.controller, 'latest_ftg_debug', None)
        if ftg_debug is not None:
            ftg_solve = float(ftg_debug.get('solve_time_ms', nan))
            t_dist = ftg_debug.get('target_dist', 0.0)
            no_gap = 1.0 if (t_dist <= 0.0 or ftg_debug.get('target') is None) else 0.0
            
            # Target angle change rate
            t_angle = ftg_debug.get('target_angle')
            if t_angle is not None:
                if self.last_ftg_target_angle is not None and self.last_ftg_target_time is not None:
                    dt_ftg = now - self.last_ftg_target_time
                    if dt_ftg > 1e-4:
                        target_angle_rate = float(abs(t_angle - self.last_ftg_target_angle) / dt_ftg)
                self.last_ftg_target_angle = t_angle
                self.last_ftg_target_time = now

        values = {
            'marker_seen': 1.0 if marker_seen else 0.0,
            'marker_x_px': nan,
            'marker_y_px': nan,
            'lateral_error_px': nan,
            'free_space_lateral_target_px': float(
                self.latest_free_space_lateral_target_px
            ),
            'heading_error_rad': nan,
            'nominal_roll_pwm': float(self.nominal_roll),
            'roll_pwm': roll_pwm_val,
            'nominal_throttle_pwm': nom_throttle,
            'throttle_pwm': throttle_pwm_val,
            'raw_track_speed_pps': float(self.raw_track_speed_pps),
            'track_speed_pps': float(self.track_speed_pps),
            'target_speed_pps': float(self.effective_target_track_speed_pps),
            'speed_error_pps': float(self.speed_error_pps),
            'safety_clearance_px': nan,
            'cbf_scale': float(self.cbf_scale),
            'cbf_active': 1.0 if self.cbf_active else 0.0,
            'nearest_static_clearance_px': float(self.nearest_static_clearance_px),
            'cbf_qp_accel': float(self.cbf_qp_accel),
            'cbf_qp_delta': float(self.cbf_qp_delta),
            'cbf_qp_h': float(self.cbf_qp_h),
            'cbf_qp_h_dot': float(self.cbf_qp_h_dot),
            'cbf_qp_h_ddot': float(self.cbf_qp_h_ddot),
            'cbf_qp_lhs_a': float(self.cbf_qp_lhs_a),
            'cbf_qp_lhs_delta': float(self.cbf_qp_lhs_delta),
            'cbf_qp_rhs': float(self.cbf_qp_rhs),
            'cbf_qp_solve_time_ms': float(self.cbf_qp_solve_time_ms),
            'cbf_qp_slack': float(self.cbf_qp_slack),
            # New metrics
            'pose_position_error_px': pose_pos_err,
            'pose_heading_error_rad': pose_heading_err,
            'pose_update_frequency_hz': pose_update_freq,
            'pose_confidence_level': 1.0 if marker_seen else 0.0,
            'control_smoothness': smoothness,
            'actuator_steer_saturated': steer_saturated,
            'actuator_throttle_saturated': throttle_saturated,
            'safety_violated': violated,
            'safety_override_magnitude': safety_override,
            'ftg_solve_time_ms': ftg_solve,
            'ftg_no_gap_found': no_gap,
            'ftg_target_angle_rate_rad_s': target_angle_rate,
        }
        if result is not None:
            values.update(
                {
                    'marker_x_px': float(result.center[0]),
                    'marker_y_px': float(result.center[1]),
                    'lateral_error_px': float(result.lateral_error_px),
                    'heading_error_rad': self.display_heading_error_rad(
                        result.heading_error_rad
                    ),
                    'safety_clearance_px': float(result.safety_clearance_px),
                }
            )

        for name, value in values.items():
            message = Float64()
            message.data = value
            self.telemetry_publishers[name].publish(message)

    def detect_marker(self, frame):
        """Return marker center, heading, and corners for the configured ID."""
        detection_frame, scale = self.detection_frame(frame)
        result = self.detect_marker_in_frame(detection_frame, scale)
        if result is not None:
            return result
        if (
            scale != 1.0
            and getattr(self.config, 'aruco_fallback_full_res', True)
        ):
            return self.detect_marker_in_frame(frame, 1.0)
        return None

    def detect_marker_in_frame(self, detection_frame, scale):
        """Detect the configured marker in a possibly resized frame."""
        gray = cv2.cvtColor(detection_frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray,
                self.aruco_dictionary,
                parameters=self.aruco_parameters,
            )
        if ids is None:
            return None

        flat_ids = ids.reshape(-1)
        for index, marker_id in enumerate(flat_ids):
            if int(marker_id) != self.config.marker_id:
                continue
            marker_corners = corners[index].reshape(4, 2).astype(np.float32)
            if scale != 1.0:
                marker_corners /= scale
            center = np.mean(marker_corners, axis=0)
            center = self.apply_aruco_parallax(center, marker_corners)
            heading = self.marker_heading(marker_corners)
            return center, heading, marker_corners
        return None

    def update_marker_scale(self, corners):
        """Update controller scale from the detected square ArUco marker."""
        side_lengths = [
            np.linalg.norm(corners[(index + 1) % 4] - corners[index])
            for index in range(4)
        ]
        self.controller.set_aruco_marker_size_px(float(np.mean(side_lengths)))

    def apply_aruco_parallax(self, center, corners):
        """Shift the marker control point to compensate for camera parallax."""
        factor = self.config.aruco_parallax_factor
        if factor == 0.0:
            return center

        front_midpoint = {
            'top': (corners[0] + corners[1]) * 0.5,
            'right': (corners[1] + corners[2]) * 0.5,
            'bottom': (corners[2] + corners[3]) * 0.5,
            'left': (corners[3] + corners[0]) * 0.5,
        }[self.config.front_edge]
        front_vector = front_midpoint - center
        norm = float(np.linalg.norm(front_vector))
        if norm <= 1e-6:
            return center

        side_lengths = [
            np.linalg.norm(corners[(index + 1) % 4] - corners[index])
            for index in range(4)
        ]
        marker_size_px = float(np.mean(side_lengths))
        return center + (front_vector / norm) * marker_size_px * factor

    def detection_frame(self, frame):
        """Return an optionally downscaled frame for faster detection."""
        if self.config.process_width <= 0:
            return frame, 1.0
        height, width = frame.shape[:2]
        if width <= self.config.process_width:
            return frame, 1.0
        scale = self.config.process_width / float(width)
        resized = cv2.resize(
            frame,
            (self.config.process_width, int(round(height * scale))),
            interpolation=cv2.INTER_AREA,
        )
        return resized, scale

    def message_is_stale(self, image_msg):
        """Drop old frames when upstream timestamps show backlog."""
        if self.config.max_frame_age <= 0.0:
            return False
        stamp = image_msg.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        return now - stamp_sec > self.config.max_frame_age

    def marker_heading(self, corners):
        """Estimate vehicle heading from the configured marker front edge."""
        midpoint = {
            'top': (corners[0] + corners[1]) * 0.5,
            'right': (corners[1] + corners[2]) * 0.5,
            'bottom': (corners[2] + corners[3]) * 0.5,
            'left': (corners[3] + corners[0]) * 0.5,
        }[self.config.front_edge]
        center = np.mean(corners, axis=0)
        front_vector = midpoint - center
        return math.atan2(float(front_vector[1]), float(front_vector[0]))

    def stop_immediately(self):
        """Neutralize command state and publish a stop as soon as marker is lost."""
        self.controller.mark_marker_lost()
        if self.output_enabled and not self.config.dry_run:
            command = self.neutral_message()
            self.command_pub.publish(command)

    def create_pid_panel(self):
        """Open live sliders for controller tuning."""
        self.tuning.create_pid_panel()

    def read_panel_values(self):
        """Read live slider values without mutating controller state."""
        return self.tuning.read_panel_values()

    def apply_tuning_values(self, values):
        """Apply tuning values to current config and controller gains."""
        self.controller.apply_tuning_values(values)

    def update_pid_from_panel(self):
        """Read live sliders and update controller gains."""
        if threading.current_thread() is not threading.main_thread():
            return
        self.tuning.update_pid_from_panel()

    def current_tuning_values(self):
        """Return latest tuning values, including CLI-only mode settings."""
        return self.tuning.current_tuning_values()

    def save_tuning_file(self):
        """Save current tuning values to disk."""
        self.tuning.save_tuning_file()

    def load_tuning_file(self):
        """Load tuning values from disk if available."""
        self.tuning.load_tuning_file()

    def save_metrics_file(self):
        """Save current ranking metrics to disk."""
        if self.controller.metrics_saved:
            return
        payload = self.metrics.summary()
        payload.update(
            {
                'controller_mode': self.config.controller_mode,
                'track_shape': self.config.track_shape,
                'drive_channel': self.config.drive_channel,
                'aruco_parallax_factor': self.config.aruco_parallax_factor,
                'steering_kp_px': self.config.steering_kp_px,
                'steering_ki_px': self.config.steering_ki_px,
                'steering_kd_px': self.config.steering_kd_px,
                'heading_kp': self.heading_kp,
                'velocity_kp_pwm': self.config.velocity_kp_pwm,
                'velocity_ki_pwm': self.config.velocity_ki_pwm,
                'velocity_kd_pwm': self.config.velocity_kd_pwm,
                'speed_error_pps_last': self.speed_error_pps,
                'velocity_delta_pwm_last': self.velocity_delta_pwm,
                'nominal_roll_pwm_last': self.nominal_roll,
                'nominal_throttle_pwm_last': self.nominal_throttle,
                'target_track_speed_pps': self.target_track_speed_pps,
                'gap_planner_mode': getattr(
                    self.config,
                    'gap_planner_mode',
                    'stable_free_space',
                ),
                'gap_switch_hysteresis_px': getattr(
                    self.config,
                    'gap_switch_hysteresis_px',
                    80.0,
                ),
                'gap_target_smoothing_alpha': getattr(
                    self.config,
                    'gap_target_smoothing_alpha',
                    0.18,
                ),
                'include_road_boundary_walls': getattr(
                    self.config,
                    'include_road_boundary_walls',
                    False,
                ),
                'track_speed_filter_alpha': self.config.track_speed_filter_alpha,
                'track_speed_slew_rate_pps2': self.config.track_speed_slew_rate_pps2,
                'raw_track_speed_pps_last': self.raw_track_speed_pps,
                'track_speed_pps_last': self.track_speed_pps,
                'gap_lookahead_points': getattr(
                    self.config,
                    'gap_lookahead_points',
                    self.config.lookahead_points,
                ),
                'setpoint_ahead_points': getattr(
                    self.config,
                    'setpoint_ahead_points',
                    self.config.lookahead_points,
                ),
                'forward_pwm': self.config.forward_pwm,
                'min_forward_pwm': self.config.min_forward_pwm,
                'max_forward_pwm': self.config.max_forward_pwm,
                'cbf_slow_error_px': self.config.cbf_slow_error_px,
                'cbf_stop_error_px': self.cbf_stop_error_px,
                'cbf_h_px': self.config.cbf_h_px,
                'cbf_alpha': self.config.cbf_alpha,
                'cbf_stop_heading_rad': self.config.cbf_stop_heading_rad,
                'cbf_edge_margin_px': self.config.cbf_edge_margin_px,
                'qp_wheelbase_px': self.config.qp_wheelbase_px,
                'cbf_a_ell': self.config.cbf_a_ell,
                'cbf_b_ell': self.config.cbf_b_ell,
                'cbf_gamma1': self.config.cbf_gamma1,
                'cbf_gamma2': self.config.cbf_gamma2,
                'cbf_gamma3': self.config.cbf_gamma3,
                'qp_min_accel': self.config.qp_min_accel,
                'qp_max_accel': self.config.qp_max_accel,
                'qp_min_delta': self.config.qp_min_delta,
                'qp_max_delta': self.config.qp_max_delta,
                'qp_solver': self.config.qp_solver,
                'qp_slack_weight': getattr(self.config, 'qp_slack_weight', 0.0),
                'cbf_qp_accel_last': self.cbf_qp_accel,
                'cbf_qp_delta_last': self.cbf_qp_delta,
                'cbf_qp_status_last': self.cbf_qp_status,
                'cbf_qp_h_last': self.cbf_qp_h,
                'cbf_qp_h_dot_last': self.cbf_qp_h_dot,
                'cbf_qp_h_ddot_last': self.cbf_qp_h_ddot,
                'cbf_qp_lhs_a_last': self.cbf_qp_lhs_a,
                'cbf_qp_lhs_delta_last': self.cbf_qp_lhs_delta,
                'cbf_qp_rhs_last': self.cbf_qp_rhs,
                'cbf_qp_solve_time_ms_last': self.cbf_qp_solve_time_ms,
                'cbf_qp_slack_last': self.cbf_qp_slack,
                'ftg_stuck_repulsion_active_last': getattr(
                    self.controller,
                    'ftg_stuck_repulsion_active',
                    False,
                ),
                'ftg_stuck_repulsion_roll_bias_last': getattr(
                    self.controller,
                    'ftg_stuck_repulsion_roll_bias',
                    0.0,
                ),
                'ftg_stuck_repulsion_throttle_pwm_last': getattr(
                    self.controller,
                    'ftg_stuck_repulsion_throttle_pwm',
                    self.config.neutral_throttle_pwm,
                ),
                'lap_limit_enabled': self.lap_limit_enabled,
                'target_laps': self.target_laps,
                'nearest_static_clearance_px_last': (
                    self.nearest_static_clearance_px
                ),
                'road_half_width_px': self.config.road_half_width_px,
                'obstacle_margin_px': self.config.obstacle_margin_px,
            }
        )
        try:
            self.metrics_file_path.parent.mkdir(parents=True, exist_ok=True)
            self.metrics_file_path.write_text(
                json.dumps(payload, indent=2) + '\n',
                encoding='utf-8',
            )
        except PermissionError as exc:
            fallback = (
                Path(__file__).resolve().parents[2]
                / 'metrics'
                / self.metrics_file_path.name
            )
            self.get_logger().warning(
                f'cannot save metrics to {self.metrics_file_path}: {exc}; '
                f'using {fallback}'
            )
            fallback.parent.mkdir(parents=True, exist_ok=True)
            fallback.write_text(
                json.dumps(payload, indent=2) + '\n',
                encoding='utf-8',
            )
            self.metrics_file_path = fallback
        self.controller.metrics_saved = True
        self.get_logger().info(f'saved metrics to {self.metrics_file_path}')

    def command_is_fresh(self):
        """Return whether a recent marker-derived command is available."""
        return self.controller.command_is_fresh()

    def neutral_message(self):
        """Create a centered and stopped RC command."""
        command = RCMessage()
        command.rc_throttle = self.config.neutral_throttle_pwm
        command.rc_roll = self.config.center_steering_pwm
        command.rc_pitch = NEUTRAL_VALUE
        command.rc_yaw = NEUTRAL_VALUE
        return command

    def drive_message(self):
        """Create an RC command using the configured drive channel."""
        command = self.neutral_message()
        command.rc_roll = self.roll
        if self.config.drive_channel == 'pitch':
            command.rc_pitch = self.throttle
        else:
            command.rc_throttle = self.throttle
        return command

    def publish_command(self):
        """Publish fresh RC output, otherwise publish neutral."""
        if threading.current_thread() is threading.main_thread():
            self.poll_keyboard()
        if self.config.dry_run or not self.output_enabled:
            return
        if not self.command_is_fresh():
            command = self.neutral_message()
            self.command_pub.publish(command)
            return

        command = self.drive_message()
        self.command_pub.publish(command)

    def poll_keyboard(self):
        """Stop the vehicle when Q is pressed in any OpenCV window."""
        if self.config.ignore_preview_keys:
            return
        if not (self.config.preview or self.tuning.enabled):
            return
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.request_stop('q pressed')
        elif key == ord('s'):
            self.save_tuning_file()
        elif key == ord('p'):
            self.save_preview_snapshot()

    def save_preview_snapshot(self):
        """Write the most recent preview frame to a timestamped PNG."""
        if self.latest_preview_frame is None:
            return
        self.config.debug_snapshot_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        path = self.config.debug_snapshot_dir / f'straight_debug_{timestamp}.png'
        cv2.imwrite(str(path), self.latest_preview_frame)
        self.get_logger().info(f'saved preview snapshot to {path}')

    def request_stop(self, reason):
        """Publish stop commands and request node shutdown."""
        if self.stop_requested:
            return
        self.controller.request_stop_state(reason)
        self.get_logger().info(f'{reason}; sending stop commands')
        self.output_enabled = False
        if not self.config.dry_run:
            self.publish_neutral_burst(SETTLE_DURATION)
        self.save_metrics_file()

    def publish_neutral_burst(self, duration):
        """Publish neutral repeatedly without spinning callbacks."""
        deadline = time.monotonic() + duration
        period = 1.0 / UPDATE_RATE_HZ
        command = self.neutral_message()
        while time.monotonic() < deadline:
            self.command_pub.publish(command)
            time.sleep(period)

    def debug_command(self, command, state):
        """Optionally print outgoing RC channels at a limited rate."""
        if not self.config.debug_commands:
            return
        now = time.monotonic()
        if now - self.last_debug_print_time < 1.0:
            return
        self.last_debug_print_time = now
        self.get_logger().info(
            f'{state}: roll={command.rc_roll} '
            f'pitch={command.rc_pitch} '
            f'throttle={command.rc_throttle} '
            f'yaw={command.rc_yaw}'
        )

    @staticmethod
    def display_heading_error_rad(heading_error_rad):
        """Return heading error in the same frame used by the controller."""
        return heading_error_rad

    def reset_virtual_vehicle(self):
        """Drop a synthetic vehicle onto the straight road test scene."""
        width = self.config.virtual_width
        height = self.config.virtual_height
        self.virtual_road_tile_index = 0
        self.controller.build_track_scene(width, height)
        if self.config.random_static_obstacles:
            self.random_static_obstacles = self.make_random_static_obstacles()
            self.detected_random_obstacles = []
            self.controller.static_obstacles = []
        point_count = len(self.track_points)
        start_index = int(
            round(self.config.virtual_start_progress * (point_count - 1))
        )
        start_index = max(0, min(start_index, point_count - 1))
        center = self.track_points[start_index].astype(np.float32)
        normal = self.road_normals[start_index]
        center = center + normal * self.config.virtual_start_lateral_offset_px
        self.virtual_vehicle_state = {
            'x': float(center[0]),
            'y': float(center[1]),
            'heading': math.radians(self.config.virtual_start_heading_deg),
            'speed': max(0.0, self.config.virtual_start_speed_pps),
        }
        self.controller.set_aruco_marker_size_px(
            float(getattr(self.config, 'virtual_marker_size_px', 10.0))
        )
        self.virtual_last_time = time.monotonic()
        self.controller.mark_marker_lost()
        self.flash_timer = 5

    def virtual_vehicle_tick(self):
        """Run one controller/simulation step for the synthetic vehicle."""
        if self.virtual_vehicle_state is None:
            self.reset_virtual_vehicle()
        now = time.monotonic()
        dt = min(0.1, max(1e-3, now - self.virtual_last_time))
        self.virtual_last_time = now

        self.update_pid_from_panel()
        state = self.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        if self.config.random_static_obstacles:
            self.update_detected_random_obstacles(center)
        result = self.controller.process_detection(
            center,
            state['heading'],
            image_size=(self.config.virtual_width, self.config.virtual_height),
            now=now,
        )
        self.publish_tuning_telemetry(result, marker_seen=True)
        previous_center = center.copy()
        if not hasattr(self, 'virtual_vehicle_trail'):
            self.virtual_vehicle_trail = []
        self.virtual_vehicle_trail.append(center.copy())
        if len(self.virtual_vehicle_trail) > 120:
            self.virtual_vehicle_trail.pop(0)
        self.advance_virtual_vehicle(dt)
        self.debug_command(self.drive_message(), 'virtual')
        preview_target = result.target
        if self.config.virtual_unlimited_path and not getattr(self.config, 'use_safe_control_env', False):
            scene_delta = self.advance_virtual_road_window(previous_center)
            preview_target = result.target + scene_delta
        elif self.config.virtual_stop_at_end and self.virtual_reached_road_end():
            self.request_stop('virtual vehicle reached road end')
        if self.config.preview:
            frame = self.make_virtual_frame()
            preview_center = np.array(
                [
                    self.virtual_vehicle_state['x'],
                    self.virtual_vehicle_state['y'],
                ],
                dtype=np.float32,
            )
            self.schedule_preview(
                frame,
                None,
                preview_center,
                preview_target,
                result.tangent,
            )

    def schedule_preview(self, frame, corners, center, target, tangent):
        """Store the latest preview data; main thread owns OpenCV UI calls."""
        max_fps = float(getattr(self.config, 'preview_max_fps', 12.0))
        if max_fps > 0.0:
            now = time.monotonic()
            min_period = 1.0 / max_fps
            if now - self.last_preview_schedule_time < min_period:
                return
            self.last_preview_schedule_time = now
        if threading.current_thread() is threading.main_thread():
            self.show_preview(frame, corners, center, target, tangent)
            return
        with self.preview_lock:
            self.pending_preview = (
                frame.copy(),
                None if corners is None else corners.copy(),
                None if center is None else center.copy(),
                None if target is None else target.copy(),
                None if tangent is None else tangent.copy(),
            )

    def render_pending_preview(self):
        """Render at most one pending preview frame on the main thread."""
        with self.preview_lock:
            pending = self.pending_preview
            self.pending_preview = None
        if pending is None:
            return
        self.show_preview(*pending)

    def advance_virtual_vehicle(self, dt):
        """Advance the synthetic vehicle using controller output."""
        state = self.virtual_vehicle_state
        throttle_span = max(
            1.0,
            self.config.max_forward_pwm - self.config.neutral_throttle_pwm,
        )
        throttle_ratio = (
            self.throttle - self.config.neutral_throttle_pwm
        ) / throttle_span
        target_speed = max(
            0.0,
            throttle_ratio * self.config.target_track_speed_pps,
        )
        speed_error = target_speed - state['speed']
        accel = speed_error * self.config.virtual_speed_response
        accel = max(
            -self.config.virtual_max_brake_pps2,
            min(self.config.virtual_max_accel_pps2, accel),
        )
        state['speed'] = max(0.0, state['speed'] + accel * dt)
        delta = self.controller.roll_pwm_to_qp_delta(self.roll)
        # The physical steering mappings are inverted in self.controller (roll_pwm_to_qp_delta),
        # so we invert the delta sign here to keep the virtual simulation steering correctly.
        heading_rate = state['speed'] / max(1.0, self.config.qp_wheelbase_px) * delta
        state['heading'] = math.atan2(
            math.sin(state['heading'] + heading_rate * dt),
            math.cos(state['heading'] + heading_rate * dt),
        )
        state['x'] += math.cos(state['heading']) * state['speed'] * dt
        state['y'] += math.sin(state['heading']) * state['speed'] * dt

    def virtual_reached_road_end(self):
        """Return whether the virtual vehicle has reached the end of the road."""
        nearest_index = self.current_virtual_nearest_index()
        if nearest_index is None or self.track_points is None:
            return False
        return nearest_index >= len(self.track_points) - 3

    def current_virtual_nearest_index(self):
        """Return the nearest road index for the current synthetic vehicle."""
        if self.virtual_vehicle_state is None or self.track_points is None:
            return None
        center = np.array(
            [
                self.virtual_vehicle_state['x'],
                self.virtual_vehicle_state['y'],
            ],
            dtype=np.float32,
        )
        distances = np.linalg.norm(self.track_points - center, axis=1)
        return int(np.argmin(distances))

    def advance_virtual_road_window(self, previous_center):
        """Scroll the virtual road in an ego-follow frame for endless tests."""
        scene_delta = self.scroll_virtual_follow_frame(previous_center)
        if self.virtual_reached_road_end():
            scene_delta = scene_delta + self.recycle_virtual_road_window()
        return scene_delta

    def scroll_virtual_follow_frame(self, previous_center):
        """Move the road opposite ego progress while keeping lateral error visible."""
        zero_delta = np.zeros(2, dtype=np.float32)
        if (
            self.virtual_vehicle_state is None
            or self.track_points is None
            or self.road_tangents is None
            or len(self.track_points) < 2
        ):
            return zero_delta
        nearest_index = self.current_virtual_nearest_index()
        if nearest_index is None:
            return zero_delta
        state = self.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        tangent = self.road_tangents[
            max(0, min(nearest_index, len(self.road_tangents) - 1))
        ]
        forward_delta = float(np.dot(center - previous_center, tangent))
        if abs(forward_delta) <= 1e-6:
            return zero_delta
        shift = tangent * forward_delta
        state['x'] = float(center[0] - shift[0])
        state['y'] = float(center[1] - shift[1])
        scene_delta = -shift
        self.translate_virtual_scene(scene_delta)
        return scene_delta

    def recycle_virtual_road_window(self):
        """Advance the finite road tile so the controller keeps seeing road ahead."""
        if self.track_points is None or len(self.track_points) < 2:
            return np.zeros(2, dtype=np.float32)
        tile_shift = self.track_points[-1] - self.track_points[0]
        self.translate_virtual_scene(tile_shift)
        self.virtual_road_tile_index += 1
        self.controller.last_progress_index = None
        self.controller.metrics.previous_progress = None
        self.get_logger().info('advanced virtual road window')
        return tile_shift

    def translate_virtual_scene(self, delta):
        """Translate road geometry and obstacle polygons by an image-space delta."""
        delta = np.asarray(delta, dtype=np.float32)
        if self.track_points is not None:
            self.track_points = self.track_points + delta
            self.controller.track_points = self.track_points
        if self.road_boundaries:
            self.road_boundaries = [
                boundary + delta for boundary in self.road_boundaries
            ]
            self.controller.road_boundaries = self.road_boundaries
        for obstacle in self.virtual_scene_obstacles():
            obstacle['center'] = obstacle['center'] + delta
            obstacle['polygon'] = obstacle['polygon'] + delta
        if self.controller.latest_free_space_target is not None:
            self.controller.latest_free_space_target = (
                self.controller.latest_free_space_target + delta
            )
        if self.controller.latest_target_center is not None:
            self.controller.latest_target_center = (
                self.controller.latest_target_center + delta
            )
        if hasattr(self, 'virtual_vehicle_trail'):
            self.virtual_vehicle_trail = [
                pt + delta for pt in self.virtual_vehicle_trail
            ]
        ftg_debug = getattr(self.controller, 'latest_ftg_debug', None)
        if ftg_debug is not None:
            for key in ('target', 'nearest_point', 'origin', 'scaled_target'):
                if ftg_debug.get(key) is not None:
                    ftg_debug[key] = ftg_debug[key] + delta
            if ftg_debug.get('bubble_points'):
                ftg_debug['bubble_points'] = [
                    p + delta for p in ftg_debug['bubble_points']
                ]

    def virtual_scene_obstacles(self):
        """Yield each currently tracked virtual obstacle once."""
        obstacle_lists = [
            self.controller.static_obstacles,
            self.random_static_obstacles,
            self.detected_random_obstacles,
        ]
        for name in ('static_scene_obstacles', 'dynamic_obstacles'):
            if hasattr(self, name):
                obstacle_lists.append(getattr(self, name))
        seen = set()
        for obstacles in obstacle_lists:
            for obstacle in obstacles:
                key = id(obstacle)
                if key in seen:
                    continue
                seen.add(key)
                yield obstacle

    def wrap_virtual_vehicle_to_start(self):
        """Move the virtual vehicle from the road end back to the road start."""
        if (
            self.virtual_vehicle_state is None
            or self.track_points is None
            or self.road_normals is None
            or len(self.track_points) < 2
        ):
            return
        state = self.virtual_vehicle_state
        position = np.array([state['x'], state['y']], dtype=np.float32)
        end_center = self.track_points[-1]
        start_center = self.track_points[0]
        end_normal = self.road_normals[-1]
        start_normal = self.road_normals[0]
        end_tangent = self.track_points[-1] - self.track_points[-2]
        start_tangent = self.track_points[1] - self.track_points[0]
        end_tangent_norm = np.linalg.norm(end_tangent)
        start_tangent_norm = np.linalg.norm(start_tangent)
        if end_tangent_norm <= 0.0 or start_tangent_norm <= 0.0:
            state['x'] = float(start_center[0])
            state['y'] = float(start_center[1])
            return
        end_tangent = end_tangent / end_tangent_norm
        start_tangent = start_tangent / start_tangent_norm
        offset = position - end_center
        lateral_offset = float(np.dot(offset, end_normal))
        forward_overshoot = max(0.0, float(np.dot(offset, end_tangent)))
        wrapped = (
            start_center
            + start_normal * lateral_offset
            + start_tangent * forward_overshoot
        )
        state['x'] = float(wrapped[0])
        state['y'] = float(wrapped[1])
        self.controller.last_progress_index = None
        self.controller.metrics.previous_progress = None
        self.get_logger().info('wrapped virtual vehicle to road start')

    def make_virtual_frame(self):
        """Create a synthetic camera frame with road-context visual texture."""
        height = self.config.virtual_height
        width = self.config.virtual_width
        y_gradient = np.linspace(248, 232, height, dtype=np.uint8)[:, None]
        frame = np.repeat(y_gradient, width, axis=1)
        frame = cv2.merge((frame, frame, frame))

        grid_color = (224, 224, 224)
        for x in range(0, width, 80):
            cv2.line(frame, (x, 0), (x, height), grid_color, 1, cv2.LINE_AA)
        for y in range(0, height, 80):
            cv2.line(frame, (0, y), (width, y), grid_color, 1, cv2.LINE_AA)

        cv2.rectangle(frame, (0, 0), (width - 1, height - 1), (205, 205, 205), 1)
        return frame

    def make_random_static_obstacles(self):
        """Create hidden random obstacles in the virtual road scene."""
        rng = np.random.default_rng(self.config.random_obstacle_seed)
        obstacles = []
        for index in range(self.config.random_obstacle_count):
            progress = float(
                rng.uniform(
                    self.config.random_obstacle_min_progress,
                    self.config.random_obstacle_max_progress,
                )
            )
            lateral_fraction = float(rng.uniform(-0.58, 0.58))
            # ponytail: Spawn smaller virtual obstacles (length 25-45px, width 15-30px)
            length_px = float(rng.uniform(25.0, 45.0))
            width_px = float(rng.uniform(15.0, 30.0))
            if self.config.virtual_vehicle_test:
                length_px *= 0.55
                width_px *= 0.55
            spec = {
                'progress': progress,
                'length_px': length_px,
                'width_px': width_px,
                'offset': lateral_fraction,
            }
            obstacle = make_laneless_static_obstacle(
                spec,
                self.track_points,
                self.road_tangents,
                self.road_normals,
                self.config,
            )
            obstacle['random_index'] = index
            obstacle['detected'] = False
            obstacles.append(obstacle)
        return obstacles

    def update_detected_random_obstacles(self, center):
        """Reveal random obstacles only when the virtual sensor can see them."""
        detected = []
        heading = self.virtual_vehicle_state['heading']
        forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
        for obstacle in self.random_static_obstacles:
            delta = obstacle['center'] - center
            ahead = float(np.dot(delta, forward))
            distance = float(np.linalg.norm(delta))
            visible = (
                ahead > -obstacle['half_length']
                and distance <= self.config.random_obstacle_detection_range_px
            )
            if visible:
                obstacle['detected'] = True
            if obstacle.get('detected'):
                detected.append(obstacle)
        self.detected_random_obstacles = detected
        self.controller.static_obstacles = detected

    def show_preview(self, frame, corners, center, target, tangent):
        """Display the virtual track and current ArUco tracking state."""
        self.update_preview_text_controls()
        if self.flash_timer > 0:
            frame[:] = 255
            self.flash_timer -= 1
        preview = frame.copy()
        if self.road_scene_enabled():
            self.draw_road_scene(preview)
        if corners is not None:
            cv2.polylines(
                preview,
                [corners.astype(np.int32)],
                True,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        if center is not None:
            cv2.circle(preview, tuple(center.astype(int)), 6, (255, 0, 0), -1)
        if self.config.virtual_vehicle_test:
            self.draw_virtual_vehicle(preview)
        if target is not None:
            cv2.circle(preview, tuple(target.astype(int)), 8, (0, 0, 255), -1)
        if center is not None and target is not None:
            cv2.line(
                preview,
                tuple(center.astype(int)),
                tuple(target.astype(int)),
                (0, 220, 255),
                2,
                cv2.LINE_AA,
            )
        if self.config.debug_visuals or self.config.virtual_vehicle_test:
            self.draw_debug_visuals(preview, center, target, tangent)
        if not self.config.virtual_vehicle_test:
            put_status(
                preview,
                self.config.controller_mode,
                self.throttle,
                self.roll,
                self.command_is_fresh(),
                self.track_speed_pps,
                self.effective_target_track_speed_pps,
                self.speed_error_pps,
                self.velocity_delta_pwm,
                self.cbf_scale,
                self.metrics.summary(),
                self.lap_limit_enabled,
                self.target_laps,
                self.nearest_static_clearance_px,
                self.cbf_qp_status,
                self.cbf_qp_accel,
                self.cbf_qp_delta,
            )
        preview = resize_for_preview(preview, self.config.preview_width)
        self.latest_preview_frame = preview.copy()
        self.prepare_preview_window(preview)
        cv2.imshow(PREVIEW_WINDOW, preview)
        self.resize_preview_window(preview)

        # ponytail: Save simple time-series log for graph visualization
        import os
        import time
        tuning_file = str(getattr(self.config, 'tuning_file', 'tuning.json') or 'tuning.json')
        is_stable = 'stable' in tuning_file.lower()
        log_name = 'telemetry_stable.csv' if is_stable else 'telemetry_unstable.csv'
        log_path = os.path.join('/home/dhruv/amcaf/simulation', log_name)
        try:
            if not os.path.exists(log_path):
                with open(log_path, 'w') as f:
                    f.write("time,cte,he,steering,h,clf_alpha,steering_kp\n")
            with open(log_path, 'a') as f:
                f.write(f"{time.time()},{self.latest_lateral_error_px:.4f},{self.latest_heading_error_rad:.4f},{self.cbf_qp_delta:.4f},{self.cbf_qp_h:.4f},{self.config.clf_alpha:.4f},{self.config.steering_kp_px:.4f}\n")
        except Exception:
            pass

    def prepare_preview_window(self, preview):
        """Create a resizable preview window once."""
        if self.preview_window_ready:
            return
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        cv2.moveWindow(PREVIEW_WINDOW, 50, 50)
        self.create_preview_text_controls()
        self.preview_window_ready = True

    def create_preview_text_controls(self):
        """Create live preview controls for text scale and visibility."""
        if self.preview_text_controls_ready:
            return
        cv2.createTrackbar('text x10', PREVIEW_WINDOW, 10, 30, noop)
        cv2.createTrackbar('text on', PREVIEW_WINDOW, 1, 1, noop)
        self.preview_text_controls_ready = True

    def update_preview_text_controls(self):
        """Apply live preview text scale and visibility controls."""
        if not self.preview_text_controls_ready:
            set_overlay_text_style(scale=1.0, visible=True)
            return
        text_scale_x10 = cv2.getTrackbarPos('text x10', PREVIEW_WINDOW)
        text_visible = cv2.getTrackbarPos('text on', PREVIEW_WINDOW) > 0
        set_overlay_text_style(
            scale=max(1, text_scale_x10) / 10.0,
            visible=text_visible,
        )

    def resize_preview_window(self, preview):
        """Set the initial preview window size without fighting user/window-manager resizes."""
        if self.preview_window_size_initialized:
            return
        height, width = preview.shape[:2]
        target_width = width
        if self.config.preview_width > 0:
            target_width = max(width, self.config.preview_width)
        target_height = int(round(height * target_width / max(1, width)))
        cv2.resizeWindow(PREVIEW_WINDOW, target_width, target_height)
        cv2.moveWindow(PREVIEW_WINDOW, 50, 50)
        self.preview_window_size_initialized = True

    def draw_virtual_vehicle(self, preview):
        """Draw the synthetic vehicle body and heading vector."""
        if self.virtual_vehicle_state is None:
            return
        state = self.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        heading = state['heading']
        forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
        right = np.array([-math.sin(heading), math.cos(heading)], dtype=np.float32)
        half_length = 34.0
        half_width = 18.0
        cabin_scale = 1.0
        wheel_scale = 1.0
        wheel_radius = 4
        if self.config.virtual_vehicle_test:
            half_length *= 0.55
            half_width *= 0.55
            cabin_scale *= 0.55
            wheel_scale *= 0.55
            wheel_radius = 2

        polygon = np.array(
            [
                center + forward * half_length + right * half_width,
                center - forward * half_length + right * half_width,
                center - forward * half_length - right * half_width,
                center + forward * half_length - right * half_width,
            ],
            dtype=np.int32,
        )
        shadow = polygon + np.array([2, 3], dtype=np.int32)
        cv2.fillConvexPoly(preview, shadow, (185, 185, 185))
        cv2.fillConvexPoly(preview, polygon, (188, 114, 0))
        cv2.polylines(preview, [polygon], True, (18, 42, 74), 2, cv2.LINE_AA)
        cabin = np.array(
            [
                center + forward * (10.0 * cabin_scale) + right * (11.0 * cabin_scale),
                center - forward * (14.0 * cabin_scale) + right * (10.0 * cabin_scale),
                center - forward * (14.0 * cabin_scale) - right * (10.0 * cabin_scale),
                center + forward * (10.0 * cabin_scale) - right * (11.0 * cabin_scale),
            ],
            dtype=np.int32,
        )
        cv2.fillConvexPoly(preview, cabin, (255, 224, 166))
        cv2.polylines(preview, [cabin], True, (95, 85, 65), 1, cv2.LINE_AA)
        for wheel_offset in (
            forward * (19.0 * wheel_scale) + right * (19.0 * wheel_scale),
            forward * (19.0 * wheel_scale) - right * (19.0 * wheel_scale),
            -forward * (21.0 * wheel_scale) + right * (19.0 * wheel_scale),
            -forward * (21.0 * wheel_scale) - right * (19.0 * wheel_scale),
        ):
            wheel_center = center + wheel_offset
            cv2.circle(preview, tuple(wheel_center.astype(int)), wheel_radius, (32, 32, 32), -1)
        nose = center + forward * (half_length + 10.0 * wheel_scale)
        cv2.arrowedLine(
            preview,
            tuple(center.astype(int)),
            tuple(nose.astype(int)),
            (0, 58, 220),
            2,
            cv2.LINE_AA,
            tipLength=0.26,
        )

    def draw_road_scene(self, preview):
        """Draw the road scene and static obstacle field."""
        if self.unlimited_virtual_road_enabled():
            self.draw_virtual_road_tiles(preview)
        elif self.road_boundaries and len(self.road_boundaries) >= 2:
            self.draw_road_band(preview)
        if self.road_boundaries and not self.unlimited_virtual_road_enabled():
            for boundary in self.road_boundaries:
                cv2.polylines(
                    preview,
                    [boundary.astype(np.int32)],
                    self.closed_road_scene_enabled(),
                    (120, 128, 132),
                    2,
                    cv2.LINE_AA,
                )
        if self.track_points is not None and not self.unlimited_virtual_road_enabled():
            cv2.polylines(
                preview,
                [self.track_points.astype(np.int32)],
                self.closed_road_scene_enabled(),
                (246, 246, 246),
                2,
                cv2.LINE_AA,
            )
            self.draw_progress_ticks(preview)
        if self.unlimited_virtual_road_enabled():
            return
        obstacles = list(self.random_static_obstacles or self.static_obstacles)
        if (
            self.config.random_static_obstacles
            and getattr(self.config, 'include_road_boundary_walls', False)
        ):
            obstacles.extend(getattr(self.controller, 'road_boundary_obstacles', []))
        for obstacle in obstacles:
            if self.config.random_static_obstacles and not obstacle.get('detected'):
                fill_color = (174, 174, 174)
            elif str(obstacle.get('kind', '')).startswith('road_boundary_wall'):
                fill_color = (88, 92, 96)
            elif obstacle.get('kind') == 'dynamic' or 'dynamic_index' in obstacle:
                fill_color = (54, 100, 230)
            else:
                fill_color = (66, 66, 196)
            shadow = obstacle['polygon'].astype(np.int32) + np.array([3, 4])
            cv2.fillConvexPoly(preview, shadow, (188, 188, 188))
            cv2.fillConvexPoly(
                preview,
                obstacle['polygon'].astype(np.int32),
                fill_color,
            )

    def unlimited_virtual_road_enabled(self):
        """Return whether the preview should draw repeated virtual road tiles."""
        return (
            self.config.virtual_vehicle_test
            and self.config.virtual_unlimited_path
            and self.track_points is not None
            and self.road_boundaries
            and len(self.road_boundaries) >= 2
            and len(self.track_points) >= 2
            and not getattr(self.config, 'use_safe_control_env', False)
        )

    def draw_virtual_road_tiles(self, preview):
        """Draw repeated road tiles so unlimited virtual tests never show an end."""
        static_obstacles = self.virtual_tile_static_obstacles()
        for delta in self.virtual_road_tile_offsets(preview):
            shifted_boundaries = [
                boundary + delta for boundary in self.road_boundaries
            ]
            shifted_track = self.track_points + delta
            self.draw_road_band_arrays(
                preview,
                shifted_boundaries,
                shifted_track,
            )
            for boundary in shifted_boundaries:
                cv2.polylines(
                    preview,
                    [boundary.astype(np.int32)],
                    False,
                    (120, 128, 132),
                    2,
                    cv2.LINE_AA,
                )
            cv2.polylines(
                preview,
                [shifted_track.astype(np.int32)],
                False,
                (246, 246, 246),
                2,
                cv2.LINE_AA,
            )
            self.draw_progress_ticks_for(preview, shifted_track)

            # Draw repeated obstacles and boundary walls for this tile
            for obstacle in static_obstacles:
                if self.config.random_static_obstacles and not obstacle.get('detected'):
                    fill_color = (174, 174, 174)
                elif str(obstacle.get('kind', '')).startswith('road_boundary_wall'):
                    fill_color = (88, 92, 96)
                elif obstacle.get('kind') == 'dynamic' or 'dynamic_index' in obstacle:
                    fill_color = (54, 100, 230)
                else:
                    fill_color = (66, 66, 196)
                shifted_poly = obstacle['polygon'] + delta
                shadow = shifted_poly.astype(np.int32) + np.array([3, 4])
                cv2.fillConvexPoly(preview, shadow, (188, 188, 188))
                cv2.fillConvexPoly(
                    preview,
                    shifted_poly.astype(np.int32),
                    fill_color,
                )
        for obstacle in self.virtual_tile_dynamic_obstacles():
            fill_color = (54, 100, 230)
            polygon = obstacle['polygon'].astype(np.int32)
            shadow = polygon + np.array([3, 4])
            cv2.fillConvexPoly(preview, shadow, (188, 188, 188))
            cv2.fillConvexPoly(preview, polygon, fill_color)
            cv2.polylines(preview, [polygon], True, (30, 30, 30), 1, cv2.LINE_AA)

    def virtual_tile_static_obstacles(self):
        """Return only scenery that should repeat with each virtual road tile."""
        if self.config.random_static_obstacles:
            obstacles = list(self.random_static_obstacles)
            if getattr(self.config, 'include_road_boundary_walls', False):
                obstacles.extend(getattr(self.controller, 'road_boundary_obstacles', []))
            return obstacles
        if hasattr(self, 'static_scene_obstacles'):
            obstacles = list(getattr(self, 'static_scene_obstacles') or [])
        else:
            obstacles = list(self.static_obstacles)
        if getattr(self.config, 'include_road_boundary_walls', False):
            obstacles.extend(getattr(self.controller, 'road_boundary_obstacles', []))
        return [
            obstacle for obstacle in obstacles
            if obstacle.get('kind') != 'dynamic' and 'dynamic_index' not in obstacle
        ]

    def virtual_tile_dynamic_obstacles(self):
        """Return moving traffic that should be drawn only once on the active tile."""
        if hasattr(self, 'dynamic_obstacles'):
            return list(getattr(self, 'dynamic_obstacles') or [])
        return []

    def virtual_road_tile_offsets(self, preview):
        """Return tile offsets that cover the current preview frame."""
        tile_shift = self.track_points[-1] - self.track_points[0]
        if np.linalg.norm(tile_shift) <= 1e-6:
            return [np.zeros(2, dtype=np.float32)]
        height, width = preview.shape[:2]
        margin = max(width, height) * 0.25
        all_points = np.vstack(self.road_boundaries + [self.track_points])
        offsets = []
        for tile_index in range(-3, 5):
            delta = tile_shift * float(tile_index)
            shifted = all_points + delta
            min_xy = np.min(shifted, axis=0)
            max_xy = np.max(shifted, axis=0)
            visible = (
                max_xy[0] >= -margin
                and min_xy[0] <= width + margin
                and max_xy[1] >= -margin
                and min_xy[1] <= height + margin
            )
            if visible:
                offsets.append(delta.astype(np.float32))
        if not offsets:
            offsets.append(np.zeros(2, dtype=np.float32))
        return offsets

    def draw_road_band_arrays(self, preview, boundaries, track_points):
        """Fill one laneless road tile from explicit boundary arrays."""
        left_boundary = boundaries[0].astype(np.int32)
        right_boundary = boundaries[1].astype(np.int32)
        road_polygon = np.vstack((left_boundary, right_boundary[::-1]))
        overlay = preview.copy()
        cv2.fillPoly(overlay, [road_polygon], (74, 84, 88))
        cv2.addWeighted(overlay, 0.74, preview, 0.26, 0.0, preview)
        inner_width = min(42.0, self.road_half_width_px * 0.35)
        for lane_edge in (
            track_points - self.road_normals * inner_width,
            track_points + self.road_normals * inner_width,
        ):
            cv2.polylines(
                preview,
                [lane_edge.astype(np.int32)],
                False,
                (112, 122, 126),
                1,
                cv2.LINE_AA,
            )

    def draw_progress_ticks_for(self, preview, track_points):
        """Draw progress ticks for one repeated road tile."""
        if self.road_normals is None or len(track_points) < 2:
            return
        step = max(1, len(track_points) // 10)
        tick_half = 12.0
        for index in range(0, len(track_points), step):
            center = track_points[index]
            normal = self.road_normals[index]
            start = center - normal * tick_half
            end = center + normal * tick_half
            cv2.line(
                preview,
                tuple(start.astype(int)),
                tuple(end.astype(int)),
                (184, 190, 192),
                1,
                cv2.LINE_AA,
            )

    def draw_road_band(self, preview):
        """Fill the laneless road area before drawing boundaries and actors."""
        left_boundary = self.road_boundaries[0].astype(np.int32)
        right_boundary = self.road_boundaries[1].astype(np.int32)
        if self.closed_road_scene_enabled():
            polygon = np.vstack((left_boundary, right_boundary[::-1]))
            overlay = preview.copy()
            cv2.fillPoly(overlay, [polygon], (72, 82, 86))
            cv2.addWeighted(overlay, 0.72, preview, 0.28, 0.0, preview)
            return

        road_polygon = np.vstack((left_boundary, right_boundary[::-1]))
        overlay = preview.copy()
        cv2.fillPoly(overlay, [road_polygon], (74, 84, 88))
        cv2.addWeighted(overlay, 0.74, preview, 0.26, 0.0, preview)
        inner_left = (
            self.track_points
            - self.road_normals * min(42.0, self.road_half_width_px * 0.35)
        )
        inner_right = (
            self.track_points
            + self.road_normals * min(42.0, self.road_half_width_px * 0.35)
        )
        for lane_edge in (inner_left, inner_right):
            cv2.polylines(
                preview,
                [lane_edge.astype(np.int32)],
                False,
                (112, 122, 126),
                1,
                cv2.LINE_AA,
            )

    def draw_progress_ticks(self, preview):
        """Draw subtle progress ticks along the road centerline."""
        if (
            self.track_points is None
            or self.road_normals is None
            or len(self.track_points) < 2
        ):
            return
        step = max(1, len(self.track_points) // 10)
        tick_half = 12.0
        for index in range(0, len(self.track_points), step):
            center = self.track_points[index]
            normal = self.road_normals[index]
            start = center - normal * tick_half
            end = center + normal * tick_half
            cv2.line(
                preview,
                tuple(start.astype(int)),
                tuple(end.astype(int)),
                (184, 190, 192),
                1,
                cv2.LINE_AA,
            )
            if index in (0, len(self.track_points) - 1):
                continue
            progress = index / float(max(1, len(self.track_points) - 1))
            draw_label(
                preview,
                tuple((end + np.array([3.0, -3.0])).astype(int)),
                f'{progress:.1f}',
                (214, 218, 218),
                scale=0.34,
            )

    @staticmethod
    def obstacle_preview_label(index, obstacle):
        """Return a compact obstacle label for the synthetic preview."""
        if 'dynamic_index' in obstacle:
            return f'dyn {obstacle["dynamic_index"]}'
        if 'random_index' in obstacle:
            state = 'seen' if obstacle.get('detected') else 'hidden'
            return f'{state} {obstacle["random_index"]}'
        return f'obs {index}'

    def draw_debug_visuals(self, preview, center, target, tangent):
        """Draw a simplified perception/safety overlay."""
        if self.config.virtual_vehicle_test:
            self.draw_virtual_vehicle_trail(preview)
        else:
            self.draw_marker_trail(preview)
        is_dclf = (self.config.controller_mode == 'pid_velocity_dclf_dcbf')
        if is_dclf:
            self.draw_dclf_dcbf_visuals(preview, center, target)
        else:
            self.draw_cbf_ellipse_debug(preview, center)
        if self.config.controller_mode == 'mpc_cbf':
            opt_traj = getattr(self.controller.mpc_controller, 'optimal_trajectory', None)
            if opt_traj is not None:
                px_pred, py_pred = opt_traj
                points = [tuple(map(int, [x, y])) for x, y in zip(px_pred, py_pred)]
                for i in range(len(points) - 1):
                    # Red line matching the red dashed trajectory in test_drift
                    cv2.line(preview, points[i], points[i+1], (0, 0, 255), 2, cv2.LINE_AA)
        self.draw_lidar_feedback(preview, center)
        if self.config.virtual_vehicle_test:
            self.draw_drift_style_hud(preview)
        else:
            self.draw_controller_debug_panel(preview)
        self.draw_free_space_interval(preview)
        self.draw_ftg_debug(preview, center)

    def draw_dclf_dcbf_visuals(self, preview, center, target):
        """Draw the DCLF-DCBF specific overlays."""
        if center is None:
            return
        
        # 1. Draw DCBF dashed neon ellipse
        color = (255, 0, 255) if self.cbf_active else (255, 255, 0)
        if self.cbf_qp_status == 'infeasible' or self.cbf_qp_h < 0.0:
            color = (0, 0, 255)
        a_ell_px, b_ell_px = self.controller.cbf_ellipse_axes_px()
        marker_scale_px = self.controller.cbf_ellipse_marker_scale_px()
        heading_rad = float(self.last_marker_heading)
        
        num_segments = 36
        pts = []
        for i in range(num_segments):
            theta = 2.0 * math.pi * i / num_segments
            x_local = a_ell_px * math.cos(theta)
            y_local = b_ell_px * math.sin(theta)
            x_rot = x_local * math.cos(heading_rad) - y_local * math.sin(heading_rad)
            y_rot = x_local * math.sin(heading_rad) + y_local * math.cos(heading_rad)
            pts.append((int(round(center[0] + x_rot)), int(round(center[1] + y_rot))))
        
        for i in range(num_segments):
            if i % 2 == 0:
                cv2.line(preview, pts[i], pts[(i + 1) % num_segments], color, 2, cv2.LINE_AA)
                
        label_origin = center + np.array([a_ell_px + 8.0, b_ell_px + 12.0])
        label_text = (
            f'DCBF a={self.config.cbf_a_ell:.2f} b={self.config.cbf_b_ell:.2f} '
            f'scale={self.format_distance_m(marker_scale_px)} -> '
            f'{self.format_distance_m(a_ell_px)}x{self.format_distance_m(b_ell_px)}'
        )
        draw_label(
            preview,
            tuple(label_origin.astype(int)),
            label_text,
            color,
            scale=0.42,
        )
        
        # 2. Draw DCLF Lyapunov convergence corridor
        if target is not None:
            overlay = preview.copy()
            for r_factor in [1.0, 2.0, 3.0]:
                cv2.ellipse(
                    overlay,
                    tuple(target.astype(int)),
                    (int(r_factor * 12), int(r_factor * 8)),
                    math.degrees(heading_rad),
                    0.0,
                    360.0,
                    (0, 255, 120),
                    1,
                    cv2.LINE_AA,
                )
            cv2.addWeighted(overlay, 0.4, preview, 0.6, 0.0, preview)
            
            cv2.line(
                preview,
                tuple(center.astype(int)),
                tuple(target.astype(int)),
                (0, 255, 120),
                2,
                cv2.LINE_AA,
            )
            midpoint = ((center + target) * 0.5).astype(int)
            draw_label(
                preview,
                tuple(midpoint + np.array([0, -10])),
                f'DCLF cte={self.latest_lateral_error_px:.1f}px',
                (0, 255, 120),
            )

    def draw_progress_debug(self, preview):
        """Annotate nearest point on the track."""
        if self.track_points is None:
            return
        if self.latest_nearest_index is not None:
            nearest = self.track_points[self.latest_nearest_index]
            cv2.circle(preview, tuple(nearest.astype(int)), 5, (255, 255, 255), -1)
            draw_label(
                preview,
                tuple(nearest.astype(int) + np.array([8, -8])),
                f'n={self.latest_nearest_index}',
                (255, 255, 255),
            )

    def draw_marker_trail(self, preview):
        """Draw recent marker centers with fading intensity."""
        if len(self.marker_trail) < 2:
            return
        count = len(self.marker_trail)
        for index in range(1, count):
            ratio = index / float(max(1, count - 1))
            color = (int(60 + 120 * ratio), int(80 + 120 * ratio), 255)
            cv2.line(
                preview,
                tuple(self.marker_trail[index - 1].astype(int)),
                tuple(self.marker_trail[index].astype(int)),
                color,
                2,
                cv2.LINE_AA,
            )

    def draw_virtual_vehicle_trail(self, preview):
        """Draw recent virtual vehicle centers with fading intensity."""
        if not hasattr(self, 'virtual_vehicle_trail') or len(self.virtual_vehicle_trail) < 2:
            return
        count = len(self.virtual_vehicle_trail)
        for index in range(1, count):
            ratio = index / float(max(1, count - 1))
            # Beautiful deep blue trail to match test_drift body/path style
            color = (int(188 * ratio), int(114 * ratio), int(0 * ratio))
            cv2.line(
                preview,
                tuple(self.virtual_vehicle_trail[index - 1].astype(int)),
                tuple(self.virtual_vehicle_trail[index].astype(int)),
                color,
                2,
                cv2.LINE_AA,
            )

    def draw_drift_style_hud(self, preview):
        """Draw clean, modern HUD indicators mimicking the test_drift style."""
        height, width = preview.shape[:2]
        
        # 1. Semi-transparent background box at the top left
        hud_width = 320
        hud_height = 95
        hx = max(12, width - hud_width - 12)
        hy = 12
        overlay = preview.copy()
        cv2.rectangle(overlay, (hx, hy), (hx + hud_width, hy + hud_height), (30, 30, 30), -1)
        cv2.addWeighted(overlay, 0.65, preview, 0.35, 0.0, preview)
        cv2.rectangle(preview, (hx, hy), (hx + hud_width, hy + hud_height), (100, 100, 100), 1, cv2.LINE_AA)

        # 2. Get current state metrics
        speed = self.track_speed_pps
        target_speed = self.effective_target_track_speed_pps
        
        # Calculate safety/barrier ratio (h / h_px)
        h_val = getattr(self, 'cbf_qp_h', 1.0)
        h_threshold = getattr(self.config, 'cbf_h_px', 42.0)
        h_ratio = np.clip(h_val / max(1.0, h_threshold), 0.0, 1.0)
        
        # Mode title
        is_shield_active = getattr(self, 'cbf_active', False) or getattr(self, 'cbf_qp_brake_gate_active', False)
        mode_str = "SAFE SHIELD" if is_shield_active else "NOMINAL"
        mode_color = (0, 0, 255) if is_shield_active else (0, 255, 0)
        
        # Collision status
        collisions = getattr(self.metrics, 'collision_samples', 0)
        coll_str = "COLLISION!" if collisions > 0 else "CLEAN"
        coll_color = (0, 0, 255) if collisions > 0 else (0, 255, 0)

        # 3. Draw text lines
        # Line 1: Mode & Collision Status
        draw_label(preview, (hx + 10, hy + 20), f"MODE: ", (255, 255, 255), scale=0.42)
        draw_label(preview, (hx + 60, hy + 20), mode_str, mode_color, scale=0.42)
        draw_label(preview, (hx + 180, hy + 20), f"COLL: ", (255, 255, 255), scale=0.42)
        draw_label(preview, (hx + 230, hy + 20), coll_str, coll_color, scale=0.42)
        
        # 4. Draw Velocity Indicator Bar
        vel_ratio = np.clip(speed / max(1.0, target_speed), 0.0, 1.0)
        cv2.putText(preview, "V:", (hx + 10, hy + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        # Bar background
        cv2.rectangle(preview, (hx + 70, hy + 36), (hx + hud_width - 15, hy + 46), (60, 60, 60), -1)
        # Filled bar (BGR blue-green matching drifting_car body_color)
        fill_w = int(vel_ratio * (hud_width - 85))
        if fill_w > 0:
            cv2.rectangle(preview, (hx + 70, hy + 36), (hx + 70 + fill_w, hy + 46), (188, 114, 0), -1)

        # 5. Draw Safety Margin (CBF h) Indicator Bar
        cv2.putText(preview, "Safety:", (hx + 10, hy + 75), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(preview, (hx + 70, hy + 66), (hx + hud_width - 15, hy + 76), (60, 60, 60), -1)
        # Fill color transitions from green to red based on ratio
        cbf_color = (0, int(255 * h_ratio), int(255 * (1.0 - h_ratio)))
        fill_h_w = int(h_ratio * (hud_width - 85))
        if fill_h_w > 0:
            cv2.rectangle(preview, (hx + 70, hy + 66), (hx + 70 + fill_h_w, hy + 76), cbf_color, -1)

    def draw_heading_debug(self, preview, center):
        """Draw marker heading and signed vehicle-right steering axis."""
        heading_vec = np.array(
            [math.cos(self.last_marker_heading), math.sin(self.last_marker_heading)],
            dtype=np.float32,
        )
        right_vec = np.array(
            [-math.sin(self.last_marker_heading), math.cos(self.last_marker_heading)],
            dtype=np.float32,
        )
        draw_vector(preview, center, heading_vec, 70.0, (0, 255, 0), 'heading')
        draw_vector(preview, center, right_vec, 55.0, (255, 170, 0), 'right')
        heading_deg = math.degrees(
            self.display_heading_error_rad(self.latest_heading_error_rad)
        )
        draw_label(
            preview,
            tuple(center.astype(int) + np.array([12, 24])),
            f'head_err={heading_deg:.1f}deg',
            (0, 255, 0),
        )

    def draw_velocity_debug(self, preview, center):
        """Draw measured and CBF-shaped target velocity vectors."""
        forward = np.array(
            [math.cos(self.last_marker_heading), math.sin(self.last_marker_heading)],
            dtype=np.float32,
        )
        right = np.array(
            [-math.sin(self.last_marker_heading), math.cos(self.last_marker_heading)],
            dtype=np.float32,
        )
        actual_speed = float(self.track_speed_pps)
        target_speed = float(self.target_track_speed_pps)
        effective_speed = float(self.effective_target_track_speed_pps)
        speed_scale = 1.6
        actual_length = max(-130.0, min(130.0, actual_speed * speed_scale))
        target_length = max(0.0, min(130.0, effective_speed * speed_scale))
        raw_target_length = max(0.0, min(130.0, target_speed * speed_scale))

        actual_origin = center - right * 18.0
        target_origin = center + right * 18.0
        raw_origin = center + right * 34.0
        actual_color = (255, 100, 20) if actual_speed >= 0.0 else (80, 80, 255)
        draw_vector(
            preview,
            actual_origin,
            forward if actual_speed >= 0.0 else -forward,
            abs(actual_length),
            actual_color,
            f'v {self.format_speed_mps(actual_speed)}',
        )
        draw_vector(
            preview,
            target_origin,
            forward,
            target_length,
            (0, 255, 255),
            f'v_ref {self.format_speed_mps(effective_speed)}',
        )
        if abs(raw_target_length - target_length) > 2.0:
            draw_vector(
                preview,
                raw_origin,
                forward,
                raw_target_length,
                (160, 160, 160),
                f'raw {self.format_speed_mps(target_speed)}',
            )

    def draw_safety_radius_debug(self, preview, center):
        """Draw vehicle-centered safety and forward slow-down regions."""
        cbf_radius = max(1, int(round(self.config.cbf_h_px)))
        color = (0, 220, 0) if self.cbf_scale > 0.75 else (0, 140, 255)
        if self.cbf_scale < 0.25:
            color = (0, 0, 255)
        cv2.circle(
            preview,
            tuple(center.astype(int)),
            cbf_radius,
            color,
            2,
            cv2.LINE_AA,
        )
        draw_label(
            preview,
            tuple(center.astype(int) + np.array([cbf_radius + 8, -8])),
            f'cbf_h {self.format_distance_m(self.config.cbf_h_px)}',
            color,
            scale=0.42,
        )

        forward = np.array(
            [math.cos(self.last_marker_heading), math.sin(self.last_marker_heading)],
            dtype=np.float32,
        )
        right = np.array(
            [-math.sin(self.last_marker_heading), math.cos(self.last_marker_heading)],
            dtype=np.float32,
        )
        a_ell_px, b_ell_px = self.controller.cbf_ellipse_axes_px()
        slow_length = max(1.0, float(a_ell_px))
        slow_half_width = max(1.0, float(b_ell_px))
        box_center = center + forward * slow_length * 0.5
        polygon = self.oriented_box_polygon(
            box_center,
            forward,
            right,
            slow_length * 0.5,
            slow_half_width,
        )
        cv2.polylines(
            preview,
            [polygon.astype(np.int32)],
            True,
            (80, 220, 255),
            1,
            cv2.LINE_AA,
        )
        draw_label(
            preview,
            tuple((box_center + right * (slow_half_width + 8.0)).astype(int)),
            'slow zone',
            (80, 220, 255),
            scale=0.42,
        )

    def draw_free_space_interval(self, preview):
        """Draw the selected lateral free interval at the lookahead point."""
        if (
            self.latest_target_index is None
            or self.latest_free_space_interval is None
            or self.road_normals is None
        ):
            return
        path_index = self.latest_target_index
        center = self.track_points[path_index]
        normal = self.road_normals[path_index]
        lower, upper = self.latest_free_space_interval
        start = center + normal * lower
        end = center + normal * upper
        cv2.line(
            preview,
            tuple(start.astype(int)),
            tuple(end.astype(int)),
            (255, 255, 0),
            4,
            cv2.LINE_AA,
        )

    def draw_ftg_debug(self, preview, center):
        """Draw Follow-the-Gap specific debug geometry (safety bubble, threat, and costmap)."""
        ftg_debug = getattr(self.controller, 'latest_ftg_debug', None)
        if not ftg_debug or center is None:
            return

        origin = ftg_debug.get('origin')
        if origin is None:
            origin = center
        origin = np.asarray(origin, dtype=np.float32)
        heading = float(ftg_debug.get('heading', self.last_marker_heading))

        # Draw the costmap if available
        costs = ftg_debug.get('costs')
        ranges = ftg_debug.get('ranges')
        angles = ftg_debug.get('angles')
        if costs is not None and ranges is not None and angles is not None:
            valid_costs = costs[np.isfinite(costs)]
            if len(valid_costs) > 0:
                min_cost = float(np.min(valid_costs))
                max_cost = float(np.max(valid_costs))
                cost_range = max_cost - min_cost if max_cost > min_cost else 1.0

                for i in range(len(ranges)):
                    angle = angles[i]
                    rng = ranges[i]
                    cost = costs[i]

                    # Compute BGR color based on cost
                    if not np.isfinite(cost):
                        color = (0, 0, 200)  # Red for blocked
                    else:
                        t = (cost - min_cost) / cost_range
                        if t < 0.5:
                            u = t * 2.0
                            color = (0, 255, int(u * 255))  # Green to Yellow
                        else:
                            u = (t - 0.5) * 2.0
                            color = (0, int((1.0 - u) * 255), 255)  # Yellow to Red

                    # Calculate end point of the lidar ray
                    global_angle = heading + angle
                    ray_end = origin + rng * np.array([math.cos(global_angle), math.sin(global_angle)], dtype=np.float32)
                    rx, ry = int(round(ray_end[0])), int(round(ray_end[1]))
                    ox, oy = int(round(origin[0])), int(round(origin[1]))

                    # Draw a thin line representing the lidar ray
                    cv2.line(preview, (ox, oy), (rx, ry), color, 1, cv2.LINE_AA)
                    # Draw a small dot at the end of the ray
                    cv2.circle(preview, (rx, ry), 2, color, -1, cv2.LINE_AA)

        nearest_point = ftg_debug.get('nearest_point')
        if nearest_point is not None:
            nx, ny = int(round(nearest_point[0])), int(round(nearest_point[1]))
            cv2.line(preview, (nx - 8, ny - 8), (nx + 8, ny + 8), (0, 140, 255), 2, cv2.LINE_AA)
            cv2.line(preview, (nx - 8, ny + 8), (nx + 8, ny - 8), (0, 140, 255), 2, cv2.LINE_AA)
            cv2.circle(preview, (nx, ny), 5, (0, 140, 255), -1, cv2.LINE_AA)
            draw_label(preview, (nx + 10, ny - 10), "Threat", (0, 140, 255), scale=0.42)
            bubble_radius = ftg_debug.get('bubble_radius', 0.0)
            if bubble_radius > 0.0:
                cv2.circle(
                    preview,
                    (nx, ny),
                    int(round(bubble_radius)),
                    (0, 100, 255),  # Safety bubble color
                    1,
                    cv2.LINE_AA
                )

        target = ftg_debug.get('target')
        if target is not None:
            tx, ty = int(round(target[0])), int(round(target[1]))
            cv2.circle(preview, (tx, ty), 5, (0, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(preview, (tx, ty), 10, (0, 0, 255), 2, cv2.LINE_AA)
            draw_label(preview, (tx + 12, ty - 12), "Raw FTG", (0, 0, 255), scale=0.42)

        scaled_target = ftg_debug.get('scaled_target')
        if scaled_target is not None:
            sx, sy = int(round(scaled_target[0])), int(round(scaled_target[1]))
            cv2.circle(preview, (sx, sy), 7, (255, 0, 255), -1, cv2.LINE_AA)
            cv2.circle(preview, (sx, sy), 14, (255, 0, 255), 2, cv2.LINE_AA)
            cv2.line(
                preview,
                tuple(origin.astype(int)),
                (sx, sy),
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            draw_label(preview, (sx + 12, sy - 12), "Steer target", (255, 0, 255), scale=0.42)

        target_angle = math.degrees(float(ftg_debug.get('target_angle', 0.0)))
        target_dist = float(ftg_debug.get('target_dist', 0.0))
        max_range = float(ftg_debug.get('max_range_cap', 0.0))
        if self.config.virtual_vehicle_test:
            text_origin = np.array([16, preview.shape[0] - 18], dtype=np.int32)
        else:
            text_origin = origin.astype(int) + np.array([10, 36])
        draw_label(
            preview,
            tuple(text_origin),
            f'ang={target_angle:.1f}deg dist={target_dist:.0f}px range={max_range:.0f}px',
            (255, 180, 60),
            scale=0.42,
        )

    def draw_obstacle_debug(self, preview, center):
        """Annotate static obstacles with progress and clearance."""
        obstacles = self.random_static_obstacles or self.static_obstacles
        if not obstacles:
            return
        for index, obstacle in enumerate(obstacles):
            obstacle_center = obstacle['center']
            detected = obstacle.get('detected', True)
            color = (80, 80, 255) if detected else (120, 120, 120)
            cv2.circle(preview, tuple(obstacle_center.astype(int)), 4, color, -1)
            prefix = 'seen' if detected else 'hidden'
            label = f'{prefix}{index} p={obstacle.get("progress", 0.0):.2f}'
            if center is not None and detected:
                clearance = self.controller.static_obstacle_clearance_px(center)
                label += f' clr={self.format_distance_m(clearance)}'
                self.draw_inflated_obstacle_zone(preview, obstacle, detected)
            draw_label(
                preview,
                tuple(obstacle_center.astype(int) + np.array([8, -8])),
                label,
                color,
                scale=0.45,
            )

    def draw_inflated_obstacle_zone(self, preview, obstacle, detected=True):
        """Draw the pixel-space obstacle zone used by CBF/lidar clearance."""
        margin = max(self.config.obstacle_margin_px, self.config.cbf_h_px)
        polygon = self.oriented_box_polygon(
            obstacle['center'],
            obstacle['tangent'],
            obstacle['normal'],
            obstacle['half_length'] + margin,
            obstacle['half_width'] + margin,
        )
        color = (0, 120, 255) if detected else (130, 130, 130)
        cv2.polylines(
            preview,
            [polygon.astype(np.int32)],
            True,
            color,
            2,
            cv2.LINE_AA,
        )
        edge_mid = obstacle['center'] + obstacle['normal'] * (
            obstacle['half_width'] + margin
        )
        draw_label(
            preview,
            tuple(edge_mid.astype(int) + np.array([6, -6])),
            f'safe {self.format_distance_m(margin)}',
            color,
            scale=0.42,
        )

    def draw_cbf_ellipse_debug(self, preview, center):
        """Draw the active QP CBF envelope around the car."""
        if center is None:
            return
        color = (120, 170, 220)
        if self.cbf_active:
            color = (0, 165, 255)
        if self.cbf_qp_status == 'infeasible' or self.cbf_qp_h < 0.0:
            color = (0, 0, 255)
        a_ell_px, b_ell_px = self.controller.cbf_ellipse_axes_px()
        marker_scale_px = self.controller.cbf_ellipse_marker_scale_px()
        angle_deg = math.degrees(float(self.last_marker_heading))
        axes = (
            max(1, int(round(a_ell_px))),
            max(1, int(round(b_ell_px))),
        )
        cv2.ellipse(
            preview,
            tuple(center.astype(int)),
            axes,
            angle_deg,
            0.0,
            360.0,
            color,
            2,
            cv2.LINE_AA,
        )
        label_origin = center + np.array([a_ell_px + 8.0, b_ell_px + 12.0])
        draw_label(
            preview,
            tuple(label_origin.astype(int)),
            (
                f'QP a={self.config.cbf_a_ell:.2f} b={self.config.cbf_b_ell:.2f} '
                f'scale={self.format_distance_m(marker_scale_px)} -> '
                f'{self.format_distance_m(a_ell_px)}x{self.format_distance_m(b_ell_px)}'
            ),
            color,
            scale=0.42,
        )

    def marker_scale_m_per_px(self):
        """Return local meters-per-pixel from the detected 10 cm ArUco marker."""
        marker_size_px = self.controller.cbf_ellipse_marker_scale_px()
        if marker_size_px <= 1e-6:
            return None
        marker_size_m = float(self.config.aruco_marker_size_cm) / 100.0
        return marker_size_m / marker_size_px

    def distance_m(self, distance_px):
        """Convert an image-space distance to local meters using ArUco scale."""
        scale = self.marker_scale_m_per_px()
        if scale is None or not math.isfinite(float(distance_px)):
            return None
        return float(distance_px) * scale

    def format_distance_m(self, distance_px):
        """Format a pixel distance as meters for debug overlays."""
        distance_m = self.distance_m(distance_px)
        if distance_m is None:
            return 'n/a m'
        return f'{distance_m:.3f}m'

    def format_speed_mps(self, speed_pps):
        """Format an image-space speed as meters per second for debug overlays."""
        speed_mps = self.distance_m(speed_pps)
        if speed_mps is None:
            return 'n/a m/s'
        return f'{speed_mps:.3f}m/s'

    @staticmethod
    def oriented_box_polygon(center, tangent, normal, half_length, half_width):
        """Return an oriented rectangle polygon for debug overlays."""
        return np.array(
            [
                center + tangent * half_length + normal * half_width,
                center - tangent * half_length + normal * half_width,
                center - tangent * half_length - normal * half_width,
                center + tangent * half_length - normal * half_width,
            ],
            dtype=np.float32,
        )

    def closest_point_on_cbf_ellipse(self, center, point):
        """Return the closest point on the drawn CBF ellipse to an image point."""
        center = np.asarray(center, dtype=np.float32)
        point = np.asarray(point, dtype=np.float32)
        a_ell_px, b_ell_px = self.controller.cbf_ellipse_axes_px()
        a_ell_px = max(1.0, float(a_ell_px))
        b_ell_px = max(1.0, float(b_ell_px))
        heading = float(self.last_marker_heading)
        forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
        lateral = np.array([-math.sin(heading), math.cos(heading)], dtype=np.float32)
        delta = point - center
        local = np.array(
            [float(np.dot(delta, forward)), float(np.dot(delta, lateral))],
            dtype=np.float64,
        )
        closest_local = self.closest_point_on_axis_aligned_ellipse(
            local,
            a_ell_px,
            b_ell_px,
        )
        closest_world = (
            center
            + forward * float(closest_local[0])
            + lateral * float(closest_local[1])
        )
        distance_px = float(np.linalg.norm(point - closest_world))
        ellipse_value = (
            (local[0] / a_ell_px) ** 2
            + (local[1] / b_ell_px) ** 2
        )
        if ellipse_value < 1.0:
            distance_px = -distance_px
        return closest_world.astype(np.float32), distance_px

    @staticmethod
    def closest_point_on_axis_aligned_ellipse(point, a_axis, b_axis):
        """Approximate the closest boundary point on an axis-aligned ellipse."""
        point = np.asarray(point, dtype=np.float64)
        if np.linalg.norm(point) <= 1e-9:
            if a_axis <= b_axis:
                return np.array([a_axis, 0.0], dtype=np.float64)
            return np.array([0.0, b_axis], dtype=np.float64)

        px, py = float(point[0]), float(point[1])
        theta = math.atan2(a_axis * py, b_axis * px)
        a_sq = a_axis * a_axis
        b_sq = b_axis * b_axis
        diff_sq = a_sq - b_sq
        for _step in range(8):
            sin_t = math.sin(theta)
            cos_t = math.cos(theta)
            gradient = (
                diff_sq * sin_t * cos_t
                - a_axis * px * sin_t
                + b_axis * py * cos_t
            )
            hessian = (
                diff_sq * (cos_t * cos_t - sin_t * sin_t)
                - a_axis * px * cos_t
                - b_axis * py * sin_t
            )
            if abs(hessian) <= 1e-9:
                break
            theta -= gradient / hessian
        return np.array(
            [a_axis * math.cos(theta), b_axis * math.sin(theta)],
            dtype=np.float64,
        )

    def draw_controller_debug_panel(self, preview):
        """Draw compact CBF/QP diagnostics."""
        height, width = preview.shape[:2]
        panel_width = 330
        panel_height = 170
        x = max(12, width - panel_width - 12)
        y = 12
        overlay = preview.copy()
        cv2.rectangle(
            overlay,
            (x, y),
            (x + panel_width, y + panel_height),
            (25, 25, 25),
            -1,
        )
        cv2.addWeighted(overlay, 0.55, preview, 0.45, 0.0, preview)
        cv2.rectangle(
            preview,
            (x, y),
            (x + panel_width, y + panel_height),
            self.cbf_debug_color(),
            1,
            cv2.LINE_AA,
        )
        speed_text = self.format_speed_mps(self.track_speed_pps)
        target_speed_text = self.format_speed_mps(
            self.effective_target_track_speed_pps
        )
        if self.config.controller_mode == 'pid_velocity_dclf_dcbf':
            mode_title = 'DCLF-DCBF'
        elif self.config.controller_mode == 'mpc_cbf':
            mode_title = 'MPC-CBF'
        else:
            mode_title = 'CBF'
        lines = [
            f'{mode_title} {self.cbf_qp_status} active={int(self.cbf_active)}',
            f'spd {speed_text}/{target_speed_text}',
            f'h {self.cbf_qp_h:.3f}  hd {self.cbf_qp_h_dot:.3f}  hdd {self.cbf_qp_h_ddot:.3f}',
            f'rhs {self.cbf_qp_rhs:.3f}',
            f'a {self.cbf_qp_accel:.4f}  d {self.cbf_qp_delta:.3f}  gate {int(self.cbf_qp_brake_gate_active)}',
            f'COLL {self.metrics.collision_samples}  {mode_title} {self.metrics.cbf_interventions}',
        ]
        for index, line in enumerate(lines):
            draw_label(
                preview,
                (x + 10, y + 24 + index * 22),
                line,
                (255, 255, 255),
                scale=0.46,
            )
        self.draw_horizontal_meter(
            preview,
            x + 10,
            y + panel_height - 18,
            panel_width - 20,
            self.cbf_debug_meter_ratio(),
            self.cbf_debug_color(),
        )

    def cbf_qp_constraint_margin(self):
        """Return lhs-rhs for the displayed closest-obstacle CBF constraint."""
        if self.cbf_qp_status == 'unused':
            return None
        lhs = (
            self.cbf_qp_lhs_a * self.cbf_qp_accel
            + self.cbf_qp_lhs_delta * self.cbf_qp_delta
            + self.cbf_qp_slack
        )
        return lhs - self.cbf_qp_rhs

    def cbf_debug_color(self):
        """Return panel color for current CBF state."""
        margin = self.cbf_qp_constraint_margin()
        if self.cbf_qp_status == 'infeasible' or self.cbf_qp_h < 0.0:
            return (0, 0, 255)
        if margin is not None and margin < 0.0:
            return (0, 140, 255)
        if self.cbf_active:
            return (0, 200, 255)
        return (0, 220, 0)

    def cbf_debug_meter_ratio(self):
        """Return a bounded health value for the CBF panel meter."""
        if self.cbf_qp_status == 'unused':
            return 1.0
        margin = self.cbf_qp_constraint_margin()
        if margin is None:
            return 1.0
        return max(0.0, min((float(margin) + 1.0) / 2.0, 1.0))

    def draw_qp_velocity_debug(self, preview, center):
        """Draw the QP acceleration result as a short velocity tendency arrow."""
        if center is None or self.cbf_qp_status == 'unused':
            return
        forward = np.array(
            [math.cos(self.last_marker_heading), math.sin(self.last_marker_heading)],
            dtype=np.float32,
        )
        accel = float(self.cbf_qp_accel)
        if abs(accel) < 1e-6:
            return
        length = max(18.0, min(95.0, abs(accel) * 90.0))
        color = (0, 230, 0) if accel > 0.0 else (0, 0, 255)
        draw_vector(
            preview,
            center + np.array([0.0, 42.0], dtype=np.float32),
            forward if accel > 0.0 else -forward,
            length,
            color,
            f'qp_a {accel:.2f}',
        )

    def velocity_debug_scale(self):
        """Return effective target speed divided by the requested target speed."""
        target = float(self.target_track_speed_pps)
        if abs(target) <= 1e-6:
            return 0.0
        return max(0.0, min(float(self.effective_target_track_speed_pps) / target, 1.0))

    def draw_virtual_motion_debug(self, preview):
        """Draw virtual vehicle velocity and predicted short-horizon pose."""
        if self.virtual_vehicle_state is None:
            return
        state = self.virtual_vehicle_state
        center = np.array([state['x'], state['y']], dtype=np.float32)
        heading = state['heading']
        speed = state['speed']
        forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
        velocity_tip = center + forward * max(20.0, min(120.0, speed * 1.5))
        cv2.arrowedLine(
            preview,
            tuple(center.astype(int)),
            tuple(velocity_tip.astype(int)),
            (255, 80, 20),
            2,
            cv2.LINE_AA,
            tipLength=0.25,
        )
        horizon = 1.0
        delta = self.controller.roll_pwm_to_qp_delta(self.roll)
        predicted_heading = heading
        predicted_center = center.copy()
        for _step in range(10):
            dt = horizon / 10.0
            predicted_heading += speed / max(1.0, self.config.qp_wheelbase_px) * delta * dt
            predicted_center += np.array(
                [math.cos(predicted_heading), math.sin(predicted_heading)],
                dtype=np.float32,
            ) * speed * dt
            cv2.circle(
                preview,
                tuple(predicted_center.astype(int)),
                2,
                (255, 80, 20),
                -1,
            )

    @staticmethod
    def draw_horizontal_meter(preview, x, y, width, ratio, color):
        """Draw a bounded 0..1 meter used by debug panels."""
        bounded_ratio = max(0.0, min(float(ratio), 1.0))
        cv2.rectangle(preview, (x, y), (x + width, y + 8), (70, 70, 70), -1)
        cv2.rectangle(
            preview,
            (x, y),
            (x + int(round(width * bounded_ratio)), y + 8),
            color,
            -1,
        )
        cv2.rectangle(preview, (x, y), (x + width, y + 8), (230, 230, 230), 1)

    def draw_safety_bars(self, preview):
        """Draw compact CBF clearance bars in the lower-left corner."""
        if not self.latest_safety_clearances:
            return
        height, _width = preview.shape[:2]
        labels = ('edge', 'lateral', 'heading', 'obstacle', 'min')
        x = 12
        y = max(24, height - 28 * len(labels) - 12)
        max_clearance = max(1.0, self.cbf_stop_error_px)
        for index, label in enumerate(labels):
            value = self.latest_safety_clearances.get(label, float('inf'))
            bar_y = y + index * 28
            if math.isfinite(value):
                ratio = max(0.0, min((value + 30.0) / (max_clearance + 30.0), 1.0))
                color = (0, 220, 0) if value >= 0.0 else (0, 0, 255)
                text_value = f'{value:.0f}'
            else:
                ratio = 1.0
                color = (180, 180, 180)
                text_value = 'inf'
            cv2.rectangle(preview, (x, bar_y), (x + 150, bar_y + 16), (20, 20, 20), -1)
            cv2.rectangle(
                preview,
                (x, bar_y),
                (x + int(round(150 * ratio)), bar_y + 16),
                color,
                -1,
            )
            draw_label(
                preview,
                (x + 160, bar_y + 13),
                f'{label}:{text_value}',
                color,
                scale=0.48,
            )

    def draw_lidar_feedback(self, preview, center):
        """Draw only the CBF-selected obstacle point."""
        if center is None:
            return
        critical_x = getattr(self.controller, 'cbf_qp_obstacle_x', None)
        critical_y = getattr(self.controller, 'cbf_qp_obstacle_y', None)
        if critical_x is not None and critical_y is not None:
            xobs = np.array([float(critical_x), float(critical_y)], dtype=np.float32)
            closest, distance_px = self.closest_point_on_cbf_ellipse(center, xobs)
            distance_color = (0, 255, 255) if distance_px >= 0.0 else (0, 0, 255)
            crit_hit = (int(round(critical_x)), int(round(critical_y)))
            closest_hit = tuple(closest.astype(int))
            cv2.line(
                preview,
                closest_hit,
                crit_hit,
                distance_color,
                2,
                cv2.LINE_AA,
            )
            cv2.circle(preview, closest_hit, 5, distance_color, -1, cv2.LINE_AA)
            cv2.circle(preview, crit_hit, 6, (0, 255, 255), -1)
            cv2.circle(preview, crit_hit, 12, (0, 0, 255), 2, cv2.LINE_AA)
            label_anchor = ((closest + xobs) * 0.5 + np.array([8.0, -8.0])).astype(int)
            draw_label(
                preview,
                tuple(label_anchor),
                f'ell-xobs {self.format_distance_m(distance_px)}',
                distance_color,
                scale=0.45,
            )
            draw_label(
                preview,
                (crit_hit[0] + 12, crit_hit[1] - 8),
                f'CBF xobs ({critical_x:.0f},{critical_y:.0f})',
                (0, 255, 255),
                scale=0.45,
            )

    def road_scene_enabled(self):
        """Return whether the current track uses road-scene planning."""
        return self.controller.road_scene_enabled()

    def closed_road_scene_enabled(self):
        """Return whether the road scene should wrap around as a loop."""
        return self.controller.closed_road_scene_enabled()

    def publish_neutral_for(self, duration):
        """Continuously publish neutral output for a fixed duration."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.command_pub.publish(self.neutral_message())
            time.sleep(1.0 / UPDATE_RATE_HZ)

    def set_armed(self, armed):
        """Request the CRSF node arming state synchronously."""
        if not self.arming_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'service unavailable: {ARMING_SERVICE}')
        request = CommandBool.Request()
        request.value = armed
        future = self.arming_client.call_async(request)
        if self.executor is not None:
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
        else:
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('arming service did not respond')
        self.armed = armed
        if armed:
            self.flash_timer = 10
        print(future.result().data)


def main(args=None):
    """Run guarded ArUco virtual-track following."""
    config, ros_args = parse_args(args)
    validate_config(config)
    if not config.dry_run and not config.confirm_propulsion_safe:
        raise SystemExit(
            'refusing ArUco track following output: securely restrain driven '
            'wheels, then pass --confirm-propulsion-safe'
        )

    rclpy.init(args=ros_args)
    follower_class = ArucoTrackFollower
    if config.dynamic_obstacles:
        from .dynamic_straight import ArucoTrackFollower as DynamicArucoTrackFollower

        follower_class = DynamicArucoTrackFollower
    node = follower_class(config)
    validate_config(config)
    print_config(config)
    executor = None
    if not config.single_thread:
        executor = MultiThreadedExecutor(num_threads=config.executor_threads)
        executor.add_node(node)
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

    def spin_or_render_once():
        if executor is None:
            rclpy.spin_once(node, timeout_sec=0.02)
        else:
            time.sleep(0.02)
        if config.preview:
            node.update_pid_from_panel()
            node.render_pending_preview()
        if config.preview or node.tuning.enabled:
            node.poll_keyboard()

    try:
        if config.dry_run:
            if config.virtual_vehicle_test:
                print('Dry run: running virtual vehicle controller test.')
            else:
                print('Dry run: detecting marker without publishing RC output.')
            while rclpy.ok() and not node.stop_requested:
                spin_or_render_once()
            return

        time.sleep(0.2)
        if not getattr(config, 'ignore_subscribers', False) and node.count_subscribers(config.command_topic) == 0:
            raise RuntimeError(
                f'no subscriber on {config.command_topic}; run crsf_ros first'
            )
        print('Waiting for ArUco marker before arming.')
        deadline = time.monotonic() + 10.0
        while not node.command_is_fresh() and time.monotonic() < deadline:
            spin_or_render_once()
        if not node.command_is_fresh():
            raise RuntimeError(
                f'no marker ID {config.marker_id} seen on {config.image_topic}'
            )

        print('Sending stop/center command before arming.')
        node.publish_neutral_for(SETTLE_DURATION)
        node.set_armed(True)
        node.output_enabled = True
        if node.command_is_fresh():
            node.command_pub.publish(node.drive_message())
        print('ArUco track follower armed; Ctrl-C stops and disarms.')
        while rclpy.ok() and not node.stop_requested:
            spin_or_render_once()
    except KeyboardInterrupt:
        print('\nInterrupted; stopping ArUco track follower.')
    finally:
        print('\nReturning to stop/center and disarming.')
        node.output_enabled = False
        try:
            if not config.dry_run:
                node.publish_neutral_for(SETTLE_DURATION)
            node.save_metrics_file()
        finally:
            try:
                if node.armed:
                    node.set_armed(False)
            finally:
                cv2.destroyAllWindows()
                if executor is not None:
                    executor.shutdown()
                    executor.remove_node(node)
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
