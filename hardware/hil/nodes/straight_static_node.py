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

from ..config.straight_static import ARMING_SERVICE
from ..config.straight_static import NEUTRAL_VALUE
from ..config.straight_static import parse_args
from ..config.straight_static import print_config
from ..config.straight_static import SETTLE_DURATION
from ..config.straight_static import UPDATE_RATE_HZ
from ..config.straight_static import validate_config
from ..control import StraightStaticController
from ..road import make_laneless_static_obstacle
from ..runtime import make_sensor_qos
from ..tuning import StraightStaticTuning
from ..ui import draw_label
from ..ui import draw_vector
from ..ui import noop
from ..ui import put_status
from ..ui import resize_for_preview
from ..ui import set_overlay_text_style

PREVIEW_WINDOW = 'Aruco Track Follower'


class ArucoTrackFollower(Node):
    """ROS transport wrapper around the reusable straight follower controller."""

    def __init__(self, config):
        super().__init__('aruco_track_follower')
        cv2.setUseOptimized(True)
        self.config = config
        self.bridge = CvBridge()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()
        self.command_callback_group = ReentrantCallbackGroup()
        self.command_pub = self.create_publisher(
            RCMessage,
            config.command_topic,
            10,
        )
        self.image_sub = self.create_subscription(
            Image,
            config.image_topic,
            self.image_callback,
            make_sensor_qos(),
            callback_group=self.control_callback_group,
        )
        self.arming_client = self.create_client(CommandBool, ARMING_SERVICE)
        self.aruco_dictionary = self.load_aruco_dictionary(config.marker_dict)
        self.aruco_parameters = self.make_detector_parameters()
        self.detector = self.make_detector()
        self.controller = self.make_controller(config)
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
        self.preview_text_controls_ready = False
        self.output_enabled = False
        self.armed = False
        self.last_debug_print_time = 0.0
        self.virtual_vehicle_state = None
        self.virtual_last_time = None
        self.virtual_road_tile_index = 0
        self.random_static_obstacles = []
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
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

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
            'roll_pwm': float(self.roll),
            'nominal_throttle_pwm': float(self.nominal_throttle),
            'throttle_pwm': float(self.throttle),
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
        }
        if result is not None:
            values.update(
                {
                    'marker_x_px': float(result.center[0]),
                    'marker_y_px': float(result.center[1]),
                    'lateral_error_px': float(result.lateral_error_px),
                    'free_space_lateral_target_px': float(
                        self.latest_free_space_lateral_target_px
                    ),
                    'heading_error_rad': self.display_heading_error_rad(
                        result.heading_error_rad
                    ),
                    'nominal_roll_pwm': float(self.nominal_roll),
                    'roll_pwm': float(result.roll_pwm),
                    'nominal_throttle_pwm': float(self.nominal_throttle),
                    'throttle_pwm': float(result.throttle_pwm),
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
        """Open live PID sliders for field tuning."""
        self.tuning.create_pid_panel()

    def read_panel_values(self):
        """Read live slider values without mutating controller state."""
        return self.tuning.read_panel_values()

    def apply_tuning_values(self, values):
        """Apply tuning values to current config and controller gains."""
        self.controller.apply_tuning_values(values)

    def update_pid_from_panel(self):
        """Read live PID sliders and update controller gains."""
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
        self.advance_virtual_vehicle(dt)
        self.debug_command(self.drive_message(), 'virtual')
        preview_target = result.target
        if self.config.virtual_unlimited_path:
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
        # Image coordinates use +y downward, so positive steering delta must
        # decrease the rendered heading to turn toward vehicle-left on screen.
        heading_rate = -state['speed'] / max(1.0, self.config.qp_wheelbase_px) * delta
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
        self.controller.track_points = self.track_points + delta
        if self.road_boundaries:
            self.controller.road_boundaries = [
                boundary + delta for boundary in self.road_boundaries
            ]
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
            length_px = float(rng.uniform(70.0, 125.0))
            width_px = float(rng.uniform(45.0, 76.0))
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

    def prepare_preview_window(self, preview):
        """Create a resizable preview window once."""
        if self.preview_window_ready:
            return
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
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
        """Force the preview window to the requested display size."""
        height, width = preview.shape[:2]
        target_width = width
        if self.config.preview_width > 0:
            target_width = max(width, self.config.preview_width)
        target_height = int(round(height * target_width / max(1, width)))
        cv2.resizeWindow(PREVIEW_WINDOW, target_width, target_height)

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
        polygon = np.array(
            [
                center + forward * half_length + right * half_width,
                center - forward * half_length + right * half_width,
                center - forward * half_length - right * half_width,
                center + forward * half_length - right * half_width,
            ],
            dtype=np.int32,
        )
        shadow = polygon + np.array([4, 5], dtype=np.int32)
        cv2.fillConvexPoly(preview, shadow, (185, 185, 185))
        cv2.fillConvexPoly(preview, polygon, (24, 134, 245))
        cv2.polylines(preview, [polygon], True, (18, 42, 74), 2, cv2.LINE_AA)
        cabin = np.array(
            [
                center + forward * 10.0 + right * 11.0,
                center - forward * 14.0 + right * 10.0,
                center - forward * 14.0 - right * 10.0,
                center + forward * 10.0 - right * 11.0,
            ],
            dtype=np.int32,
        )
        cv2.fillConvexPoly(preview, cabin, (255, 224, 166))
        cv2.polylines(preview, [cabin], True, (95, 85, 65), 1, cv2.LINE_AA)
        for wheel_offset in (
            forward * 19.0 + right * 19.0,
            forward * 19.0 - right * 19.0,
            -forward * 21.0 + right * 19.0,
            -forward * 21.0 - right * 19.0,
        ):
            wheel_center = center + wheel_offset
            cv2.circle(preview, tuple(wheel_center.astype(int)), 4, (32, 32, 32), -1)
        nose = center + forward * (half_length + 18.0)
        cv2.arrowedLine(
            preview,
            tuple(center.astype(int)),
            tuple(nose.astype(int)),
            (0, 58, 220),
            3,
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
        obstacles = self.random_static_obstacles or self.static_obstacles
        for obstacle in obstacles:
            if self.config.random_static_obstacles and not obstacle.get('detected'):
                fill_color = (174, 174, 174)
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
        )

    def draw_virtual_road_tiles(self, preview):
        """Draw repeated road tiles so unlimited virtual tests never show an end."""
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
        self.draw_marker_trail(preview)
        self.draw_cbf_ellipse_debug(preview, center)
        self.draw_controller_debug_panel(preview)
        self.draw_free_space_interval(preview)

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
            f'v {actual_speed:.1f}',
        )
        draw_vector(
            preview,
            target_origin,
            forward,
            target_length,
            (0, 255, 255),
            f'v_ref {effective_speed:.1f}',
        )
        if abs(raw_target_length - target_length) > 2.0:
            draw_vector(
                preview,
                raw_origin,
                forward,
                raw_target_length,
                (160, 160, 160),
                f'raw {target_speed:.1f}',
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
            f'cbf_h {self.config.cbf_h_px:.0f}px',
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
                label += f' clr={clearance:.0f}'
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
            f'safe {margin:.0f}px',
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
                f'scale={marker_scale_px:.1f}px -> '
                f'{a_ell_px:.0f}x{b_ell_px:.0f}px'
            ),
            color,
            scale=0.42,
        )

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

    def draw_controller_debug_panel(self, preview):
        """Draw compact CBF/QP diagnostics."""
        height, width = preview.shape[:2]
        panel_width = 300
        panel_height = 172
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
        margin = self.cbf_qp_constraint_margin()
        margin_text = 'n/a' if margin is None else f'{margin:.3f}'
        clear_text = (
            'inf'
            if not math.isfinite(self.nearest_static_clearance_px)
            else f'{self.nearest_static_clearance_px:.1f}px'
        )
        lines = [
            f'CBF {self.cbf_qp_status} active={int(self.cbf_active)}',
            f'h {self.cbf_qp_h:.3f}  hd {self.cbf_qp_h_dot:.3f}  hdd {self.cbf_qp_h_ddot:.3f}',
            f'margin {margin_text}  rhs {self.cbf_qp_rhs:.3f}',
            f'a {self.cbf_qp_accel:.4f}  d {self.cbf_qp_delta:.3f}  gate {int(self.cbf_qp_brake_gate_active)}',
            f'slack {self.cbf_qp_slack:.3f}  clear {clear_text}',
            f'COLL {self.metrics.collision_samples}  CBF {self.metrics.cbf_interventions}',
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
        """Draw virtual lidar returns from the marker/car center."""
        if not self.latest_lidar_points:
            return
        origin = tuple(center.astype(int))
        for point in self.latest_lidar_points:
            hit = (int(round(point.x_px)), int(round(point.y_px)))
            cv2.line(preview, origin, hit, (255, 0, 255), 1, cv2.LINE_AA)
            cv2.circle(preview, hit, 3, (255, 0, 255), -1)

        critical_x = getattr(self.controller, 'cbf_qp_obstacle_x', None)
        critical_y = getattr(self.controller, 'cbf_qp_obstacle_y', None)
        if critical_x is not None and critical_y is not None:
            crit_hit = (int(round(critical_x)), int(round(critical_y)))
            # Draw a thicker yellow line to the critical point
            cv2.line(preview, origin, crit_hit, (0, 255, 255), 2, cv2.LINE_AA)
            # Draw a larger yellow circle at the critical point
            cv2.circle(preview, crit_hit, 6, (0, 255, 255), -1)
            # Draw an outer red ring to highlight it
            cv2.circle(preview, crit_hit, 8, (0, 0, 255), 1, cv2.LINE_AA)
            # Add a small text label
            draw_label(preview, (crit_hit[0] + 10, crit_hit[1] - 5), "x_obs", (0, 255, 255), scale=0.45)

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
        from .dynamic_straight_node import ArucoTrackFollower as DynamicArucoTrackFollower

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
