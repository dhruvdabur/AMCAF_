"""ROS node for the closed-ellipse ArUco hil follower."""

import json
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

from ..config.ellipse_static import ARMING_SERVICE
from ..config.ellipse_static import NEUTRAL_VALUE
from ..config.ellipse_static import parse_args
from ..config.ellipse_static import print_config
from ..config.ellipse_static import SETTLE_DURATION
from ..config.ellipse_static import UPDATE_RATE_HZ
from ..config.ellipse_static import validate_config
from ..control import EllipseStaticController
from ..runtime import make_sensor_qos
from ..tuning import EllipseStaticTuning
from ..ui import draw_label
from ..ui import draw_vector
from ..ui import put_status
from ..ui import resize_for_preview

PREVIEW_WINDOW = 'Aruco Track Follower'


class ArucoTrackFollower(Node):
    """ROS transport wrapper around the reusable ellipse follower controller."""

    def __init__(self, config):
        super().__init__('aruco_track_follower')
        cv2.setUseOptimized(True)
        self.config = config
        self.bridge = CvBridge()
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
        )
        self.arming_client = self.create_client(CommandBool, ARMING_SERVICE)
        self.aruco_dictionary = self.load_aruco_dictionary(config.marker_dict)
        self.aruco_parameters = self.make_detector_parameters()
        self.detector = self.make_detector()
        self.controller = EllipseStaticController(config)
        self.telemetry_publishers = self.create_tuning_telemetry_publishers()
        self.tuning = EllipseStaticTuning(
            config,
            self.controller,
            self.get_logger(),
            enabled=not config.no_pid_panel,
        )
        self.latest_preview_frame = None
        self.preview_window_ready = False
        self.output_enabled = False
        self.armed = False
        self.last_debug_print_time = 0.0
        if config.load_tuning:
            self.tuning.load_tuning_file()
            self.controller.refresh_cbf_qp_config()
        if self.tuning.enabled:
            self.tuning.create_pid_panel()
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.publish_command)

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
        if self.message_is_stale(image_msg):
            return
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        detection = self.detect_marker(frame)
        if detection is None:
            self.stop_immediately()
            self.publish_tuning_telemetry(marker_seen=False)
            if self.config.preview:
                self.show_preview(frame, None, None, None, None)
            return

        center, heading, corners = detection
        self.update_marker_scale(corners)
        self.tuning.update_pid_from_panel()
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
            self.debug_command(command, 'tracking')

        if self.config.preview:
            self.show_preview(frame, corners, center, result.target, result.tangent)

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
            self.debug_command(command, 'marker lost')

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
        import threading
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
        import threading
        if threading.current_thread() is threading.main_thread():
            self.poll_keyboard()
        if self.config.dry_run or not self.output_enabled:
            return
        if not self.command_is_fresh():
            command = self.neutral_message()
            self.command_pub.publish(command)
            self.debug_command(command, 'stale')
            return

        command = self.drive_message()
        self.command_pub.publish(command)
        self.debug_command(command, 'tracking')

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
        path = self.config.debug_snapshot_dir / f'ellipse_debug_{timestamp}.png'
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

    def show_preview(self, frame, corners, center, target, tangent):
        """Display the virtual track and current ArUco tracking state."""
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
            cv2.circle(preview, tuple(center.astype(int)), 5, (255, 0, 0), -1)
        if target is not None:
            cv2.circle(preview, tuple(target.astype(int)), 7, (0, 0, 255), -1)
        if self.config.debug_visuals:
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
        self.preview_window_ready = True

    def resize_preview_window(self, preview):
        """Force the preview window to the requested display size."""
        height, width = preview.shape[:2]
        target_width = width
        if self.config.preview_width > 0:
            target_width = max(width, self.config.preview_width)
        target_height = int(round(height * target_width / max(1, width)))
        cv2.resizeWindow(PREVIEW_WINDOW, target_width, target_height)

    def draw_road_scene(self, preview):
        """Draw the road scene and static obstacle field."""
        if self.road_boundaries:
            for boundary in self.road_boundaries:
                cv2.polylines(
                    preview,
                    [boundary.astype(np.int32)],
                    self.closed_road_scene_enabled(),
                    (180, 180, 180),
                    2,
                    cv2.LINE_AA,
                )
        for obstacle in self.static_obstacles:
            cv2.fillConvexPoly(
                preview,
                obstacle['polygon'].astype(np.int32),
                (40, 40, 220),
            )
            cv2.polylines(
                preview,
                [obstacle['polygon'].astype(np.int32)],
                True,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        if self.latest_free_space_target is not None:
            cv2.circle(
                preview,
                tuple(self.latest_free_space_target.astype(int)),
                6,
                (255, 255, 0),
                -1,
            )

    def draw_debug_visuals(self, preview, center, target, tangent):
        """Draw dense geometry and CBF debug overlays."""
        self.draw_progress_debug(preview)
        self.draw_marker_trail(preview)
        if center is not None:
            self.draw_heading_debug(preview, center)
        if center is not None and target is not None:
            cv2.line(
                preview,
                tuple(center.astype(int)),
                tuple(target.astype(int)),
                (0, 180, 255),
                2,
                cv2.LINE_AA,
            )
            midpoint = ((center + target) * 0.5).astype(int)
            draw_label(
                preview,
                tuple(midpoint),
                f'cte={self.latest_lateral_error_px:.1f}px',
                (0, 180, 255),
            )
        if target is not None and tangent is not None:
            draw_vector(preview, target, tangent, 70.0, (0, 255, 255), 'target')
        self.draw_cbf_ellipse_debug(preview, center)
        self.draw_controller_debug_panel(preview)
        self.draw_free_space_interval(preview)
        self.draw_safety_bars(preview)

    def draw_progress_debug(self, preview):
        """Annotate nearest and lookahead points on the track."""
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
        if self.latest_target_index is not None:
            target_center = self.track_points[self.latest_target_index]
            cv2.circle(
                preview,
                tuple(target_center.astype(int)),
                5,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            draw_label(
                preview,
                tuple(target_center.astype(int) + np.array([8, 14])),
                f'la={self.config.lookahead_points}',
                (0, 255, 255),
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

    def draw_cbf_ellipse_debug(self, preview, center):
        """Draw the marker-scaled QP CBF ellipse around the car."""
        if center is None:
            return
        color = (0, 165, 255) if self.cbf_active else (120, 170, 220)
        a_ell_px, b_ell_px = self.controller.cbf_ellipse_axes_px()
        a_ell_cm, b_ell_cm = self.controller.cbf_ellipse_axes_cm()
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
                f'{a_ell_px:.0f}x{b_ell_px:.0f}px '
                f'({a_ell_cm:.0f}x{b_ell_cm:.0f}cm)'
            ),
            color,
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
        center = self.track_points[self.latest_target_index]
        normal = self.road_normals[self.latest_target_index]
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
        draw_label(
            preview,
            tuple(((start + end) * 0.5).astype(int) + np.array([8, -10])),
            f'free={upper - lower:.0f}px',
            (255, 255, 0),
        )

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
            f'CBF {self.cbf_qp_status}',
            f'h {self.cbf_qp_h:.3f}  hd {self.cbf_qp_h_dot:.3f}  hdd {self.cbf_qp_h_ddot:.3f}',
            f'margin {margin_text}  rhs {self.cbf_qp_rhs:.3f}',
            f'a {self.cbf_qp_accel:.2f}  d {self.cbf_qp_delta:.2f}',
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
            rclpy.spin_once(self, timeout_sec=0.0)
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
    node = ArucoTrackFollower(config)
    validate_config(config)
    print_config(config)
    try:
        if config.dry_run:
            print('Dry run: detecting marker without publishing RC output.')
            while rclpy.ok() and not node.stop_requested:
                rclpy.spin_once(node, timeout_sec=0.05)
            return

        time.sleep(0.2)
        if not getattr(config, 'ignore_subscribers', False) and node.count_subscribers(config.command_topic) == 0:
            raise RuntimeError(
                f'no subscriber on {config.command_topic}; run crsf_ros first'
            )
        print('Waiting for ArUco marker before arming.')
        deadline = time.monotonic() + 10.0
        while not node.command_is_fresh() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
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
            rclpy.spin_once(node, timeout_sec=0.05)
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
                node.destroy_node()
                rclpy.try_shutdown()


if __name__ == '__main__':
    main()
