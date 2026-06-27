#!/usr/bin/env python3
"""hil ArUco follower with a plain straight camera-frame road."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
from rc_msgs.msg import RCMessage
from rc_msgs.srv import CommandBool
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image

SOURCE_ROOT = Path(__file__).resolve().parents[2]
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

try:
    from crsf_ros2.hil.controllers import CONTROLLER_MODES
    from crsf_ros2.hil.controllers import PID
    from crsf_ros2.hil.controllers import PIDVelocityCBFController
    from crsf_ros2.hil.controllers import PIDController
    from crsf_ros2.hil.controllers import PID_VELOCITY_CBF
except ImportError:
    from controllers import CONTROLLER_MODES
    from controllers import PID
    from controllers import PIDVelocityCBFController
    from controllers import PIDController
    from controllers import PID_VELOCITY_CBF


IMAGE_TOPIC = '/image_raw'
COMMAND_TOPIC = '/drone/rc_command'
ARMING_SERVICE = '/drone/cmd/arming'
NEUTRAL_VALUE = 1500
UPDATE_RATE_HZ = 50.0
SETTLE_DURATION = 0.5
PID_WINDOW = 'Tuning Panel'
PID_SCALE = 1000
HEADING_SCALE = 10
VELOCITY_SCALE = 10
CBF_ERROR_SCALE = 1
CBF_ALPHA_SCALE = 100


def bounded(value, lower, upper):
    """Return value restricted to inclusive lower and upper limits."""
    return max(lower, min(value, upper))


def wrap_angle(angle):
    """Wrap an angle to -pi..pi."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def unique_metrics_path(path):
    """Return a timestamped metrics path so runs never overwrite."""
    timestamp = time.strftime('%Y%m%d_%H%M%S')
    return path.with_name(f'{path.stem}_{timestamp}{path.suffix}')


class FollowerMetrics:
    """Collect ArUco follower ranking metrics analogous to the 2D simulator."""

    def __init__(self):
        self.reset()

    def reset(self):
        """Clear accumulated metrics."""
        self.start_time = None
        self.last_time = None
        self.sample_count = 0
        self.abs_cte_sum_px = 0.0
        self.cte_square_sum_px = 0.0
        self.max_abs_cte_px = 0.0
        self.abs_heading_sum_rad = 0.0
        self.abs_speed_error_sum_pps = 0.0
        self.steering_effort_pwm_s = 0.0
        self.throttle_effort_pwm_s = 0.0
        self.min_safety_clearance_px = float('inf')
        self.collision_samples = 0
        self.cbf_interventions = 0
        self.laps_completed = 0
        self.previous_progress = None
        self.marker_lost_events = 0

    def update(
        self,
        now,
        cte_px,
        heading_error_rad,
        speed_error_pps,
        roll_pwm,
        throttle_pwm,
        safety_clearance_px,
        cbf_active,
        nearest_index,
        track_size,
        config,
    ):
        """Accumulate one detected-marker sample."""
        if self.start_time is None:
            self.start_time = now
        if self.last_time is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self.last_time)
        self.last_time = now

        abs_cte = abs(cte_px)
        self.sample_count += 1
        self.abs_cte_sum_px += abs_cte
        self.cte_square_sum_px += cte_px * cte_px
        self.max_abs_cte_px = max(self.max_abs_cte_px, abs_cte)
        self.abs_heading_sum_rad += abs(heading_error_rad)
        self.abs_speed_error_sum_pps += abs(speed_error_pps)
        self.steering_effort_pwm_s += (
            abs(roll_pwm - config.center_steering_pwm) * dt
        )
        self.throttle_effort_pwm_s += (
            abs(throttle_pwm - config.neutral_throttle_pwm) * dt
        )
        self.min_safety_clearance_px = min(
            self.min_safety_clearance_px,
            safety_clearance_px,
        )
        if safety_clearance_px < 0.0:
            self.collision_samples += 1
        if cbf_active:
            self.cbf_interventions += 1

        progress = nearest_index / float(track_size)
        if (
            self.previous_progress is not None
            and self.previous_progress > 0.8
            and progress < 0.2
        ):
            self.laps_completed += 1
        self.previous_progress = progress

    def marker_lost(self):
        """Count a transition into marker-lost state."""
        self.marker_lost_events += 1

    def summary(self):
        """Return aggregate metrics for display and JSON output."""
        if self.sample_count == 0:
            return {
                'samples': 0,
                'duration_s': 0.0,
                'laps_completed': 0,
                'mean_abs_cte_px': 0.0,
                'rmse_cte_px': 0.0,
                'max_abs_cte_px': 0.0,
                'mean_abs_heading_error_deg': 0.0,
                'mean_abs_speed_error_pps': 0.0,
                'steering_effort_pwm_s': 0.0,
                'throttle_effort_pwm_s': 0.0,
                'min_obstacle_clearance_px': 0.0,
                'collision_samples': 0,
                'cbf_interventions': 0,
                'marker_lost_events': self.marker_lost_events,
            }
        duration = 0.0 if self.start_time is None else self.last_time - self.start_time
        return {
            'samples': self.sample_count,
            'duration_s': duration,
            'laps_completed': self.laps_completed,
            'mean_abs_cte_px': self.abs_cte_sum_px / self.sample_count,
            'rmse_cte_px': math.sqrt(self.cte_square_sum_px / self.sample_count),
            'max_abs_cte_px': self.max_abs_cte_px,
            'mean_abs_heading_error_deg': math.degrees(
                self.abs_heading_sum_rad / self.sample_count
            ),
            'mean_abs_speed_error_pps': (
                self.abs_speed_error_sum_pps / self.sample_count
            ),
            'steering_effort_pwm_s': self.steering_effort_pwm_s,
            'throttle_effort_pwm_s': self.throttle_effort_pwm_s,
            'min_obstacle_clearance_px': self.min_safety_clearance_px,
            'collision_samples': self.collision_samples,
            'cbf_interventions': self.cbf_interventions,
            'marker_lost_events': self.marker_lost_events,
        }


def parse_args(args=None):
    """Read vision, control, and safety settings while retaining ROS args."""
    parser = argparse.ArgumentParser(
        description=(
            'Detect ArUco DICT_4X4 marker ID 0 on /image_raw, draw a plain '
            'straight road, and publish bounded RC commands to follow it.'
        )
    )
    parser.add_argument('--image-topic', default=IMAGE_TOPIC)
    parser.add_argument('--command-topic', default=COMMAND_TOPIC)
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-dict', default='DICT_4X4_50')
    parser.add_argument(
        '--front-edge',
        choices=('top', 'right', 'bottom', 'left'),
        default='top',
        help='Which ArUco marker edge points toward vehicle front.',
    )
    parser.add_argument('--track-center-x', type=float, default=0.5)
    parser.add_argument('--track-center-y', type=float, default=0.55)
    parser.add_argument('--track-radius-x', type=float, default=0.20)
    parser.add_argument('--track-radius-y', type=float, default=0.24)
    parser.add_argument(
        '--track-shape',
        choices=(
            'straight_road',
            'oval',
            'figure8',
            'chicane',
            'hairpin',
            's_curve_road',
        ),
        default='straight_road',
        help='Virtual image-space track shape.',
    )
    parser.add_argument('--lookahead-points', type=int, default=10)
    parser.add_argument('--road-lane-width-px', type=float, default=150.0)
    parser.add_argument('--road-amplitude-x', type=float, default=0.18)
    parser.add_argument('--road-length-y', type=float, default=0.82)
    parser.add_argument(
        '--static-obstacles',
        default='',
        help=(
            'JSON list of static road obstacles. Each item may set lane, '
            'progress, length_px, and width_px. Empty keeps straight_road clear.'
        ),
    )
    parser.add_argument('--obstacle-margin-px', type=float, default=42.0)
    parser.add_argument('--lane-switch-lookahead-points', type=int, default=42)
    parser.add_argument(
        '--process-width',
        type=int,
        default=960,
        help='Downscale image to this width before ArUco detection; 0 disables.',
    )
    parser.add_argument(
        '--max-frame-age',
        type=float,
        default=0.2,
        help='Drop stamped images older than this many seconds; 0 disables.',
    )
    parser.add_argument(
        '--preview-width',
        type=int,
        default=960,
        help='Resize debug preview to this width; 0 shows full size.',
    )
    parser.add_argument('--steering-kp-px', type=float, default=1.4)
    parser.add_argument('--steering-ki-px', type=float, default=0.0)
    parser.add_argument('--steering-kd-px', type=float, default=0.25)
    parser.add_argument('--heading-kp', type=float, default=120.0)
    parser.add_argument('--integral-limit-px-s', type=float, default=250.0)
    parser.add_argument(
        '--controller-mode',
        choices=CONTROLLER_MODES,
        default=PID,
        help='Control variant used for throttle/pitch output.',
    )
    parser.add_argument('--target-track-speed-pps', type=float, default=38.0)
    parser.add_argument('--velocity-kp-pwm', type=float, default=2.0)
    parser.add_argument('--velocity-ki-pwm', type=float, default=0.0)
    parser.add_argument('--velocity-kd-pwm', type=float, default=0.05)
    parser.add_argument('--velocity-integral-limit', type=float, default=120.0)
    parser.add_argument('--max-forward-pwm', type=int, default=1590)
    parser.add_argument('--cbf-slow-error-px', type=float, default=55.0)
    parser.add_argument('--cbf-stop-error-px', type=float, default=120.0)
    parser.add_argument('--cbf-stop-heading-rad', type=float, default=1.2)
    parser.add_argument('--cbf-edge-margin-px', type=float, default=45.0)
    parser.add_argument(
        '--cbf-h-px',
        type=float,
        default=42.0,
        help='CBF obstacle barrier distance h in pixels.',
    )
    parser.add_argument(
        '--cbf-alpha',
        type=float,
        default=1.0,
        help='CBF aggressiveness: lower slows earlier, higher allows more speed.',
    )
    parser.add_argument('--forward-pwm', type=int, default=1590)
    parser.add_argument('--min-forward-pwm', type=int, default=1590)
    parser.add_argument('--neutral-throttle-pwm', type=int, default=1500)
    parser.add_argument(
        '--drive-channel',
        choices=('pitch', 'throttle'),
        default='pitch',
        help='RC channel used for vehicle drive output (default: pitch).',
    )
    parser.add_argument('--left-pwm', type=int, default=1850)
    parser.add_argument('--center-steering-pwm', type=int, default=1500)
    parser.add_argument('--right-pwm', type=int, default=1150)
    parser.add_argument('--command-timeout', type=float, default=1.0)
    parser.add_argument(
        '--preview',
        action='store_true',
        help='Show OpenCV debug preview with marker, track, and target.',
    )
    parser.add_argument(
        '--no-tuning-panel',
        '--no-pid-panel',
        dest='no_pid_panel',
        action='store_true',
        help='Do not open the live controller tuning slider panel.',
    )
    parser.add_argument(
        '--debug-commands',
        action='store_true',
        help='Print outgoing RC command values once per second.',
    )
    parser.add_argument(
        '--tuning-file',
        type=Path,
        default=Path('aruco_track_follower_tuning.json'),
        help='Path used when saving/loading tuned gains.',
    )
    parser.add_argument(
        '--load-tuning',
        action='store_true',
        help='Load tuning-file values on startup.',
    )
    parser.add_argument(
        '--metrics-file',
        type=Path,
        default=Path('aruco_track_follower_metrics.json'),
        help='Path used when saving ranking metrics on exit.',
    )
    parser.add_argument(
        '--target-laps',
        type=int,
        default=0,
        help='Stop after this many completed laps; 0 disables lap limit.',
    )
    parser.add_argument(
        '--enable-lap-limit',
        action='store_true',
        help='Enable target-laps stopping from startup.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Detect marker and preview without publishing or arming.',
    )
    parser.add_argument(
        '--confirm-propulsion-safe',
        action='store_true',
        help='Required for RC output; confirms driven wheels cannot injure.',
    )
    return parser.parse_known_args(args)


class ArucoTrackFollower(Node):
    """Use image-space ArUco feedback to follow a virtual track."""

    def __init__(self, config):
        super().__init__('aruco_track_follower')
        cv2.setUseOptimized(True)
        self.config = config
        mode = getattr(config, 'controller_mode', 'pid')
        if mode == 'pid_velocity_dclf_dcbf':
            mode_title = 'DCLF-DCBF'
        elif mode == 'mpc_cbf':
            mode_title = 'MPC-CBF'
        elif mode == 'pid_velocity_cbf_qp_ellipse':
            mode_title = 'CBF-QP'
        else:
            mode_title = mode.replace('_', '-').upper()
        self.window_name = f"Tuning Panel ({mode_title})"
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
        self.track_points = None
        self.lane_tracks = None
        self.road_boundaries = None
        self.static_obstacles = []
        self.last_image_size = None
        self.last_command_time = None
        self.last_detection_time = None
        self.last_marker_heading = 0.0
        self.output_enabled = False
        self.armed = False
        self.throttle = config.neutral_throttle_pwm
        self.roll = config.center_steering_pwm
        self.last_debug_print_time = 0.0
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
        self.pid_velocity_cbf = PIDVelocityCBFController(self.velocity_pid)
        self.pid_panel_enabled = not config.no_pid_panel
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
        self.latest_lidar_points = []
        self.selected_lane_index = 0
        self.previous_lane_index = None
        self.lane_switch_count = 0
        self.nearest_static_clearance_px = float('inf')
        self.metrics = FollowerMetrics()
        self.metrics_saved = False
        self.metrics_file_path = unique_metrics_path(config.metrics_file)
        self.lap_limit_enabled = config.enable_lap_limit
        self.target_laps = max(0, config.target_laps)
        if config.load_tuning:
            self.load_tuning_file()
        if self.pid_panel_enabled:
            self.create_pid_panel()
        self.create_timer(1.0 / UPDATE_RATE_HZ, self.publish_command)

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
        params.adaptiveThreshWinSizeStep = 15
        return params

    def image_callback(self, image_msg):
        """Process one camera frame and update the latest RC command."""
        if self.message_is_stale(image_msg):
            return
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        if self.last_image_size != (width, height):
            self.last_image_size = (width, height)
            self.build_track_scene(width, height)

        detection = self.detect_marker(frame)
        if detection is None:
            self.stop_immediately()
            if self.config.preview:
                self.show_preview(frame, None, None, None, None)
            return

        center, heading, corners = detection
        self.last_marker_heading = heading
        target, tangent, error, nearest_index = self.track_error(center, heading)
        heading_error = wrap_angle(math.atan2(tangent[1], tangent[0]) - heading)
        now = time.monotonic()
        self.update_pid_from_panel()
        self.roll = self.steering_to_pwm(error, heading_error, now)
        self.throttle = self.throttle_to_pwm(
            center,
            error,
            heading_error,
            nearest_index,
            now,
        )
        speed_error = self.target_track_speed_pps - self.track_speed_pps
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
        if self.target_lap_reached():
            self.request_stop(
                f'target laps completed ({self.metrics.laps_completed}/'
                f'{self.target_laps})'
            )
            return
        self.last_command_time = now
        self.last_detection_time = now
        self.last_marker_seen = True

        if self.output_enabled and not self.config.dry_run:
            command = self.drive_message()
            self.command_pub.publish(command)
            self.debug_command(command, 'tracking')

        if self.config.preview:
            self.show_preview(frame, corners, center, target, tangent)

    def build_track_scene(self, width, height):
        """Create the virtual track or road scene for the current image size."""
        if self.config.track_shape == 'straight_road':
            scene = make_straight_road_scene(width, height, self.config)
            self.lane_tracks = scene['lanes']
            self.road_boundaries = scene['boundaries']
            self.static_obstacles = scene['obstacles']
            self.selected_lane_index = min(
                self.selected_lane_index,
                len(self.lane_tracks) - 1,
            )
            self.track_points = self.lane_tracks[self.selected_lane_index]
            return

        if self.config.track_shape == 's_curve_road':
            scene = make_s_curve_road_scene(width, height, self.config)
            self.lane_tracks = scene['lanes']
            self.road_boundaries = scene['boundaries']
            self.static_obstacles = scene['obstacles']
            self.selected_lane_index = min(
                self.selected_lane_index,
                len(self.lane_tracks) - 1,
            )
            self.track_points = self.lane_tracks[self.selected_lane_index]
            return

        self.lane_tracks = None
        self.road_boundaries = None
        self.static_obstacles = []
        self.track_points = make_oval_track(
            width,
            height,
            self.config.track_center_x,
            self.config.track_center_y,
            self.config.track_radius_x,
            self.config.track_radius_y,
            self.config.track_shape,
        )

    def road_scene_enabled(self):
        """Return whether the current track uses two-lane road planning."""
        return self.config.track_shape in ('straight_road', 's_curve_road')

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
            heading = self.marker_heading(marker_corners)
            return center, heading, marker_corners
        return None

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

    def track_error(self, center, heading):
        """Find a lookahead target and signed image-space steering error."""
        if self.road_scene_enabled():
            self.track_points = self.select_road_lane(center)
        distances = np.linalg.norm(self.track_points - center, axis=1)
        nearest_index = int(np.argmin(distances))
        if self.road_scene_enabled():
            target_index = min(
                nearest_index + self.config.lookahead_points,
                len(self.track_points) - 1,
            )
        else:
            target_index = (
                nearest_index + self.config.lookahead_points
            ) % len(self.track_points)
        target = self.track_points[target_index]
        if self.road_scene_enabled():
            next_index = min(target_index + 1, len(self.track_points) - 1)
            previous_index = max(target_index - 1, 0)
        else:
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

    def select_road_lane(self, center):
        """Pick the lowest-cost lane through the S-curve obstacle field."""
        if not self.lane_tracks:
            return self.track_points

        costs = []
        for lane_index, lane_points in enumerate(self.lane_tracks):
            distances = np.linalg.norm(lane_points - center, axis=1)
            nearest_index = int(np.argmin(distances))
            end_index = min(
                nearest_index + self.config.lane_switch_lookahead_points,
                len(lane_points) - 1,
            )
            lookahead = lane_points[nearest_index:end_index + 1]
            if len(lookahead) == 0:
                lookahead = lane_points[nearest_index:nearest_index + 1]

            obstacle_cost = 0.0
            for point in lookahead:
                clearance = self.static_obstacle_clearance_px(point)
                if clearance < 0.0:
                    obstacle_cost += 100000.0
                elif clearance < self.config.obstacle_margin_px:
                    obstacle_cost += (
                        self.config.obstacle_margin_px - clearance
                    ) ** 2

            switch_cost = 0.0
            if self.previous_lane_index is not None:
                switch_cost = (
                    2500.0
                    if lane_index != self.previous_lane_index
                    else 0.0
                )
            position_cost = float(np.min(distances)) * 0.5
            costs.append(
                (obstacle_cost + switch_cost + position_cost, lane_index)
            )

        _cost, selected_lane = min(costs, key=lambda item: item[0])
        if (
            self.previous_lane_index is not None
            and selected_lane != self.previous_lane_index
        ):
            self.lane_switch_count += 1
        self.previous_lane_index = selected_lane
        self.selected_lane_index = selected_lane
        return self.lane_tracks[selected_lane]

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
        if self.config.controller_mode == PID:
            self.cbf_scale = 1.0
            self.cbf_active = False
            return self.config.forward_pwm

        self.update_track_speed(nearest_index, now)
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

        if self.config.controller_mode == PID_VELOCITY_CBF:
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
            throttle = result.throttle_pwm
            self.cbf_scale = result.scale
            self.cbf_active = result.active
            self.nearest_static_clearance_px = result.lidar_clearance_px
            self.latest_lidar_points = (
                self.pid_velocity_cbf.virtual_lidar.latest_points
            )
        else:
            self.cbf_scale = 1.0
            self.cbf_active = False
            self.latest_lidar_points = []

        return round(throttle)

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
        if self.road_scene_enabled():
            delta = nearest_index - self.last_progress_index
        else:
            point_count = len(self.track_points)
            forward_delta = (
                nearest_index - self.last_progress_index
            ) % point_count
            reverse_delta = forward_delta - point_count
            if abs(forward_delta) < abs(reverse_delta):
                delta = forward_delta
            else:
                delta = reverse_delta
        self.track_speed_pps = delta / dt
        self.last_progress_index = nearest_index
        self.last_progress_time = now

    def cbf_safety_scale(self, center, lateral_error_px, heading_error_rad):
        """Scale throttle down as image-space safety limits are approached."""
        result = self.pid_velocity_cbf.apply_cbf(
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
        )
        self.latest_lidar_points = (
            self.pid_velocity_cbf.virtual_lidar.latest_points
        )
        self.nearest_static_clearance_px = result.lidar_clearance_px
        return result.scale

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
            self.pid_velocity_cbf.virtual_lidar.scan(
                center,
                self.last_marker_heading,
                self.static_obstacles,
            )
            static_clearance = (
                self.pid_velocity_cbf.virtual_lidar.nearest_clearance_px(
                    self.config.cbf_h_px
                )
            )
            self.latest_lidar_points = (
                self.pid_velocity_cbf.virtual_lidar.latest_points
            )
            self.nearest_static_clearance_px = static_clearance
        return min(
            edge_clearance,
            error_clearance,
            heading_px,
            static_clearance,
        )

    def static_obstacle_clearance_px(self, point):
        """Return clearance from a point to the nearest virtual road obstacle."""
        if not self.static_obstacles:
            return float('inf')
        clearances = [
            obstacle_clearance_px(point, obstacle)
            for obstacle in self.static_obstacles
        ]
        return min(clearances) - self.config.obstacle_margin_px

    def stop_immediately(self):
        """Neutralize command state and publish a stop as soon as marker is lost."""
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
        self.latest_lidar_points = []
        if self.output_enabled and not self.config.dry_run:
            command = self.neutral_message()
            self.command_pub.publish(command)
            self.debug_command(command, 'marker lost')

    def create_pid_panel(self):
        """Open live sliders for controller tuning."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 520, 400)
        cv2.createTrackbar(
            'CBF slack wt',
            self.window_name,
            int(getattr(self.config, 'qp_slack_weight', 2000)),
            5000,
            noop,
        )
        cv2.createTrackbar(
            'CLF slack wt',
            self.window_name,
            int(getattr(self.config, 'clf_slack_weight', 500)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'Lookahead pts',
            self.window_name,
            self.config.lookahead_points,
            50,
            noop,
        )
        cv2.createTrackbar(
            'CLF alpha x100',
            self.window_name,
            int(round(getattr(self.config, 'clf_alpha', 0.1) * 100)),
            100,
            noop,
        )
        cv2.createTrackbar(
            'Kp x1000',
            self.window_name,
            int(round(self.config.steering_kp_px * PID_SCALE)),
            5000,
            noop,
        )
        cv2.createTrackbar(
            'Ki x1000',
            self.window_name,
            int(round(self.config.steering_ki_px * PID_SCALE)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'Kd x1000',
            self.window_name,
            int(round(self.config.steering_kd_px * PID_SCALE)),
            5000,
            noop,
        )
        cv2.createTrackbar(
            'Heading x10',
            self.window_name,
            int(round(self.config.heading_kp * HEADING_SCALE)),
            3000,
            noop,
        )
        cv2.createTrackbar(
            'Throttle',
            self.window_name,
            self.config.forward_pwm,
            self.config.max_forward_pwm,
            noop,
        )
        cv2.createTrackbar(
            'Target pps x10',
            self.window_name,
            int(round(self.config.target_track_speed_pps * VELOCITY_SCALE)),
            1200,
            noop,
        )
        cv2.createTrackbar(
            'VelKp x10',
            self.window_name,
            int(round(self.config.velocity_kp_pwm * VELOCITY_SCALE)),
            200,
            noop,
        )
        cv2.createTrackbar(
            'CBF stop px',
            self.window_name,
            int(round(self.config.cbf_stop_error_px / CBF_ERROR_SCALE)),
            400,
            noop,
        )
        cv2.createTrackbar(
            'CBF h px',
            self.window_name,
            int(round(self.config.cbf_h_px)),
            300,
            noop,
        )
        cv2.createTrackbar(
            'CBF alpha x100',
            self.window_name,
            int(round(self.config.cbf_alpha * CBF_ALPHA_SCALE)),
            500,
            noop,
        )
        cv2.createTrackbar(
            'Lap limit',
            self.window_name,
            1 if self.lap_limit_enabled else 0,
            1,
            noop,
        )
        cv2.createTrackbar(
            'Target laps',
            self.window_name,
            self.target_laps,
            20,
            noop,
        )
        if self.config.forward_pwm < self.config.min_forward_pwm:
            cv2.setTrackbarPos('Throttle', self.window_name, self.config.min_forward_pwm)

    def read_panel_values(self):
        """Read live slider values without mutating controller state."""
        values = {
            'steering_kp_px': cv2.getTrackbarPos('Kp x1000', self.window_name)
            / PID_SCALE,
            'steering_ki_px': cv2.getTrackbarPos('Ki x1000', self.window_name)
            / PID_SCALE,
            'steering_kd_px': cv2.getTrackbarPos('Kd x1000', self.window_name)
            / PID_SCALE,
            'heading_kp': cv2.getTrackbarPos('Heading x10', self.window_name)
            / HEADING_SCALE,
            'forward_pwm': int(cv2.getTrackbarPos('Throttle', self.window_name)),
            'target_track_speed_pps': cv2.getTrackbarPos(
                'Target pps x10',
                self.window_name,
            )
            / VELOCITY_SCALE,
            'velocity_kp_pwm': cv2.getTrackbarPos('VelKp x10', self.window_name)
            / VELOCITY_SCALE,
            'cbf_stop_error_px': cv2.getTrackbarPos('CBF stop px', self.window_name)
            * CBF_ERROR_SCALE,
            'cbf_h_px': float(cv2.getTrackbarPos('CBF h px', self.window_name)),
            'cbf_alpha': cv2.getTrackbarPos('CBF alpha x100', self.window_name)
            / CBF_ALPHA_SCALE,
            'lap_limit_enabled': bool(cv2.getTrackbarPos('Lap limit', self.window_name)),
            'target_laps': int(cv2.getTrackbarPos('Target laps', self.window_name)),
            'lookahead_points': int(cv2.getTrackbarPos('Lookahead pts', self.window_name)),
            'clf_alpha': cv2.getTrackbarPos('CLF alpha x100', self.window_name) / 100.0,
            'qp_slack_weight': float(cv2.getTrackbarPos('CBF slack wt', self.window_name)),
            'clf_slack_weight': float(cv2.getTrackbarPos('CLF slack wt', self.window_name)),
        }
        values['forward_pwm'] = int(
            bounded(
                values['forward_pwm'],
                self.config.min_forward_pwm,
                self.config.max_forward_pwm,
            )
        )
        values['cbf_stop_error_px'] = max(
            self.config.cbf_slow_error_px + 1.0,
            values['cbf_stop_error_px'],
        )
        values['cbf_h_px'] = max(1.0, values['cbf_h_px'])
        values['cbf_alpha'] = max(0.01, values['cbf_alpha'])
        return values

    def apply_tuning_values(self, values):
        """Apply tuning values to current config and controller gains."""
        self.config.steering_kp_px = values['steering_kp_px']
        self.config.steering_ki_px = values['steering_ki_px']
        self.config.steering_kd_px = values['steering_kd_px']
        self.heading_kp = values['heading_kp']
        self.config.forward_pwm = values['forward_pwm']
        self.target_track_speed_pps = values['target_track_speed_pps']
        self.config.velocity_kp_pwm = values['velocity_kp_pwm']
        self.cbf_stop_error_px = values['cbf_stop_error_px']
        self.config.cbf_h_px = values['cbf_h_px']
        self.config.cbf_alpha = values['cbf_alpha']
        if 'lookahead_points' in values:
            self.config.lookahead_points = max(1, int(values['lookahead_points']))
        if 'clf_alpha' in values:
            self.config.clf_alpha = float(values['clf_alpha'])
        if 'qp_slack_weight' in values:
            self.config.qp_slack_weight = float(values['qp_slack_weight'])
        if 'clf_slack_weight' in values:
            self.config.clf_slack_weight = float(values['clf_slack_weight'])
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
        self.velocity_pid.update_gains(
            self.config.velocity_kp_pwm,
            self.config.velocity_ki_pwm,
            self.config.velocity_kd_pwm,
            self.config.velocity_integral_limit,
        )

    def update_pid_from_panel(self):
        """Read live sliders and update controller gains."""
        if not self.pid_panel_enabled:
            return
        self.apply_tuning_values(self.read_panel_values())

    def current_tuning_values(self):
        """Return latest tuning values, including CLI-only mode settings."""
        if self.pid_panel_enabled:
            values = self.read_panel_values()
        else:
            values = {
                'steering_kp_px': self.config.steering_kp_px,
                'steering_ki_px': self.config.steering_ki_px,
                'steering_kd_px': self.config.steering_kd_px,
                'heading_kp': self.heading_kp,
                'forward_pwm': self.config.forward_pwm,
                'target_track_speed_pps': self.target_track_speed_pps,
                'velocity_kp_pwm': self.config.velocity_kp_pwm,
                'cbf_stop_error_px': self.cbf_stop_error_px,
                'cbf_h_px': self.config.cbf_h_px,
                'cbf_alpha': self.config.cbf_alpha,
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
                'cbf_slow_error_px': self.config.cbf_slow_error_px,
                'cbf_stop_heading_rad': self.config.cbf_stop_heading_rad,
                'cbf_edge_margin_px': self.config.cbf_edge_margin_px,
                'cbf_h_px': self.config.cbf_h_px,
                'cbf_alpha': self.config.cbf_alpha,
                'lap_limit_enabled': self.lap_limit_enabled,
                'target_laps': self.target_laps,
                'road_lane_width_px': self.config.road_lane_width_px,
                'road_amplitude_x': self.config.road_amplitude_x,
                'road_length_y': self.config.road_length_y,
                'obstacle_margin_px': self.config.obstacle_margin_px,
                'lane_switch_lookahead_points': (
                    self.config.lane_switch_lookahead_points
                ),
                'completion_reason': self.completion_reason,
            }
        )
        return values

    def save_tuning_file(self):
        """Save current tuning values to disk."""
        values = self.current_tuning_values()
        self.config.tuning_file.parent.mkdir(parents=True, exist_ok=True)
        self.config.tuning_file.write_text(
            json.dumps(values, indent=2) + '\n',
            encoding='utf-8',
        )
        self.get_logger().info(f'saved tuning to {self.config.tuning_file}')

    def save_metrics_file(self):
        """Save current ranking metrics to disk."""
        if self.metrics_saved:
            return
        payload = self.metrics.summary()
        payload.update(
            {
                'controller_mode': self.config.controller_mode,
                'track_shape': self.config.track_shape,
                'drive_channel': self.config.drive_channel,
                'steering_kp_px': self.config.steering_kp_px,
                'steering_ki_px': self.config.steering_ki_px,
                'steering_kd_px': self.config.steering_kd_px,
                'heading_kp': self.heading_kp,
                'velocity_kp_pwm': self.config.velocity_kp_pwm,
                'velocity_ki_pwm': self.config.velocity_ki_pwm,
                'velocity_kd_pwm': self.config.velocity_kd_pwm,
                'speed_error_pps_last': self.speed_error_pps,
                'velocity_delta_pwm_last': self.velocity_delta_pwm,
                'target_track_speed_pps': self.target_track_speed_pps,
                'forward_pwm': self.config.forward_pwm,
                'min_forward_pwm': self.config.min_forward_pwm,
                'max_forward_pwm': self.config.max_forward_pwm,
                'cbf_slow_error_px': self.config.cbf_slow_error_px,
                'cbf_stop_error_px': self.cbf_stop_error_px,
                'cbf_h_px': self.config.cbf_h_px,
                'cbf_alpha': self.config.cbf_alpha,
                'cbf_stop_heading_rad': self.config.cbf_stop_heading_rad,
                'cbf_edge_margin_px': self.config.cbf_edge_margin_px,
                'lap_limit_enabled': self.lap_limit_enabled,
                'target_laps': self.target_laps,
                'selected_lane_index': self.selected_lane_index,
                'lane_switch_count': self.lane_switch_count,
                'nearest_static_clearance_px_last': (
                    self.nearest_static_clearance_px
                ),
                'road_lane_width_px': self.config.road_lane_width_px,
                'road_amplitude_x': self.config.road_amplitude_x,
                'road_length_y': self.config.road_length_y,
                'obstacle_margin_px': self.config.obstacle_margin_px,
                'lane_switch_lookahead_points': (
                    self.config.lane_switch_lookahead_points
                ),
            }
        )
        self.metrics_file_path.parent.mkdir(parents=True, exist_ok=True)
        self.metrics_file_path.write_text(
            json.dumps(payload, indent=2) + '\n',
            encoding='utf-8',
        )
        self.metrics_saved = True
        self.get_logger().info(f'saved metrics to {self.metrics_file_path}')

    def load_tuning_file(self):
        """Load tuning values from disk if available."""
        if not self.config.tuning_file.exists():
            self.get_logger().warning(
                f'tuning file not found: {self.config.tuning_file}'
            )
            return
        payload = json.loads(self.config.tuning_file.read_text(encoding='utf-8'))
        for key in (
            'min_forward_pwm',
            'max_forward_pwm',
            'velocity_ki_pwm',
            'velocity_kd_pwm',
            'cbf_slow_error_px',
            'cbf_stop_heading_rad',
            'cbf_edge_margin_px',
            'cbf_h_px',
            'cbf_alpha',
            'enable_lap_limit',
            'target_laps',
        ):
            if key in payload:
                setattr(self.config, key, payload[key])
        self.apply_tuning_values(
            {
                'steering_kp_px': payload.get(
                    'steering_kp_px',
                    self.config.steering_kp_px,
                ),
                'steering_ki_px': payload.get(
                    'steering_ki_px',
                    self.config.steering_ki_px,
                ),
                'steering_kd_px': payload.get(
                    'steering_kd_px',
                    self.config.steering_kd_px,
                ),
                'heading_kp': payload.get('heading_kp', self.heading_kp),
                'forward_pwm': payload.get('forward_pwm', self.config.forward_pwm),
                'target_track_speed_pps': payload.get(
                    'target_track_speed_pps',
                    self.target_track_speed_pps,
                ),
                'velocity_kp_pwm': payload.get(
                    'velocity_kp_pwm',
                    self.config.velocity_kp_pwm,
                ),
                'cbf_stop_error_px': payload.get(
                    'cbf_stop_error_px',
                    self.cbf_stop_error_px,
                ),
                'cbf_h_px': payload.get('cbf_h_px', self.config.cbf_h_px),
                'cbf_alpha': payload.get('cbf_alpha', self.config.cbf_alpha),
                'lap_limit_enabled': payload.get(
                    'lap_limit_enabled',
                    payload.get('enable_lap_limit', self.lap_limit_enabled),
                ),
                'target_laps': payload.get('target_laps', self.target_laps),
            }
        )
        self.get_logger().info(f'loaded tuning from {self.config.tuning_file}')

    def command_is_fresh(self):
        """Return whether a recent marker-derived command is available."""
        return (
            self.last_command_time is not None
            and time.monotonic() - self.last_command_time
            <= self.config.command_timeout
        )

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
        if not (self.config.preview or self.pid_panel_enabled):
            return
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.request_stop('q pressed')
        elif key == ord('s'):
            self.save_tuning_file()

    def request_stop(self, reason):
        """Publish stop commands and request node shutdown."""
        if self.stop_requested:
            return
        self.stop_requested = True
        self.completion_reason = reason
        self.get_logger().info(f'{reason}; sending stop commands')
        self.throttle = self.config.neutral_throttle_pwm
        self.roll = self.config.center_steering_pwm
        self.last_command_time = None
        self.output_enabled = False
        self.pid.reset()
        self.pid_velocity_cbf.reset()
        self.latest_lidar_points = []
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

    def show_preview(self, frame, corners, center, target, tangent):
        """Display the virtual track and current ArUco tracking state."""
        preview = frame.copy()
        if self.road_scene_enabled():
            self.draw_road_scene(preview)
        if self.track_points is not None:
            cv2.polylines(
                preview,
                [self.track_points.astype(np.int32)],
                not self.road_scene_enabled(),
                (0, 255, 255),
                3 if self.road_scene_enabled() else 2,
                cv2.LINE_AA,
            )
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
            self.draw_lidar_feedback(preview, center)
        if target is not None:
            cv2.circle(preview, tuple(target.astype(int)), 7, (0, 0, 255), -1)
            cv2.line(
                preview,
                tuple(center.astype(int)),
                tuple(target.astype(int)),
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        if target is not None and tangent is not None:
            end = target + tangent * 45.0
            cv2.arrowedLine(
                preview,
                tuple(target.astype(int)),
                tuple(end.astype(int)),
                (255, 255, 0),
                2,
                cv2.LINE_AA,
            )
        put_status(
            preview,
            self.config.controller_mode,
            self.throttle,
            self.roll,
            self.command_is_fresh(),
            self.track_speed_pps,
            self.target_track_speed_pps,
            self.speed_error_pps,
            self.velocity_delta_pwm,
            self.cbf_scale,
            self.metrics.summary(),
            self.lap_limit_enabled,
            self.target_laps,
            self.selected_lane_index if self.road_scene_enabled() else None,
            self.lane_switch_count,
            self.nearest_static_clearance_px,
        )
        preview = resize_for_preview(preview, self.config.preview_width)
        cv2.imshow('Aruco Track Follower', preview)

    def draw_road_scene(self, preview):
        """Draw the two-lane S-curve road and static obstacle field."""
        if self.road_boundaries:
            for boundary in self.road_boundaries:
                cv2.polylines(
                    preview,
                    [boundary.astype(np.int32)],
                    False,
                    (180, 180, 180),
                    2,
                    cv2.LINE_AA,
                )
        if self.lane_tracks:
            for lane_index, lane_points in enumerate(self.lane_tracks):
                if lane_index == self.selected_lane_index:
                    color = (90, 220, 90)
                else:
                    color = (80, 120, 255)
                cv2.polylines(
                    preview,
                    [lane_points.astype(np.int32)],
                    False,
                    color,
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

    def draw_lidar_feedback(self, preview, center):
        """Draw virtual lidar returns from the marker/car center."""
        if not self.latest_lidar_points:
            return
        origin = tuple(center.astype(int))
        for point in self.latest_lidar_points:
            hit = (int(round(point.x_px)), int(round(point.y_px)))
            cv2.line(preview, origin, hit, (255, 0, 255), 1, cv2.LINE_AA)
            cv2.circle(preview, hit, 3, (255, 0, 255), -1)

    def publish_neutral_for(self, duration):
        """Continuously publish neutral output for a fixed duration."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.command_pub.publish(self.neutral_message())
            if self.executor is None:
                rclpy.spin_once(self, timeout_sec=0.0)
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
        print(future.result().data)


def make_oval_track(width, height, center_x, center_y, radius_x, radius_y, shape):
    """Create a virtual image-space track."""
    angles = np.linspace(0.0, 2.0 * math.pi, 240, endpoint=False)
    if shape == 'figure8':
        points = np.column_stack(
            (
                center_x * width + np.sin(angles) * radius_x * width,
                center_y * height
                + np.sin(angles) * np.cos(angles) * radius_y * height,
            )
        )
        return points.astype(np.float32)
    if shape == 'chicane':
        points = np.column_stack(
            (
                center_x * width + np.cos(angles) * radius_x * width,
                center_y * height
                + (
                    0.55 * np.sin(angles)
                    + 0.45 * np.sin(3.0 * angles)
                )
                * radius_y
                * height,
            )
        )
        return points.astype(np.float32)
    if shape == 'hairpin':
        radius_scale = 0.65 + 0.35 * np.cos(angles)
        points = np.column_stack(
            (
                center_x * width
                + np.cos(angles) * radius_x * width * radius_scale,
                center_y * height + np.sin(angles) * radius_y * height,
            )
        )
        return points.astype(np.float32)
    points = np.column_stack(
        (
            center_x * width + np.cos(angles) * radius_x * width,
            center_y * height + np.sin(angles) * radius_y * height,
        )
    )
    return points.astype(np.float32)


def make_s_curve_road_scene(width, height, config):
    """Create two S-curve lane centerlines and static road obstacles."""
    point_count = 260
    progress = np.linspace(0.0, 1.0, point_count)
    road_length_x = bounded(config.road_length_y, 0.25, 0.96) * width
    start_x = config.track_center_x * width - road_length_x * 0.5
    end_x = config.track_center_x * width + road_length_x * 0.5
    x_values = np.linspace(start_x, end_x, point_count)
    y_center = config.track_center_y * height
    amplitude = config.road_amplitude_x * height
    y_values = y_center + amplitude * np.sin(2.0 * math.pi * progress)
    base = np.column_stack((x_values, y_values)).astype(np.float32)
    tangents, normals = path_tangents_normals(base)

    lane_half_offset = config.road_lane_width_px * 0.5
    lanes = [
        (base - normals * lane_half_offset).astype(np.float32),
        (base + normals * lane_half_offset).astype(np.float32),
    ]
    boundaries = [
        (base - normals * config.road_lane_width_px).astype(np.float32),
        (base + normals * config.road_lane_width_px).astype(np.float32),
    ]
    obstacle_specs = parse_static_obstacle_specs(config.static_obstacles)
    obstacles = [
        make_static_obstacle(spec, lanes, tangents, normals, config)
        for spec in obstacle_specs
    ]
    obstacles.extend(make_road_end_walls(base, tangents, normals, config))
    return {
        'lanes': lanes,
        'boundaries': boundaries,
        'obstacles': obstacles,
    }


def make_straight_road_scene(width, height, config):
    """Create two straight landscape lane centerlines for hil preview."""
    point_count = 260
    road_length_x = bounded(config.road_length_y, 0.25, 0.96) * width
    start_x = config.track_center_x * width - road_length_x * 0.5
    end_x = config.track_center_x * width + road_length_x * 0.5
    x_values = np.linspace(start_x, end_x, point_count)
    y_values = np.full(point_count, config.track_center_y * height)
    base = np.column_stack((x_values, y_values)).astype(np.float32)
    tangents, normals = path_tangents_normals(base)

    lane_half_offset = config.road_lane_width_px * 0.5
    lanes = [
        (base - normals * lane_half_offset).astype(np.float32),
        (base + normals * lane_half_offset).astype(np.float32),
    ]
    boundaries = [
        (base - normals * config.road_lane_width_px).astype(np.float32),
        (base + normals * config.road_lane_width_px).astype(np.float32),
    ]
    obstacle_specs = (
        parse_static_obstacle_specs(config.static_obstacles)
        if config.static_obstacles
        else []
    )
    obstacles = [
        make_static_obstacle(spec, lanes, tangents, normals, config)
        for spec in obstacle_specs
    ]
    return {
        'lanes': lanes,
        'boundaries': boundaries,
        'obstacles': obstacles,
    }


def parse_static_obstacle_specs(raw_payload):
    """Return static obstacle specs from CLI JSON or defaults."""
    if not raw_payload:
        return [
            {
                'lane': 0,
                'progress': 0.24,
                'length_px': 95.0,
                'width_px': 58.0,
            },
            {
                'lane': 1,
                'progress': 0.52,
                'length_px': 105.0,
                'width_px': 58.0,
            },
            {
                'lane': 0,
                'progress': 0.75,
                'length_px': 95.0,
                'width_px': 58.0,
            },
        ]
    payload = json.loads(raw_payload)
    if not isinstance(payload, list):
        raise ValueError('--static-obstacles must be a JSON list')
    for item in payload:
        if not isinstance(item, dict):
            raise ValueError('each static obstacle must be a JSON object')
    return payload


def path_tangents_normals(points):
    """Return unit tangent and normal vectors for a polyline."""
    previous_points = np.vstack((points[0], points[:-1]))
    next_points = np.vstack((points[1:], points[-1]))
    tangents = next_points - previous_points
    norms = np.linalg.norm(tangents, axis=1)
    norms[norms <= 1e-6] = 1.0
    tangents = tangents / norms[:, None]
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    return tangents.astype(np.float32), normals.astype(np.float32)


def make_static_obstacle(spec, lanes, tangents, normals, config):
    """Create an oriented rectangular obstacle on a lane centerline."""
    lane_index = int(spec.get('lane', 0))
    lane_index = int(bounded(lane_index, 0, len(lanes) - 1))
    progress = bounded(float(spec.get('progress', 0.5)), 0.0, 1.0)
    point_index = int(round(progress * (len(lanes[lane_index]) - 1)))
    length_px = float(spec.get('length_px', config.road_lane_width_px))
    width_px = float(spec.get('width_px', config.road_lane_width_px * 0.58))
    center = lanes[lane_index][point_index]
    tangent = tangents[point_index]
    normal = normals[point_index]
    half_length = max(1.0, length_px * 0.5)
    half_width = max(1.0, width_px * 0.5)
    polygon = np.array(
        [
            center + tangent * half_length + normal * half_width,
            center - tangent * half_length + normal * half_width,
            center - tangent * half_length - normal * half_width,
            center + tangent * half_length - normal * half_width,
        ],
        dtype=np.float32,
    )
    return {
        'center': center,
        'tangent': tangent,
        'normal': normal,
        'half_length': half_length,
        'half_width': half_width,
        'polygon': polygon,
        'lane': lane_index,
        'progress': progress,
    }


def make_road_end_walls(base, tangents, normals, config):
    """Create obstacle polygons that close the start and end of the road."""
    wall_thickness = max(18.0, config.obstacle_margin_px * 0.5)
    wall_half_width = config.road_lane_width_px + config.obstacle_margin_px
    walls = []
    for point_index, progress in ((0, 0.0), (len(base) - 1, 1.0)):
        center = base[point_index]
        tangent = tangents[point_index]
        normal = normals[point_index]
        half_length = wall_thickness * 0.5
        polygon = np.array(
            [
                center + tangent * half_length + normal * wall_half_width,
                center - tangent * half_length + normal * wall_half_width,
                center - tangent * half_length - normal * wall_half_width,
                center + tangent * half_length - normal * wall_half_width,
            ],
            dtype=np.float32,
        )
        walls.append(
            {
                'center': center,
                'tangent': tangent,
                'normal': normal,
                'half_length': half_length,
                'half_width': wall_half_width,
                'polygon': polygon,
                'lane': None,
                'progress': progress,
                'kind': 'road_end_wall',
            }
        )
    return walls


def obstacle_clearance_px(point, obstacle):
    """Return signed point clearance to an oriented rectangular obstacle."""
    delta = point - obstacle['center']
    local_x = float(np.dot(delta, obstacle['tangent']))
    local_y = float(np.dot(delta, obstacle['normal']))
    dx = abs(local_x) - obstacle['half_length']
    dy = abs(local_y) - obstacle['half_width']
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    inside = min(max(dx, dy), 0.0)
    return outside + inside


def make_sensor_qos():
    """Create low-latency QoS that keeps only the newest image."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


def noop(_value):
    """Opencv trackbar callback placeholder."""
    return None


def resize_for_preview(frame, target_width):
    """Resize preview output while preserving aspect ratio."""
    if target_width <= 0:
        return frame
    height, width = frame.shape[:2]
    if width <= target_width:
        return frame
    scale = target_width / float(width)
    return cv2.resize(
        frame,
        (target_width, int(round(height * scale))),
        interpolation=cv2.INTER_AREA,
    )


def put_status(
    frame,
    mode,
    throttle,
    roll,
    fresh,
    track_speed_pps,
    target_track_speed_pps,
    speed_error_pps,
    velocity_delta_pwm,
    cbf_scale,
    metrics,
    lap_limit_enabled,
    target_laps,
    selected_lane_index=None,
    lane_switch_count=0,
    nearest_static_clearance_px=float('inf'),
):
    """Draw controller status text onto a preview frame."""
    lines = [
        f'mode={mode} throttle={throttle} roll={roll}',
        f'command={"fresh" if fresh else "neutral"}',
        (
            f'track_speed={track_speed_pps:.1f}/{target_track_speed_pps:.1f} '
            f'pps cbf_scale={cbf_scale:.2f}'
        ),
        f'vel_err={speed_error_pps:.1f}pps vel_delta={velocity_delta_pwm:.1f}pwm',
        (
            f'MAE={metrics["mean_abs_cte_px"]:.1f}px '
            f'RMSE={metrics["rmse_cte_px"]:.1f}px '
            f'MAX={metrics["max_abs_cte_px"]:.1f}px'
        ),
        (
            f'head={metrics["mean_abs_heading_error_deg"]:.1f}deg '
            f'speed_err={metrics["mean_abs_speed_error_pps"]:.1f}pps'
        ),
        (
            f'steer_eff={metrics["steering_effort_pwm_s"]:.0f} '
            f'thr_eff={metrics["throttle_effort_pwm_s"]:.0f}'
        ),
        (
            f'clear={metrics["min_obstacle_clearance_px"]:.1f}px '
            f'coll={metrics["collision_samples"]} '
            f'cbf={metrics["cbf_interventions"]} '
            f'laps={metrics["laps_completed"]}'
        ),
        (
            f'lap_limit={"on" if lap_limit_enabled else "off"} '
            f'target={target_laps}'
        ),
    ]
    if selected_lane_index is not None:
        clearance = nearest_static_clearance_px
        if np.isfinite(clearance):
            clearance_text = f'{clearance:.1f}px'
        else:
            clearance_text = 'inf'
        lines.append(
            f'lane={selected_lane_index} switches={lane_switch_count} '
            f'static_clear={clearance_text}'
        )
    for index, line in enumerate(lines):
        origin = (12, 30 + index * 28)
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def validate_config(config):
    """Reject invalid controller calibration before hardware is armed."""
    for label in (
        'forward_pwm',
        'neutral_throttle_pwm',
        'left_pwm',
        'center_steering_pwm',
        'right_pwm',
        'min_forward_pwm',
        'max_forward_pwm',
    ):
        value = getattr(config, label)
        if not 988 <= value <= 2012:
            raise SystemExit(f'{label} must be between 988 and 2012')
    if config.forward_pwm < config.min_forward_pwm:
        raise SystemExit(
            'forward-pwm must be greater than or equal to min-forward-pwm'
        )
    if config.max_forward_pwm < config.min_forward_pwm:
        raise SystemExit(
            'max-forward-pwm must be greater than or equal to min-forward-pwm'
        )
    if config.cbf_stop_error_px <= config.cbf_slow_error_px:
        raise SystemExit('cbf-stop-error-px must be greater than cbf-slow-error-px')
    if config.cbf_stop_heading_rad <= 0.0:
        raise SystemExit('cbf-stop-heading-rad must be positive')
    if config.cbf_h_px <= 0.0:
        raise SystemExit('cbf-h-px must be positive')
    if config.cbf_alpha <= 0.0:
        raise SystemExit('cbf-alpha must be positive')
    if config.command_timeout <= 0.0:
        raise SystemExit('command timeout must be positive')
    if config.lookahead_points < 1:
        raise SystemExit('lookahead-points must be at least 1')
    if config.road_lane_width_px <= 0.0:
        raise SystemExit('road-lane-width-px must be positive')
    if config.road_amplitude_x < 0.0:
        raise SystemExit('road-amplitude-x must be non-negative')
    if not 0.0 < config.road_length_y <= 1.0:
        raise SystemExit('road-length-y must be between 0 and 1')
    if config.obstacle_margin_px <= 0.0:
        raise SystemExit('obstacle-margin-px must be positive')
    if config.lane_switch_lookahead_points < 1:
        raise SystemExit('lane-switch-lookahead-points must be at least 1')
    if config.static_obstacles:
        try:
            parse_static_obstacle_specs(config.static_obstacles)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise SystemExit(f'invalid static-obstacles JSON: {exc}') from exc


def print_config(config):
    """Print the configured topics and controller bounds."""
    print(
        f'image={config.image_topic} output={config.command_topic} '
        f'marker={config.marker_dict} id={config.marker_id}'
    )
    print(
        f'controller={config.controller_mode} track={config.track_shape} '
        f'target_speed={config.target_track_speed_pps:.1f} pps'
    )
    if config.track_shape in ('straight_road', 's_curve_road'):
        print(
            'road: '
            f'lane_width={config.road_lane_width_px:.1f}px '
            f'amplitude={config.road_amplitude_x:.2f} '
            f'length={config.road_length_y:.2f} '
            f'obstacle_margin={config.obstacle_margin_px:.1f}px'
        )
    print(
        'steering PWM: '
        f'left={config.left_pwm} center={config.center_steering_pwm} '
        f'right={config.right_pwm}'
    )
    print(
        'throttle PWM: '
        f'forward={config.forward_pwm} '
        f'min_forward={config.min_forward_pwm} '
        f'max_forward={config.max_forward_pwm} '
        f'neutral={config.neutral_throttle_pwm} '
        f'channel={config.drive_channel}'
    )


def main(args=None):
    """Run guarded ArUco virtual-track following."""
    config, ros_args = parse_args(args)
    validate_config(config)
    print_config(config)
    if not config.dry_run and not config.confirm_propulsion_safe:
        raise SystemExit(
            'refusing ArUco track following output: securely restrain driven '
            'wheels, then pass --confirm-propulsion-safe'
        )

    rclpy.init(args=ros_args)
    node = ArucoTrackFollower(config)
    try:
        if config.dry_run:
            print('Dry run: detecting marker without publishing RC output.')
            while rclpy.ok() and not node.stop_requested:
                rclpy.spin_once(node, timeout_sec=0.05)
            return

        time.sleep(0.2)
        if node.count_subscribers(config.command_topic) == 0:
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
