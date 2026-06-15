"""Command-line configuration for the straight-road hil follower."""

import argparse
import json
from pathlib import Path

from ..controllers import CONTROLLER_MODES
from ..controllers import PID
from ..controllers import PID_VELOCITY_CBF_QP_ELLIPSE
from ..road import parse_static_obstacle_specs

IMAGE_TOPIC = '/image_raw'
COMMAND_TOPIC = '/drone/rc_command'
ARMING_SERVICE = '/drone/cmd/arming'
NEUTRAL_VALUE = 1500
UPDATE_RATE_HZ = 50.0
SETTLE_DURATION = 0.5
PID_WINDOW = 'Aruco PID Tuning'
PID_SCALE = 1000
HEADING_SCALE = 10
VELOCITY_SCALE = 10
CBF_ERROR_SCALE = 1
CBF_ALPHA_SCALE = 100
CBF_ELLIPSE_SCALE = 10
CBF_GAMMA_SCALE = 100
QP_WHEELBASE_PX = 90.0
QP_MIN_ACCEL = -5.0
QP_MAX_ACCEL = 0.5
QP_MIN_DELTA = -0.4
QP_MAX_DELTA = 0.4
QP_SOLVER = 'OSQP'
QP_SLACK_WEIGHT = 0.0
GAP_SWITCH_HYSTERESIS_PX = 80.0
GAP_TARGET_SMOOTHING_ALPHA = 0.18
STEERING_KP_PX = 15
STEERING_KI_PX = 0.0
STEERING_KD_PX = 0.25
HEADING_KP_RAD = 120.0
INTEGRAL_LIMIT_PX_S = 250.0
ARUCO_MARKER_SIZE_CM = 10.0
DEFAULT_METRICS_FILE = (
    Path(__file__).resolve().parents[1]
    / 'metrics'
    / 'aruco_track_follower_metrics.json'
)


def parse_args(args=None):
    """Read vision, control, and safety settings while retaining ROS args."""
    parser = argparse.ArgumentParser(
        description=(
            'Detect ArUco DICT_4X4 marker ID 0 on /image_raw, draw an open '
            'straight road with static obstacles, and publish bounded RC '
            'commands to follow it.'
        )
    )
    parser.add_argument('--image-topic', default=IMAGE_TOPIC)
    parser.add_argument('--command-topic', default=COMMAND_TOPIC)
    parser.add_argument(
        '--telemetry-prefix',
        default='/aruco_track_follower/tuning',
        help='ROS topic prefix for live Float64 tuning telemetry.',
    )
    parser.add_argument(
        '--no-telemetry',
        action='store_true',
        help='Disable live tuning telemetry topics.',
    )
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-dict', default='DICT_4X4_50')
    parser.add_argument(
        '--aruco-marker-size-cm',
        type=float,
        default=ARUCO_MARKER_SIZE_CM,
        help='Physical side length of the ArUco marker used for CBF scaling.',
    )
    parser.add_argument(
        '--aruco-parallax-factor',
        type=float,
        default=0.0,
        help=(
            'Shift the detected ArUco control point along the marker front '
            'direction by this many marker-side lengths. Positive moves toward '
            'front-edge; negative moves away.'
        ),
    )
    parser.add_argument(
        '--front-edge',
        choices=('top', 'right', 'bottom', 'left'),
        default='top',
        help='Which ArUco marker edge points toward vehicle front.',
    )
    parser.add_argument('--track-center-x', type=float, default=0.5)
    parser.add_argument('--track-center-y', type=float, default=0.55)
    parser.add_argument(
        '--road-length-x',
        type=float,
        default=0.82,
        help='Straight road length as a fraction of image width.',
    )
    parser.add_argument(
        '--track-shape',
        choices=('straight_road',),
        default='straight_road',
        help='Virtual image-space track shape.',
    )
    parser.add_argument('--lookahead-points', type=int, default=10)
    parser.add_argument('--road-half-width-px', type=float, default=250.0)
    parser.add_argument(
        '--static-obstacles',
        default='',
        help=(
            'JSON list of static road obstacles. Each item may set progress, '
            'length_px, width_px, and lateral_offset_px.'
        ),
    )
    parser.add_argument('--obstacle-margin-px', type=float, default=42.0)
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
        default=1600,
        help='Resize debug preview to this width; 0 shows full size.',
    )
    parser.add_argument(
        '--virtual-vehicle-test',
        action='store_true',
        help='Run a synthetic image-space vehicle through the controller.',
    )
    parser.add_argument('--virtual-width', type=int, default=960)
    parser.add_argument('--virtual-height', type=int, default=540)
    parser.add_argument('--virtual-start-progress', type=float, default=0.04)
    parser.add_argument('--virtual-start-lateral-offset-px', type=float, default=70.0)
    parser.add_argument('--virtual-start-heading-deg', type=float, default=0.0)
    parser.add_argument('--virtual-start-speed-pps', type=float, default=0.0)
    parser.add_argument('--virtual-max-accel-pps2', type=float, default=120.0)
    parser.add_argument('--virtual-max-brake-pps2', type=float, default=220.0)
    parser.add_argument('--virtual-speed-response', type=float, default=2.0)
    parser.add_argument(
        '--virtual-stop-at-end',
        action='store_true',
        help='Stop the virtual test when the vehicle reaches the road end.',
    )
    parser.add_argument(
        '--virtual-unlimited-path',
        action='store_true',
        help='Wrap the virtual vehicle back to the road start at the road end.',
    )
    parser.add_argument(
        '--random-static-obstacles',
        action='store_true',
        help='Spawn random virtual obstacles that are not known at startup.',
    )
    parser.add_argument('--random-obstacle-count', type=int, default=5)
    parser.add_argument('--random-obstacle-seed', type=int, default=7)
    parser.add_argument('--random-obstacle-detection-range-px', type=float, default=260.0)
    parser.add_argument('--random-obstacle-min-progress', type=float, default=0.18)
    parser.add_argument('--random-obstacle-max-progress', type=float, default=0.92)
    parser.add_argument(
        '--dynamic-obstacles',
        action='store_true',
        help='Move virtual obstacles along the straight road over time.',
    )
    parser.add_argument(
        '--traffic-scenario',
        choices=(
            'free_flow',
            'lead_slowdown',
            'cut_in',
            'overtake_merge',
            'stop_go_platoon',
            'dense_flow',
            'bottleneck_merge',
            'lane_weave',
            'crossing_conflict',
            'signal_phase',
            'looping_flow',
        ),
        default='free_flow',
        help='Preset dynamic-traffic pattern used by straight_dynamic.',
    )
    parser.add_argument('--dynamic-obstacle-count', type=int, default=3)
    parser.add_argument('--dynamic-obstacle-seed', type=int, default=13)
    parser.add_argument('--dynamic-obstacle-speed-pps', type=float, default=0.045)
    parser.add_argument('--dynamic-obstacle-min-progress', type=float, default=0.18)
    parser.add_argument('--dynamic-obstacle-max-progress', type=float, default=0.92)
    parser.add_argument('--dynamic-obstacle-min-gap-progress', type=float, default=0.08)
    parser.add_argument(
        '--dynamic-obstacle-min-longitudinal-gap-px',
        type=float,
        default=18.0,
        help='Minimum bumper-to-bumper gap between dynamic obstacles.',
    )
    parser.add_argument(
        '--dynamic-obstacle-min-lateral-gap-px',
        type=float,
        default=8.0,
        help='Minimum side-to-side gap before obstacles may share progress.',
    )
    parser.add_argument('--steering-kp-px', type=float, default=STEERING_KP_PX)
    parser.add_argument('--steering-ki-px', type=float, default=STEERING_KI_PX)
    parser.add_argument('--steering-kd-px', type=float, default=STEERING_KD_PX)
    parser.add_argument('--heading-kp', type=float, default=HEADING_KP_RAD)
    parser.add_argument(
        '--integral-limit-px-s',
        type=float,
        default=INTEGRAL_LIMIT_PX_S,
    )
    parser.add_argument(
        '--controller-mode',
        choices=CONTROLLER_MODES,
        default=PID,
        help='Control variant used for throttle/pitch output.',
    )
    parser.add_argument('--target-track-speed-pps', type=float, default=38.0)
    parser.add_argument(
        '--gap-planner-mode',
        choices=('stable_free_space', 'free_space', 'centerline'),
        default='stable_free_space',
        help='How the lookahead setpoint is chosen around obstacles.',
    )
    parser.add_argument(
        '--gap-switch-hysteresis-px',
        type=float,
        default=GAP_SWITCH_HYSTERESIS_PX,
        help='Extra free width required before switching to another gap.',
    )
    parser.add_argument(
        '--gap-target-smoothing-alpha',
        type=float,
        default=GAP_TARGET_SMOOTHING_ALPHA,
        help='Low-pass coefficient for lateral free-space setpoint.',
    )
    parser.add_argument(
        '--track-speed-filter-alpha',
        type=float,
        default=0.35,
        help=(
            'Low-pass coefficient for progress speed. 1.0 disables smoothing; '
            'lower values reduce velocity PID jitter.'
        ),
    )
    parser.add_argument('--velocity-kp-pwm', type=float, default=2.0)
    parser.add_argument('--velocity-ki-pwm', type=float, default=0.0)
    parser.add_argument('--velocity-kd-pwm', type=float, default=0.05)
    parser.add_argument('--velocity-integral-limit', type=float, default=120.0)
    parser.add_argument('--max-forward-pwm', type=int, default=1600)
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
    parser.add_argument('--cbf-a-ell', type=float, default=2.0)
    parser.add_argument('--cbf-b-ell', type=float, default=1.0)
    parser.add_argument('--cbf-gamma1', type=float, default=2.0)
    parser.add_argument('--cbf-gamma2', type=float, default=1.0)
    parser.add_argument('--cbf-gamma3', type=float, default=1.0)
    parser.add_argument(
        '--qp-slack-weight',
        type=float,
        default=QP_SLACK_WEIGHT,
        help='Optional CBF-QP slack penalty. 0 disables slack.',
    )
    parser.add_argument('--forward-pwm', type=int, default=1590)
    parser.add_argument('--min-forward-pwm', type=int, default=1585)
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
        '--no-pid-panel',
        action='store_true',
        help='Do not open the live PID tuning slider panel.',
    )
    parser.add_argument(
        '--debug-commands',
        action='store_true',
        help='Print outgoing RC command values once per second.',
    )
    parser.add_argument(
        '--debug-visuals',
        action='store_true',
        help='Add dense control, safety, and planner overlays to preview.',
    )
    parser.add_argument(
        '--debug-trail-length',
        type=int,
        default=120,
        help='Number of marker centers to retain in the debug trail.',
    )
    parser.add_argument(
        '--debug-snapshot-dir',
        type=Path,
        default=Path('debug_snapshots'),
        help='Directory used for preview snapshots when P is pressed.',
    )
    parser.add_argument(
        '--ignore-preview-keys',
        action='store_true',
        help='Ignore OpenCV preview key shortcuts; useful if stale key events close the node.',
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
        default=DEFAULT_METRICS_FILE,
        help='Path used when saving ranking metrics on exit.',
    )
    parser.add_argument(
        '--target-laps',
        type=int,
        default=0,
        help='Reserved for compatibility; straight roads do not count laps.',
    )
    parser.add_argument(
        '--enable-lap-limit',
        action='store_true',
        help='Reserved for compatibility; straight roads do not count laps.',
    )
    parser.add_argument(
        '--add-road-end-walls',
        action='store_true',
        help='Add static obstacle walls at the start and end of the road.',
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
    parser.set_defaults(
        qp_wheelbase_px=QP_WHEELBASE_PX,
        qp_min_accel=QP_MIN_ACCEL,
        qp_max_accel=QP_MAX_ACCEL,
        qp_min_delta=QP_MIN_DELTA,
        qp_max_delta=QP_MAX_DELTA,
        qp_solver=QP_SOLVER,
        qp_slack_weight=QP_SLACK_WEIGHT,
    )
    return parser.parse_known_args(args)


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
    if not 0.0 < config.track_speed_filter_alpha <= 1.0:
        raise SystemExit('track-speed-filter-alpha must be in (0, 1]')
    if config.cbf_stop_heading_rad <= 0.0:
        raise SystemExit('cbf-stop-heading-rad must be positive')
    if config.cbf_h_px <= 0.0:
        raise SystemExit('cbf-h-px must be positive')
    if config.cbf_alpha <= 0.0:
        raise SystemExit('cbf-alpha must be positive')
    if config.cbf_a_ell <= 0.0:
        raise SystemExit('cbf-a-ell must be positive')
    if config.cbf_b_ell <= 0.0:
        raise SystemExit('cbf-b-ell must be positive')
    if config.cbf_gamma1 <= 0.0:
        raise SystemExit('cbf-gamma1 must be positive')
    if config.cbf_gamma2 <= 0.0:
        raise SystemExit('cbf-gamma2 must be positive')
    if config.cbf_gamma3 <= 0.0:
        raise SystemExit('cbf-gamma3 must be positive')
    if config.qp_wheelbase_px <= 0.0:
        raise SystemExit('qp-wheelbase-px must be positive')
    if config.qp_max_accel <= 0.0:
        raise SystemExit('qp-max-accel must be positive')
    if config.qp_min_accel >= 0.0:
        raise SystemExit('qp-min-accel must be negative')
    if config.qp_max_delta <= 0.0:
        raise SystemExit('qp-max-delta must be positive')
    if config.qp_min_delta >= 0.0:
        raise SystemExit('qp-min-delta must be negative')
    if config.qp_slack_weight < 0.0:
        raise SystemExit('qp-slack-weight must be non-negative')
    if config.aruco_marker_size_cm <= 0.0:
        raise SystemExit('aruco-marker-size-cm must be positive')
    if config.controller_mode == PID_VELOCITY_CBF_QP_ELLIPSE:
        try:
            import cvxpy  # noqa: F401
        except ImportError as exc:
            raise SystemExit(
                f'controller-mode {config.controller_mode} requires cvxpy; '
                'install cvxpy in this Python environment'
            ) from exc
    if config.command_timeout <= 0.0:
        raise SystemExit('command timeout must be positive')
    if config.lookahead_points < 1:
        raise SystemExit('lookahead-points must be at least 1')
    if config.gap_switch_hysteresis_px < 0.0:
        raise SystemExit('gap-switch-hysteresis-px must be non-negative')
    if not 0.0 < config.gap_target_smoothing_alpha <= 1.0:
        raise SystemExit('gap-target-smoothing-alpha must be in (0, 1]')
    if not 0.0 < config.road_length_x <= 1.0:
        raise SystemExit('road-length-x must be between 0 and 1')
    if config.virtual_width < 160:
        raise SystemExit('virtual-width must be at least 160')
    if config.virtual_height < 120:
        raise SystemExit('virtual-height must be at least 120')
    if not 0.0 <= config.virtual_start_progress <= 1.0:
        raise SystemExit('virtual-start-progress must be between 0 and 1')
    if config.virtual_max_accel_pps2 <= 0.0:
        raise SystemExit('virtual-max-accel-pps2 must be positive')
    if config.virtual_max_brake_pps2 <= 0.0:
        raise SystemExit('virtual-max-brake-pps2 must be positive')
    if config.virtual_speed_response <= 0.0:
        raise SystemExit('virtual-speed-response must be positive')
    if config.random_obstacle_count < 0:
        raise SystemExit('random-obstacle-count must be non-negative')
    if config.random_obstacle_detection_range_px <= 0.0:
        raise SystemExit('random-obstacle-detection-range-px must be positive')
    if not 0.0 <= config.random_obstacle_min_progress <= 1.0:
        raise SystemExit('random-obstacle-min-progress must be between 0 and 1')
    if not 0.0 <= config.random_obstacle_max_progress <= 1.0:
        raise SystemExit('random-obstacle-max-progress must be between 0 and 1')
    if config.random_obstacle_max_progress < config.random_obstacle_min_progress:
        raise SystemExit(
            'random-obstacle-max-progress must be >= random-obstacle-min-progress'
        )
    if config.dynamic_obstacle_count < 0:
        raise SystemExit('dynamic-obstacle-count must be non-negative')
    if config.dynamic_obstacle_speed_pps < 0.0:
        raise SystemExit('dynamic-obstacle-speed-pps must be non-negative')
    if not 0.0 <= config.dynamic_obstacle_min_progress <= 1.0:
        raise SystemExit('dynamic-obstacle-min-progress must be between 0 and 1')
    if not 0.0 <= config.dynamic_obstacle_max_progress <= 1.0:
        raise SystemExit('dynamic-obstacle-max-progress must be between 0 and 1')
    if config.dynamic_obstacle_max_progress < config.dynamic_obstacle_min_progress:
        raise SystemExit(
            'dynamic-obstacle-max-progress must be >= dynamic-obstacle-min-progress'
        )
    if not 0.0 <= config.dynamic_obstacle_min_gap_progress <= 1.0:
        raise SystemExit('dynamic-obstacle-min-gap-progress must be between 0 and 1')
    if config.dynamic_obstacle_min_longitudinal_gap_px < 0.0:
        raise SystemExit('dynamic-obstacle-min-longitudinal-gap-px must be non-negative')
    if config.dynamic_obstacle_min_lateral_gap_px < 0.0:
        raise SystemExit('dynamic-obstacle-min-lateral-gap-px must be non-negative')
    if config.road_half_width_px <= 0.0:
        raise SystemExit('road-half-width-px must be positive')
    if config.obstacle_margin_px <= 0.0:
        raise SystemExit('obstacle-margin-px must be positive')
    if not -3.0 <= config.aruco_parallax_factor <= 3.0:
        raise SystemExit('aruco-parallax-factor must be between -3.0 and 3.0')
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
    if not config.no_telemetry:
        print(f'telemetry={config.telemetry_prefix}')
    print(
        f'controller={config.controller_mode} track={config.track_shape} '
        f'target_speed={config.target_track_speed_pps:.1f} pps'
    )
    print(
        'road: '
        f'length={config.road_length_x:.2f} '
        f'half_width={config.road_half_width_px:.1f}px '
        f'obstacle_margin={config.obstacle_margin_px:.1f}px'
    )
    print(
        'gap target: '
        f'mode={config.gap_planner_mode} '
        f'hysteresis={config.gap_switch_hysteresis_px:.1f}px '
        f'smoothing_alpha={config.gap_target_smoothing_alpha:.2f}'
    )
    print(
        f'aruco: marker_size={config.aruco_marker_size_cm:.1f}cm '
        f'parallax_factor={config.aruco_parallax_factor:.3f}'
    )
    if config.virtual_vehicle_test:
        print(
            'virtual: '
            f'size={config.virtual_width}x{config.virtual_height} '
            f'start={config.virtual_start_progress:.2f} '
            f'offset={config.virtual_start_lateral_offset_px:.1f}px '
            f'unlimited_path={config.virtual_unlimited_path}'
        )
    if config.random_static_obstacles:
        print(
            'random obstacles: '
            f'count={config.random_obstacle_count} '
            f'seed={config.random_obstacle_seed} '
            f'detect={config.random_obstacle_detection_range_px:.0f}px'
        )
    if config.dynamic_obstacles:
        print(
            'dynamic obstacles: '
            f'scenario={config.traffic_scenario} '
            f'count={config.dynamic_obstacle_count} '
            f'seed={config.dynamic_obstacle_seed} '
            f'speed={config.dynamic_obstacle_speed_pps:.3f} progress/s '
            f'gap={config.dynamic_obstacle_min_gap_progress:.2f} '
            f'obs_gap={config.dynamic_obstacle_min_longitudinal_gap_px:.0f}px'
        )
    print(
        'steering PWM: '
        f'left={config.left_pwm} center={config.center_steering_pwm} '
        f'right={config.right_pwm} heading_kp={config.heading_kp:.1f}'
    )
    print(
        'throttle PWM: '
        f'forward={config.forward_pwm} '
        f'min_forward={config.min_forward_pwm} '
        f'max_forward={config.max_forward_pwm} '
        f'neutral={config.neutral_throttle_pwm} '
        f'channel={config.drive_channel}'
    )
    if config.controller_mode == PID_VELOCITY_CBF_QP_ELLIPSE:
        print(
            'qp-cbf: '
            f'a_ell={config.cbf_a_ell:.2f} '
            f'b_ell={config.cbf_b_ell:.2f} '
            f'gamma1={config.cbf_gamma1:.2f} '
            f'gamma2={config.cbf_gamma2:.2f} '
            f'gamma3={config.cbf_gamma3:.2f} '
            f'slack_weight={config.qp_slack_weight:.1f}'
        )
