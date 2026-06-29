"""Command-line configuration parser and validator for HIL simulation."""

import argparse
import json
from pathlib import Path

# Controller modes definition for compatibility
PID = 'pid'
PID_VELOCITY_CBF_QP_ELLIPSE = 'pid_velocity_cbf_qp_ellipse'
PID_VELOCITY_DCLF_DCBF = 'pid_velocity_dclf_dcbf'
MPC_CBF = 'mpc_cbf'
CONTROLLER_MODES = (PID, PID_VELOCITY_CBF_QP_ELLIPSE, PID_VELOCITY_DCLF_DCBF, MPC_CBF)

# Defaults
IMAGE_TOPIC = '/image_raw'
COMMAND_TOPIC = '/drone/rc_command'
ARMING_SERVICE = '/drone/cmd/arming'
NEUTRAL_VALUE = 1500
UPDATE_RATE_HZ = 50.0
SETTLE_DURATION = 0.5

QP_WHEELBASE_PX = 2.0
QP_MIN_ACCEL = -10.0
QP_MAX_ACCEL = 0.01
QP_MIN_DELTA = -1.3
QP_MAX_DELTA = 1.3
QP_SOLVER = 'OSQP'
QP_SLACK_WEIGHT = 2000.0
QP_MAX_OBSTACLES = 4

GAP_SWITCH_HYSTERESIS_PX = 25.0
GAP_TARGET_SMOOTHING_ALPHA = 0.18
FTG_MAX_RANGE_PX = 500.0
FTG_BUBBLE_RADIUS_PX = 0.0

STEERING_KP_PX = 15.0
STEERING_KI_PX = 0.0
STEERING_KD_PX = 0.25
HEADING_KP_RAD = 120.0
INTEGRAL_LIMIT_PX_S = 250.0
ARUCO_MARKER_SIZE_CM = 10.0

DEFAULT_METRICS_FILE = Path(__file__).resolve().parent / 'dynamic_straight_metrics.json'

def parse_args(args=None):
    """Read simulator settings and parameters."""
    parser = argparse.ArgumentParser(
        description='Simulation launcher and visualizer.'
    )
    parser.add_argument('--image-topic', default=IMAGE_TOPIC)
    parser.add_argument('--command-topic', default=COMMAND_TOPIC)
    parser.add_argument('--ignore-subscribers', action='store_true')
    parser.add_argument('--telemetry-prefix', default='/dynamic_straight/tuning')
    parser.add_argument('--no-telemetry', action='store_true')
    parser.add_argument('--telemetry-max-fps', type=float, default=10.0)
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-dict', default='DICT_4X4_50')
    parser.add_argument('--aruco-marker-size-cm', type=float, default=ARUCO_MARKER_SIZE_CM)
    parser.add_argument('--aruco-parallax-factor', type=float, default=0.0)
    parser.add_argument('--front-edge', choices=('top', 'right', 'bottom', 'left'), default='top')
    parser.add_argument('--track-center-x', type=float, default=0.5)
    parser.add_argument('--track-center-y', type=float, default=0.55)
    parser.add_argument('--road-length-x', type=float, default=0.82)
    parser.add_argument('--track-shape', choices=('straight_road',), default='straight_road')
    parser.add_argument('--lookahead-points', type=int, default=10)
    parser.add_argument('--road-half-width-px', type=float, default=250.0)
    parser.add_argument('--lidar-heading-offset-rad', type=float, default=0.0)
    parser.add_argument('--static-obstacles', default='')
    parser.add_argument('--obstacle-margin-px', type=float, default=42.0)
    parser.add_argument('--process-width', type=int, default=640)
    parser.add_argument('--aruco-fallback-full-res', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--max-frame-age', type=float, default=0.08)
    parser.add_argument('--preview-width', type=int, default=0)
    parser.add_argument('--virtual-vehicle-test', action='store_true', default=True)
    parser.add_argument('--virtual-width', type=int, default=960)
    parser.add_argument('--virtual-height', type=int, default=540)
    parser.add_argument('--virtual-start-progress', type=float, default=0.04)
    parser.add_argument('--virtual-start-lateral-offset-px', type=float, default=70.0)
    parser.add_argument('--virtual-start-heading-deg', type=float, default=0.0)
    parser.add_argument('--virtual-start-speed-pps', type=float, default=0.0)
    parser.add_argument('--virtual-marker-size-px', type=float, default=10.0)
    parser.add_argument('--virtual-max-accel-pps2', type=float, default=120.0)
    parser.add_argument('--virtual-max-brake-pps2', type=float, default=220.0)
    parser.add_argument('--virtual-speed-response', type=float, default=2.0)
    parser.add_argument('--virtual-stop-at-end', action='store_true')
    parser.add_argument('--virtual-unlimited-path', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--random-static-obstacles', action='store_true')
    parser.add_argument('--random-obstacle-count', type=int, default=5)
    parser.add_argument('--random-obstacle-seed', type=int, default=7)
    parser.add_argument('--random-obstacle-detection-range-px', type=float, default=260.0)
    parser.add_argument('--random-obstacle-min-progress', type=float, default=0.18)
    parser.add_argument('--random-obstacle-max-progress', type=float, default=0.92)
    
    parser.add_argument('--dynamic-obstacles', action='store_false', default=True)
    parser.add_argument(
        '--traffic-scenario',
        choices=(
            'free_flow', 'lead_slowdown', 'cut_in', 'overtake_merge',
            'stop_go_platoon', 'dense_flow', 'bottleneck_merge', 'lane_weave',
            'crossing_conflict', 'signal_phase', 'looping_flow', 'three_sparse',
            'head_on', 'endless_walls'
        ),
        default='free_flow'
    )
    parser.add_argument('--dynamic-obstacle-count', type=int, default=6)
    parser.add_argument('--dynamic-obstacle-seed', type=int, default=13)
    parser.add_argument('--dynamic-obstacle-speed-pps', type=float, default=0.045)
    parser.add_argument('--dynamic-obstacle-min-progress', type=float, default=0.18)
    parser.add_argument('--dynamic-obstacle-max-progress', type=float, default=0.92)
    parser.add_argument('--dynamic-obstacle-min-gap-progress', type=float, default=0.08)
    parser.add_argument('--dynamic-obstacle-min-longitudinal-gap-px', type=float, default=18.0)
    parser.add_argument('--dynamic-obstacle-min-lateral-gap-px', type=float, default=8.0)
    
    # Steering PID
    parser.add_argument('--steering-kp-px', type=float, default=STEERING_KP_PX)
    parser.add_argument('--steering-ki-px', type=float, default=STEERING_KI_PX)
    parser.add_argument('--steering-kd-px', type=float, default=STEERING_KD_PX)
    parser.add_argument('--heading-kp', type=float, default=HEADING_KP_RAD)
    parser.add_argument('--integral-limit-px-s', type=float, default=INTEGRAL_LIMIT_PX_S)
    
    # Mode and throttle targets
    parser.add_argument('--controller-mode', choices=CONTROLLER_MODES, default=PID)
    parser.add_argument('--target-track-speed-pps', type=float, default=38.0)
    parser.add_argument('--gap-planner-mode', choices=('stable_free_space', 'free_space', 'centerline', 'follow_the_gap_advanced'), default='follow_the_gap_advanced')
    parser.add_argument('--gap-switch-hysteresis-px', type=float, default=GAP_SWITCH_HYSTERESIS_PX)
    parser.add_argument('--gap-target-smoothing-alpha', type=float, default=GAP_TARGET_SMOOTHING_ALPHA)
    parser.add_argument('--ftg-max-range-px', type=float, default=FTG_MAX_RANGE_PX)
    parser.add_argument('--ftg-bubble-radius-px', type=float, default=FTG_BUBBLE_RADIUS_PX)
    parser.add_argument('--ftg-stuck-speed-pps', type=float, default=2.0)
    parser.add_argument('--ftg-stuck-forward-pwm', type=float, default=18.0)
    parser.add_argument('--ftg-stuck-steering-gain-pwm', type=float, default=80.0)
    parser.add_argument('--ftg-stuck-max-steering-bias-pwm', type=float, default=70.0)
    
    # Velocity PID & PWM maps
    parser.add_argument('--track-speed-filter-alpha', type=float, default=0.35)
    parser.add_argument('--track-speed-slew-rate-pps2', type=float, default=120.0)
    parser.add_argument('--velocity-kp-pwm', type=float, default=2.0)
    parser.add_argument('--velocity-ki-pwm', type=float, default=0.0)
    parser.add_argument('--velocity-kd-pwm', type=float, default=0.05)
    parser.add_argument('--velocity-integral-limit', type=float, default=120.0)
    parser.add_argument('--max-forward-pwm', type=int, default=1590)
    
    # CBF-QP Ellipse limits
    parser.add_argument('--cbf-slow-error-px', type=float, default=55.0)
    parser.add_argument('--cbf-stop-error-px', type=float, default=120.0)
    parser.add_argument('--cbf-stop-heading-rad', type=float, default=1.2)
    parser.add_argument('--cbf-h-px', type=float, default=250.0)
    parser.add_argument('--cbf-alpha', type=float, default=2.0)
    parser.add_argument('--clf-alpha', type=float, default=0.1)
    parser.add_argument('--clf-slack-weight', type=float, default=500.0)
    parser.add_argument('--cbf-a-ell', type=float, default=4.3)
    parser.add_argument('--cbf-b-ell', type=float, default=2.6)
    parser.add_argument('--cbf-gamma1', type=float, default=5.02)
    parser.add_argument('--cbf-gamma2', type=float, default=1.57)
    parser.add_argument('--cbf-gamma3', type=float, default=7.79)
    
    # Hardware constraints
    parser.add_argument('--forward-pwm', type=int, default=1590)
    parser.add_argument('--min-forward-pwm', type=int, default=1585)
    parser.add_argument('--neutral-throttle-pwm', type=int, default=1500)
    parser.add_argument('--left-pwm', type=int, default=1850)
    parser.add_argument('--center-steering-pwm', type=int, default=1500)
    parser.add_argument('--right-pwm', type=int, default=1150)
    parser.add_argument('--drive-channel', default='pitch')
    parser.add_argument('--command-timeout', type=float, default=0.25)
    
    # OSQP/Tuning
    parser.add_argument('--preview', action='store_true', default=True)
    parser.add_argument('--no-pid-panel', action='store_true')
    parser.add_argument('--snapshot-dir', default='snapshots')
    parser.add_argument('--ignore-preview-keys', action='store_true')
    parser.add_argument('--single-thread', action='store_true')
    parser.add_argument('--executor-threads', type=int, default=4)
    parser.add_argument('--tuning-file', type=Path, default=Path('aruco_track_follower_tuning.json'))
    parser.add_argument('--load-tuning', action='store_true')
    parser.add_argument('--load-cbf-tuning', action='store_true', default=True)
    parser.add_argument('--metrics-file', type=Path, default=DEFAULT_METRICS_FILE)
    parser.add_argument('--add-road-end-walls', action='store_true')
    parser.add_argument('--include-road-boundary-walls', action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument('--dry-run', action='store_true', default=True)
    parser.add_argument('--confirm-propulsion-safe', action='store_true', default=True)
    parser.add_argument('--load', choices=('tuning',), default=None, help=argparse.SUPPRESS)
    parser.add_argument('--enable-lap-limit', action='store_true')
    parser.add_argument('--target-laps', type=int, default=0)
    parser.add_argument('--cbf-edge-margin-px', type=float, default=45.0)
    parser.add_argument('--debug-commands', action='store_true')
    parser.add_argument('--debug-print-interval-s', type=float, default=1.0)
    parser.add_argument('--debug-snapshot-dir', type=Path, default=Path('debug_snapshots'))
    parser.add_argument('--debug-trail-length', type=int, default=120)
    parser.add_argument('--debug-visuals', action='store_true')
    parser.add_argument('--use-safe-control-env', action='store_true')

    config, remaining = parser.parse_known_args(args)
    if config.load == 'tuning':
        config.load_tuning = True

    # Inject defaults for setup options
    setattr(config, 'qp_wheelbase_px', QP_WHEELBASE_PX)
    setattr(config, 'qp_min_accel', QP_MIN_ACCEL)
    setattr(config, 'qp_max_accel', QP_MAX_ACCEL)
    setattr(config, 'qp_min_delta', QP_MIN_DELTA)
    setattr(config, 'qp_max_delta', QP_MAX_DELTA)
    setattr(config, 'qp_solver', QP_SOLVER)
    setattr(config, 'qp_slack_weight', QP_SLACK_WEIGHT)
    setattr(config, 'qp_max_obstacles', QP_MAX_OBSTACLES)

    return config, remaining

def validate_config(config):
    """Reject invalid controller calibrations."""
    for label in (
        'forward_pwm', 'neutral_throttle_pwm', 'left_pwm',
        'center_steering_pwm', 'right_pwm', 'min_forward_pwm', 'max_forward_pwm',
    ):
        value = getattr(config, label)
        if not 988 <= value <= 2012:
            raise SystemExit(f'{label} must be between 988 and 2012')
    if config.forward_pwm < config.min_forward_pwm:
        raise SystemExit('forward-pwm must be >= min-forward-pwm')
    if config.max_forward_pwm < config.min_forward_pwm:
        raise SystemExit('max-forward-pwm must be >= min-forward-pwm')
    if config.cbf_stop_error_px <= config.cbf_slow_error_px:
        raise SystemExit('cbf-stop-error-px must be > cbf-slow-error-px')
    if not 0.0 < config.track_speed_filter_alpha <= 1.0:
        raise SystemExit('track-speed-filter-alpha must be in (0, 1]')
    if config.track_speed_slew_rate_pps2 <= 0.0:
        raise SystemExit('track-speed-slew-rate-pps2 must be positive')
    if config.cbf_stop_heading_rad <= 0.0:
        raise SystemExit('cbf-stop-heading-rad must be positive')
    if config.cbf_h_px <= 0.0:
        raise SystemExit('cbf-h-px must be positive')
    if config.cbf_alpha <= 0.0:
        raise SystemExit('cbf-alpha must be positive')
    if config.clf_alpha < 0.0:
        raise SystemExit('clf-alpha must be non-negative')
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
        raise SystemExit('random-obstacle-max-progress must be >= random-obstacle-min-progress')
    if config.dynamic_obstacle_count < 0:
        raise SystemExit('dynamic-obstacle-count must be non-negative')
    if config.dynamic_obstacle_speed_pps < 0.0:
        raise SystemExit('dynamic-obstacle-speed-pps must be non-negative')
    if not 0.0 <= config.dynamic_obstacle_min_progress <= 1.0:
        raise SystemExit('dynamic-obstacle-min-progress must be between 0 and 1')
    if not 0.0 <= config.dynamic_obstacle_max_progress <= 1.0:
        raise SystemExit('dynamic-obstacle-max-progress must be between 0 and 1')
    if config.dynamic_obstacle_max_progress < config.dynamic_obstacle_min_progress:
        raise SystemExit('dynamic-obstacle-max-progress must be >= dynamic-obstacle-min-progress')
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

def print_config(config):
    """Print settings of launcher."""
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
        f'right={config.right_pwm}'
    )
    print(
        'throttle PWM: '
        f'forward={config.forward_pwm} '
        f'min_forward={config.min_forward_pwm} '
        f'max_forward={config.max_forward_pwm} '
        f'neutral={config.neutral_throttle_pwm}'
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
        print(
            'qp bounds: '
            f'wheelbase={config.qp_wheelbase_px:.3f}px '
            f'accel=[{config.qp_min_accel:.4f}, {config.qp_max_accel:.4f}] '
            f'delta=[{config.qp_min_delta:.3f}, {config.qp_max_delta:.3f}] '
            f'solver={config.qp_solver}'
        )
