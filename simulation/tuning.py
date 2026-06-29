"""Interactive OpenCV tuning panel for simulation parameters."""

import json
from pathlib import Path
import cv2

PID_SCALE = 1000.0
HEADING_SCALE = 10.0
CBF_ELLIPSE_SCALE = 10.0
CBF_GAMMA_SCALE = 100.0

def noop(val):
    """Placeholder callback for trackbars."""
    pass

class StraightStaticTuning:
    """OpenCV tuning panel window for dynamic parameter adjustment."""

    def __init__(self, config, controller, logger=None, enabled=True):
        self.config = config
        self.controller = controller
        self.logger = logger
        self.enabled = enabled

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

    def create_pid_panel(self):
        """Open trackbars for live HIL tuning."""
        if not self.enabled:
            return
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 720, 620)
        preview_width = getattr(self.config, 'virtual_width', 960) or 640
        cv2.moveWindow(self.window_name, 50 + int(preview_width) + 20, 50)
        
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
            int(round(self.config.clf_alpha * 100)),
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
            int(round(self.controller.heading_kp * HEADING_SCALE)),
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
            'A_ELL x10',
            self.window_name,
            int(round(self.config.cbf_a_ell * CBF_ELLIPSE_SCALE)),
            50,
            noop,
        )
        cv2.createTrackbar(
            'B_ELL x10',
            self.window_name,
            int(round(self.config.cbf_b_ell * CBF_ELLIPSE_SCALE)),
            50,
            noop,
        )
        cv2.createTrackbar(
            'GAMMA1 x100',
            self.window_name,
            int(round(self.config.cbf_gamma1 * CBF_GAMMA_SCALE)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'GAMMA2 x100',
            self.window_name,
            int(round(self.config.cbf_gamma2 * CBF_GAMMA_SCALE)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'GAMMA3 x100',
            self.window_name,
            int(round(self.config.cbf_gamma3 * CBF_GAMMA_SCALE)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'Lap limit',
            self.window_name,
            1 if self.controller.lap_limit_enabled else 0,
            1,
            noop,
        )
        cv2.createTrackbar(
            'Target laps',
            self.window_name,
            self.controller.target_laps,
            20,
            noop,
        )
        if self.config.forward_pwm < self.config.min_forward_pwm:
            cv2.setTrackbarPos('Throttle', self.window_name, self.config.min_forward_pwm)

    def read_panel_values(self):
        """Read current trackbar slider settings."""
        if not self.enabled:
            return {}
        
        # Safe lookup mapping in case trackbars aren't created yet
        def read_pos(name, default):
            try:
                pos = cv2.getTrackbarPos(name, self.window_name)
                return pos if pos >= 0 else default
            except Exception:
                return default

        values = {
            'steering_kp_px': read_pos('Kp x1000', int(round(self.config.steering_kp_px * PID_SCALE))) / PID_SCALE,
            'steering_ki_px': read_pos('Ki x1000', int(round(self.config.steering_ki_px * PID_SCALE))) / PID_SCALE,
            'steering_kd_px': read_pos('Kd x1000', int(round(self.config.steering_kd_px * PID_SCALE))) / PID_SCALE,
            'heading_kp': read_pos('Heading x10', int(round(self.controller.heading_kp * HEADING_SCALE))) / HEADING_SCALE,
            'forward_pwm': int(read_pos('Throttle', self.config.forward_pwm)),
            'cbf_a_ell': read_pos('A_ELL x10', int(round(self.config.cbf_a_ell * CBF_ELLIPSE_SCALE))) / CBF_ELLIPSE_SCALE,
            'cbf_b_ell': read_pos('B_ELL x10', int(round(self.config.cbf_b_ell * CBF_ELLIPSE_SCALE))) / CBF_ELLIPSE_SCALE,
            'cbf_gamma1': read_pos('GAMMA1 x100', int(round(self.config.cbf_gamma1 * CBF_GAMMA_SCALE))) / CBF_GAMMA_SCALE,
            'cbf_gamma2': read_pos('GAMMA2 x100', int(round(self.config.cbf_gamma2 * CBF_GAMMA_SCALE))) / CBF_GAMMA_SCALE,
            'cbf_gamma3': read_pos('GAMMA3 x100', int(round(self.config.cbf_gamma3 * CBF_GAMMA_SCALE))) / CBF_GAMMA_SCALE,
            'lap_limit_enabled': bool(read_pos('Lap limit', 0)),
            'target_laps': int(read_pos('Target laps', 0)),
            'lookahead_points': int(read_pos('Lookahead pts', self.config.lookahead_points)),
            'clf_alpha': read_pos('CLF alpha x100', int(round(self.config.clf_alpha * 100))) / 100.0,
            'qp_slack_weight': float(read_pos('CBF slack wt', int(getattr(self.config, 'qp_slack_weight', 2000)))),
            'clf_slack_weight': float(read_pos('CLF slack wt', int(getattr(self.config, 'clf_slack_weight', 500)))),
        }
        
        # Clamp to bounds
        values['forward_pwm'] = int(
            max(self.config.min_forward_pwm, min(values['forward_pwm'], self.config.max_forward_pwm))
        )
        values['cbf_a_ell'] = max(0.01, values['cbf_a_ell'])
        values['cbf_b_ell'] = max(0.01, values['cbf_b_ell'])
        values['cbf_gamma1'] = max(0.01, values['cbf_gamma1'])
        values['cbf_gamma2'] = max(0.01, values['cbf_gamma2'])
        values['cbf_gamma3'] = max(0.01, values['cbf_gamma3'])
        return values

    def update_pid_from_panel(self):
        if not self.enabled:
            return
        self.controller.apply_tuning_values(self.read_panel_values())

    def current_tuning_values(self):
        values = self.read_panel_values() if self.enabled else None
        # Gather CLI parameters and controller values
        c_values = {
            'steering_kp_px': self.config.steering_kp_px,
            'steering_ki_px': self.config.steering_ki_px,
            'steering_kd_px': self.config.steering_kd_px,
            'heading_kp': self.controller.heading_kp,
            'forward_pwm': self.config.forward_pwm,
            'min_forward_pwm': self.config.min_forward_pwm,
            'max_forward_pwm': self.config.max_forward_pwm,
            'target_track_speed_pps': self.controller.target_track_speed_pps,
            'track_speed_filter_alpha': self.config.track_speed_filter_alpha,
            'track_speed_slew_rate_pps2': self.config.track_speed_slew_rate_pps2,
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
            'road_half_width_px': self.config.road_half_width_px,
            'obstacle_margin_px': self.config.obstacle_margin_px,
            'lidar_heading_offset_rad': getattr(self.config, 'lidar_heading_offset_rad', 0.0),
            'lookahead_points': self.config.lookahead_points,
            'clf_alpha': self.config.clf_alpha,
            'clf_slack_weight': self.config.clf_slack_weight,
            'lap_limit_enabled': self.controller.lap_limit_enabled,
            'target_laps': self.controller.target_laps,
        }
        if values:
            c_values.update(values)
        return c_values

    def save_tuning_file(self):
        values = self.current_tuning_values()
        tuning_path = Path(self.config.tuning_file)
        tuning_path.parent.mkdir(parents=True, exist_ok=True)
        tuning_path.write_text(json.dumps(values, indent=2) + '\n', encoding='utf-8')
        if self.logger is not None:
            self.logger.info(f'saved tuning to {tuning_path}')

    def load_tuning_file(self):
        tuning_path = Path(self.config.tuning_file)
        if not tuning_path.exists():
            if self.logger is not None:
                self.logger.warning(f'tuning file not found: {tuning_path}')
            return
        payload = json.loads(tuning_path.read_text(encoding='utf-8'))
        
        config_keys = [
            'min_forward_pwm', 'max_forward_pwm', 'aruco_parallax_factor',
            'target_track_speed_pps', 'track_speed_filter_alpha', 'track_speed_slew_rate_pps2',
            'velocity_kp_pwm', 'velocity_ki_pwm', 'velocity_kd_pwm', 'heading_kp',
            'cbf_a_ell', 'cbf_b_ell', 'cbf_gamma1', 'cbf_gamma2', 'cbf_gamma3',
            'qp_wheelbase_px', 'qp_min_accel', 'qp_max_accel', 'qp_min_delta', 'qp_max_delta',
            'qp_solver', 'qp_slack_weight', 'qp_max_obstacles', 'road_half_width_px',
            'obstacle_margin_px', 'lidar_heading_offset_rad', 'lookahead_points',
            'clf_alpha', 'clf_slack_weight', 'target_laps'
        ]
        for key in config_keys:
            if key in payload:
                setattr(self.config, key, payload[key])
                
        value_keys = config_keys + [
            'steering_kp_px', 'steering_ki_px', 'steering_kd_px', 'forward_pwm', 'lap_limit_enabled'
        ]
        values = {key: payload[key] for key in value_keys if key in payload}
        self.controller.apply_tuning_values(values)
        if self.logger is not None:
            self.logger.info(f'loaded tuning from {tuning_path}')
