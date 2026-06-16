"""Tuning panel and tuning-file support for ellipse_static."""

import json

import cv2

from ..common import bounded
from ..config.ellipse_static import CBF_ELLIPSE_SCALE
from ..config.ellipse_static import CBF_GAMMA_SCALE
from ..config.ellipse_static import HEADING_SCALE
from ..config.ellipse_static import PID_SCALE
from ..config.ellipse_static import PID_WINDOW
from ..ui import noop


class EllipseStaticTuning:
    """Own OpenCV tuning widgets and tuning-file load/save."""

    def __init__(self, config, controller, logger=None, enabled=True):
        self.config = config
        self.controller = controller
        self.logger = logger
        self.enabled = enabled

    def create_pid_panel(self):
        """Open live PID sliders for field tuning."""
        cv2.namedWindow(PID_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(PID_WINDOW, 520, 320)
        cv2.createTrackbar(
            'Kp x1000',
            PID_WINDOW,
            int(round(self.config.steering_kp_px * PID_SCALE)),
            5000,
            noop,
        )
        cv2.createTrackbar(
            'Ki x1000',
            PID_WINDOW,
            int(round(self.config.steering_ki_px * PID_SCALE)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'Kd x1000',
            PID_WINDOW,
            int(round(self.config.steering_kd_px * PID_SCALE)),
            5000,
            noop,
        )
        cv2.createTrackbar(
            'Heading x10',
            PID_WINDOW,
            int(round(self.controller.heading_kp * HEADING_SCALE)),
            3000,
            noop,
        )
        cv2.createTrackbar(
            'Throttle',
            PID_WINDOW,
            self.config.forward_pwm,
            self.config.max_forward_pwm,
            noop,
        )
        cv2.createTrackbar(
            'A_ELL x10',
            PID_WINDOW,
            int(round(self.config.cbf_a_ell * CBF_ELLIPSE_SCALE)),
            50,
            noop,
        )
        cv2.createTrackbar(
            'B_ELL x10',
            PID_WINDOW,
            int(round(self.config.cbf_b_ell * CBF_ELLIPSE_SCALE)),
            50,
            noop,
        )
        cv2.createTrackbar(
            'GAMMA1 x100',
            PID_WINDOW,
            int(round(self.config.cbf_gamma1 * CBF_GAMMA_SCALE)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'GAMMA2 x100',
            PID_WINDOW,
            int(round(self.config.cbf_gamma2 * CBF_GAMMA_SCALE)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'GAMMA3 x100',
            PID_WINDOW,
            int(round(self.config.cbf_gamma3 * CBF_GAMMA_SCALE)),
            2000,
            noop,
        )
        cv2.createTrackbar(
            'Lap limit',
            PID_WINDOW,
            1 if self.controller.lap_limit_enabled else 0,
            1,
            noop,
        )
        cv2.createTrackbar(
            'Target laps',
            PID_WINDOW,
            self.controller.target_laps,
            20,
            noop,
        )
        if self.config.forward_pwm < self.config.min_forward_pwm:
            cv2.setTrackbarPos('Throttle', PID_WINDOW, self.config.min_forward_pwm)

    def read_panel_values(self):
        """Read live slider values without mutating controller state."""
        values = {
            'steering_kp_px': cv2.getTrackbarPos('Kp x1000', PID_WINDOW)
            / PID_SCALE,
            'steering_ki_px': cv2.getTrackbarPos('Ki x1000', PID_WINDOW)
            / PID_SCALE,
            'steering_kd_px': cv2.getTrackbarPos('Kd x1000', PID_WINDOW)
            / PID_SCALE,
            'heading_kp': cv2.getTrackbarPos('Heading x10', PID_WINDOW)
            / HEADING_SCALE,
            'forward_pwm': int(cv2.getTrackbarPos('Throttle', PID_WINDOW)),
            'cbf_a_ell': cv2.getTrackbarPos('A_ELL x10', PID_WINDOW)
            / CBF_ELLIPSE_SCALE,
            'cbf_b_ell': cv2.getTrackbarPos('B_ELL x10', PID_WINDOW)
            / CBF_ELLIPSE_SCALE,
            'cbf_gamma1': cv2.getTrackbarPos('GAMMA1 x100', PID_WINDOW)
            / CBF_GAMMA_SCALE,
            'cbf_gamma2': cv2.getTrackbarPos('GAMMA2 x100', PID_WINDOW)
            / CBF_GAMMA_SCALE,
            'cbf_gamma3': cv2.getTrackbarPos('GAMMA3 x100', PID_WINDOW)
            / CBF_GAMMA_SCALE,
            'lap_limit_enabled': bool(cv2.getTrackbarPos('Lap limit', PID_WINDOW)),
            'target_laps': int(cv2.getTrackbarPos('Target laps', PID_WINDOW)),
        }
        values['forward_pwm'] = int(
            bounded(
                values['forward_pwm'],
                self.config.min_forward_pwm,
                self.config.max_forward_pwm,
            )
        )
        values['cbf_a_ell'] = max(0.01, values['cbf_a_ell'])
        values['cbf_b_ell'] = max(0.01, values['cbf_b_ell'])
        values['cbf_gamma1'] = max(0.01, values['cbf_gamma1'])
        values['cbf_gamma2'] = max(0.01, values['cbf_gamma2'])
        values['cbf_gamma3'] = max(0.01, values['cbf_gamma3'])
        return values

    def update_pid_from_panel(self):
        """Read live PID sliders and update controller gains."""
        if not self.enabled:
            return
        self.controller.apply_tuning_values(self.read_panel_values())

    def current_tuning_values(self):
        """Return latest tuning values, including CLI-only mode settings."""
        values = self.read_panel_values() if self.enabled else None
        return self.controller.current_tuning_values(values)

    def save_tuning_file(self):
        """Save current tuning values to disk."""
        values = self.current_tuning_values()
        self.config.tuning_file.parent.mkdir(parents=True, exist_ok=True)
        self.config.tuning_file.write_text(
            json.dumps(values, indent=2) + '\n',
            encoding='utf-8',
        )
        if self.logger is not None:
            self.logger.info(f'saved tuning to {self.config.tuning_file}')

    def load_tuning_file(self):
        """Load tuning values from disk if available."""
        if not self.config.tuning_file.exists():
            if self.logger is not None:
                self.logger.warning(
                    f'tuning file not found: {self.config.tuning_file}'
                )
            return
        payload = json.loads(self.config.tuning_file.read_text(encoding='utf-8'))
        config_keys = [
            'min_forward_pwm',
            'max_forward_pwm',
            'aruco_parallax_factor',
            'target_track_speed_pps',
            'track_speed_filter_alpha',
            'track_speed_slew_rate_pps2',
            'velocity_kp_pwm',
            'velocity_ki_pwm',
            'velocity_kd_pwm',
            'heading_kp',
            'cbf_a_ell',
            'cbf_b_ell',
            'cbf_gamma1',
            'cbf_gamma2',
            'cbf_gamma3',
            'qp_wheelbase_px',
            'qp_min_accel',
            'qp_max_accel',
            'qp_min_delta',
            'qp_max_delta',
            'qp_solver',
            'qp_slack_weight',
            'enable_lap_limit',
            'target_laps',
        ]
        for key in config_keys:
            if key in payload:
                setattr(self.config, key, payload[key])
        value_keys = (
            'steering_kp_px',
            'steering_ki_px',
            'steering_kd_px',
            'heading_kp',
            'forward_pwm',
            'target_track_speed_pps',
            'track_speed_filter_alpha',
            'track_speed_slew_rate_pps2',
            'velocity_kp_pwm',
            'velocity_ki_pwm',
            'velocity_kd_pwm',
            'lap_limit_enabled',
            'target_laps',
            'cbf_a_ell',
            'cbf_b_ell',
            'cbf_gamma1',
            'cbf_gamma2',
            'cbf_gamma3',
            'qp_wheelbase_px',
            'qp_min_accel',
            'qp_max_accel',
            'qp_min_delta',
            'qp_max_delta',
            'qp_solver',
            'qp_slack_weight',
        )
        values = {key: payload[key] for key in value_keys if key in payload}
        if 'enable_lap_limit' in payload and 'lap_limit_enabled' not in values:
            values['lap_limit_enabled'] = payload['enable_lap_limit']
        print(f"[TUNING_LOAD_DEBUG] File={self.config.tuning_file} | payload steering_kp_px={payload.get('steering_kp_px')} | values steering_kp_px={values.get('steering_kp_px')}")
        self.controller.apply_tuning_values(values)
        if self.logger is not None:
            self.logger.info(f'loaded tuning from {self.config.tuning_file}')
