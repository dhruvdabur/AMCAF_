"""Metrics collection for image-space ArUco followers."""

import math
import time


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
