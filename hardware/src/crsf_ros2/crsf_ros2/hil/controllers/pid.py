"""PID and PID+CBF controller primitives used by hil follower modes."""

from dataclasses import dataclass
import math
import time


@dataclass
class LidarPoint:
    """One virtual lidar return in image-pixel coordinates."""

    distance_px: float
    angle_rad: float
    x_px: float
    y_px: float


@dataclass
class CBFResult:
    """Safety-filter output for one controller step."""

    throttle_pwm: float
    scale: float
    active: bool
    safety_clearance_px: float
    lidar_clearance_px: float
    edge_clearance_px: float
    error_clearance_px: float
    heading_clearance_px: float


@dataclass
class BarrierState:
    """One safety barrier sample for derivative-based CBF filtering."""

    name: str
    h: float
    h_dot: float = 0.0
    scale: float = 1.0


def bounded(value, lower, upper):
    """Return value restricted to inclusive lower and upper limits."""
    return max(lower, min(value, upper))


class PIDController:
    """Small PID controller with clamped integral state."""

    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def reset(self):
        """Clear accumulated integral and derivative history."""
        self.integral = 0.0
        self.previous_error = None
        self.previous_time = None

    def update_gains(self, kp, ki, kd, integral_limit):
        """Update gains without losing current state."""
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit

    def step(self, error, now):
        """Return PID output for one control sample."""
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self.previous_time)

        if dt > 0.0:
            self.integral += error * dt
            self.integral = max(
                -self.integral_limit,
                min(self.integral, self.integral_limit),
            )

        if self.previous_error is None or dt <= 0.0:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt

        self.previous_error = error
        self.previous_time = now
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class VirtualLidar:
    """Image-space lidar inspired by the av_control_guide ray-casting sensor."""

    def __init__(self, max_range_px=500.0, resolution_rad=math.radians(3.0)):
        self.max_range_px = max_range_px
        self.resolution_rad = resolution_rad
        self._contour_steps = 24
        self.latest_points = []

    def scan(self, origin_px, heading_rad, obstacles):
        """Return nearest obstacle contour hits binned by relative angle."""
        if not obstacles:
            self.latest_points = []
            return []

        bin_count = int(math.floor((2.0 * math.pi) / self.resolution_rad)) + 1
        nearest_by_bin = {}
        origin_x = float(origin_px[0])
        origin_y = float(origin_px[1])

        for obstacle in obstacles:
            polygon = obstacle.get('polygon')
            if polygon is None:
                continue
            for point_x, point_y in self._polygon_contour_points(polygon):
                dx = float(point_x) - origin_x
                dy = float(point_y) - origin_y
                distance = math.hypot(dx, dy)
                if distance <= 0.0 or distance > self.max_range_px:
                    continue
                angle = self._wrap_angle(math.atan2(dy, dx) - heading_rad)
                bin_id = int(round((angle + math.pi) / self.resolution_rad))
                bin_id = max(0, min(bin_id, bin_count - 1))
                current = nearest_by_bin.get(bin_id)
                if current is None or distance < current.distance_px:
                    nearest_by_bin[bin_id] = LidarPoint(
                        distance,
                        angle,
                        float(point_x),
                        float(point_y),
                    )

        self.latest_points = [
            nearest_by_bin[key]
            for key in sorted(nearest_by_bin)
        ]
        return self.latest_points

    def nearest_clearance_px(self, safety_margin_px):
        """Return closest lidar range minus the configured safety margin."""
        if not self.latest_points:
            return float('inf')
        nearest = min(point.distance_px for point in self.latest_points)
        return nearest - safety_margin_px

    def reset(self):
        """Clear the last scan."""
        self.latest_points = []

    def _polygon_contour_points(self, polygon):
        """Yield interpolated points around a rectangular obstacle polygon."""
        points = list(polygon)
        if len(points) < 2:
            return
        for index, start in enumerate(points):
            end = points[(index + 1) % len(points)]
            for step in range(self._contour_steps):
                ratio = step / float(self._contour_steps)
                yield (
                    (1.0 - ratio) * start[0] + ratio * end[0],
                    (1.0 - ratio) * start[1] + ratio * end[1],
                )

    @staticmethod
    def _wrap_angle(angle):
        """Wrap an angle to -pi..pi."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


class PIDVelocityCBFController:
    """Velocity PID plus CBF throttle scaling using virtual-lidar feedback."""

    def __init__(self, velocity_pid, virtual_lidar=None):
        self.velocity_pid = velocity_pid
        self.virtual_lidar = virtual_lidar or VirtualLidar()
        self._previous_barriers = {}
        self._previous_barrier_time = None

    def reset(self):
        """Reset controller history and sensor memory."""
        self.velocity_pid.reset()
        self.virtual_lidar.reset()
        self._previous_barriers = {}
        self._previous_barrier_time = None

    def update_velocity_gains(self, kp, ki, kd, integral_limit):
        """Update the embedded velocity PID gains."""
        self.velocity_pid.update_gains(kp, ki, kd, integral_limit)

    def velocity_throttle(
        self,
        target_speed_pps,
        measured_speed_pps,
        nominal_forward_pwm,
        min_forward_pwm,
        max_forward_pwm,
        now,
    ):
        """Return bounded throttle and raw velocity-PID delta."""
        speed_error = target_speed_pps - measured_speed_pps
        speed_delta = self.velocity_pid.step(speed_error, now)
        throttle = bounded(
            nominal_forward_pwm + speed_delta,
            min_forward_pwm,
            max_forward_pwm,
        )
        return throttle, speed_delta, speed_error

    def apply_cbf(
        self,
        throttle_pwm,
        neutral_throttle_pwm,
        center_px,
        heading_rad,
        lateral_error_px,
        heading_error_rad,
        image_size,
        obstacles,
        slow_error_px,
        stop_error_px,
        stop_heading_rad,
        edge_margin_px,
        obstacle_margin_px,
        cbf_h_px,
        cbf_alpha,
        now=None,
    ):
        """
        Apply derivative-based CBF throttle filtering.

        Each barrier uses h >= 0 as its safe set. The filter estimates h_dot
        from recent samples and enforces h_dot + alpha*h >= 0 by reducing the
        forward-throttle scale when a barrier is moving toward violation.
        """
        if now is None:
            now = time.monotonic()
        self.virtual_lidar.scan(center_px, heading_rad, obstacles)
        lidar_clearance = self.virtual_lidar.nearest_clearance_px(cbf_h_px)
        safety_clearance, edge_clearance, error_clearance, heading_clearance = (
            self._safety_clearance(
                center_px,
                lateral_error_px,
                heading_error_rad,
                image_size,
                stop_error_px,
                stop_heading_rad,
                edge_margin_px,
                lidar_clearance,
            )
        )
        barrier_values = {
            'edge': edge_clearance,
            'lateral_error': error_clearance,
            'heading': heading_clearance,
            'obstacle': lidar_clearance,
        }
        barriers = self._estimate_barrier_derivatives(barrier_values, now)
        scale = self._derivative_cbf_scale(barriers, cbf_alpha)
        filtered = neutral_throttle_pwm + (
            throttle_pwm - neutral_throttle_pwm
        ) * scale
        return CBFResult(
            throttle_pwm=filtered,
            scale=scale,
            active=scale < 0.999,
            safety_clearance_px=safety_clearance,
            lidar_clearance_px=lidar_clearance,
            edge_clearance_px=edge_clearance,
            error_clearance_px=error_clearance,
            heading_clearance_px=heading_clearance,
        )

    def _estimate_barrier_derivatives(self, barrier_values, now):
        """Return barrier states with finite-difference h_dot estimates."""
        if self._previous_barrier_time is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self._previous_barrier_time)

        barriers = []
        for name, h in barrier_values.items():
            previous_h = self._previous_barriers.get(name)
            if previous_h is None or dt <= 0.0 or not math.isfinite(h):
                h_dot = 0.0
            else:
                h_dot = (h - previous_h) / dt
            barriers.append(BarrierState(name=name, h=h, h_dot=h_dot))

        self._previous_barriers = dict(barrier_values)
        self._previous_barrier_time = now
        return barriers

    def _derivative_cbf_scale(self, barriers, cbf_alpha):
        """Return the largest safe 0..1 throttle scale for all barriers."""
        scale = 1.0
        for barrier in barriers:
            if not math.isfinite(barrier.h):
                barrier.scale = 1.0
                continue
            if barrier.h < 0.0:
                barrier.scale = 0.0
            elif barrier.h_dot < 0.0:
                barrier.scale = cbf_alpha * barrier.h / max(
                    1e-6,
                    -barrier.h_dot,
                )
            else:
                barrier.scale = 1.0
            barrier.scale = bounded(barrier.scale, 0.0, 1.0)
            scale = min(scale, barrier.scale)
        return scale

    def _safety_clearance(
        self,
        center_px,
        lateral_error_px,
        heading_error_rad,
        image_size,
        stop_error_px,
        stop_heading_rad,
        edge_margin_px,
        lidar_clearance_px,
    ):
        """Return aggregate and individual CBF clearances in pixels."""
        width, height = image_size
        edge_distance = min(
            center_px[0],
            center_px[1],
            width - center_px[0],
            height - center_px[1],
        )
        edge_clearance = edge_distance - edge_margin_px
        error_clearance = stop_error_px - abs(lateral_error_px)
        heading_clearance = (
            stop_heading_rad - abs(heading_error_rad)
        ) * stop_error_px / stop_heading_rad
        safety_clearance = min(
            edge_clearance,
            error_clearance,
            heading_clearance,
            lidar_clearance_px,
        )
        return (
            safety_clearance,
            edge_clearance,
            error_clearance,
            heading_clearance,
        )
