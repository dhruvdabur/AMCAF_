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
class LidarCluster:
    """One obstacle cluster extracted from adjacent lidar returns."""

    points: list
    nearest_point: LidarPoint
    y_left_px: float
    y_right_px: float
    width_px: float
    x_min_px: float
    x_max_px: float


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

    def decay_integral(self, ratio=0.5):
        """Bleed off stored integral when a downstream safety filter dominates."""
        self.integral *= max(0.0, min(float(ratio), 1.0))

    def step(self, error, now, output_limits=None, output_offset=0.0):
        """Return PID output for one control sample."""
        if self.previous_time is None:
            dt = 0.0
        else:
            dt = max(0.0, now - self.previous_time)

        previous_integral = self.integral
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
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if output_limits is None:
            return output

        lower, upper = output_limits
        commanded = output_offset + output
        if (
            (commanded > upper and error > 0.0)
            or (commanded < lower and error < 0.0)
        ):
            self.integral = previous_integral
            output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output


class VirtualLidar:
    """Image-space forward lidar inspired by the av_control_guide ray-casting sensor."""

    def __init__(
        self,
        max_range_px=500.0,
        resolution_rad=math.radians(3.0),
        front_view_rad=math.radians(120.0),
    ):
        self.max_range_px = max_range_px
        self.resolution_rad = resolution_rad
        self.front_view_rad = front_view_rad
        self._contour_steps = 24
        self.cluster_gap_threshold_px = 30.0
        self.cluster_min_points = 3
        self.cluster_max_range_epsilon_px = 1.0
        self.latest_points = []
        self.latest_clusters = []

    def scan(self, origin_px, heading_rad, obstacles):
        """Return nearest polygon surface hits from front-sector ray-cast beams."""
        if not obstacles:
            self.latest_points = []
            self.latest_clusters = []
            return []

        front_view_rad = float(self.front_view_rad)
        half_view_rad = 0.5 * max(0.0, min(2.0 * math.pi, front_view_rad))
        bin_count = int(math.floor((2.0 * half_view_rad) / self.resolution_rad)) + 1
        origin_x = float(origin_px[0])
        origin_y = float(origin_px[1])
        points = []

        for bin_id in range(bin_count):
            angle = -half_view_rad + bin_id * self.resolution_rad
            ray_angle = heading_rad + angle
            hit = self._nearest_obstacle_hit(
                origin_x,
                origin_y,
                ray_angle,
                obstacles,
            )
            if hit is not None:
                points.append(
                    LidarPoint(
                        hit.distance_px,
                        angle,
                        hit.x_px,
                        hit.y_px,
                    )
                )

        self.latest_points = points
        self.latest_clusters = []
        return self.latest_points

    def scan_nearest_corners(self, origin_px, heading_rad, obstacles):
        """Return one nearest visible polygon corner per obstacle."""
        if not obstacles:
            self.latest_points = []
            return []

        origin_x = float(origin_px[0])
        origin_y = float(origin_px[1])
        points = []
        for obstacle in obstacles:
            polygon = obstacle.get('polygon')
            if polygon is None:
                continue
            nearest = None
            for corner_x, corner_y in polygon:
                dx = float(corner_x) - origin_x
                dy = float(corner_y) - origin_y
                distance = math.hypot(dx, dy)
                if distance <= 0.0 or distance > self.max_range_px:
                    continue
                angle = self._wrap_angle(math.atan2(dy, dx) - heading_rad)
                point = LidarPoint(
                    distance,
                    angle,
                    float(corner_x),
                    float(corner_y),
                )
                if nearest is None or point.distance_px < nearest.distance_px:
                    nearest = point
            if nearest is not None:
                points.append(nearest)

        self.latest_points = sorted(points, key=lambda point: point.distance_px)
        return self.latest_points

    def scan_edge_centers(self, origin_px, heading_rad, obstacles):
        """Return one visible edge-midpoint observation per obstacle."""
        if not obstacles:
            self.latest_points = []
            self.latest_clusters = []
            return []

        origin_x = float(origin_px[0])
        origin_y = float(origin_px[1])
        points = []
        for obstacle in obstacles:
            polygon = obstacle.get('polygon')
            if polygon is None:
                continue

            returns = self._visible_obstacle_hits(
                origin_x,
                origin_y,
                heading_rad,
                polygon,
                obstacles,
            )
            if len(returns) < 2:
                continue

            left_edge = min(returns, key=lambda point: point.angle_rad)
            right_edge = max(returns, key=lambda point: point.angle_rad)
            x_obs = 0.5 * (left_edge.x_px + right_edge.x_px)
            y_obs = 0.5 * (left_edge.y_px + right_edge.y_px)
            dx = x_obs - origin_x
            dy = y_obs - origin_y
            distance = math.hypot(dx, dy)
            angle = self._wrap_angle(math.atan2(dy, dx) - heading_rad)
            points.append(LidarPoint(distance, angle, x_obs, y_obs))

        self.latest_points = sorted(points, key=lambda point: point.distance_px)
        self.latest_clusters = []
        return self.latest_points

    def detect_obstacles_from_scan(
        self,
        origin_px,
        heading_rad,
        obstacles,
        max_range_px=None,
        gap_threshold_px=None,
        min_points=None,
        epsilon_px=None,
    ):
        """Cluster lidar returns into obstacle observations."""
        points = self.scan(origin_px, heading_rad, obstacles)
        clusters = self.cluster_points(
            origin_px,
            points,
            max_range_px=max_range_px,
            gap_threshold_px=gap_threshold_px,
            min_points=min_points,
            epsilon_px=epsilon_px,
        )
        self.latest_clusters = clusters
        return clusters

    def cluster_points(
        self,
        origin_px,
        points=None,
        max_range_px=None,
        gap_threshold_px=None,
        min_points=None,
        epsilon_px=None,
    ):
        """Group adjacent lidar points using Euclidean gaps."""
        if points is None:
            points = self.latest_points
        max_range_px = self.max_range_px if max_range_px is None else max_range_px
        gap_threshold_px = (
            self.cluster_gap_threshold_px
            if gap_threshold_px is None
            else gap_threshold_px
        )
        min_points = self.cluster_min_points if min_points is None else min_points
        epsilon_px = (
            self.cluster_max_range_epsilon_px
            if epsilon_px is None
            else epsilon_px
        )
        origin_x = float(origin_px[0])
        origin_y = float(origin_px[1])

        filtered = [
            point for point in points
            if point.distance_px < max_range_px - epsilon_px
        ]
        if not filtered:
            return []

        filtered = sorted(filtered, key=lambda point: point.angle_rad)
        raw_clusters = []
        current = [filtered[0]]
        previous = filtered[0]
        for point in filtered[1:]:
            gap = math.hypot(
                point.x_px - previous.x_px,
                point.y_px - previous.y_px,
            )
            if gap > gap_threshold_px:
                raw_clusters.append(current)
                current = []
            current.append(point)
            previous = point
        raw_clusters.append(current)

        clusters = []
        for cluster_points in raw_clusters:
            if len(cluster_points) < min_points:
                continue
            nearest = min(
                cluster_points,
                key=lambda point: math.hypot(
                    point.x_px - origin_x,
                    point.y_px - origin_y,
                ),
            )
            y_left = max(point.y_px for point in cluster_points)
            y_right = min(point.y_px for point in cluster_points)
            x_min = min(point.x_px for point in cluster_points)
            x_max = max(point.x_px for point in cluster_points)
            clusters.append(
                LidarCluster(
                    points=list(cluster_points),
                    nearest_point=nearest,
                    y_left_px=y_left,
                    y_right_px=y_right,
                    width_px=y_left - y_right,
                    x_min_px=x_min,
                    x_max_px=x_max,
                )
            )
        return clusters

    def nearest_clearance_px(self, safety_margin_px):
        """Return closest lidar range minus the configured safety margin."""
        if not self.latest_points:
            return float('inf')
        nearest = min(point.distance_px for point in self.latest_points)
        return nearest - safety_margin_px

    def reset(self):
        """Clear the last scan."""
        self.latest_points = []
        self.latest_clusters = []

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

    def _visible_obstacle_hits(
        self,
        origin_x,
        origin_y,
        heading_rad,
        polygon,
        obstacles,
    ):
        """Ray-cast across one obstacle's angular span and keep front hits."""
        polygon_points = list(polygon)
        if len(polygon_points) < 2:
            return []

        centroid_x = sum(
            float(point[0]) for point in polygon_points
        ) / len(polygon_points)
        centroid_y = sum(
            float(point[1]) for point in polygon_points
        ) / len(polygon_points)
        center_angle = math.atan2(centroid_y - origin_y, centroid_x - origin_x)
        unwrapped_angles = []
        for point_x, point_y in self._polygon_contour_points(polygon):
            angle = math.atan2(
                float(point_y) - origin_y,
                float(point_x) - origin_x,
            )
            unwrapped_angles.append(
                center_angle + self._wrap_angle(angle - center_angle)
            )
        if not unwrapped_angles:
            return []

        min_angle = min(unwrapped_angles)
        max_angle = max(unwrapped_angles)
        span = max_angle - min_angle
        if span <= 1e-6:
            sample_angles = [center_angle]
        else:
            step = max(self.resolution_rad * 0.5, span / 48.0)
            sample_count = max(2, int(math.ceil(span / step)))
            sample_angles = [
                min_angle + span * index / float(sample_count)
                for index in range(sample_count + 1)
            ]

        hits = []
        for ray_angle in sample_angles:
            hit = self._ray_polygon_hit(origin_x, origin_y, ray_angle, polygon)
            if hit is None or hit.distance_px > self.max_range_px:
                continue
            nearest = self._nearest_obstacle_hit(
                origin_x,
                origin_y,
                ray_angle,
                obstacles,
            )
            if (
                nearest is None
                or abs(nearest.distance_px - hit.distance_px) > 1e-5
            ):
                continue
            hits.append(
                LidarPoint(
                    hit.distance_px,
                    self._wrap_angle(ray_angle - heading_rad),
                    hit.x_px,
                    hit.y_px,
                )
            )
        return hits

    def _nearest_obstacle_hit(self, origin_x, origin_y, ray_angle, obstacles):
        """Return the nearest obstacle hit along one ray."""
        nearest = None
        for obstacle in obstacles:
            polygon = obstacle.get('polygon')
            if polygon is None:
                continue
            hit = self._ray_polygon_hit(origin_x, origin_y, ray_angle, polygon)
            if hit is None or hit.distance_px > self.max_range_px:
                continue
            if nearest is None or hit.distance_px < nearest.distance_px:
                nearest = hit
        return nearest

    def _ray_polygon_hit(self, origin_x, origin_y, ray_angle, polygon):
        """Return the nearest intersection between one ray and a polygon."""
        direction_x = math.cos(ray_angle)
        direction_y = math.sin(ray_angle)
        nearest = None
        points = list(polygon)
        if len(points) < 2:
            return None
        for index, start in enumerate(points):
            end = points[(index + 1) % len(points)]
            hit = self._ray_segment_hit(
                origin_x,
                origin_y,
                direction_x,
                direction_y,
                float(start[0]),
                float(start[1]),
                float(end[0]),
                float(end[1]),
            )
            if hit is None:
                continue
            if nearest is None or hit.distance_px < nearest.distance_px:
                nearest = hit
        return nearest

    @staticmethod
    def _ray_segment_hit(
        origin_x,
        origin_y,
        direction_x,
        direction_y,
        start_x,
        start_y,
        end_x,
        end_y,
    ):
        """Return a lidar hit if a ray intersects a finite segment."""
        segment_x = end_x - start_x
        segment_y = end_y - start_y
        denom = direction_x * segment_y - direction_y * segment_x
        if abs(denom) <= 1e-9:
            return None

        delta_x = start_x - origin_x
        delta_y = start_y - origin_y
        distance = (delta_x * segment_y - delta_y * segment_x) / denom
        segment_ratio = (delta_x * direction_y - delta_y * direction_x) / denom
        if (
            distance <= 1e-6
            or segment_ratio < -1e-6
            or segment_ratio > 1.0 + 1e-6
        ):
            return None

        x_px = origin_x + distance * direction_x
        y_px = origin_y + distance * direction_y
        return LidarPoint(distance, 0.0, x_px, y_px)

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
        speed_delta = self.velocity_pid.step(
            speed_error,
            now,
            output_limits=(min_forward_pwm, max_forward_pwm),
            output_offset=nominal_forward_pwm,
        )
        throttle = bounded(
            nominal_forward_pwm + speed_delta,
            min_forward_pwm,
            max_forward_pwm,
        )
        return throttle, speed_delta, speed_error

    def apply_throttle_anti_windup(
        self,
        nominal_throttle,
        applied_throttle,
        speed_error,
    ):
        """Unwind velocity integral when safety filtering reduces throttle."""
        if applied_throttle < nominal_throttle - 1.0 and speed_error > 0.0:
            self.velocity_pid.decay_integral(0.35)

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
