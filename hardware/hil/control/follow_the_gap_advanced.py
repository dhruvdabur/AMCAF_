"""Advanced Follow-the-Gap target selection for LiDAR scans."""

import math
from pathlib import Path
import re

import numpy as np


DEFAULT_BUBBLE_RADIUS = 38.125
DEFAULT_MAX_RANGE_CAP = 583.173
DEFAULT_DEEP_CLUSTER_RATIO = 0.95
DEFAULT_OBSTACLE_EPSILON = 1.0
FORWARD_VIEW_RAD = math.radians(120.0)


def calculate_follow_the_gap_point(
    lidar_points,
    origin,
    heading,
    max_range,
    resolution,
    bubble_radius,
    max_range_cap=DEFAULT_MAX_RANGE_CAP,
):
    """Return an image-space target point from sparse forward LiDAR returns."""
    if max_range_cap is not None:
        max_range = min(float(max_range), float(max_range_cap))
    ranges, angles = build_forward_scan(
        lidar_points,
        max_range,
        resolution,
    )
    target_angle, target_dist = calculate_follow_the_gap_target(
        ranges,
        angles,
        bubble_radius,
    )
    if target_dist <= 0.0:
        return None, target_angle, target_dist

    global_angle = heading + target_angle
    target = np.asarray(origin, dtype=np.float32) + target_dist * np.array(
        [math.cos(global_angle), math.sin(global_angle)],
        dtype=np.float32,
    )
    return target, target_angle, target_dist


def calculate_follow_the_gap_debug(
    lidar_points,
    origin,
    heading,
    max_range,
    resolution,
    bubble_radius,
    max_range_cap=DEFAULT_MAX_RANGE_CAP,
):
    """Return Follow-the-Gap target and nearest-obstacle debug geometry."""
    if max_range_cap is not None:
        max_range = min(float(max_range), float(max_range_cap))
    ranges, angles = build_forward_scan(lidar_points, max_range, resolution)
    safe_ranges, bubble_center = apply_safety_bubble(
        ranges,
        angles,
        bubble_radius,
        max_range,
    )
    target_angle, target_dist = calculate_follow_the_gap_target(
        ranges,
        angles,
        bubble_radius,
        safe_ranges=safe_ranges,
    )
    nearest_angle, nearest_dist = nearest_obstacle_point(ranges, angles)
    target = None
    if target_dist > 0.0:
        global_angle = heading + target_angle
        target = np.asarray(origin, dtype=np.float32) + target_dist * np.array(
            [math.cos(global_angle), math.sin(global_angle)],
            dtype=np.float32,
        )
    nearest_point = None
    if nearest_dist is not None:
        nearest_global_angle = heading + nearest_angle
        nearest_point = np.asarray(origin, dtype=np.float32) + nearest_dist * np.array(
            [math.cos(nearest_global_angle), math.sin(nearest_global_angle)],
            dtype=np.float32,
        )
    bubble_points = []
    if bubble_center is not None:
        bubble_angle, bubble_dist = bubble_center
        bubble_global_angle = heading + bubble_angle
        bubble_points.append(
            np.asarray(origin, dtype=np.float32)
            + bubble_dist
            * np.array(
                [math.cos(bubble_global_angle), math.sin(bubble_global_angle)],
                dtype=np.float32,
            )
        )
    return {
        'target': target,
        'target_angle': target_angle,
        'target_dist': target_dist,
        'nearest_point': nearest_point,
        'nearest_angle': nearest_angle,
        'nearest_dist': nearest_dist,
        'bubble_radius': float(bubble_radius),
        'max_range_cap': float(max_range),
        'safe_ranges': safe_ranges,
        'angles': angles,
        'bubble_center': bubble_center,
        'bubble_points': bubble_points,
    }


def build_forward_scan(lidar_points, max_range, resolution):
    """Build forward-facing ranges and angles from sparse hits."""
    resolution = float(resolution)
    min_angle = -0.5 * FORWARD_VIEW_RAD
    max_angle = 0.5 * FORWARD_VIEW_RAD
    bin_count = int(math.floor((max_angle - min_angle) / resolution)) + 1
    angles = np.array(
        [min_angle + index * resolution for index in range(bin_count)],
        dtype=np.float32,
    )
    ranges = np.full(bin_count, float(max_range), dtype=np.float32)

    for point in lidar_points:
        if point.angle_rad < min_angle or point.angle_rad > max_angle:
            continue
        bin_id = int(round((point.angle_rad - min_angle) / resolution))
        if 0 <= bin_id < bin_count:
            ranges[bin_id] = min(ranges[bin_id], float(point.distance_px))
    return ranges, angles


def follow_the_gap_bubble_radius(config):
    """Return the safety bubble radius used by Follow-the-Gap Advanced."""
    return max(
        float(getattr(config, 'obstacle_margin_px', 0.0)),
        float(getattr(config, 'vehicle_radius_px', 0.0)),
        DEFAULT_BUBBLE_RADIUS,
    )


def calculate_follow_the_gap_target(
    ranges,
    angles,
    bubble_radius,
    deep_cluster_ratio=DEFAULT_DEEP_CLUSTER_RATIO,
    safe_ranges=None,
):
    """Return the safest target angle and distance from a LiDAR scan."""
    ranges = np.asarray(ranges, dtype=np.float32)
    angles = np.asarray(angles, dtype=np.float32)
    if ranges.size == 0 or angles.size == 0 or ranges.size != angles.size:
        return 0.0, 0.0

    finite_mask = np.isfinite(ranges)
    positive_mask = ranges > 0.0
    valid_mask = finite_mask & positive_mask
    if not np.any(valid_mask):
        return 0.0, 0.0

    if safe_ranges is None:
        safe_ranges, _bubble_center = apply_safety_bubble(
            ranges,
            angles,
            bubble_radius,
        )
    else:
        safe_ranges = np.asarray(safe_ranges, dtype=np.float32)

    current_start = -1
    best_start = -1
    best_length = 0

    for i in range(len(safe_ranges)):
        if safe_ranges[i] > 0.0:
            if current_start == -1:
                current_start = i
        else:
            if current_start != -1:
                current_length = i - current_start
                if current_length > best_length:
                    best_length = current_length
                    best_start = current_start
                current_start = -1

    final_length = len(safe_ranges) - current_start
    if current_start != -1 and final_length > best_length:
        best_length = final_length
        best_start = current_start

    if best_length == 0:
        return 0.0, 0.0

    best_end = best_start + best_length
    gap_ranges = safe_ranges[best_start:best_end]
    max_gap_dist = np.max(gap_ranges)
    depth_threshold = max_gap_dist * float(deep_cluster_ratio)
    deep_cluster_indices = np.where(gap_ranges >= depth_threshold)[0]
    furthest_idx_in_gap = int(np.mean(deep_cluster_indices))

    target_idx = best_start + furthest_idx_in_gap
    target_angle = float(angles[target_idx])
    target_dist = float(ranges[target_idx])

    return target_angle, target_dist


def apply_safety_bubble(
    ranges,
    angles,
    bubble_radius,
    max_range=None,
    obstacle_epsilon=DEFAULT_OBSTACLE_EPSILON,
):
    """Zero scan rays that pass through the nearest obstacle safety bubble."""
    ranges = np.asarray(ranges, dtype=np.float32)
    angles = np.asarray(angles, dtype=np.float32)
    if ranges.size == 0 or angles.size == 0 or ranges.size != angles.size:
        return np.zeros_like(ranges, dtype=np.float32), []

    safe_ranges = np.where(
        np.isfinite(ranges) & (ranges > 0.0),
        ranges,
        0.0,
    ).astype(np.float32, copy=True)
    if bubble_radius <= 0.0:
        return safe_ranges, []

    if max_range is None:
        max_range = float(np.max(ranges[np.isfinite(ranges)]))
    obstacle_mask = (
        np.isfinite(ranges)
        & (ranges > 0.0)
        & (ranges < float(max_range) - float(obstacle_epsilon))
    )
    obstacle_indices = np.flatnonzero(obstacle_mask)
    if obstacle_indices.size == 0:
        return safe_ranges, None

    obstacle_idx = obstacle_indices[np.argmin(ranges[obstacle_indices])]
    obstacle_dist = float(ranges[obstacle_idx])
    obstacle_angle = float(angles[obstacle_idx])
    for ray_idx in range(len(safe_ranges)):
        if safe_ranges[ray_idx] <= 0.0:
            continue
        dist_to_obstacle = math.sqrt(
            obstacle_dist**2
            + float(ranges[ray_idx]) ** 2
            - 2.0
            * obstacle_dist
            * float(ranges[ray_idx])
            * math.cos(float(angles[ray_idx]) - obstacle_angle)
        )
        if dist_to_obstacle < bubble_radius:
            safe_ranges[ray_idx] = 0.0
    return safe_ranges, (obstacle_angle, obstacle_dist)


def nearest_obstacle_point(ranges, angles):
    """Return nearest valid obstacle as angle and distance."""
    ranges = np.asarray(ranges, dtype=np.float32)
    angles = np.asarray(angles, dtype=np.float32)
    valid_mask = np.isfinite(ranges) & (ranges > 0.0)
    if ranges.size == 0 or angles.size == 0 or ranges.size != angles.size:
        return None, None
    if not np.any(valid_mask):
        return None, None
    valid_indices = np.flatnonzero(valid_mask)
    min_idx = valid_indices[np.argmin(ranges[valid_indices])]
    return float(angles[min_idx]), float(ranges[min_idx])


def bubble_outline(min_angle, min_dist, bubble_radius, sample_count=96):
    """Return polar points for a safety bubble around the nearest obstacle."""
    if min_angle is None or min_dist is None:
        return np.empty(0), np.empty(0)

    center_x = min_dist * np.sin(min_angle)
    center_y = min_dist * np.cos(min_angle)
    theta = np.linspace(0.0, 2.0 * np.pi, sample_count)
    x_points = center_x + bubble_radius * np.cos(theta)
    y_points = center_y + bubble_radius * np.sin(theta)
    angles = np.arctan2(x_points, y_points)
    ranges = np.hypot(x_points, y_points)
    return angles, ranges


def generate_fake_lidar():
    """Create a synthetic 180-degree forward LiDAR scan for tuning."""
    angles = np.linspace(-np.pi / 2, np.pi / 2, 180)
    ranges = np.full_like(angles, 500.0)

    obstacle_mask = (angles > 0.2) & (angles < 0.6)
    ranges[obstacle_mask] = 400.0

    ranges += np.random.normal(0, 5.0, size=ranges.shape)
    return angles, ranges


def save_tuned_values(bubble_radius, max_range_cap, file_path=None):
    """Persist tuned Follow-the-Gap values into this module."""
    path = Path(__file__) if file_path is None else Path(file_path)
    source = path.read_text(encoding='utf-8')
    replacements = {
        'DEFAULT_BUBBLE_RADIUS': float(bubble_radius),
        'DEFAULT_MAX_RANGE_CAP': float(max_range_cap),
    }
    for name, value in replacements.items():
        source = re.sub(
            rf'^{name} = .*$',
            f'{name} = {value:.6g}',
            source,
            count=1,
            flags=re.MULTILINE,
        )
    path.write_text(source, encoding='utf-8')


def run_tuning_panel():
    """Launch an interactive Follow-the-Gap tuning panel."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button
    from matplotlib.widgets import Slider

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    plt.subplots_adjust(bottom=0.42)

    angles, ranges_raw = generate_fake_lidar()
    raw_plot = ax.scatter(angles, ranges_raw, c='blue', s=10, label='Raw Scan')
    bubble_plot, = ax.plot([], [], c='orange', linewidth=2, label='Safety Bubble')
    threat_plot = ax.scatter(
        [],
        [],
        c='orange',
        marker='x',
        s=120,
        label='Nearest Obstacle',
    )
    target_plot = ax.scatter(
        [],
        [],
        c='red',
        marker='*',
        s=200,
        label='Target Heading',
    )

    ax.set_ylim(0, 600)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.legend(loc='upper right')

    ax_bubble = plt.axes([0.2, 0.24, 0.65, 0.03])
    ax_cap = plt.axes([0.2, 0.14, 0.65, 0.03])
    ax_save = plt.axes([0.72, 0.04, 0.13, 0.05])

    slider_bubble = Slider(
        ax_bubble,
        'Bubble Radius',
        10.0,
        400.0,
        valinit=DEFAULT_BUBBLE_RADIUS,
    )
    slider_cap = Slider(
        ax_cap,
        'Max Range Cap',
        100.0,
        600.0,
        valinit=DEFAULT_MAX_RANGE_CAP,
    )
    save_button = Button(ax_save, 'Save')

    def update(_val):
        bubble_r = slider_bubble.val
        cap_val = slider_cap.val
        ranges_capped = np.clip(ranges_raw, 0.0, cap_val)
        target_angle, target_dist = calculate_follow_the_gap_target(
            ranges_capped,
            angles,
            bubble_r,
        )
        min_angle, min_dist = nearest_obstacle_point(ranges_capped, angles)
        bubble_angles, bubble_ranges = bubble_outline(
            min_angle,
            min_dist,
            bubble_r,
        )
        bubble_plot.set_data(bubble_angles, bubble_ranges)
        if min_dist is not None:
            threat_plot.set_offsets(np.c_[min_angle, min_dist])
        else:
            threat_plot.set_offsets(np.empty((0, 2)))
        if target_dist > 0.0:
            target_plot.set_offsets(np.c_[target_angle, target_dist])
        else:
            target_plot.set_offsets(np.empty((0, 2)))
        raw_plot.set_offsets(np.c_[angles, ranges_capped])
        fig.canvas.draw_idle()

    def save(_event):
        save_tuned_values(slider_bubble.val, slider_cap.val)
        fig.suptitle('Saved tuned Follow-the-Gap values')
        fig.canvas.draw_idle()

    slider_bubble.on_changed(update)
    slider_cap.on_changed(update)
    save_button.on_clicked(save)

    update(0)
    plt.show()


if __name__ == '__main__':
    run_tuning_panel()
