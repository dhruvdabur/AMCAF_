"""Advanced Follow-the-Gap target selection for LiDAR scans."""

import math
from pathlib import Path
import re

import numpy as np


DEFAULT_BUBBLE_RADIUS = 87.8333
DEFAULT_MAX_RANGE_CAP = 552.381
DEFAULT_DEEP_CLUSTER_RATIO = 0.839286
DEFAULT_OBSTACLE_EPSILON = 1.0
FORWARD_VIEW_RAD = math.radians(120.714)

# Disparity Extension parameters
DEFAULT_DISPARITY_THRESHOLD = 32.1143
DEFAULT_DISPARITY_WIDTH = 49.3333

# Cost function weights
DEFAULT_DEPTH_SCALE = 189.524
DEFAULT_HEADING_WEIGHT = 2.92857

# Steering output smoothing
DEFAULT_SMOOTHING_ALPHA = 0.9835


_prev_angle = 0.0
_f1tenth_prev_angle = 0.0


def extend_disparities(ranges, angles, vehicle_width=None):
    """Draw a virtual safety radius around depth disparities to clear vehicle width."""
    if vehicle_width is None:
        vehicle_width = DEFAULT_DISPARITY_WIDTH
    ranges = np.asarray(ranges, dtype=np.float32)
    angles = np.asarray(angles, dtype=np.float32)
    extended_ranges = np.copy(ranges)
    
    # Auto-detect scale: if max range is < 10, assume meters (F1Tenth), else pixels
    is_meters = np.max(ranges) < 10.0
    threshold = 0.5 if is_meters else DEFAULT_DISPARITY_THRESHOLD
    width = 0.35 if is_meters else float(vehicle_width)
    
    for i in range(1, len(ranges)):
        diff = abs(ranges[i] - ranges[i - 1])
        if diff > threshold:
            closer_idx = i if ranges[i] < ranges[i - 1] else i - 1
            closer_dist = ranges[closer_idx]
            alpha = abs(angles[i] - angles[i - 1])
            if closer_dist * alpha > 0:
                num_indices = int(width / (closer_dist * alpha))
                start = max(0, closer_idx - num_indices)
                end = min(len(ranges), closer_idx + num_indices + 1)
                for j in range(start, end):
                    if j < len(extended_ranges) and extended_ranges[j] > closer_dist:
                        extended_ranges[j] = closer_dist
    return extended_ranges


def select_best_gap_index(gaps, ranges, angles, goal_angle=0.0):
    """Evaluate all gaps using a cost function balancing depth and heading error."""
    best_idx = -1
    min_cost = float('inf')
    for start, end in gaps:
        for i in range(start, end + 1):
            depth = float(ranges[i])
            heading_error = abs(angles[i] - goal_angle)
            # Scaling depth index based on units
            is_meters = np.max(ranges) < 10.0
            depth_scale = 1.0 if is_meters else DEFAULT_DEPTH_SCALE
            cost = (depth_scale / (depth + 1e-5)) + (DEFAULT_HEADING_WEIGHT * heading_error)
            if cost < min_cost:
                min_cost = cost
                best_idx = i
    return best_idx


def calculate_follow_the_gap_point(
    lidar_points,
    origin,
    heading,
    max_range,
    resolution,
    bubble_radius,
    max_range_cap=DEFAULT_MAX_RANGE_CAP,
    fov_rad=FORWARD_VIEW_RAD,
):
    """Return an image-space target point from sparse forward LiDAR returns."""
    if max_range_cap is not None:
        max_range = min(float(max_range), float(max_range_cap))
    ranges, angles = build_forward_scan(
        lidar_points,
        max_range,
        resolution,
        fov_rad=fov_rad,
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
    fov_rad=FORWARD_VIEW_RAD,
):
    """Return Follow-the-Gap target and nearest-obstacle debug geometry."""
    if max_range_cap is not None:
        max_range = min(float(max_range), float(max_range_cap))
    ranges, angles = build_forward_scan(
        lidar_points,
        max_range,
        resolution,
        fov_rad=fov_rad,
    )
    extended_ranges = extend_disparities(ranges, angles, vehicle_width=bubble_radius)
    safe_ranges, bubble_center = apply_safety_bubble(
        extended_ranges,
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
    nearest_angle, nearest_dist = nearest_obstacle_point(extended_ranges, angles, max_range=max_range)
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

    # Compute cost for each index to construct the costmap
    costs = np.full(ranges.size, float('inf'), dtype=np.float32)
    is_meters = np.max(ranges) < 10.0
    depth_scale = 1.0 if is_meters else DEFAULT_DEPTH_SCALE
    for i in range(ranges.size):
        if safe_ranges[i] > 0.0:
            depth = float(safe_ranges[i])
            heading_error = abs(angles[i])
            costs[i] = (depth_scale / (depth + 1e-5)) + (DEFAULT_HEADING_WEIGHT * heading_error)

    return {
        'target': target,
        'target_angle': target_angle,
        'target_dist': target_dist,
        'nearest_point': nearest_point,
        'nearest_angle': nearest_angle,
        'nearest_dist': nearest_dist,
        'bubble_radius': float(bubble_radius),
        'max_range_cap': float(max_range),
        'ranges': ranges,
        'safe_ranges': safe_ranges,
        'angles': angles,
        'bubble_center': bubble_center,
        'bubble_points': bubble_points,
        'costs': costs,
    }


def build_forward_scan(lidar_points, max_range, resolution, fov_rad=FORWARD_VIEW_RAD):
    """Build forward-facing ranges and angles from sparse hits."""
    resolution = float(resolution)
    view_rad = max(0.0, min(2.0 * math.pi, float(fov_rad)))
    min_angle = -0.5 * view_rad
    max_angle = 0.5 * view_rad
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
    global _prev_angle
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
        extended_ranges = extend_disparities(ranges, angles, vehicle_width=bubble_radius)
        safe_ranges, _bubble_center = apply_safety_bubble(
            extended_ranges,
            angles,
            bubble_radius,
        )
    else:
        safe_ranges = np.asarray(safe_ranges, dtype=np.float32)

    current_start = -1
    gaps = []
    for i in range(len(safe_ranges)):
        if safe_ranges[i] > 0.0:
            if current_start == -1:
                current_start = i
        else:
            if current_start != -1:
                gaps.append((current_start, i - 1))
                current_start = -1
    if current_start != -1:
        gaps.append((current_start, len(safe_ranges) - 1))

    if not gaps:
        return 0.0, 0.0

    best_gap_idx = select_best_gap_index(gaps, safe_ranges, angles, goal_angle=0.0)
    if best_gap_idx == -1:
        return 0.0, 0.0

    target_angle = float(angles[best_gap_idx])
    target_dist = float(safe_ranges[best_gap_idx])

    # Smooth the target steering heading with a low-pass filter
    alpha = DEFAULT_SMOOTHING_ALPHA
    smoothed_angle = (alpha * target_angle) + ((1.0 - alpha) * _prev_angle)
    _prev_angle = smoothed_angle
    target_angle = smoothed_angle

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
    angle_diffs = angles - obstacle_angle
    ray_lengths = safe_ranges.astype(np.float32, copy=False)
    valid_rays = np.isfinite(ray_lengths) & (ray_lengths > 0.0)

    # Project the obstacle onto each ray and clamp to the finite ray segment.
    # This masks rays whose path crosses the bubble, even if the endpoint is beyond it.
    projection = obstacle_dist * np.cos(angle_diffs)
    closest_along_ray = np.clip(projection, 0.0, ray_lengths)
    closest_dist_sq = (
        obstacle_dist**2
        + closest_along_ray**2
        - 2.0 * obstacle_dist * closest_along_ray * np.cos(angle_diffs)
    )
    closest_dist = np.sqrt(np.maximum(closest_dist_sq, 0.0))
    bubble_mask = valid_rays & (closest_dist < bubble_radius)
    safe_ranges[bubble_mask] = 0.0
    return safe_ranges, (obstacle_angle, obstacle_dist)


def nearest_obstacle_point(ranges, angles, max_range=None, obstacle_epsilon=DEFAULT_OBSTACLE_EPSILON):
    """Return nearest valid obstacle as angle and distance."""
    ranges = np.asarray(ranges, dtype=np.float32)
    angles = np.asarray(angles, dtype=np.float32)
    if ranges.size == 0 or angles.size == 0 or ranges.size != angles.size:
        return None, None
    if max_range is None:
        valid_mask = np.isfinite(ranges) & (ranges > 0.0)
    else:
        valid_mask = (
            np.isfinite(ranges)
            & (ranges > 0.0)
            & (ranges < float(max_range) - float(obstacle_epsilon))
        )
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
    rng = np.random.default_rng(7)
    angles = np.linspace(-0.5 * FORWARD_VIEW_RAD, 0.5 * FORWARD_VIEW_RAD, 121)
    ranges = np.full_like(angles, 500.0)

    near_obstacle = (angles > -0.10) & (angles < 0.18)
    left_obstacle = (angles > -0.78) & (angles < -0.52)
    right_obstacle = (angles > 0.50) & (angles < 0.72)
    ranges[near_obstacle] = 150.0
    ranges[left_obstacle] = 230.0
    ranges[right_obstacle] = 190.0

    ranges += rng.normal(0.0, 3.0, size=ranges.shape)
    return angles, ranges


def save_tuned_values(
    bubble_radius,
    max_range_cap,
    deep_cluster_ratio,
    forward_view_rad,
    disparity_threshold,
    disparity_width,
    depth_scale,
    heading_weight,
    smoothing_alpha,
    file_path=None
):
    """Persist tuned Follow-the-Gap values into this module."""
    path = Path(__file__) if file_path is None else Path(file_path)
    source = path.read_text(encoding='utf-8')
    replacements = {
        'DEFAULT_BUBBLE_RADIUS': float(bubble_radius),
        'DEFAULT_MAX_RANGE_CAP': float(max_range_cap),
        'DEFAULT_DEEP_CLUSTER_RATIO': float(deep_cluster_ratio),
        'FORWARD_VIEW_RAD': f"math.radians({math.degrees(forward_view_rad):.6g})",
        'DEFAULT_DISPARITY_THRESHOLD': float(disparity_threshold),
        'DEFAULT_DISPARITY_WIDTH': float(disparity_width),
        'DEFAULT_DEPTH_SCALE': float(depth_scale),
        'DEFAULT_HEADING_WEIGHT': float(heading_weight),
        'DEFAULT_SMOOTHING_ALPHA': float(smoothing_alpha),
    }
    for name, value in replacements.items():
        if name == 'FORWARD_VIEW_RAD':
            source = re.sub(
                rf'^{name} = .*$',
                f'{name} = {value}',
                source,
                count=1,
                flags=re.MULTILINE,
            )
        else:
            source = re.sub(
                rf'^{name} = .*$',
                f'{name} = {value:.6g}',
                source,
                count=1,
                flags=re.MULTILINE,
            )
    path.write_text(source, encoding='utf-8')


def run_tuning_panel():
    """Launch an interactive Follow-the-Gap tuning panel with 9 parameters."""
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button
    from matplotlib.widgets import Slider

    fig = plt.figure(figsize=(14.0, 9.0))
    ax = fig.add_axes([0.06, 0.32, 0.58, 0.62], projection='polar')
    info_ax = fig.add_axes([0.68, 0.32, 0.28, 0.62])
    info_ax.axis('off')

    angles_raw, ranges_raw = generate_fake_lidar()
    raw_plot = ax.scatter(angles_raw, ranges_raw, c='tab:blue', s=10, label='Clipped scan')
    safe_plot = ax.scatter(
        [],
        [],
        c='tab:green',
        s=16,
        label='Safe rays',
    )
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
    status_text = info_ax.text(
        0.0,
        0.98,
        '',
        transform=info_ax.transAxes,
        fontsize=9,
        va='top',
        family='monospace',
    )

    ax.set_ylim(0, 520)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    handles, labels = ax.get_legend_handles_labels()
    info_ax.legend(
        handles,
        labels,
        loc='upper left',
        bbox_to_anchor=(0.0, 0.72),
        borderaxespad=0.0,
        frameon=False,
        fontsize=9,
    )
    info_ax.text(
        0.0,
        0.54,
        (
            'Symbols\n'
            'Blue dots: range scan after cap\n'
            'Green dots: safe rays after bubble\n'
            'Orange X: nearest obstacle\n'
            'Orange circle: safety bubble\n'
            'Red star: chosen FTG target\n\n'
            'Readout\n'
            'target: steering direction and distance\n'
            'nearest: obstacle angle and distance\n'
            'masked: blocked rays / total rays'
        ),
        transform=info_ax.transAxes,
        fontsize=9,
        va='top',
        linespacing=1.35,
    )

    # 2-column layout for the 9 sliders
    # Column 1 Sliders
    ax_bubble = fig.add_axes([0.15, 0.22, 0.30, 0.03])
    ax_cap = fig.add_axes([0.15, 0.17, 0.30, 0.03])
    ax_cluster = fig.add_axes([0.15, 0.12, 0.30, 0.03])
    ax_view = fig.add_axes([0.15, 0.07, 0.30, 0.03])
    ax_alpha = fig.add_axes([0.15, 0.02, 0.30, 0.03])

    # Column 2 Sliders
    ax_disp_thresh = fig.add_axes([0.58, 0.22, 0.30, 0.03])
    ax_disp_width = fig.add_axes([0.58, 0.17, 0.30, 0.03])
    ax_depth_scale = fig.add_axes([0.58, 0.12, 0.30, 0.03])
    ax_heading_weight = fig.add_axes([0.58, 0.07, 0.30, 0.03])

    # Save button
    ax_save = fig.add_axes([0.90, 0.02, 0.08, 0.06])

    slider_bubble = Slider(
        ax_bubble, 'Bubble Rad', 5.0, 250.0, valinit=DEFAULT_BUBBLE_RADIUS
    )
    slider_cap = Slider(
        ax_cap, 'Max Cap', 100.0, 600.0, valinit=DEFAULT_MAX_RANGE_CAP
    )
    slider_cluster = Slider(
        ax_cluster, 'Cluster Ratio', 0.5, 1.0, valinit=DEFAULT_DEEP_CLUSTER_RATIO
    )
    slider_view = Slider(
        ax_view, 'View Arc (deg)', 30.0, 180.0, valinit=math.degrees(FORWARD_VIEW_RAD)
    )
    slider_alpha = Slider(
        ax_alpha, 'Alpha', 0.01, 1.0, valinit=DEFAULT_SMOOTHING_ALPHA
    )

    slider_disp_thresh = Slider(
        ax_disp_thresh, 'Disp Thresh', 1.0, 100.0, valinit=DEFAULT_DISPARITY_THRESHOLD
    )
    slider_disp_width = Slider(
        ax_disp_width, 'Disp Width', 5.0, 100.0, valinit=DEFAULT_DISPARITY_WIDTH
    )
    slider_depth_scale = Slider(
        ax_depth_scale, 'Depth Scale', 10.0, 300.0, valinit=DEFAULT_DEPTH_SCALE
    )
    slider_heading_weight = Slider(
        ax_heading_weight, 'Heading Wt', 0.0, 10.0, valinit=DEFAULT_HEADING_WEIGHT
    )

    save_button = Button(ax_save, 'Save')

    def update(_val):
        bubble_r = slider_bubble.val
        cap_val = slider_cap.val
        deep_cluster_ratio = slider_cluster.val
        forward_view_rad = math.radians(slider_view.val)
        disp_thresh = slider_disp_thresh.val
        disp_width = slider_disp_width.val
        depth_scale = slider_depth_scale.val
        heading_weight = slider_heading_weight.val
        alpha = slider_alpha.val

        # Temporarily override global parameters in the module namespace for evaluation
        global DEFAULT_BUBBLE_RADIUS, DEFAULT_MAX_RANGE_CAP, DEFAULT_DEEP_CLUSTER_RATIO
        global FORWARD_VIEW_RAD, DEFAULT_DISPARITY_THRESHOLD, DEFAULT_DISPARITY_WIDTH
        global DEFAULT_DEPTH_SCALE, DEFAULT_HEADING_WEIGHT, DEFAULT_SMOOTHING_ALPHA

        DEFAULT_BUBBLE_RADIUS = bubble_r
        DEFAULT_MAX_RANGE_CAP = cap_val
        DEFAULT_DEEP_CLUSTER_RATIO = deep_cluster_ratio
        FORWARD_VIEW_RAD = forward_view_rad
        DEFAULT_DISPARITY_THRESHOLD = disp_thresh
        DEFAULT_DISPARITY_WIDTH = disp_width
        DEFAULT_DEPTH_SCALE = depth_scale
        DEFAULT_HEADING_WEIGHT = heading_weight
        DEFAULT_SMOOTHING_ALPHA = alpha

        # Re-generate scan angles based on forward view arc
        min_angle = -0.5 * forward_view_rad
        max_angle = 0.5 * forward_view_rad
        scan_angles = np.linspace(min_angle, max_angle, len(ranges_raw))
        ranges_capped = np.clip(ranges_raw, 0.0, cap_val)

        extended_ranges = extend_disparities(ranges_capped, scan_angles, vehicle_width=disp_width)
        safe_ranges, bubble_center = apply_safety_bubble(
            extended_ranges,
            scan_angles,
            bubble_r,
            cap_val,
        )
        target_angle, target_dist = calculate_follow_the_gap_target(
            ranges_capped,
            scan_angles,
            bubble_r,
            deep_cluster_ratio=deep_cluster_ratio,
            safe_ranges=safe_ranges,
        )
        min_angle_obs, min_dist_obs = nearest_obstacle_point(
            ranges_capped,
            scan_angles,
            max_range=cap_val,
        )
        bubble_angles, bubble_ranges = bubble_outline(
            min_angle_obs,
            min_dist_obs,
            bubble_r,
        )
        bubble_plot.set_data(bubble_angles, bubble_ranges)
        if min_dist_obs is not None:
            threat_plot.set_offsets(np.c_[min_angle_obs, min_dist_obs])
        else:
            threat_plot.set_offsets(np.empty((0, 2)))
        if target_dist > 0.0:
            target_plot.set_offsets(np.c_[target_angle, target_dist])
        else:
            target_plot.set_offsets(np.empty((0, 2)))
        raw_plot.set_offsets(np.c_[scan_angles, ranges_capped])
        safe_mask = safe_ranges > 0.0
        if np.any(safe_mask):
            safe_plot.set_offsets(np.c_[scan_angles[safe_mask], safe_ranges[safe_mask]])
        else:
            safe_plot.set_offsets(np.empty((0, 2)))
        zeroed = int(np.count_nonzero(~safe_mask))
        target_deg = math.degrees(target_angle)
        obstacle_text = 'none'
        if bubble_center is not None:
            obstacle_text = f'{math.degrees(bubble_center[0]):.1f}deg/{bubble_center[1]:.1f}px'
        status_text.set_text(
            'Live FTG Tuning\n'
            f'target : {target_deg:6.1f} deg, {target_dist:6.1f} px\n'
            f'nearest: {obstacle_text}\n'
            f'masked : {zeroed}/{len(safe_ranges)} rays'
        )
        ax.set_ylim(0, max(520.0, cap_val * 1.08))
        fig.canvas.draw_idle()

    def save(_event):
        save_tuned_values(
            slider_bubble.val,
            slider_cap.val,
            slider_cluster.val,
            math.radians(slider_view.val),
            slider_disp_thresh.val,
            slider_disp_width.val,
            slider_depth_scale.val,
            slider_heading_weight.val,
            slider_alpha.val,
        )
        fig.suptitle('Saved all 9 tuned FTG values to file!')
        fig.canvas.draw_idle()

    for sl in (slider_bubble, slider_cap, slider_cluster, slider_view, slider_alpha,
               slider_disp_thresh, slider_disp_width, slider_depth_scale, slider_heading_weight):
        sl.on_changed(update)
    save_button.on_clicked(save)

    update(0)
    plt.show()


def f1tenth_follow_the_gap(raw_ranges, angle_min, angle_increment, car_width=0.3): # Define the core function taking raw ROS LiDAR data
    global _f1tenth_prev_angle
    # --- 1. PREPROCESSING ---
    ranges = np.array(raw_ranges) # Convert the raw ROS tuple into a mutable numpy array
    
    # Calculate indices to crop the LiDAR sweep to a 180-degree forward arc (-pi/2 to pi/2)
    start_idx = int((-np.pi/2 - angle_min) / angle_increment) # Find the array index for -90 degrees
    end_idx = int((np.pi/2 - angle_min) / angle_increment) # Find the array index for +90 degrees
    
    cropped_ranges = ranges[start_idx:end_idx] # Slice the array to isolate only the forward-facing laser rays
    
    # Scrub LiDAR noise: Replace infinities and NaNs with a flat 0.0 so they don't break the math
    cropped_ranges[np.isinf(cropped_ranges)] = 0.0 # Overwrite 'inf' values to 0.0
    cropped_ranges[np.isnan(cropped_ranges)] = 0.0 # Overwrite 'NaN' values to 0.0
    
    # Clamp maximum distance: Cap all rays to 3.0 meters so the car doesn't violently chase distant noise
    max_range_cap = 3.0 # Define the physical distance cutoff in meters
    cropped_ranges = np.clip(cropped_ranges, 0.0, max_range_cap) # Force any value above 3.0 down to exactly 3.0
    
    # Calculate cropped angles for disparity extension
    cropped_angles = np.array([angle_min + (start_idx + i) * angle_increment for i in range(len(cropped_ranges))])
    
    # Apply disparity extension (safety bubble expansion)
    cropped_ranges = extend_disparities(cropped_ranges, cropped_angles, vehicle_width=car_width)
    
    # --- 2. FIND CLOSEST THREAT & APPLY BUBBLE ---
    closest_idx = np.argmin(cropped_ranges) # Find the array index of the closest physical object
    
    # --- DYNAMIC BUBBLE SIZING PATCH ---
    car_width = 0.35 # Define the physical width of your F1Tenth chassis in meters plus a tiny safety margin
    closest_dist = cropped_ranges[closest_idx] # Extract the exact physical distance to the detected threat in meters
    safe_dist = max(0.1, closest_dist) # Prevent divide-by-zero math errors if the obstacle is touching the sensor
    
    # Use trigonometry (ArcSine) to find the angular width of the car at this specific distance
    theta_bubble = np.arcsin(min(1.0, (car_width / 2.0) / safe_dist)) # Calculate the angle required to cover half the car
    
    # Convert that physical angle into the exact number of array indices based on the specific LiDAR's hardware resolution
    bubble_radius_indices = int(theta_bubble / angle_increment) # Divide angle by resolution to get the index count
    # --- END PATCH ---
    
    # Calculate the left and right array boundaries for the safety bubble, ensuring they don't go out of bounds
    bubble_start = max(0, closest_idx - bubble_radius_indices) # Find the left edge of the bubble
    bubble_end = min(len(cropped_ranges), closest_idx + bubble_radius_indices) # Find the right edge of the bubble
    
    cropped_ranges[bubble_start:bubble_end] = 0.0 # Overwrite the entire bubble area to 0.0 (creating a virtual wall)
    
    # --- 3. FIND THE MAX GAP ---
    # Find all contiguous gaps
    current_start = -1
    gaps = []
    for i in range(len(cropped_ranges)):
        if cropped_ranges[i] > 0.0:
            if current_start == -1:
                current_start = i
        else:
            if current_start != -1:
                gaps.append((current_start, i - 1))
                current_start = -1
    if current_start != -1:
        gaps.append((current_start, len(cropped_ranges) - 1))
        
    if not gaps:
        return 0.0, 0.0
        
    # --- 4. FIND TARGET SETPOINT ---
    # Choose target index using the Cost-Based Gap Selector
    target_idx = select_best_gap_index(gaps, cropped_ranges, cropped_angles, goal_angle=0.0)
    if target_idx == -1:
        return 0.0, 0.0
        
    target_angle = float(cropped_angles[target_idx])
    
    # Smooth the target steering heading with a low-pass filter
    alpha = DEFAULT_SMOOTHING_ALPHA
    smoothed_angle = (alpha * target_angle) + ((1.0 - alpha) * _f1tenth_prev_angle)
    _f1tenth_prev_angle = smoothed_angle
    target_angle = smoothed_angle
    
    # --- 5. VELOCITY PROFILING ---
    # Calculate how fast to drive based on how sharp the target angle is
    abs_angle = abs(target_angle) # Get the absolute magnitude of the required steering angle
    
    if abs_angle < 0.17: # If the steering angle is less than ~10 degrees (straightaway)
        target_velocity = 4.0 # Command high speed (e.g., 4.0 m/s)
    elif abs_angle < 0.35: # If the steering angle is between ~10 and ~20 degrees (sweeping corner)
        target_velocity = 2.0 # Command medium speed (e.g., 2.0 m/s)
    else: # If the steering angle is very sharp (hairpin turn)
        target_velocity = 1.0 # Command slow speed to prevent drifting (e.g., 1.0 m/s)
        
    return target_angle, target_velocity # Return the final Ackerman drive commands for the VESC


def select_advanced_free_space_target(controller, path_index):
    """Compute follow-the-gap target with dynamic centerline cost evaluation."""
    import time
    import math
    
    config = controller.config
    origin = controller.latest_vehicle_center
    if origin is None:
        origin = np.asarray(controller.track_points[path_index], dtype=np.float32)
    heading = controller.lidar_heading_rad()
    max_range = getattr(
        config,
        'ftg_max_range_px',
        getattr(controller.virtual_lidar, 'max_range_px', 500.0),
    )
    resolution = getattr(controller.virtual_lidar, 'resolution_rad', 0.052359877)
    fov_deg = float(getattr(config, 'ftg_fov_deg', math.degrees(FORWARD_VIEW_RAD)))
    fov_rad = math.radians(max(0.0, min(360.0, fov_deg)))
    bubble_radius = float(getattr(config, 'ftg_bubble_radius_px', 0.0))
    if bubble_radius <= 0.0:
        bubble_radius = follow_the_gap_bubble_radius(config)
    
    obstacle_source = getattr(controller, 'gap_planner_obstacles', None)
    if obstacle_source is None:
        obstacles = getattr(controller, 'static_obstacles', [])
    else:
        obstacles = obstacle_source()

    # Refresh lidar scan with current combined obstacles
    controller.virtual_lidar.front_view_rad = fov_rad
    controller.virtual_lidar.scan(origin, heading, obstacles)
    lidar_points = controller.virtual_lidar.latest_points
    
    t0 = time.perf_counter()
    ftg_debug = calculate_follow_the_gap_debug(
        lidar_points=lidar_points,
        origin=origin,
        heading=heading,
        max_range=max_range,
        resolution=resolution,
        bubble_radius=bubble_radius,
        fov_rad=fov_rad,
    )
    ftg_solve_time = (time.perf_counter() - t0) * 1000.0
    ftg_debug['solve_time_ms'] = ftg_solve_time
    target_angle = ftg_debug['target_angle']
    target_dist = ftg_debug['target_dist']
    ftg_debug['origin'] = np.asarray(origin, dtype=np.float32)
    ftg_debug['heading'] = float(heading)
    ftg_debug['scan_ranges'] = ftg_debug.get('ranges')
    raw_target = ftg_debug.get('target')
    controller.latest_ftg_debug = ftg_debug

    # Dynamic centerline-based lookahead search using the costmap
    # Disabled by default so that the advanced planner uses the true follow-the-gap target rather than snapping to the centerline.
    use_centerline_search = getattr(config, 'ftg_use_centerline_search', False)
    if use_centerline_search:
        nearest_idx = getattr(controller, 'latest_nearest_index', path_index)
        best_cand_target = None
        best_cand_cost = float('inf')
        best_cand_index = path_index

        # Scan candidate centerline points in a sliding lookahead window
        for offset in range(4, 30):
            cand_idx = (nearest_idx + offset) % len(controller.track_points)
            cand_point = controller.track_points[cand_idx]
            delta_vec = cand_point - origin
            d = np.linalg.norm(delta_vec)
            ang = math.atan2(delta_vec[1], delta_vec[0]) - heading
            ang = (ang + math.pi) % (2.0 * math.pi) - math.pi

            if abs(ang) <= math.radians(60.0):
                bin_idx = int(np.argmin(np.abs(ftg_debug['angles'] - ang)))
                safe_r = ftg_debug['safe_ranges'][bin_idx]
                
                if safe_r > 0.0 and d < safe_r:
                    is_meters = np.max(ftg_debug['ranges']) < 10.0
                    depth_scale = 1.0 if is_meters else DEFAULT_DEPTH_SCALE
                    cost = (depth_scale / (d + 1e-5)) + DEFAULT_HEADING_WEIGHT * abs(ang)
                    
                    if cost < best_cand_cost:
                        best_cand_cost = cost
                        best_cand_target = cand_point
                        best_cand_index = cand_idx

        if best_cand_target is not None:
            ftg_debug['target'] = best_cand_target
            delta_vec = best_cand_target - origin
            target_dist = float(np.linalg.norm(delta_vec))
            target_angle = float(math.atan2(delta_vec[1], delta_vec[0]) - heading)
            target_angle = (target_angle + math.pi) % (2.0 * math.pi) - math.pi

            # Apply low-pass steering angle smoothing
            global _prev_angle
            alpha = DEFAULT_SMOOTHING_ALPHA
            smoothed_angle = (alpha * target_angle) + ((1.0 - alpha) * _prev_angle)
            _prev_angle = smoothed_angle
            target_angle = smoothed_angle

            ftg_debug['target_angle'] = target_angle
            ftg_debug['target_dist'] = target_dist
            controller.latest_target_index = best_cand_index
            raw_target = best_cand_target

    if raw_target is not None:
        controller.latest_free_space_target = raw_target
        return raw_target
    return controller.track_points[path_index]


if __name__ == '__main__':
    run_tuning_panel()
