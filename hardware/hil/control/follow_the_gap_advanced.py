"""Advanced Follow-the-Gap target selection for LiDAR scans."""

import math
from pathlib import Path
import re

import numpy as np


DEFAULT_BUBBLE_RADIUS = 95.7812
DEFAULT_MAX_RANGE_CAP = 222.024
DEFAULT_DEEP_CLUSTER_RATIO = 0.95
DEFAULT_OBSTACLE_EPSILON = 1.0
FORWARD_VIEW_RAD = math.radians(120.0)


_prev_angle = 0.0
_f1tenth_prev_angle = 0.0


def extend_disparities(ranges, angles, vehicle_width=38.125):
    """Draw a virtual safety radius around depth disparities to clear vehicle width."""
    ranges = np.asarray(ranges, dtype=np.float32)
    angles = np.asarray(angles, dtype=np.float32)
    extended_ranges = np.copy(ranges)
    
    # Auto-detect scale: if max range is < 10, assume meters (F1Tenth), else pixels
    is_meters = np.max(ranges) < 10.0
    threshold = 0.5 if is_meters else 25.0
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
            depth_scale = 1.0 if is_meters else 100.0
            cost = (depth_scale / (depth + 1e-5)) + (2.0 * heading_error)
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
    alpha = 0.25
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

    fig = plt.figure(figsize=(12.0, 7.0))
    ax = fig.add_axes([0.06, 0.27, 0.58, 0.66], projection='polar')
    info_ax = fig.add_axes([0.68, 0.27, 0.28, 0.66])
    info_ax.axis('off')

    angles, ranges_raw = generate_fake_lidar()
    raw_plot = ax.scatter(angles, ranges_raw, c='tab:blue', s=10, label='Clipped scan')
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
            'Tuning\n'
            'Bubble Radius: clears rays near obstacle\n'
            'Max Range Cap: farthest usable distance\n\n'
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

    ax_bubble = fig.add_axes([0.18, 0.16, 0.56, 0.035])
    ax_cap = fig.add_axes([0.18, 0.09, 0.56, 0.035])
    ax_save = fig.add_axes([0.80, 0.07, 0.12, 0.06])

    slider_bubble = Slider(
        ax_bubble,
        'Bubble Radius',
        5.0,
        250.0,
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
        safe_ranges, bubble_center = apply_safety_bubble(
            ranges_capped,
            angles,
            bubble_r,
            cap_val,
        )
        target_angle, target_dist = calculate_follow_the_gap_target(
            ranges_capped,
            angles,
            bubble_r,
            safe_ranges=safe_ranges,
        )
        min_angle, min_dist = nearest_obstacle_point(
            ranges_capped,
            angles,
            max_range=cap_val,
        )
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
        safe_mask = safe_ranges > 0.0
        if np.any(safe_mask):
            safe_plot.set_offsets(np.c_[angles[safe_mask], safe_ranges[safe_mask]])
        else:
            safe_plot.set_offsets(np.empty((0, 2)))
        zeroed = int(np.count_nonzero(~safe_mask))
        target_deg = math.degrees(target_angle)
        obstacle_text = 'none'
        if bubble_center is not None:
            obstacle_text = f'{math.degrees(bubble_center[0]):.1f}deg/{bubble_center[1]:.1f}px'
        status_text.set_text(
            'Live FTG\n'
            f'target : {target_deg:6.1f} deg, {target_dist:6.1f} px\n'
            f'nearest: {obstacle_text}\n'
            f'masked : {zeroed}/{len(safe_ranges)} rays'
        )
        ax.set_ylim(0, max(520.0, cap_val * 1.08))
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
    alpha = 0.25
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


if __name__ == '__main__':
    run_tuning_panel()
