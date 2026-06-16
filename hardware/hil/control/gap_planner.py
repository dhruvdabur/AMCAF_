"""Simple lateral interval gap planner for HIL road followers."""

import numpy as np
from ..common import bounded
from ..road import free_lateral_intervals

def free_space_target(controller, path_index):
    """
    Selects the lateral free-space target point based on obstacle projections.
    
    This replaces complex LiDAR scan logic with a clean, geometric obstacle 
    projection onto the road's lateral coordinate axis.
    """
    # 1. Check if the road geometry is available
    if (
        controller.road_normals is None
        or controller.road_tangents is None
        or controller.road_half_width_px <= 0.0
    ):
        return controller.track_points[path_index]

    # 2. Extract road local coordinate frame at the lookahead index
    center = controller.track_points[path_index]
    tangent = controller.road_tangents[path_index]
    normal = controller.road_normals[path_index]

    # 3. Define the lateral road boundaries (incorporating obstacle margin)
    margin = max(4.0, getattr(controller.config, 'obstacle_margin_px', 20.0))
    lower = -controller.road_half_width_px + margin
    upper = controller.road_half_width_px - margin

    # 4. Project obstacles within the lookahead window onto the lateral axis
    lookahead_length = max(
        controller.config.road_half_width_px,
        controller.config.lookahead_points * 4.0,
    )
    obstacle_source = getattr(controller, 'gap_planner_obstacles', None)
    if obstacle_source is None:
        obstacles = getattr(controller, 'static_obstacles', [])
    else:
        obstacles = obstacle_source()

    blocked = []
    for obstacle in obstacles:
        # Calculate longitudinal (along) and lateral (across) offsets
        delta = obstacle['center'] - center
        along = float(np.dot(delta, tangent))
        
        # Check if the obstacle is within the longitudinal lookahead window
        if abs(along) <= lookahead_length + obstacle['half_length']:
            lateral = float(np.dot(delta, normal))
            half_width = obstacle['half_width'] + margin
            
            # Map the obstacle boundary to blocked lateral coordinates
            blocked_lower = max(lower, lateral - half_width)
            blocked_upper = min(upper, lateral + half_width)
            if blocked_lower < blocked_upper:
                blocked.append((blocked_lower, blocked_upper))

    # 5. Compute the remaining free lateral intervals (gaps)
    free_intervals = free_lateral_intervals(lower, upper, blocked)
    # Filter out gaps that are too narrow for the vehicle
    free_intervals = [
        gap for gap in free_intervals 
        if (gap[1] - gap[0]) >= margin
    ]

    # 6. Select the best gap and determine the lateral target
    planner_mode = getattr(controller.config, 'gap_planner_mode', 'stable_free_space')
    
    if not free_intervals or planner_mode == 'centerline':
        # Default to the centerline if no gaps exist or centerline mode is selected
        best_lower, best_upper = lower, upper
        lateral_target = 0.0
    else:
        # Hysteresis selection to prevent gap snapping/chattering
        previous = getattr(controller, 'smoothed_free_space_lateral_target_px', None)
        if previous is not None and planner_mode == 'stable_free_space':
            # 1. Prioritize gaps containing the previous target
            containing = [
                gap for gap in free_intervals
                if gap[0] <= previous <= gap[1]
            ]
            if containing:
                # Keep tracking the same gap (pick the widest of those containing previous)
                best_lower, best_upper = max(containing, key=lambda gap: gap[1] - gap[0])
            else:
                # 2. Otherwise, pick the gap closest to the previous target
                best_lower, best_upper = min(
                    free_intervals,
                    key=lambda gap: min(abs(previous - gap[0]), abs(previous - gap[1]))
                )
        else:
            # Default/raw mode: Pick the widest gap overall
            best_lower, best_upper = max(
                free_intervals,
                key=lambda gap: gap[1] - gap[0],
            )
        lateral_target = 0.5 * (best_lower + best_upper)

    # 7. Apply simple low-pass smoothing if tracking in stable mode
    if planner_mode == 'stable_free_space':
        alpha = float(getattr(controller.config, 'gap_target_smoothing_alpha', 0.18))
        previous = getattr(controller, 'smoothed_free_space_lateral_target_px', None)
        if previous is not None:
            # Smoothly transition from previous target, clamped inside the chosen gap
            safe_previous = bounded(previous, best_lower, best_upper)
            lateral_target = safe_previous + alpha * (lateral_target - safe_previous)

    # 8. Save planning state to the controller for debugging and visualization
    controller.latest_free_space_blocked_intervals = blocked
    controller.latest_free_space_intervals = free_intervals
    controller.latest_free_space_interval = (best_lower, best_upper)
    controller.latest_free_space_lateral_target_px = lateral_target
    controller.smoothed_free_space_lateral_target_px = lateral_target
    controller.latest_free_space_target = center + normal * lateral_target

    return controller.latest_free_space_target
