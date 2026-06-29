"""Straight road geometry for image-space hil tests."""

import numpy as np

from ..common import bounded
from .ellipse import make_laneless_static_obstacle
from .ellipse import parse_static_obstacle_specs
from .ellipse import path_tangents_normals


def make_straight_road_scene(width, height, config):
    """Create an open laneless straight road scene."""
    if getattr(config, 'use_safe_control_env', False):
        return make_safe_control_road_scene(width, height, config)
    point_count = 220
    road_length = bounded(config.road_length_x, 0.25, 0.98) * width
    start_x = config.track_center_x * width - road_length * 0.5
    end_x = config.track_center_x * width + road_length * 0.5
    x_values = np.linspace(start_x, end_x, point_count)
    y_values = np.full(point_count, config.track_center_y * height)
    centerline = np.column_stack((x_values, y_values)).astype(np.float32)
    tangents, normals = path_tangents_normals(centerline, closed=False)

    road_half_width_px = config.road_half_width_px
    boundaries = [
        (centerline - normals * road_half_width_px).astype(np.float32),
        (centerline + normals * road_half_width_px).astype(np.float32),
    ]
    obstacle_specs = parse_static_obstacle_specs(config.static_obstacles)
    obstacles = [
        make_laneless_static_obstacle(spec, centerline, tangents, normals, config)
        for spec in obstacle_specs
    ]
    obstacles.extend(
        make_straight_road_boundary_walls(centerline, tangents, normals, config)
    )
    if config.add_road_end_walls:
        obstacles.extend(
            make_straight_road_end_walls(centerline, tangents, normals, config)
        )

    return {
        'centerline': centerline,
        'boundaries': boundaries,
        'tangents': tangents,
        'normals': normals,
        'road_half_width_px': road_half_width_px,
        'obstacles': obstacles,
    }


def make_straight_road_boundary_walls(centerline, tangents, normals, config):
    """Create two solid obstacle walls along the straight road boundaries."""
    wall_thickness = max(8.0, float(config.obstacle_margin_px) * 0.35)
    half_width = wall_thickness * 0.5
    road_half_width_px = float(config.road_half_width_px)
    start = centerline[0]
    end = centerline[-1]
    segment = end - start
    length = float(np.linalg.norm(segment))
    if length <= 1e-6:
        return []
    tangent = (segment / length).astype(np.float32)
    base_center = 0.5 * (start + end)
    normal = normals[0].astype(np.float32)
    half_length = 0.5 * length + 1.0

    walls = []
    for side_name, side_sign in (('left', -1.0), ('right', 1.0)):
        outward = (normal * side_sign).astype(np.float32)
        boundary_center = base_center + normal * (side_sign * road_half_width_px)
        center = boundary_center + outward * half_width
        polygon = np.array(
            [
                center + tangent * half_length + outward * half_width,
                center - tangent * half_length + outward * half_width,
                center - tangent * half_length - outward * half_width,
                center + tangent * half_length - outward * half_width,
            ],
            dtype=np.float32,
        )
        walls.append(
            {
                'center': center.astype(np.float32),
                'tangent': tangent,
                'normal': outward,
                'half_length': half_length,
                'half_width': half_width,
                'polygon': polygon,
                'lateral_offset_px': 0.0,
                'progress': 0.5,
                'kind': f'road_boundary_wall_{side_name}',
            }
        )
    return walls


def make_straight_road_end_walls(centerline, tangents, normals, config):
    """Create obstacle polygons that close the start and end of the road."""
    wall_thickness = max(18.0, config.obstacle_margin_px * 0.5)
    wall_half_width = config.road_half_width_px + config.obstacle_margin_px
    walls = []
    for progress, point_index in ((0.0, 0), (1.0, len(centerline) - 1)):
        center = centerline[point_index]
        tangent = tangents[point_index]
        normal = normals[point_index]
        half_length = wall_thickness * 0.5
        half_width = wall_half_width
        polygon = np.array(
            [
                center + tangent * half_length + normal * half_width,
                center - tangent * half_length + normal * half_width,
                center - tangent * half_length - normal * half_width,
                center + tangent * half_length - normal * half_width,
            ],
            dtype=np.float32,
        )
        walls.append(
            {
                'center': center.astype(np.float32),
                'tangent': tangent,
                'normal': normal,
                'half_length': half_length,
                'half_width': half_width,
                'polygon': polygon,
                'lateral_offset_px': 0.0,
                'progress': progress,
                'kind': 'road_end_wall',
            }
        )
    return walls


def make_safe_control_road_scene(width, height, config):
    """Create the safe_control square closed-loop road scene with obstacles."""
    # Scale factors from 14x14 meter environment to pixel width x height
    scale_x = float(width) / 14.0
    scale_y = float(height) / 14.0

    def map_coords(x, y):
        return [float(x) * scale_x, float(height) - float(y) * scale_y]

    waypoints = np.array([
        map_coords(2.0, 2.0),
        map_coords(2.0, 12.0),
        map_coords(12.0, 12.0),
        map_coords(12.0, 2.0),
        map_coords(2.0, 2.0)
    ], dtype=np.float32)

    # Interpolate waypoints to get a smooth centerline
    segment_points = 55
    centerline = []
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i+1]
        for t in np.linspace(0.0, 1.0, segment_points, endpoint=False):
            centerline.append(start + t * (end - start))
    centerline = np.array(centerline, dtype=np.float32)

    # Calculate tangents and normals
    tangents, normals = path_tangents_normals(centerline, closed=True)

    # Create road boundaries
    road_half_width_px = float(config.road_half_width_px)
    boundaries = [
        (centerline - normals * road_half_width_px).astype(np.float32),
        (centerline + normals * road_half_width_px).astype(np.float32),
    ]

    # Create the circular/octagonal obstacles from safe_control
    known_obs = [
        [2.2, 5.0, 0.2],
        [3.0, 5.0, 0.2],
        [4.0, 9.0, 0.3],
        [1.5, 10.0, 0.5],
        [9.0, 11.0, 1.0],
        [7.0, 7.0, 3.0],
        [4.0, 3.5, 1.5],
        [10.0, 7.3, 0.4],
        [6.0, 13.0, 0.7],
        [5.0, 10.0, 0.6],
        [11.0, 5.0, 0.8],
        [13.5, 11.0, 0.6],
        [2.0, 7.0, 0.7],
        [2.0, 8.0, 0.5]
    ]

    obstacles = []
    for i, obs in enumerate(known_obs):
        ox, oy, radius = obs
        cx, cy = map_coords(ox, oy)
        
        # Scale the radius using scale_x
        rad_px = float(radius) * scale_x
        
        # Build an 8-sided polygon to represent the circle
        theta_vals = np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
        polygon = np.array([
            [cx + rad_px * np.cos(t), cy + rad_px * np.sin(t)]
            for t in theta_vals
        ], dtype=np.float32)

        # Estimate closest progress on centerline
        dists = np.linalg.norm(centerline - np.array([cx, cy]), axis=1)
        closest_idx = int(np.argmin(dists))
        progress = float(closest_idx) / len(centerline)

        obstacles.append({
            'center': np.array([cx, cy], dtype=np.float32),
            'tangent': tangents[closest_idx],
            'normal': normals[closest_idx],
            'half_length': rad_px,
            'half_width': rad_px,
            'polygon': polygon,
            'lateral_offset_px': float(np.dot(np.array([cx, cy]) - centerline[closest_idx], normals[closest_idx])),
            'progress': progress,
            'kind': f'circle_obstacle_{i}',
        })

    return {
        'centerline': centerline,
        'boundaries': boundaries,
        'tangents': tangents,
        'normals': normals,
        'road_half_width_px': road_half_width_px,
        'obstacles': obstacles,
    }
