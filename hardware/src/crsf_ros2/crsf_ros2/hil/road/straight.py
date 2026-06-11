"""Straight road geometry for image-space hil tests."""

import numpy as np

from ..common import bounded
from .ellipse import make_laneless_static_obstacle
from .ellipse import parse_static_obstacle_specs
from .ellipse import path_tangents_normals


def make_straight_road_scene(width, height, config):
    """Create an open laneless straight road scene."""
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
