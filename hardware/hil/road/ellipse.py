"""Closed elliptical road geometry for image-space hil tests."""

import json
import math

import numpy as np

from ..common import bounded


def make_ellipse_road_scene(width, height, config):
    """Create a closed laneless elliptical road scene."""
    point_count = 280
    angles = np.linspace(0.0, 2.0 * math.pi, point_count, endpoint=False)
    base = np.column_stack(
        (
            config.track_center_x * width
            + np.cos(angles) * config.track_radius_x * width,
            config.track_center_y * height
            + np.sin(angles) * config.track_radius_y * height,
        )
    ).astype(np.float32)
    tangents, normals = path_tangents_normals(base, closed=True)

    road_half_width_px = config.road_half_width_px
    boundaries = [
        (base - normals * road_half_width_px).astype(np.float32),
        (base + normals * road_half_width_px).astype(np.float32),
    ]
    obstacle_specs = parse_static_obstacle_specs(config.static_obstacles)
    obstacles = [
        make_laneless_static_obstacle(spec, base, tangents, normals, config)
        for spec in obstacle_specs
    ]
    return {
        'centerline': base,
        'boundaries': boundaries,
        'tangents': tangents,
        'normals': normals,
        'road_half_width_px': road_half_width_px,
        'obstacles': obstacles,
    }


def parse_static_obstacle_specs(raw_payload):
    """Return static obstacle specs from CLI JSON or defaults."""
    if not raw_payload:
        return [
            {
                'progress': 0.24,
                'length_px': 95.0,
                'width_px': 58.0,
                'offset': -0.45,
            },
            {
                'progress': 0.52,
                'length_px': 105.0,
                'width_px': 58.0,
                'offset': 0.45,
            },
            {
                'progress': 0.75,
                'length_px': 95.0,
                'width_px': 58.0,
                'offset': -0.45,
            },
        ]
    payload = json.loads(raw_payload)
    if not isinstance(payload, list):
        raise ValueError('--static-obstacles must be a JSON list')
    for item in payload:
        if not isinstance(item, dict):
            raise ValueError('each static obstacle must be a JSON object')
    return payload


def path_tangents_normals(points, closed=False):
    """Return unit tangent and normal vectors for a polyline."""
    if closed:
        previous_points = np.vstack((points[-1], points[:-1]))
        next_points = np.vstack((points[1:], points[0]))
    else:
        previous_points = np.vstack((points[0], points[:-1]))
        next_points = np.vstack((points[1:], points[-1]))
    tangents = next_points - previous_points
    norms = np.linalg.norm(tangents, axis=1)
    norms[norms <= 1e-6] = 1.0
    tangents = tangents / norms[:, None]
    normals = np.column_stack((-tangents[:, 1], tangents[:, 0]))
    return tangents.astype(np.float32), normals.astype(np.float32)


def make_laneless_static_obstacle(spec, centerline, tangents, normals, config):
    """Create an oriented rectangular obstacle inside a laneless road band."""
    progress = bounded(float(spec.get('progress', 0.5)), 0.0, 1.0)
    point_index = int(round(progress * (len(centerline) - 1)))
    length_px = float(spec.get('length_px', config.road_half_width_px))
    width_px = float(spec.get('width_px', config.road_half_width_px * 0.45))
    road_half_width_px = config.road_half_width_px

    if 'lateral_offset_px' in spec:
        lateral_offset = float(spec['lateral_offset_px'])
    elif 'offset_px' in spec:
        lateral_offset = float(spec['offset_px'])
    elif 'offset' in spec:
        lateral_offset = float(spec['offset']) * road_half_width_px
    else:
        lateral_offset = 0.0

    max_offset = max(0.0, road_half_width_px - width_px * 0.5)
    lateral_offset = bounded(lateral_offset, -max_offset, max_offset)
    tangent = tangents[point_index]
    normal = normals[point_index]
    center = centerline[point_index] + normal * lateral_offset
    half_length = max(1.0, length_px * 0.5)
    half_width = max(1.0, width_px * 0.5)
    polygon = np.array(
        [
            center + tangent * half_length + normal * half_width,
            center - tangent * half_length + normal * half_width,
            center - tangent * half_length - normal * half_width,
            center + tangent * half_length - normal * half_width,
        ],
        dtype=np.float32,
    )
    return {
        'center': center.astype(np.float32),
        'tangent': tangent,
        'normal': normal,
        'half_length': half_length,
        'half_width': half_width,
        'polygon': polygon,
        'lateral_offset_px': lateral_offset,
        'progress': progress,
    }


def free_lateral_intervals(lower, upper, blocked_intervals):
    """Return unblocked lateral intervals after removing blocked spans."""
    if lower >= upper:
        return []

    blocked = sorted(blocked_intervals, key=lambda interval: interval[0])
    merged = []
    for start, end in blocked:
        start = bounded(start, lower, upper)
        end = bounded(end, lower, upper)
        if start >= end:
            continue
        if not merged or start > merged[-1][1]:
            merged.append([start, end])
        else:
            merged[-1][1] = max(merged[-1][1], end)

    free = []
    cursor = lower
    for start, end in merged:
        if cursor < start:
            free.append((cursor, start))
        cursor = max(cursor, end)
    if cursor < upper:
        free.append((cursor, upper))
    return free


def obstacle_clearance_px(point, obstacle):
    """Return signed point clearance to an oriented rectangular obstacle."""
    delta = point - obstacle['center']
    local_x = float(np.dot(delta, obstacle['tangent']))
    local_y = float(np.dot(delta, obstacle['normal']))
    dx = abs(local_x) - obstacle['half_length']
    dy = abs(local_y) - obstacle['half_width']
    outside = math.hypot(max(dx, 0.0), max(dy, 0.0))
    inside = min(max(dx, dy), 0.0)
    return outside + inside
