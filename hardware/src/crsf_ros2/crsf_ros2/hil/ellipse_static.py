#!/usr/bin/env python3
"""Compatibility launcher for the closed-ellipse ArUco hil follower."""

from .common import bounded
from .common import wrap_angle
from .config.ellipse_static import ARMING_SERVICE
from .config.ellipse_static import COMMAND_TOPIC
from .config.ellipse_static import IMAGE_TOPIC
from .config.ellipse_static import NEUTRAL_VALUE
from .config.ellipse_static import parse_args
from .config.ellipse_static import print_config
from .config.ellipse_static import SETTLE_DURATION
from .config.ellipse_static import UPDATE_RATE_HZ
from .config.ellipse_static import validate_config
from .control import EllipseControlResult
from .control import EllipseStaticController
from .metrics import FollowerMetrics
from .metrics import unique_metrics_path
from .road import free_lateral_intervals
from .road import make_ellipse_road_scene
from .road import make_laneless_static_obstacle
from .road import obstacle_clearance_px
from .road import parse_static_obstacle_specs
from .road import path_tangents_normals
from .tuning import EllipseStaticTuning
from .ui import draw_label
from .ui import draw_vector
from .ui import noop
from .ui import put_status
from .ui import resize_for_preview


def __getattr__(name):
    """Load ROS-only node objects lazily for simulator-friendly imports."""
    if name == 'ArucoTrackFollower':
        from .nodes.ellipse_static_node import ArucoTrackFollower

        return ArucoTrackFollower
    if name == 'make_sensor_qos':
        from .runtime import make_sensor_qos

        return make_sensor_qos
    raise AttributeError(name)


def main(args=None):
    """Run the ROS ellipse_static node."""
    from .nodes.ellipse_static_node import main as node_main

    return node_main(args)


__all__ = [
    'ARMING_SERVICE',
    'COMMAND_TOPIC',
    'EllipseStaticTuning',
    'EllipseStaticController',
    'EllipseControlResult',
    'FollowerMetrics',
    'IMAGE_TOPIC',
    'NEUTRAL_VALUE',
    'SETTLE_DURATION',
    'UPDATE_RATE_HZ',
    'bounded',
    'draw_label',
    'draw_vector',
    'free_lateral_intervals',
    'main',
    'make_ellipse_road_scene',
    'make_laneless_static_obstacle',
    'noop',
    'obstacle_clearance_px',
    'parse_args',
    'parse_static_obstacle_specs',
    'path_tangents_normals',
    'print_config',
    'put_status',
    'resize_for_preview',
    'unique_metrics_path',
    'validate_config',
    'wrap_angle',
]

if __name__ == '__main__':
    main()
