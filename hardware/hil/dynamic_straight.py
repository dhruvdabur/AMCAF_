#!/usr/bin/env python3
"""Compatibility launcher for the straight-road follower with dynamic obstacles."""

from .straight_static import *  # noqa: F401,F403
from .config.dynamic_straight import parse_args
from .config.dynamic_straight import print_config
from .config.dynamic_straight import validate_config
from .control import DynamicStraightControlResult
from .control import DynamicStraightController


def __getattr__(name):
    """Load ROS-only node objects lazily for simulator-friendly imports."""
    if name == 'ArucoTrackFollower':
        from .nodes.dynamic_straight_node import ArucoTrackFollower

        return ArucoTrackFollower
    if name == 'make_sensor_qos':
        from .runtime import make_sensor_qos

        return make_sensor_qos
    raise AttributeError(name)


def main(args=None):
    """Run the ROS dynamic_straight node."""
    from .nodes.dynamic_straight_node import main as node_main

    return node_main(args)


if __name__ == '__main__':
    main()
