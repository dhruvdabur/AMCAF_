"""ROS node implementations for hil follower executables."""

__all__ = [
    'ArucoTrackFollower',
    'DynamicStraightArucoTrackFollower',
    'StraightArucoTrackFollower',
    'dynamic_straight_main',
    'main',
    'straight_main',
]


def __getattr__(name):
    """Load ROS node classes only when callers ask for them."""
    if name in ('ArucoTrackFollower', 'main'):
        from .ellipse_static_node import ArucoTrackFollower
        from .ellipse_static_node import main

        return {'ArucoTrackFollower': ArucoTrackFollower, 'main': main}[name]
    if name in ('DynamicStraightArucoTrackFollower', 'dynamic_straight_main'):
        from .dynamic_straight_node import ArucoTrackFollower
        from .dynamic_straight_node import main

        return {
            'DynamicStraightArucoTrackFollower': ArucoTrackFollower,
            'dynamic_straight_main': main,
        }[name]
    if name in ('StraightArucoTrackFollower', 'straight_main'):
        from .straight_static_node import ArucoTrackFollower
        from .straight_static_node import main

        return {
            'StraightArucoTrackFollower': ArucoTrackFollower,
            'straight_main': main,
        }[name]
    raise AttributeError(name)
