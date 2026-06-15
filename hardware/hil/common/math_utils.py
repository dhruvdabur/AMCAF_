"""Small math helpers shared by hil follower modules."""

import math


def bounded(value, lower, upper):
    """Return value restricted to inclusive lower and upper limits."""
    return max(lower, min(value, upper))


def wrap_angle(angle):
    """Wrap an angle to -pi..pi."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
