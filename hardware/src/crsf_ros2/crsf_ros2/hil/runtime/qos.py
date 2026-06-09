"""ROS QoS profiles used by hil follower nodes."""

from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


def make_sensor_qos():
    """Create low-latency QoS that keeps only the newest image."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )
