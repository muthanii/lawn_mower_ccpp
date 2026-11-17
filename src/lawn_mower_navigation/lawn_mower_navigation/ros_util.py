"""Small helpers shared by the nodes."""

import math

from rclpy.qos import DurabilityPolicy, QoSProfile

QOS_DEPTH = 10


def latched():
    """QoS of a topic whose last message late subscribers still get."""
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def set_yaw(q, yaw):
    q.x, q.y = 0.0, 0.0
    q.z, q.w = math.sin(yaw / 2), math.cos(yaw / 2)
