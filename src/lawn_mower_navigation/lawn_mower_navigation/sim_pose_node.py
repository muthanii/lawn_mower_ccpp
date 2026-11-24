"""Pose node for the simulation: the true pose from Gazebo -> the mission's pose topic.

The robot has no GNSS receiver for now, so the simulation does not estimate
the pose. This node passes Gazebo's ground truth on as the pose, in the lawn
frame. The lawn frame and the Gazebo world frame are the same: the world is
built from the lawn polygon.

localization.py keeps the EKF (gyro, driven speed and position fixes) for
when a position sensor is added.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lawn_mower_electronics import topics as hw

from . import topics
from .ros_util import QOS_DEPTH


class SimPoseNode(Node):

    def __init__(self):
        super().__init__('sim_pose')
        self.pub = self.create_publisher(Odometry, topics.POSE, QOS_DEPTH)
        self.create_subscription(Odometry, hw.GROUND_TRUTH, self.on_truth, QOS_DEPTH)
        self.get_logger().info('Pose from the Gazebo ground truth (no GNSS)')

    def on_truth(self, msg):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = topics.FRAME
        out.child_frame_id = topics.BASE_FRAME
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimPoseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
