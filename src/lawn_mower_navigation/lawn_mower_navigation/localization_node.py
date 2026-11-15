"""Localisation node: GNSS fixes, the gyro and the driven speed -> pose.

The forward speed comes from the pulses the controller sends to the motor
driver (speed_source:=pulses), because the robot has no wheel encoders, or
from wheel odometry (speed_source:=odometry). The speed from the pulses goes
through a model of the driver's lag and the drive's acceleration limit. The first fix sets the position.
The first heading is the initial_heading parameter; driving corrects it.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import UInt16MultiArray

from lawn_mower_electronics import topics as hw
from lawn_mower_electronics.config import load_electronics_config, resolve_electronics_config
from lawn_mower_electronics.motor_driver import MotorDriver
from lawn_mower_electronics.sensors import GNSS_TOPIC, IMU_TOPIC
from lawn_mower_robot.kinematics import SkidSteerKinematics
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config

from . import topics
from .geo import to_local
from .localization import PoseFilter, SpeedModel
from .ros_util import QOS_DEPTH, set_yaw, stamp_seconds
from .world import DEFAULT_LATITUDE, DEFAULT_LONGITUDE

SPEED_SOURCES = ('pulses', 'odometry')
DEFAULT_SPEED_STDDEV = 0.1          # m/s, error of the speed taken from the pulses
DEFAULT_YAW_RATE_STDDEV = 0.01      # rad/s, gyro noise and drift
DEFAULT_HEADING_STDDEV = 0.3        # rad, error of initial_heading


class LocalizationNode(Node):

    def __init__(self):
        super().__init__('localization')
        robot = load_robot_config(resolve_robot_config(
            self.declare_parameter('robot', 'faculty_robot_estimate').value))
        electronics = load_electronics_config(resolve_electronics_config(
            self.declare_parameter('electronics', 'faculty_robot').value))
        self.origin = (self.declare_parameter('origin_latitude', DEFAULT_LATITUDE).value,
                       self.declare_parameter('origin_longitude', DEFAULT_LONGITUDE).value)
        self.initial_heading = self.declare_parameter('initial_heading', 0.0).value
        source = self.declare_parameter('speed_source', SPEED_SOURCES[0]).value
        if source not in SPEED_SOURCES:
            raise ValueError(f'speed_source must be one of {SPEED_SOURCES}')
        self.gnss_stddev = self.declare_parameter(
            'gnss_stddev', electronics.sensors.gnss.horizontal_stddev).value
        self.filter = PoseFilter(
            self.declare_parameter('speed_stddev', DEFAULT_SPEED_STDDEV).value,
            self.declare_parameter('yaw_rate_stddev', DEFAULT_YAW_RATE_STDDEV).value,
            self.declare_parameter('heading_stddev', DEFAULT_HEADING_STDDEV).value)

        self.kin = SkidSteerKinematics.from_config(robot)
        self.driver = MotorDriver(electronics, robot.drive.wheel_max_speed)
        self.speed_model = SpeedModel(electronics.motor_driver.time_constant,
                                      robot.limits.max_linear_acceleration)
        self.from_pulses = source == 'pulses'
        self.target_speed = 0.0
        self.speed = 0.0
        self.last_imu = None
        self.pub = self.create_publisher(Odometry, topics.POSE, QOS_DEPTH)
        self.create_subscription(NavSatFix, GNSS_TOPIC, self.on_fix, QOS_DEPTH)
        self.create_subscription(Imu, IMU_TOPIC, self.on_imu, QOS_DEPTH)
        if source == 'pulses':
            self.create_subscription(UInt16MultiArray, hw.PWM_DRIVE, self.on_pulses, QOS_DEPTH)
        else:
            self.create_subscription(Odometry, topics.WHEEL_ODOM, self.on_wheel_odom, QOS_DEPTH)
        self.get_logger().info(f'Speed from {source}; origin {self.origin}')

    def on_pulses(self, msg):
        left, right = (self.driver.target(p) for p in msg.data[:2])
        self.target_speed, _ = self.kin.forward(left, right)

    def on_wheel_odom(self, msg):
        self.speed = msg.twist.twist.linear.x

    def on_fix(self, msg):
        x, y = to_local(msg.latitude, msg.longitude, self.origin)
        if not self.filter.ready:
            self.filter.reset(x, y, self.initial_heading, self.gnss_stddev)
            self.get_logger().info(f'First fix at ({x:.2f}, {y:.2f}) m')
        else:
            self.filter.correct(x, y, self.gnss_stddev)

    def on_imu(self, msg):
        t = stamp_seconds(msg.header.stamp)
        omega = msg.angular_velocity.z
        if self.last_imu is not None:
            dt = t - self.last_imu
            if self.from_pulses:
                self.speed = self.speed_model.update(self.target_speed, dt)
            self.filter.predict(self.speed, omega, dt)
        self.last_imu = t
        if not self.filter.ready:
            return
        x, y, heading = self.filter.pose
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = topics.FRAME
        out.child_frame_id = topics.BASE_FRAME
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        set_yaw(out.pose.pose.orientation, heading)
        out.twist.twist.linear.x = self.speed
        out.twist.twist.angular.z = omega
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
