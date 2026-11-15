"""ROS 2 nodes of the electronics models, all in one process.

    ros2 run lawn_mower_electronics electronics --ros-args \\
        -p robot:=faculty_robot_estimate -p electronics:=faculty_robot

Each node wraps one model of this package and talks over the topics in
topics.py, so a node can be replaced by the real part: the controller node by
the controller board, the driver node by the L-faster, and so on.
"""

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contacts
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64, Float64MultiArray, String, UInt16MultiArray

from lawn_mower_robot.energy import SkidSteerEnergyModel
from lawn_mower_robot.kinematics import SkidSteerKinematics
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config

from . import topics
from .battery import Battery, PowerMonitor
from .blade import Blade
from .bumper import Bumper
from .config import load_electronics_config, resolve_electronics_config
from .controller import Controller
from .motor_driver import MotorDriver
from .rc import Receiver
from .sensors import CONTACT_TOPIC

QOS_DEPTH = 10
DEFAULT_ROBOT = 'faculty_robot_estimate'
DEFAULT_ELECTRONICS = 'faculty_robot'
MONITOR_RATE = 10.0  # Hz, battery and energy reports


class ModelNode(Node):
    """Base: loads both configs and gives the time in seconds."""

    def __init__(self, name):
        super().__init__(name)
        robot = self.declare_parameter('robot', DEFAULT_ROBOT).value
        electronics = self.declare_parameter('electronics', DEFAULT_ELECTRONICS).value
        self.robot = load_robot_config(resolve_robot_config(robot))
        self.electronics = load_electronics_config(resolve_electronics_config(electronics))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9


class ReceiverNode(ModelNode):

    def __init__(self):
        super().__init__('rc_receiver')
        self.model = Receiver(self.electronics.rc)
        self.channels_pub = self.create_publisher(UInt16MultiArray, topics.RC_CHANNELS, QOS_DEPTH)
        self.failsafe_pub = self.create_publisher(Bool, topics.RC_FAILSAFE, QOS_DEPTH)
        self.create_subscription(UInt16MultiArray, topics.RC_TRANSMITTER, self.on_frame, QOS_DEPTH)
        self.create_timer(self.electronics.rc.frame_period, self.step)

    def on_frame(self, msg):
        try:
            self.model.feed(self.now(), msg.data)
        except ValueError as e:
            self.get_logger().warning(str(e), throttle_duration_sec=1.0)

    def step(self):
        channels, failsafe = self.model.output(self.now())
        self.channels_pub.publish(UInt16MultiArray(data=channels))
        self.failsafe_pub.publish(Bool(data=failsafe))


class BumperNode(ModelNode):

    def __init__(self):
        super().__init__('bumper')
        self.model = Bumper(self.electronics.bumper.hold_time)
        self.pub = self.create_publisher(Bool, topics.BUMPER, QOS_DEPTH)
        self.create_subscription(Contacts, CONTACT_TOPIC, self.on_contacts, QOS_DEPTH)
        self.create_timer(1.0 / self.electronics.controller.rate, self.step)

    def on_contacts(self, msg):
        if msg.contacts:
            self.model.touch(self.now())

    def step(self):
        self.pub.publish(Bool(data=self.model.pressed(self.now())))


class ControllerNode(ModelNode):

    def __init__(self):
        super().__init__('controller')
        self.model = Controller(self.electronics, SkidSteerKinematics.from_config(self.robot))
        self.channels = None
        self.failsafe = True
        self.estop = False
        self.bumper = False
        self.state = None
        self.pwm_pub = self.create_publisher(UInt16MultiArray, topics.PWM_DRIVE, QOS_DEPTH)
        self.relay_pub = self.create_publisher(Bool, topics.BLADE_RELAY, QOS_DEPTH)
        self.state_pub = self.create_publisher(String, topics.CONTROLLER_STATE, QOS_DEPTH)
        self.create_subscription(UInt16MultiArray, topics.RC_CHANNELS, self.on_channels, QOS_DEPTH)
        self.create_subscription(Bool, topics.RC_FAILSAFE, self.on_failsafe, QOS_DEPTH)
        self.create_subscription(Twist, topics.CMD_VEL, self.on_command, QOS_DEPTH)
        self.create_subscription(Bool, topics.BLADE_REQUEST,
                                 lambda m: self.model.set_blade_request(m.data), QOS_DEPTH)
        self.create_subscription(Bool, topics.ESTOP, self.on_estop, QOS_DEPTH)
        self.create_subscription(Bool, topics.BUMPER,
                                 lambda m: setattr(self, 'bumper', m.data), QOS_DEPTH)
        self.create_timer(1.0 / self.electronics.controller.rate, self.step)

    def on_channels(self, msg):
        self.channels = list(msg.data)

    def on_failsafe(self, msg):
        self.failsafe = msg.data

    def on_estop(self, msg):
        self.estop = msg.data

    def on_command(self, msg):
        self.model.set_command(self.now(), msg.linear.x, msg.angular.z)

    def step(self):
        # No receiver output yet is the same as a lost link.
        channels = self.channels or list(self.electronics.rc.failsafe)
        out = self.model.step(self.now(), channels, self.failsafe or self.channels is None,
                              self.estop, self.bumper)
        self.pwm_pub.publish(UInt16MultiArray(data=[out.left, out.right]))
        self.relay_pub.publish(Bool(data=out.blade))
        self.state_pub.publish(String(data=out.state))
        if out.state != self.state:
            self.get_logger().info(f'Controller state: {out.state}')
            self.state = out.state


class MotorDriverNode(ModelNode):

    def __init__(self):
        super().__init__('motor_driver')
        self.kin = SkidSteerKinematics.from_config(self.robot)
        self.model = MotorDriver(self.electronics, self.robot.drive.wheel_max_speed)
        self.estop = False
        self.twist_pub = self.create_publisher(Twist, topics.MOTOR_CMD_VEL, QOS_DEPTH)
        self.speeds_pub = self.create_publisher(Float64MultiArray, topics.WHEEL_SPEEDS, QOS_DEPTH)
        self.create_subscription(UInt16MultiArray, topics.PWM_DRIVE, self.on_pulses, QOS_DEPTH)
        self.create_subscription(Bool, topics.ESTOP, self.on_estop, QOS_DEPTH)
        self.create_timer(1.0 / self.electronics.controller.rate, self.step)

    def on_pulses(self, msg):
        self.model.set_pulses(self.now(), msg.data[0], msg.data[1])

    def on_estop(self, msg):
        self.estop = msg.data

    def step(self):
        # The emergency stop opens the supply of the driver.
        left, right = self.model.step(self.now(), powered=not self.estop)
        twist = Twist()
        twist.linear.x, twist.angular.z = self.kin.forward(left, right)
        self.twist_pub.publish(twist)
        self.speeds_pub.publish(Float64MultiArray(data=[left, right]))


class BladeNode(ModelNode):

    def __init__(self):
        super().__init__('blade')
        self.model = Blade(self.electronics, self.robot.mower.blade_power)
        self.relay = False
        self.estop = False
        self.speed_pub = self.create_publisher(Float64, topics.BLADE_SPEED, QOS_DEPTH)
        self.running_pub = self.create_publisher(Bool, topics.BLADE_RUNNING, QOS_DEPTH)
        self.create_subscription(Bool, topics.BLADE_RELAY, self.on_relay, QOS_DEPTH)
        self.create_subscription(Bool, topics.ESTOP, self.on_estop, QOS_DEPTH)
        self.create_timer(1.0 / self.electronics.controller.rate, self.step)

    def on_relay(self, msg):
        self.relay = msg.data

    def on_estop(self, msg):
        self.estop = msg.data

    def step(self):
        self.model.step(self.now(), self.relay, powered=not self.estop)
        self.speed_pub.publish(Float64(data=self.model.speed))
        self.running_pub.publish(Bool(data=self.model.running))


class PowerMonitorNode(ModelNode):

    def __init__(self):
        super().__init__('power_monitor')
        self.battery = Battery.from_configs(self.robot, self.electronics)
        self.model = PowerMonitor(SkidSteerEnergyModel.from_config(self.robot), self.battery)
        self.blade = Blade(self.electronics, self.robot.mower.blade_power)
        self.battery_pub = self.create_publisher(BatteryState, topics.BATTERY, QOS_DEPTH)
        self.energy_pub = self.create_publisher(Float64MultiArray, topics.ENERGY, QOS_DEPTH)
        self.create_subscription(Odometry, topics.GROUND_TRUTH, self.on_odom, QOS_DEPTH)
        self.create_subscription(Float64, topics.BLADE_SPEED, self.on_blade, QOS_DEPTH)
        self.create_timer(1.0 / MONITOR_RATE, self.report)

    def on_blade(self, msg):
        self.blade.fraction = msg.data / self.electronics.blade.rated_speed

    def on_odom(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.model.update(t, msg.twist.twist.linear.x, msg.twist.twist.angular.z,
                          self.blade.power())

    def report(self):
        b = self.battery
        if b.terminal_voltage is None:
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = b.terminal_voltage
        msg.current = -b.current   # negative while discharging
        msg.design_capacity = b.capacity
        msg.capacity = b.capacity * b.state_of_charge
        msg.percentage = b.state_of_charge
        msg.present = True
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIFE
        self.battery_pub.publish(msg)
        self.energy_pub.publish(Float64MultiArray(data=[
            b.energy.get(name, 0.0)
            for name in (PowerMonitor.TRACTION, PowerMonitor.BLADE, PowerMonitor.AUXILIARY)]))


NODES = (ReceiverNode, BumperNode, ControllerNode, MotorDriverNode, BladeNode, PowerMonitorNode)


def main(args=None):
    rclpy.init(args=args)
    executor = SingleThreadedExecutor()
    nodes = [cls() for cls in NODES]
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # A callback that was already queued can fail while the context shuts down.
        if rclpy.ok():
            raise
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
