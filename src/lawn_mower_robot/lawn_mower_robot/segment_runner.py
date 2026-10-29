"""Drive a list of segments in Gazebo and log the energy of each one.

Smoke test for the robot config: each segment is "straight <metres>" or
"turn <degrees>" (in place). The node drives it closed loop on the ground-truth
odometry, integrates the energy model over the measured velocities, and writes
one CSV row per segment next to the model's closed-form prediction.
"""

import csv
import math
import os
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .energy import EnergyIntegrator, SkidSteerEnergyModel
from .kinematics import SkidSteerKinematics
from .robot_config import load_robot_config, resolve_robot_config

DEFAULT_SEGMENTS = ['straight 2.0', 'turn 90']
DEFAULT_SPEED_FRACTION = 0.5
DEFAULT_CONTROL_RATE = 20.0  # Hz
DEFAULT_SETTLE_TIME = 1.0    # s, coasting to rest after each segment
DEFAULT_START_DELAY = 2.0    # s, lets the robot settle on the ground plane
QOS_DEPTH = 10
CSV_DIGITS = 6

CSV_FIELDS = ['index', 'kind', 'target', 'measured', 'unit', 'duration_s',
              'rolling_J', 'turning_J', 'kinetic_J', 'auxiliary_J',
              'electrical_J', 'electrical_Wh', 'predicted_electrical_J']


@dataclass
class Segment:
    kind: str     # straight | turn
    target: float  # m or rad, signed

    @classmethod
    def parse(cls, text):
        parts = text.split()
        if len(parts) != 2 or parts[0] not in ('straight', 'turn'):
            raise ValueError(f'bad segment "{text}", expected "straight <m>" or "turn <deg>"')
        value = float(parts[1])
        return cls(parts[0], math.radians(value) if parts[0] == 'turn' else value)


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SegmentRunner(Node):

    def __init__(self):
        super().__init__('segment_runner')
        robot = self.declare_parameter('robot', 'pioneer3at').value
        segments = self.declare_parameter('segments', DEFAULT_SEGMENTS).value
        self.speed_fraction = self.declare_parameter('speed_fraction', DEFAULT_SPEED_FRACTION).value
        self.settle_time = self.declare_parameter('settle_time', DEFAULT_SETTLE_TIME).value
        self.start_delay = self.declare_parameter('start_delay', DEFAULT_START_DELAY).value
        rate = self.declare_parameter('control_rate', DEFAULT_CONTROL_RATE).value
        self.log_path = os.path.expanduser(
            self.declare_parameter('energy_log', 'energy_segments.csv').value)

        self.cfg = load_robot_config(resolve_robot_config(robot))
        self.kin = SkidSteerKinematics.from_config(self.cfg)
        self.model = SkidSteerEnergyModel.from_config(self.cfg)
        self.integrator = EnergyIntegrator(self.model)
        for name in self.cfg.assumptions:
            self.get_logger().warning(f'{self.cfg.name}: {name} is an assumption, not a measurement')

        self.segments = [Segment.parse(s) for s in segments]
        self.rows = []
        self.index = 0
        self.phase = 'wait'
        self.phase_start = None
        self.segment_start = None
        self.done = False
        self.odom = None
        self.start_pose = None
        self.yaw_travelled = 0.0
        self.last_yaw = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', QOS_DEPTH)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, QOS_DEPTH)
        self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(f'Robot {self.cfg.name} from {self.cfg.path}; '
                               f'{len(self.segments)} segments')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        self.odom = msg
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.phase in ('drive', 'settle'):
            self.integrator.add(t, msg.twist.twist.linear.x, msg.twist.twist.angular.z)
            yaw = _yaw(msg.pose.pose.orientation)
            if self.last_yaw is not None:
                self.yaw_travelled += _wrap(yaw - self.last_yaw)
            self.last_yaw = yaw

    def progress(self):
        seg = self.segments[self.index]
        if seg.kind == 'turn':
            return self.yaw_travelled
        p = self.odom.pose.pose.position
        x0, y0, yaw0 = self.start_pose
        # Distance along the start heading, so a backwards target works too.
        return (p.x - x0) * math.cos(yaw0) + (p.y - y0) * math.sin(yaw0)

    def start_segment(self):
        p = self.odom.pose.pose
        self.start_pose = (p.position.x, p.position.y, _yaw(p.orientation))
        self.yaw_travelled = 0.0
        self.last_yaw = None
        self.integrator.reset()
        self.phase = 'drive'
        self.segment_start = self.now()

    def command(self, seg):
        sign = math.copysign(1.0, seg.target)
        if seg.kind == 'straight':
            v, w = sign * self.speed_fraction * self.kin.max_linear_speed, 0.0
        else:
            v, w = 0.0, sign * self.speed_fraction * self.kin.max_angular_speed
        return self.kin.saturate(v, w)

    def braking_distance(self, seg):
        v, w = self.command(seg)
        if seg.kind == 'straight':
            return v ** 2 / (2 * self.cfg.limits.max_linear_acceleration)
        return w ** 2 / (2 * self.cfg.limits.max_angular_acceleration)

    def predicted(self, seg):
        v, w = self.command(seg)
        if seg.kind == 'straight':
            return self.model.straight(abs(seg.target), v).electrical
        return self.model.turn(abs(seg.target), w).electrical

    def step(self):
        if self.odom is None:
            return
        if self.phase == 'wait':
            if self.phase_start is None:
                self.phase_start = self.now()
            if self.now() - self.phase_start >= self.start_delay:
                self.start_segment()
            return
        if self.phase == 'done':
            return

        seg = self.segments[self.index]
        twist = Twist()
        if self.phase == 'drive':
            # Stop commanding once the rest fits in the braking distance at the
            # config's deceleration limit, so the robot stops near the target.
            if abs(self.progress()) + self.braking_distance(seg) >= abs(seg.target):
                self.phase = 'settle'
                self.phase_start = self.now()
            else:
                twist.linear.x, twist.angular.z = self.command(seg)
        elif self.phase == 'settle' and self.now() - self.phase_start >= self.settle_time:
            self.finish_segment(seg)
        self.cmd_pub.publish(twist)

    def finish_segment(self, seg):
        e = self.integrator.energy
        duration = self.now() - self.segment_start
        measured = self.progress()
        unit = 'm' if seg.kind == 'straight' else 'deg'
        if seg.kind == 'turn':
            target, measured = math.degrees(seg.target), math.degrees(measured)
        else:
            target = seg.target
        row = {
            'index': self.index, 'kind': seg.kind, 'target': round(target, CSV_DIGITS),
            'measured': round(measured, CSV_DIGITS), 'unit': unit,
            'duration_s': round(duration, CSV_DIGITS),
            'rolling_J': round(e.rolling, CSV_DIGITS), 'turning_J': round(e.turning, CSV_DIGITS),
            'kinetic_J': round(e.kinetic, CSV_DIGITS), 'auxiliary_J': round(e.auxiliary, CSV_DIGITS),
            'electrical_J': round(e.electrical, CSV_DIGITS),
            'electrical_Wh': round(e.electrical_wh, CSV_DIGITS),
            'predicted_electrical_J': round(self.predicted(seg), CSV_DIGITS),
        }
        self.rows.append(row)
        self.get_logger().info(
            f'Segment {self.index} {seg.kind}: target {row["target"]} {unit}, '
            f'measured {row["measured"]} {unit}, energy {row["electrical_J"]} J '
            f'({row["electrical_Wh"]} Wh), model {row["predicted_electrical_J"]} J')
        self.index += 1
        if self.index < len(self.segments):
            self.start_segment()
        else:
            self.write_log()
            self.phase = 'done'
            self.done = True

    def write_log(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.log_path)), exist_ok=True)
        with open(self.log_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f'Energy per segment written to {self.log_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SegmentRunner()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
