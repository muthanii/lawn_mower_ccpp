"""Evaluator node, for the simulation: scores the mission from Gazebo's ground truth.

It records the true pose and whether the blade is cutting. When the mission
reports DONE it writes one row of results and the driven trajectory, and
exits, which ends the launch.

    <results_dir>/mission.csv      one row per run (appended)
    <results_dir>/trajectory.csv   t, x, y, heading, cutting of the last run
"""

import csv
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float64MultiArray, String

from lawn_mower_electronics import topics as hw
from lawn_mower_robot.energy import JOULES_PER_WH
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config

from . import topics
from .evaluation import (Sample, coverage, footprint_excursion, path_length, thin,
                         tracking_error)
from .lawns import find_lawn
from .mission_node import DONE
from .ros_util import QOS_DEPTH, latched, stamp_seconds, yaw_of

SETTLE_TIME = 2.0   # s after DONE, so the robot is at rest and the blade has stopped
DIGITS = 4
FIELDS = ['lawn', 'robot', 'method', 'seed', 'duration_s', 'distance_m', 'coverage_pct',
          'overlap_pct', 'mowed_area_m2', 'lawn_area_m2', 'footprint_excursion_m',
          'tracking_mean_m', 'tracking_max_m', 'localization_mean_m', 'localization_max_m',
          'traction_wh', 'blade_wh', 'auxiliary_wh', 'total_wh', 'battery_pct']


class EvaluatorNode(Node):

    def __init__(self):
        super().__init__('evaluator')
        self.robot = load_robot_config(resolve_robot_config(
            self.declare_parameter('robot', 'faculty_robot_estimate').value))
        self.lawn = find_lawn(self.declare_parameter('lawn', 'demo').value)
        self.method = self.declare_parameter('method', '').value
        self.seed = self.declare_parameter('seed', 0).value
        self.results_dir = os.path.expanduser(
            self.declare_parameter('results_dir', 'mission_results').value)

        self.samples = []
        self.cutting = False
        self.path = []
        self.energy = [0.0, 0.0, 0.0]
        self.battery = None
        self.truth = None
        self.localization_errors = []
        self.done_at = None
        self.finished = False
        self.row = None
        self.create_subscription(Odometry, hw.GROUND_TRUTH, self.on_truth, QOS_DEPTH)
        self.create_subscription(Odometry, topics.POSE, self.on_estimate, QOS_DEPTH)
        self.create_subscription(Bool, hw.BLADE_RUNNING, self.on_blade, QOS_DEPTH)
        self.create_subscription(Float64MultiArray, hw.ENERGY, self.on_energy, QOS_DEPTH)
        self.create_subscription(BatteryState, hw.BATTERY, self.on_battery, QOS_DEPTH)
        self.create_subscription(Path, topics.MISSION_PATH, self.on_path, latched())
        self.create_subscription(String, topics.MISSION_STATUS, self.on_status, QOS_DEPTH)

    def on_truth(self, msg):
        p = msg.pose.pose
        self.truth = (p.position.x, p.position.y)
        t = stamp_seconds(msg.header.stamp)
        self.samples.append(Sample(t, p.position.x, p.position.y, yaw_of(p.orientation),
                                   self.cutting))
        if self.done_at is not None and t - self.done_at >= SETTLE_TIME and not self.finished:
            self.finish()

    def on_estimate(self, msg):
        if self.truth is not None and self.done_at is None:
            p = msg.pose.pose.position
            self.localization_errors.append(math.hypot(p.x - self.truth[0], p.y - self.truth[1]))

    def on_blade(self, msg):
        self.cutting = msg.data

    def on_energy(self, msg):
        self.energy = list(msg.data)

    def on_battery(self, msg):
        self.battery = msg.percentage

    def on_path(self, msg):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def on_status(self, msg):
        if msg.data == DONE and self.done_at is None and self.samples:
            self.done_at = self.samples[-1].t

    def finish(self):
        self.finished = True
        samples = thin(self.samples)
        g = self.robot.geometry
        result = coverage(samples, self.lawn.area, self.robot.mower.cutting_width)
        mean, worst = tracking_error(samples, self.path)
        errors = self.localization_errors or [0.0]
        traction, blade, auxiliary = (e / JOULES_PER_WH for e in self.energy)
        row = {
            'lawn': self.lawn.name, 'robot': self.robot.name, 'method': self.method,
            'seed': self.seed,
            'duration_s': self.done_at - samples[0].t,
            'distance_m': path_length(samples),
            'coverage_pct': result['coverage'], 'overlap_pct': result['overlap'],
            'mowed_area_m2': result['mowed_area'], 'lawn_area_m2': self.lawn.area.area,
            'footprint_excursion_m': footprint_excursion(samples, self.lawn.area,
                                                         g.body_length, g.body_width),
            'tracking_mean_m': mean, 'tracking_max_m': worst,
            'localization_mean_m': sum(errors) / len(errors), 'localization_max_m': max(errors),
            'traction_wh': traction, 'blade_wh': blade, 'auxiliary_wh': auxiliary,
            'total_wh': traction + blade + auxiliary,
            'battery_pct': 100 * self.battery if self.battery is not None else '',
        }
        self.row = {k: round(v, DIGITS) if isinstance(v, float) else v for k, v in row.items()}
        os.makedirs(self.results_dir, exist_ok=True)
        path = os.path.join(self.results_dir, 'mission.csv')
        new = not os.path.exists(path)
        with open(path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, FIELDS)
            if new:
                writer.writeheader()
            writer.writerow(self.row)
        with open(os.path.join(self.results_dir, 'trajectory.csv'), 'w', newline='',
                  encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['t', 'x', 'y', 'heading', 'cutting'])
            writer.writerows([round(s.t, DIGITS), round(s.x, DIGITS), round(s.y, DIGITS),
                              round(s.heading, DIGITS), int(s.cutting)] for s in samples)
        self.get_logger().info('Mission results: ' + ', '.join(
            f'{k} {v}' for k, v in self.row.items()))
        self.get_logger().info(f'Written to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
