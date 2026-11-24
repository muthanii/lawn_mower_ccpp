"""Draw the planned path and the mowed swath into Gazebo, for screenshots.

    python3 docs/tools/gazebo_overlay.py      # after coverage_sim has spawned the robot

The planned path (/mission/path) becomes thin orange strips. While the blade
runs (/blade/running), the ground-truth path widened by the cutting width
becomes green strips, added every 3 s. Both are static models with visuals
only, no collision, so they do not change the physics of the run. Set
GZ_WORLD if the world is not called "lawn".

The GUI marker service (/marker) did not answer under software rendering, so
the strips are spawned through the world's create service instead.
"""
import math
import os
import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool

HALF_CUT = 0.25  # m, half the cutting width of faculty_robot_estimate
WORLD = os.environ.get('GZ_WORLD', 'lawn')


def box_visual(name, a, b, width, height, z, rgba):
    (x0, y0), (x1, y1) = a, b
    length = math.hypot(x1 - x0, y1 - y0)
    if length < 1e-3:
        return ''
    yaw = math.atan2(y1 - y0, x1 - x0)
    col = ' '.join(map(str, rgba))
    return (f'<visual name="{name}"><pose>{(x0 + x1) / 2} {(y0 + y1) / 2} {z} 0 0 {yaw}</pose>'
            f'<geometry><box><size>{length + width * 0.02} {width} {height}</size></box></geometry>'
            f'<material><ambient>{col}</ambient><diffuse>{col}</diffuse></material>'
            f'<cast_shadows>false</cast_shadows></visual>')


def spawn(name, visuals):
    sdf = (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>'
           f'<link name="link">{"".join(visuals)}</link></model></sdf>')
    fd, path = tempfile.mkstemp(suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(sdf)
    subprocess.run(['gz', 'service', '-s', f'/world/{WORLD}/create', '--reqtype',
                    'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean', '--timeout', '5000',
                    '--req', f'sdf_filename: "{path}", name: "{name}"'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class Overlay(Node):
    def __init__(self):
        super().__init__('screenshot_overlay')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Path, '/mission/path', self.on_path, latched)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
        self.create_subscription(Bool, '/blade/running', self.on_blade, 10)
        self.blade = False
        self.pending, self.last, self.batch = [], None, 0
        self.create_timer(3.0, self.flush)
        self.drawn_path = False

    def on_blade(self, m):
        self.blade = m.data
        if not m.data:
            self.last = None

    def on_path(self, m):
        if self.drawn_path:
            return
        xy = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        vis = [box_visual(f'p{i}', a, b, 0.05, 0.01, 0.045, (0.95, 0.42, 0.1, 1))
               for i, (a, b) in enumerate(zip(xy, xy[1:]))]
        spawn('planned_path', vis)
        self.drawn_path = True
        self.get_logger().info(f'path with {len(xy)} poses drawn')

    def on_odom(self, m):
        if not self.blade:
            return
        p = m.pose.pose.position
        if self.last is None:
            self.last = (p.x, p.y)
            return
        if math.hypot(p.x - self.last[0], p.y - self.last[1]) < 0.2:
            return
        self.pending.append((self.last, (p.x, p.y)))
        self.last = (p.x, p.y)

    def flush(self):
        if not self.pending:
            return
        vis = [box_visual(f's{i}', a, b, 2 * HALF_CUT, 0.01, 0.025, (0.62, 0.88, 0.42, 1))
               for i, (a, b) in enumerate(self.pending)]
        self.pending = []
        spawn(f'mowed_{self.batch}', vis)
        self.batch += 1


def main():
    rclpy.init()
    rclpy.spin(Overlay())


if __name__ == '__main__':
    main()
