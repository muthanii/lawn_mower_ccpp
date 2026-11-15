"""Mission node: plans the coverage of a lawn and drives it.

    ros2 run lawn_mower_navigation mission --ros-args -p lawn:=demo -p method:=energy_ga

It plans once at start, publishes the path, and waits until the controller is
in automatic mode and the pose is known. Then it sends velocity commands and
the blade request to the controller. When the operator takes over, or the
controller reports a failsafe or an emergency stop, the mission pauses, and it
goes on from where the robot is when automatic mode returns.
"""

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, String

from lawn_mower_electronics import topics as hw
from lawn_mower_electronics.controller import AUTO
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config

from . import topics
from .lawns import find_lawn
from .plan import DEFAULT_BLADE_OFF_MOVE, DEFAULT_METHOD, default_clearance, plan_mission
from .ros_util import QOS_DEPTH, latched, yaw_of
from .tracker import PathTracker, TrackerParams

DEFAULT_SPEED_FRACTION = 0.5   # of the robot's speed limits
DEFAULT_RATE = 20.0            # Hz
DECELERATION_FRACTION = 0.6    # of the robot's acceleration limits, so the ramps are reachable
NO_BUDGET = 0

PLANNED, WAITING, RUNNING, PAUSED, DONE = 'PLANNED', 'WAITING', 'RUNNING', 'PAUSED', 'DONE'


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission')
        robot = load_robot_config(resolve_robot_config(
            self.declare_parameter('robot', 'faculty_robot_estimate').value))
        lawn = find_lawn(self.declare_parameter('lawn', 'demo').value)
        method = self.declare_parameter('method', DEFAULT_METHOD).value
        seed = self.declare_parameter('seed', 0).value
        budget = self.declare_parameter('budget', NO_BUDGET).value
        fraction = self.declare_parameter('speed_fraction', DEFAULT_SPEED_FRACTION).value
        clearance = self.declare_parameter('clearance', default_clearance(robot)).value
        blade_off_move = self.declare_parameter('blade_off_move', DEFAULT_BLADE_OFF_MOVE).value
        rate = self.declare_parameter('rate', DEFAULT_RATE).value
        for name in robot.assumptions:
            self.get_logger().warning(f'{robot.name}: {name} is an assumption, not a measurement',
                                      once=True)

        lim = robot.limits
        params = TrackerParams(
            cruise_speed=fraction * lim.max_linear_speed,
            turn_rate=fraction * lim.max_angular_speed,
            linear_deceleration=DECELERATION_FRACTION * lim.max_linear_acceleration,
            angular_deceleration=DECELERATION_FRACTION * lim.max_angular_acceleration)
        self.get_logger().info(f'Planning {lawn.name} with {method} for {robot.name} ...')
        self.mission = plan_mission(lawn, robot, method, seed, params.cruise_speed,
                                    params.turn_rate, clearance, budget or None,
                                    blade_off_move=blade_off_move)
        m, predicted = self.mission, self.mission.predicted
        self.get_logger().info(
            f'Plan: {len(m.plan.order)} cells, {len(m.segments)} segments, '
            f'{predicted["length"]:.1f} m, {predicted["turns"]:.0f} turns, '
            f'{predicted.get("energy", 0.0) / 3600:.2f} Wh traction, '
            f'planned coverage {m.planned_coverage["coverage"]:.1f} %, '
            f'clearance {m.clearance:.2f} m, planned in {m.planning_seconds:.1f} s')

        self.tracker = PathTracker(m.segments, params)
        self.pose = None
        self.state = None
        self.blade_running = False
        self.status = PLANNED
        self.cmd_pub = self.create_publisher(Twist, hw.CMD_VEL, QOS_DEPTH)
        self.blade_pub = self.create_publisher(Bool, hw.BLADE_REQUEST, QOS_DEPTH)
        self.status_pub = self.create_publisher(String, topics.MISSION_STATUS, QOS_DEPTH)
        self.path_pub = self.create_publisher(Path, topics.MISSION_PATH, latched())
        self.path_pub.publish(self.path_message())
        self.create_subscription(Odometry, topics.POSE, self.on_pose, QOS_DEPTH)
        self.create_subscription(String, hw.CONTROLLER_STATE, self.on_state, QOS_DEPTH)
        self.create_subscription(Bool, hw.BLADE_RUNNING, self.on_blade, QOS_DEPTH)
        self.create_timer(1.0 / rate, self.step)

    def path_message(self):
        path = Path()
        path.header.frame_id = topics.FRAME
        for x, y in self.mission.points:
            pose = PoseStamped()
            pose.header.frame_id = topics.FRAME
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            path.poses.append(pose)
        return path

    def on_pose(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def on_state(self, msg):
        self.state = msg.data

    def on_blade(self, msg):
        self.blade_running = msg.data

    def set_status(self, status):
        if status != self.status:
            self.get_logger().info(f'Mission {status}')
            self.status = status

    def step(self):
        self.status_pub.publish(String(data=self.status))
        if self.status == DONE:
            self.blade_pub.publish(Bool(data=False))
            self.cmd_pub.publish(Twist())
            return
        if self.pose is None or self.state != AUTO:
            # Not ours to drive: send nothing, and turn onto the path again afterwards.
            self.tracker.hold()
            self.blade_pub.publish(Bool(data=False))
            self.set_status(WAITING if self.status in (PLANNED, WAITING) else PAUSED)
            return
        cmd = self.tracker.update(self.pose, self.blade_running)
        twist = Twist()
        twist.linear.x, twist.angular.z = cmd.v, cmd.omega
        self.cmd_pub.publish(twist)
        self.blade_pub.publish(Bool(data=cmd.blade))
        self.set_status(DONE if cmd.done else RUNNING)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
