"""Play the operator during a simulated mission and check how the robot reacts.

Start the simulation, then this node:

    ros2 launch lawn_mower_navigation coverage_sim.launch.py headless:=true
    ros2 run lawn_mower_navigation safety_check --ros-args -p use_sim_time:=true

It waits until the robot mows a lane, then takes over by radio, switches the
transmitter off, and presses the emergency stop. After each step it prints the
controller state, the mission status, the true speed and the blade speed, and
compares them with what must happen. It exits with 1 if a step fails.

The bumper scenario drives the robot into the flower bed of the demo lawn by
hand, and checks that automatic mode refuses to start against the obstacle:

    ros2 launch lawn_mower_navigation coverage_sim.launch.py headless:=true auto_after:=-1.0
    ros2 run lawn_mower_navigation safety_check --ros-args -p use_sim_time:=true -p scenario:=bumper
"""

import math
import sys
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Float64, String

from lawn_mower_electronics import topics as hw
from lawn_mower_electronics.controller import AUTO, ESTOP, FAILSAFE, MANUAL

from . import topics
from .lawns import DEMO
from .mission_node import PAUSED, RUNNING, WAITING
from .ros_util import QOS_DEPTH, yaw_of

TRANSMITTER = '/rc_transmitter'
MOVING = 0.2          # m/s, faster than this is driving
STILL = 0.02          # m/s and rad/s, slower than this is stopped
BLADE_STOPPED = 600.0  # rpm, below this the blade has stopped
BLADE_CUTTING = 5000.0  # rpm
POLL = 0.02           # s, real time
START_TIMEOUT = 120.0  # s, real time, to wait for the mission
SCENARIOS = ('operator', 'bumper')
TURN_STICK = 0.12     # steer stick while the scripted operator turns the robot
TURN_TRIES = 6
AIM_TOLERANCE = math.radians(4)
DRIVE_STICK = 0.5     # throttle stick while it drives
DRIVE_TIMEOUT = 40.0  # s, simulated, to reach the obstacle


class SafetyCheck(Node):

    def __init__(self):
        super().__init__('safety_check')
        self.state = self.mission = None
        self.speed = self.yaw_rate = self.blade = 0.0
        self.pose = None
        self.bumper = False
        self.failures = self.steps = 0
        self.scenario = self.declare_parameter('scenario', SCENARIOS[0]).value
        if self.scenario not in SCENARIOS:
            raise ValueError(f'scenario must be one of {SCENARIOS}')
        self.create_subscription(Bool, hw.BUMPER,
                                 lambda m: setattr(self, 'bumper', m.data), QOS_DEPTH)
        self.create_subscription(String, hw.CONTROLLER_STATE,
                                 lambda m: setattr(self, 'state', m.data), QOS_DEPTH)
        self.create_subscription(String, topics.MISSION_STATUS,
                                 lambda m: setattr(self, 'mission', m.data), QOS_DEPTH)
        self.create_subscription(Float64, hw.BLADE_SPEED,
                                 lambda m: setattr(self, 'blade', m.data), QOS_DEPTH)
        self.create_subscription(Odometry, hw.GROUND_TRUTH, self.on_truth, QOS_DEPTH)
        self.estop_pub = self.create_publisher(Bool, hw.ESTOP, QOS_DEPTH)
        self.params = self.create_client(SetParameters, f'{TRANSMITTER}/set_parameters')

    def on_truth(self, msg):
        self.speed = abs(msg.twist.twist.linear.x)
        self.yaw_rate = abs(msg.twist.twist.angular.z)
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def wait(self, seconds):
        """Sleep for simulated seconds; the node spins in another thread."""
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            time.sleep(POLL)

    def transmitter(self, **values):
        """Move the switches and sticks of the scripted transmitter."""
        request = SetParameters.Request(parameters=[
            Parameter(name, value=value).to_parameter_msg() for name, value in values.items()])
        self.params.wait_for_service()
        future = self.params.call_async(request)
        while not future.done():
            time.sleep(POLL)

    def estop(self, pressed, seconds):
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            self.estop_pub.publish(Bool(data=pressed))
            time.sleep(POLL)

    def check(self, step, state, mission, moving=None, blade_stopped=None, bumper=None):
        ok = self.state == state and self.mission == mission
        if bumper is not None:
            ok = ok and self.bumper == bumper
        self.steps += 1
        if moving is True:
            ok = ok and self.speed > MOVING
        if moving is False:
            ok = ok and self.speed < STILL and self.yaw_rate < STILL
        if blade_stopped is True:
            ok = ok and self.blade < BLADE_STOPPED
        if blade_stopped is False:
            ok = ok and self.blade > BLADE_CUTTING
        self.failures += not ok
        bump = ' bumper pressed' if self.bumper else ''
        print(f'{"ok  " if ok else "FAIL"} {step:44s} controller {self.state:8s} '
              f'mission {self.mission:7s} speed {self.speed:.2f} m/s '
              f'blade {self.blade:4.0f} rpm{bump}', flush=True)

    def heading_error(self, bearing):
        return math.atan2(math.sin(bearing - self.pose[2]), math.cos(bearing - self.pose[2]))

    def turn_to(self, bearing):
        """Turn in place by the sticks until the robot faces the bearing [rad]."""
        for _ in range(TURN_TRIES):
            error = self.heading_error(bearing)
            if abs(error) < AIM_TOLERANCE:
                return
            self.transmitter(steer=math.copysign(TURN_STICK, error))
            # Let go early: the robot turns on while the drive slows down.
            while abs(self.heading_error(bearing)) > 2 * AIM_TOLERANCE \
                    and self.heading_error(bearing) * error > 0:
                time.sleep(POLL)
            self.transmitter(steer=0.0)
            self.wait(2.0)

    def report(self):
        print(f'{self.failures} of {self.steps} steps failed' if self.failures
              else f'All {self.steps} steps passed')
        return 1 if self.failures else 0

    def run(self):
        return self.run_bumper() if self.scenario == 'bumper' else self.run_operator()

    def run_bumper(self):
        """Drive into the flower bed of the demo lawn in manual mode."""
        deadline = time.monotonic() + START_TIMEOUT
        while self.state != MANUAL or self.mission != WAITING or self.pose is None:
            if time.monotonic() > deadline:
                print('FAIL the controller is not in manual mode; launch with auto_after:=-1.0')
                return 1
            time.sleep(POLL)
        bed = DEMO.area.interiors[0].centroid
        bearing = math.atan2(bed.y - self.pose[1], bed.x - self.pose[0])
        self.turn_to(bearing)
        self.transmitter(throttle=DRIVE_STICK)
        end = self.now() + DRIVE_TIMEOUT
        while not self.bumper and self.now() < end:
            time.sleep(POLL)
        self.wait(2.0)
        bed_box = DEMO.area.interiors[0].bounds
        outside = not (bed_box[0] < self.pose[0] < bed_box[2]
                       and bed_box[1] < self.pose[1] < bed_box[3])
        self.check('driven into the flower bed by hand', MANUAL, WAITING, moving=False,
                   bumper=True)
        self.steps += 1
        self.failures += not outside
        print(f'{"ok  " if outside else "FAIL"} the obstacle stops the robot: centre at '
              f'({self.pose[0]:.2f}, {self.pose[1]:.2f}), bed from x {bed_box[0]:g} '
              f'to {bed_box[2]:g}, y {bed_box[1]:g} to {bed_box[3]:g}', flush=True)
        self.transmitter(throttle=0.0)
        self.wait(1.0)
        self.transmitter(mode='auto')
        self.wait(3.0)
        self.check('automatic mode selected against the obstacle', ESTOP, WAITING, moving=False,
                   bumper=True)
        self.transmitter(mode='manual', throttle=-DRIVE_STICK)
        self.wait(5.0)
        self.check('operator reverses off in manual', MANUAL, WAITING, moving=True, bumper=False)
        self.transmitter(throttle=0.0)
        self.wait(2.0)
        self.transmitter(mode='auto', blade=True)
        self.wait(8.0)
        self.check('automatic mode again: back to the path', AUTO, RUNNING, bumper=False)
        return self.report()

    def run_operator(self):
        deadline = time.monotonic() + START_TIMEOUT
        while self.mission != RUNNING or self.blade < BLADE_CUTTING or self.speed < MOVING:
            if time.monotonic() > deadline:
                print('FAIL the mission did not start')
                return 1
            time.sleep(POLL)
        self.check('mowing in automatic mode', AUTO, RUNNING, moving=True, blade_stopped=False)

        self.transmitter(mode='manual')
        self.wait(4.0)
        self.check('operator switches to manual, sticks centred', MANUAL, PAUSED, moving=False)
        self.transmitter(throttle=0.5)
        self.wait(3.0)
        self.check('operator pushes the throttle', MANUAL, PAUSED, moving=True)
        self.transmitter(throttle=0.0)
        self.wait(3.0)
        self.transmitter(mode='auto')
        self.wait(12.0)
        self.check('operator switches back to automatic', AUTO, RUNNING, blade_stopped=False)

        self.transmitter(link=False)
        self.wait(5.0)
        self.check('transmitter off: radio link lost', FAILSAFE, PAUSED, moving=False,
                   blade_stopped=True)
        self.transmitter(link=True)
        self.wait(12.0)
        self.check('transmitter back on', AUTO, RUNNING, blade_stopped=False)

        self.estop(True, 5.0)
        self.check('emergency stop pressed', ESTOP, PAUSED, moving=False, blade_stopped=True)
        self.estop(False, 4.0)
        self.check('emergency stop released: still latched', ESTOP, PAUSED, moving=False,
                   blade_stopped=True)
        self.transmitter(mode='manual')
        self.wait(2.0)
        self.check('operator re-arms by switching to manual', MANUAL, PAUSED, moving=False)
        self.transmitter(mode='auto')
        self.wait(12.0)
        self.check('operator switches to automatic again', AUTO, RUNNING, blade_stopped=False)
        return self.report()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCheck()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 1
    executor.shutdown()
    spinner.join()
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
