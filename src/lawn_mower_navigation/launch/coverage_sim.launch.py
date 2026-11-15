"""Simulate an autonomous coverage mission in Gazebo, electronics included.

    ros2 launch lawn_mower_navigation coverage_sim.launch.py
    ros2 launch lawn_mower_navigation coverage_sim.launch.py lawn:=two_beds headless:=true

Starts Gazebo with the world of the lawn and the robot of the config, the
electronics models (radio, controller, motor driver, blade, battery), the
localisation, the mission and the evaluator. A scripted operator switches the
radio to automatic mode `auto_after` seconds in. The launch ends when the
evaluator has written the results.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from lawn_mower_electronics import topics as hw
from lawn_mower_electronics.config import load_electronics_config, resolve_electronics_config
from lawn_mower_electronics.sensors import CONTACT_TOPIC, GNSS_TOPIC, IMU_TOPIC, sensors_sdf
from lawn_mower_navigation.lawns import find_lawn
from lawn_mower_navigation.world import DEFAULT_LATITUDE, DEFAULT_LONGITUDE, lawn_world_sdf
from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
from lawn_mower_robot.sdf_model import build_sdf

ARGUMENTS = (
    ('robot', 'faculty_robot_estimate', 'Robot config name or path'),
    ('electronics', 'faculty_robot', 'Electronics config name or path'),
    ('lawn', 'demo', 'Lawn name: demo, or one of lawn_mower_planning.maps'),
    ('method', 'energy_ga', 'Cell ordering method of lawn_mower_planning'),
    ('seed', '0', 'Seed of the ordering method'),
    ('budget', '0', 'Evaluated orders of the ordering method, 0 for its default'),
    ('speed_fraction', '0.5', 'Driving speed as a fraction of the robot limits'),
    ('speed_source', 'pulses', 'Speed for localisation: pulses or odometry'),
    ('auto_after', '3.0', 'Seconds before the operator switches to automatic mode'),
    ('clearance', '-1', 'Distance the robot centre keeps from edges and obstacles [m]; '
                        '-1 for the radius the body sweeps in a turn plus a margin'),
    ('real_time_factor', '1', 'Simulation speed. Above about 3 the message delays, which '
                              'stay the same in real time, start to disturb the control'),
    ('headless', 'false', 'Run the Gazebo server without the GUI'),
    ('results_dir', 'mission_results', 'Folder for mission.csv and trajectory.csv'),
)


def launch_setup(context):
    arg = context.launch_configurations
    cfg = load_robot_config(resolve_robot_config(arg['robot']))
    electronics = load_electronics_config(resolve_electronics_config(arg['electronics']))
    lawn = find_lawn(arg['lawn'])
    x, y, heading = lawn.start

    folder = tempfile.mkdtemp(prefix='lawn_mower_sim_')
    world_file = os.path.join(folder, f'{lawn.name}.sdf')
    with open(world_file, 'w', encoding='utf-8') as f:
        f.write(lawn_world_sdf(lawn.area, real_time_factor=float(arg['real_time_factor'])))
    model_file = os.path.join(folder, f'{cfg.name}.sdf')
    with open(model_file, 'w', encoding='utf-8') as f:
        f.write(build_sdf(cfg, sensors_sdf(electronics), hw.MOTOR_CMD_VEL))

    common = {'use_sim_time': True, 'robot': arg['robot']}
    both = dict(common, electronics=arg['electronics'])
    run = dict(common, lawn=arg['lawn'], method=arg['method'], seed=int(arg['seed']))
    headless = arg['headless'].lower() == 'true'
    evaluator = Node(package='lawn_mower_navigation', executable='evaluator', output='screen',
                     parameters=[dict(run, results_dir=arg['results_dir'])])
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {"-s " if headless else ""}{world_file}',
                              'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', cfg.gazebo.model_name, '-file', model_file,
                        '-x', str(x), '-y', str(y), '-Y', str(heading)]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 f'{hw.MOTOR_CMD_VEL}@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 f'{hw.GROUND_TRUTH}@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 f'{IMU_TOPIC}@sensor_msgs/msg/Imu[gz.msgs.IMU',
                 f'{GNSS_TOPIC}@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
                 f'{CONTACT_TOPIC}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
             ]),
        Node(package='lawn_mower_electronics', executable='electronics', output='screen',
             parameters=[both]),
        Node(package='lawn_mower_electronics', executable='rc_transmitter', output='screen',
             parameters=[dict(both, auto_after=float(arg['auto_after']))]),
        Node(package='lawn_mower_navigation', executable='localization', output='screen',
             parameters=[dict(both, initial_heading=float(heading),
                              speed_source=arg['speed_source'],
                              origin_latitude=DEFAULT_LATITUDE,
                              origin_longitude=DEFAULT_LONGITUDE)]),
        Node(package='lawn_mower_navigation', executable='mission', output='screen',
             parameters=[dict(run, budget=int(arg['budget']),
                              speed_fraction=float(arg['speed_fraction']),
                              **({'clearance': float(arg['clearance'])}
                                 if float(arg['clearance']) >= 0 else {}))]),
        evaluator,
        RegisterEventHandler(OnProcessExit(target_action=evaluator,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default, description=text)
         for name, default, text in ARGUMENTS] + [OpaqueFunction(function=launch_setup)])
