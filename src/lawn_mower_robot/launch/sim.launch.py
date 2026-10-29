"""Start Gazebo with the robot built from a robot config.

    ros2 launch lawn_mower_robot sim.launch.py robot:=pioneer3at
    ros2 launch lawn_mower_robot sim.launch.py robot:=/path/to/my_robot.yaml

`robot` is a config name in share/lawn_mower_robot/config/robot or a path to a
YAML file. `model_path` is the folder that holds the downloaded Gazebo model
folder (e.g. ~/gazebo_models, which contains pioneer3at/), so the config's
model:// mesh URIs resolve.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from lawn_mower_robot.robot_config import load_robot_config, resolve_robot_config
from lawn_mower_robot.sdf_model import build_sdf


def launch_setup(context):
    robot = context.launch_configurations['robot']
    world = context.launch_configurations['world']
    headless = context.launch_configurations['headless'].lower() == 'true'
    model_path = os.path.expanduser(context.launch_configurations['model_path'])

    # Fails here, with every problem listed, if the config is incomplete.
    cfg = load_robot_config(resolve_robot_config(robot))
    sdf_file = os.path.join(tempfile.mkdtemp(prefix='lawn_mower_robot_'), f'{cfg.name}.sdf')
    with open(sdf_file, 'w', encoding='utf-8') as f:
        f.write(build_sdf(cfg))

    actions = []
    if model_path:
        actions.append(AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    gz_args = f'-r {"-s " if headless else ""}{world}'
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-name', cfg.gazebo.model_name, '-file', sdf_file]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=[
                 '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                 '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                 '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/ground_truth/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                 '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
             ]),
    ]
    return actions


def generate_launch_description():
    share = get_package_share_directory('lawn_mower_robot')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='pioneer3at',
                              description='Robot config name or path to a YAML file'),
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'flat.sdf')),
        DeclareLaunchArgument('model_path', default_value='',
                              description='Folder that holds the downloaded Gazebo model folders'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run the Gazebo server without the GUI'),
        OpaqueFunction(function=launch_setup),
    ])
