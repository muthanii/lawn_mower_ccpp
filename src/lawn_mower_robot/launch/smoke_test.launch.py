"""Drive a straight line and an in-place turn in Gazebo and log energy per segment.

    ros2 launch lawn_mower_robot smoke_test.launch.py robot:=pioneer3at \
        model_path:=~/gazebo_models energy_log:=~/energy_segments.csv

Everything shuts down when the last segment is logged.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('lawn_mower_robot')
    runner = Node(
        package='lawn_mower_robot', executable='segment_runner', output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot': LaunchConfiguration('robot'),
            'energy_log': LaunchConfiguration('energy_log'),
            'segments': ['straight 2.0', 'turn 90', 'turn -90', 'straight -1.0'],
        }])
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='pioneer3at'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('energy_log', default_value='energy_segments.csv'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sim.launch.py')),
            launch_arguments={
                'robot': LaunchConfiguration('robot'),
                'model_path': LaunchConfiguration('model_path'),
                'headless': LaunchConfiguration('headless'),
            }.items()),
        runner,
        RegisterEventHandler(OnProcessExit(target_action=runner,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
