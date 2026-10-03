"""NAVIGATION WHILE MAPPING: SLAM Toolbox + Nav2 (no saved map, no AMCL).

Easiest way to test Nav2 first: SLAM provides /map and map->odom, Nav2 plans on
the map as it is being built. Nav2's final velocity goes out on /cmd_vel,
which your diff drive controller already listens to.

Usage:
  ros2 launch amr_navigation nav_slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('amr_navigation')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file')
    slam_params = LaunchConfiguration('slam_params_file')
    use_composition = LaunchConfiguration('use_composition')
    nav2_delay = LaunchConfiguration('nav2_start_delay')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': use_composition,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml'),
            description='Nav2 parameter file'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg, 'config', 'slam_toolbox_params.yaml'),
            description='SLAM Toolbox parameter file'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Run Nav2 servers in one process (saves RAM, harder to debug)'),
        DeclareLaunchArgument(
            'nav2_start_delay', default_value='5.0',
            description='Seconds to wait after SLAM starts before starting Nav2'),

        slam,
        TimerAction(period=nav2_delay, actions=[nav2]),
    ])
