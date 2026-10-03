"""Static TF base_link -> laser (RPLIDAR mount position on the robot).

This is the ONLY place the LiDAR mount offsets are defined; amr_bringup.sh starts this file.
  x = 0.175 m (forward of base_link), y = 0, z = 0.100 m, no rotation.

Usage:
  ros2 launch amr_navigation laser_tf.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.175', '0', '0.100',
            '0', '0', '0',
            'base_link', 'laser'
        ]
    )

    return LaunchDescription([
        static_tf
    ])
