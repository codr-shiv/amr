from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    pkg_dir = get_package_share_directory('amr_slam')

    config_file = os.path.join(
        pkg_dir,
        'config',
        'mapper_params_real.yaml'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        slam_toolbox_node
    ])
