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
