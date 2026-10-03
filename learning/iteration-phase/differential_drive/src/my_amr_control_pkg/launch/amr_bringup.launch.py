import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('my_amr_control_pkg')
    
    # Parse the URDF file
    xacro_file = os.path.join(pkg_path, 'urdf', 'amr_robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}
    
    # Path to the controllers.yaml file we created earlier
    controller_config = os.path.join(pkg_path, 'config', 'controllers.yaml')

    # Start the core controller manager node
    # Start the core controller manager node with remapped topics
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_config],
        remappings=[
            ("/diff_drive_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        output="both",
    )

    # Spawner for the Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner for the Differential Drive Controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        diff_drive_spawner
    ])