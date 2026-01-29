"""
Monitor Launcher for MyActuator Arm - Read Only Mode

Only reads position and velocity data from motors. No control commands sent.
Useful for verifying EtherCAT communication and monitoring motor states.

Usage:
    ros2 launch myactuator_arm monitor.launch.py                 # Monitor all 5 joints (default)
    ros2 launch myactuator_arm monitor.launch.py num_joints:=1   # Monitor joint 1 only
    ros2 launch myactuator_arm monitor.launch.py num_joints:=2   # Monitor joints 1-2
    ros2 launch myactuator_arm monitor.launch.py num_joints:=3   # Monitor joints 1-3

Features:
    - EtherCAT driver reads motor states
    - Joint states published to /joint_states topic
    - RViz visualization
    - NO controllers active (read-only)
    - Safe for checking motor positions without risk of commanding them

Monitor joint states:
    ros2 topic echo /joint_states
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('myactuator_arm')

    # Declare launch argument for number of joints
    num_joints_arg = DeclareLaunchArgument(
        'num_joints',
        default_value='5',
        description='Number of joints to monitor (1-5)'
    )

    num_joints = LaunchConfiguration('num_joints')

    urdf_file = os.path.join(pkg_dir, 'description', 'urdf', 'arm_5dof.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers', 'arm_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' pkg_share:=', pkg_dir, ' num_joints:=', num_joints]),
        value_type=str
    )

    return LaunchDescription([
        num_joints_arg,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Controller Manager (loads EtherCAT driver for reading)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_file
            ],
            output='screen'
        ),

        # Joint State Broadcaster ONLY (no control, just publishes /joint_states)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'rviz', 'visualize.rviz')],
            output='screen'
        ),
    ])
