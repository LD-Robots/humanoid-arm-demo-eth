"""
Position Control Test Launcher - CSP Mode (Mode 8)

Tests position control for MoveIt integration.
Uses trajectory controller to send position commands.

WARNING: In position mode, motors are NOT backdrivable!
         Make sure joints are in safe positions before launching.

Usage:
    ros2 launch myactuator_arm arm_position_test.launch.py

Then use the test script:
    ros2 run myactuator_arm test_position.py --joint 1 --angle 10
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    arm_pkg = get_package_share_directory('myactuator_arm')
    moveit_pkg = get_package_share_directory('myactuator_arm_moveit_new')

    # Use MoveIt config URDF (has mode 8 for position control)
    urdf_file = os.path.join(moveit_pkg, 'config', 'myactuator_arm.urdf.xacro')

    # Use arm controllers config
    controllers_file = os.path.join(arm_pkg, 'config', 'controllers', 'arm_controllers.yaml')

    # Build robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' pkg_share:=', arm_pkg]),
        value_type=str
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Controller Manager (loads EtherCAT driver)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_file
            ],
            output='screen'
        ),

        # Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Trajectory Controller (ACTIVE for position control testing)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(arm_pkg, 'config', 'rviz', 'visualize.rviz')],
            output='screen'
        ),
    ])
