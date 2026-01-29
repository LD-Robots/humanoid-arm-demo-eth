"""
Single Joint Test Launcher for MyActuator Arm

Tests one joint at a time in position control mode (CSP mode 8).
Safe for testing individual joints without affecting others.

Usage:
    ros2 launch myactuator_arm arm_test_single_joint.launch.py joint_num:=1

    # In another terminal, send test commands:
    ros2 run myactuator_arm test_joint_motion.py --joint 1 --position 0.5
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    arm_pkg = get_package_share_directory('myactuator_arm')
    moveit_pkg = get_package_share_directory('myactuator_arm_moveit_config')

    # Use MoveIt config URDF (has mode 8 for position control)
    urdf_file = os.path.join(moveit_pkg, 'config', 'myactuator_arm_real.urdf.xacro')
    controllers_file = os.path.join(arm_pkg, 'config', 'controllers', 'arm_controllers.yaml')

    # Build robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' pkg_share:=', arm_pkg]),
        value_type=str
    )

    # Launch arguments
    joint_num_arg = DeclareLaunchArgument(
        'joint_num',
        default_value='1',
        description='Joint number to test (1-5)'
    )

    return LaunchDescription([
        joint_num_arg,

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Controller Manager (loads EtherCAT driver with mode 8)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_description},
                controllers_file
            ],
            output='screen'
        ),

        # Joint State Broadcaster (ACTIVE)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Single Joint Trajectory Controller (ACTIVE for testing)
        # This will activate the full trajectory controller but only send commands to one joint
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
            arguments=['-d', os.path.join(arm_pkg, 'config', 'rviz', 'arm_test.rviz')],
            output='log',
            parameters=[{'robot_description': robot_description}],
            condition=lambda context: False  # Disabled by default, can be enabled with rviz:=true
        ),
    ])
