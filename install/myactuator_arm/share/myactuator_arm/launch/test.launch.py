"""
Test Launcher for MyActuator Arm - No Virtual Wall Protection

Allows testing motors one by one without safety limits.
Use with CAUTION - no virtual wall protection active.

Usage:
    ros2 launch myactuator_arm test.launch.py num_joints:=1    # Test joint 1 only
    ros2 launch myactuator_arm test.launch.py num_joints:=2    # Test joints 1-2
    ros2 launch myactuator_arm test.launch.py num_joints:=3    # Test joints 1-3
    ros2 launch myactuator_arm test.launch.py num_joints:=4    # Test joints 1-4
    ros2 launch myactuator_arm test.launch.py num_joints:=5    # Test all 5 joints

WARNING: No virtual wall protection. Motors can move beyond safe limits.
         Only use for initial testing and configuration.
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
        default_value='1',
        description='Number of joints to activate (1-5)'
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

        # Effort Controller (starts ACTIVE - sends 0.0 torque by default)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['effort_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Trajectory Controller (starts INACTIVE)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager', '--inactive'],
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
