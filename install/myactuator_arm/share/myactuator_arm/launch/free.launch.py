"""
Free Mode Launcher for MyActuator Arm

Starts motors in CST (torque control) mode with zero torque,
allowing free manual movement while visualizing in RViz.

Usage:
    ros2 launch myactuator_arm free.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('myactuator_arm')

    urdf_file = os.path.join(pkg_dir, 'description', 'urdf', 'arm.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers', 'arm_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' pkg_share:=', pkg_dir]),
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

        # Effort Controller (starts ACTIVE - sends 0.0 torque by default)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['effort_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Trajectory Controller (starts INACTIVE - available if needed)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager', '--inactive'],
            output='screen'
        ),

        # Virtual Wall Safety Node (per-joint asymmetric limits)
        Node(
            package='myactuator_arm',
            executable='virtual_wall.py',
            name='virtual_wall',
            parameters=[{
                'joints': ['joint1', 'joint2'],
                'lower_limits': [-0.75, -2.98],
                'upper_limits': [3.14, 0.2],
                'soft_limit_margin': 0.2618,
                'wall_stiffness': 10.0,
                'wall_damping': 5.0,
                'max_wall_torque': 3.0,
            }],
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
