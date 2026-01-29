"""
Simple RViz Visualizer for MyActuator X6

Shows robot in RViz with effort control and virtual wall safety.
Motor is backdrivable and safe to move by hand.

Usage:
    ros2 launch myactuator_x6_test x6_visualize.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('myactuator_x6_test')

    # Use position mode URDF (supports both position and effort control)
    urdf_file = os.path.join(pkg_dir, 'urdf', 'x6_test.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers_dual.yaml')

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

        # Effort Controller (safe, backdrivable) - starts ACTIVE
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['effort_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Position Controller (for trajectories/MoveIt) - starts INACTIVE
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager', '--inactive'],
            output='screen'
        ),

        # Virtual Wall Safety Node
        Node(
            package='myactuator_x6_test',
            executable='virtual_wall.py',
            name='virtual_wall',
            parameters=[{
                'position_limit': 1.5708,       # 90 degrees
                'soft_limit_margin': 0.2618,    # 15 degrees margin
                'wall_stiffness': 10.0,         # Nm/rad
                'wall_damping': 5.0,            # Nm/(rad/s)
                'max_wall_torque': 3.0,         # Nm
                'joints': ['motor0_joint', 'motor1_joint']
            }],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'config', 'visualize.rviz')],
            output='screen'
        ),
    ])
