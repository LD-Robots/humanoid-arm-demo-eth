"""
Free Mode Launcher for MyActuator X6 Dual Motors

Starts both motors in CST (torque control) mode with zero torque,
allowing free manual movement while visualizing in RViz.

Usage:
    ros2 launch myactuator_x6_test x6_free.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('myactuator_x6_test')

    # Use free mode URDF (CST torque control mode)
    urdf_file = os.path.join(pkg_dir, 'urdf', 'x6_test_free.urdf.xacro')
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
            package='myactuator_x6_test',
            executable='virtual_wall.py',
            name='virtual_wall',
            parameters=[{
                'joints': ['motor0_joint', 'motor1_joint'],
                'lower_limits': [-0.75, -2.98],       # motor0: -43°, motor1: -171°
                'upper_limits': [3.14, 0.2],         # motor0: +180°, motor1: +11°
                'soft_limit_margin': 0.2618,        # 15 degrees margin
                'wall_stiffness': 10.0,             # Nm/rad
                'wall_damping': 5.0,                # Nm/(rad/s)
                'max_wall_torque': 3.0,             # Nm
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
