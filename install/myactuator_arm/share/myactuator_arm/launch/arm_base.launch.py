"""
Base launcher for MyActuator Arm - Headless with mode 8 (CSP) for MoveIt

Starts:
- EtherCAT hardware interface with mode 8 URDF
- Joint state broadcaster
- Trajectory controller (INACTIVE - motors free-floating until MoveIt activates)
- Virtual wall safety node (OPTIONAL)

Use this as the base, then launch arm_moveit.launch.py to activate MoveIt.

Usage:
    Terminal 1: ros2 launch myactuator_arm arm_base.launch.py
    Terminal 2: ros2 launch myactuator_arm arm_moveit.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    arm_pkg = get_package_share_directory('myactuator_arm')
    moveit_pkg = get_package_share_directory('myactuator_arm_moveit_new')

    # Use MoveIt config URDF (already has mode 8 for all joints)
    urdf_file = os.path.join(moveit_pkg, 'config', 'myactuator_arm.urdf.xacro')

    # Use arm controllers config (defines both effort + trajectory controllers)
    controllers_file = os.path.join(arm_pkg, 'config', 'controllers', 'arm_controllers.yaml')

    # Build robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' pkg_share:=', arm_pkg]),
        value_type=str
    )

    # Launch arguments
    enable_virtual_wall_arg = DeclareLaunchArgument(
        'enable_virtual_wall',
        default_value='false',
        description='Enable virtual wall safety (spring-damper near limits)'
    )

    return LaunchDescription([
        enable_virtual_wall_arg,

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

        # Joint State Broadcaster (ACTIVE)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Trajectory Controller (loaded but INACTIVE - motors free-floating until MoveIt activates)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--inactive', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Virtual Wall Safety Node (OPTIONAL)
        Node(
            package='myactuator_arm',
            executable='virtual_wall.py',
            name='virtual_wall',
            parameters=[{
                'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
                'lower_limits': [-1.5708, -1.13446, -1.57119, -1.57087, -1.5708],
                'upper_limits': [1.5708, 2.00713, 1.5704, 1.57072, 1.5708],
                'soft_limit_margin': 0.1,         # 5.7° margin before hard limit
                'wall_stiffness': 15.0,           # Moderate spring constant (Nm/rad)
                'wall_damping': 0.5,              # Light damping (Nm/(rad/s))
                'max_wall_torque': 3.0,           # 15% of rated torque (safe)
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_virtual_wall'))
        ),
    ])
