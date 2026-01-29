"""
Base launcher for MyActuator X6 - Headless with effort control (safe default)

Starts:
- EtherCAT hardware interface
- Joint state broadcaster
- Effort controller (default)
- Virtual wall safety node

Use this as the base, then switch controllers as needed.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('myactuator_x6_test')

    # Use position mode URDF for MoveIt compatibility (mode 8 = CSP)
    urdf_file = os.path.join(pkg_dir, 'urdf', 'x6_test.urdf.xacro')
    # Use position config which defines BOTH controllers (effort + trajectory)
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers_position.yaml')

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

        # Trajectory Controller (loaded but INACTIVE - motor is back-drivable until activated)
        # This provides safe compliant behavior by default
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '--inactive', '--controller-manager', '/controller_manager'],
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
    ])
