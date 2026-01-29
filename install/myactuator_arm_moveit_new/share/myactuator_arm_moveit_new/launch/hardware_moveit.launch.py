"""
MoveIt Hardware Launch - Real EtherCAT Control

Launches the complete MoveIt stack with real EtherCAT hardware:
- Controller manager with EtherCAT driver (CSP mode 8)
- Joint trajectory controller
- Move group for motion planning
- RViz with MoveIt interface

WARNING: Motors will be in position control mode (not backdrivable)
         Ensure workspace is clear before executing motions

Usage:
    ros2 launch myactuator_arm_moveit_new hardware_moveit.launch.py
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get MoveIt config
    moveit_config = MoveItConfigsBuilder(
        "myactuator_arm",
        package_name="myactuator_arm_moveit_new"
    ).to_moveit_configs()

    # Get package directories
    arm_pkg = get_package_share_directory('myactuator_arm')
    moveit_pkg = get_package_share_directory('myactuator_arm_moveit_new')

    # URDF file path (MoveIt config URDF with EtherCAT)
    urdf_file = os.path.join(moveit_pkg, 'config', 'myactuator_arm.urdf.xacro')

    # Controllers config (from myactuator_arm package)
    controllers_file = os.path.join(arm_pkg, 'config', 'controllers', 'arm_controllers.yaml')

    # Build robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' pkg_share:=', arm_pkg]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Controller Manager (loads EtherCAT hardware interface)
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    # Joint State Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Joint Trajectory Controller spawner
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Static TF for virtual joint (world frame)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

    # Move Group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # RViz with MoveIt
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'moveit_rviz.launch.py')
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        static_tf,
        move_group,
        rviz,
    ])
