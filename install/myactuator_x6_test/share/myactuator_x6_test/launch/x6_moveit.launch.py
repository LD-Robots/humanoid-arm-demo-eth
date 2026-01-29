"""
MoveIt launcher for MyActuator X6

Requires x6_base.launch.py to be running first!

This launcher:
1. Switches from effort to trajectory controller
2. Launches MoveIt move_group and RViz
3. Stays running until Ctrl+C
4. On shutdown, switches back to effort controller (safe mode)

Usage:
    Terminal 1: ros2 launch myactuator_x6_test x6_base.launch.py
    Terminal 2: ros2 launch myactuator_x6_test x6_moveit.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "x6_test", 
        package_name="x6_moveit_config"
    ).to_moveit_configs()
    
    # Get package directories
    moveit_pkg = get_package_share_directory('x6_moveit_config')
    
    return LaunchDescription([
        # Position Mode Manager - switches controllers and stays alive
        Node(
            package='myactuator_x6_test',
            executable='position_mode_manager.py',
            name='position_mode_manager',
            output='screen'
        ),

        # MoveIt move_group
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {'publish_robot_description_semantic': True},
            ],
        ),

        # RViz with MoveIt config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', os.path.join(moveit_pkg, 'config', 'moveit.rviz')],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
        ),
    ])
