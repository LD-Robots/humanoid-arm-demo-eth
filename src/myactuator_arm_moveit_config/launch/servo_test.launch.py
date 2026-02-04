"""
MoveIt Servo Test Launch File

Launches the complete stack for testing MoveIt Servo with fake hardware.

Usage:
    ros2 launch myactuator_arm_moveit_config servo_test.launch.py

Testing Servo Commands:
    ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped \
        "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1}}}" -r 50
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

# Try to import ParameterBuilder
try:
    from launch_param_builder import ParameterBuilder
    HAS_PARAM_BUILDER = True
except ImportError:
    HAS_PARAM_BUILDER = False


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')

    # Package directory
    moveit_config_pkg = get_package_share_directory('myactuator_arm_moveit_config')

    # MoveIt config
    moveit_config = MoveItConfigsBuilder(
        "myactuator_arm",
        package_name="myactuator_arm_moveit_config"
    ).to_moveit_configs()

    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('myactuator_arm_moveit_config'),
            'config', 'myactuator_arm.urdf.xacro'
        ])
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Config paths
    servo_params_file = os.path.join(moveit_config_pkg, 'config', 'servo_params.yaml')
    ros2_controllers = os.path.join(moveit_config_pkg, 'config', 'ros2_controllers.yaml')

    # ========== NODES ==========

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
        output='screen'
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'world', '--child-frame-id', 'table'],
        output='screen'
    )

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
            # Disable start state tolerance check (0.0 = disabled, safe for simulation)
            {'trajectory_execution.allowed_start_tolerance': 0.0},
        ],
    )

    # MoveIt Servo Node - use parameter file directly for proper loading
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        name='servo_node',
        parameters=[
            servo_params_file,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output='screen',
    )

    # RViz
    rviz_config = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher,
        static_tf,
        TimerAction(period=1.0, actions=[ros2_control_node]),
        TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=4.0, actions=[joint_trajectory_controller_spawner]),
        TimerAction(period=5.0, actions=[move_group_node]),
        TimerAction(period=5.0, actions=[rviz_node]),
        TimerAction(period=7.0, actions=[servo_node]),
    ])
