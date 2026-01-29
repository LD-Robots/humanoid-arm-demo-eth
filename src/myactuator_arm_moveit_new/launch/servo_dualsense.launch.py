"""
MoveIt Servo with DualSense Controller

Launches the complete stack for joystick teleoperation:
- EtherCAT hardware + controllers
- MoveIt components (planning scene, kinematics)
- Servo node for real-time control
- Joy node for DualSense controller
- Joystick-to-Servo mapping node

Prerequisites:
- DualSense controller paired via Bluetooth
- Install joy package: sudo apt install ros-humble-joy

Usage:
    ros2 launch myactuator_arm_moveit_new servo_dualsense.launch.py

Controls:
- Left Stick: Translate X/Y
- Right Stick Y: Translate Z
- Right Stick X: Rotate Z (yaw)
- L2/R2: Rotate Y/X (pitch/roll)
- Circle: Cartesian mode
- Cross: Joint mode
- Triangle: Emergency stop
- L1/R1: Increase/decrease speed
"""
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get MoveIt config
    moveit_config = MoveItConfigsBuilder(
        "myactuator_arm",
        package_name="myactuator_arm_moveit_new"
    ).to_moveit_configs()

    # Get package directories
    arm_pkg = get_package_share_directory('myactuator_arm')
    moveit_pkg = get_package_share_directory('myactuator_arm_moveit_new')

    # URDF file path
    urdf_file = os.path.join(moveit_pkg, 'config', 'myactuator_arm.urdf.xacro')

    # Controllers config
    controllers_file = os.path.join(arm_pkg, 'config', 'controllers', 'arm_controllers.yaml')

    # Servo config
    servo_config_file = os.path.join(moveit_pkg, 'config', 'servo_config.yaml')

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

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Joint Trajectory Controller
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link']
    )

    # Move Group (required for Servo)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')
        )
    )

    # Servo Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node',
        parameters=[
            servo_config_file,
            moveit_config.to_dict(),
        ],
        output='screen'
    )

    # Joy Node (DualSense controller)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.05,
            'autorepeat_rate': 50.0,
        }],
        output='screen'
    )

    # DualSense to Servo mapper
    dualsense_servo = Node(
        package='myactuator_arm_moveit_new',
        executable='dualsense_servo.py',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        joint_trajectory_controller,
        static_tf,
        move_group,
        servo_node,
        joy_node,
        dualsense_servo,
    ])
