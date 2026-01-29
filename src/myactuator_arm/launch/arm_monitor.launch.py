"""
Full Arm Monitor Launcher - With Virtual Wall Safety

Monitors all 5 DOF of the full robotic arm with real meshes.
Virtual wall modes:
  - active (default): Applies opposing torque near limits
  - monitor: Logs warnings only, no torque commands (read-only)
  - disabled: No virtual wall at all

Usage:
    ros2 launch myactuator_arm arm_monitor.launch.py
    ros2 launch myactuator_arm arm_monitor.launch.py virtual_wall_mode:=monitor
    ros2 launch myactuator_arm arm_monitor.launch.py virtual_wall_mode:=disabled
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('myactuator_arm')

    urdf_file = os.path.join(pkg_dir, 'description', 'urdf', 'arm_full.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers', 'arm_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' pkg_share:=', pkg_dir]),
        value_type=str
    )

    # Launch arguments
    virtual_wall_mode_arg = DeclareLaunchArgument(
        'virtual_wall_mode',
        default_value='active',
        description='Virtual wall mode: active (torque), monitor (warnings only), disabled'
    )

    # Conditions for virtual wall modes
    is_active = PythonExpression([
        "'", LaunchConfiguration('virtual_wall_mode'), "' == 'active'"
    ])
    is_monitor = PythonExpression([
        "'", LaunchConfiguration('virtual_wall_mode'), "' == 'monitor'"
    ])

    return LaunchDescription([
        virtual_wall_mode_arg,

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

        # Joint State Broadcaster (publishes /joint_states)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Effort Controller (only needed for active mode to send torque commands)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['effort_controller', '--controller-manager', '/controller_manager'],
            output='screen',
            condition=IfCondition(is_active)
        ),

        # Virtual Wall Safety Node (active mode - applies torque)
        Node(
            package='myactuator_arm',
            executable='virtual_wall.py',
            name='virtual_wall',
            parameters=[{
                'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
                'lower_limits': [-1.5708, -3.14159, -1.57119, -1.57087, -1.5708],
                'upper_limits': [3.14159, 0.112, 1.5704, 1.57072, 1.5708],
                'soft_limit_margin': 0.05,
                'wall_stiffness': 15.0,
                'wall_damping': 0.5,
                'max_wall_torque': 5.0,
                'monitor_only': False,
            }],
            output='screen',
            condition=IfCondition(is_active)
        ),

        # Virtual Wall Safety Node (monitor mode - warnings only)
        Node(
            package='myactuator_arm',
            executable='virtual_wall.py',
            name='virtual_wall',
            parameters=[{
                'joints': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
                'lower_limits': [-1.5708, -3.14159, -1.57119, -1.57087, -1.5708],
                'upper_limits': [3.14159, 0.112, 1.5704, 1.57072, 1.5708],
                'soft_limit_margin': 0.05,
                'wall_stiffness': 15.0,
                'wall_damping': 0.5,
                'max_wall_torque': 3.0,
                'monitor_only': True,
            }],
            output='screen',
            condition=IfCondition(is_monitor)
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
