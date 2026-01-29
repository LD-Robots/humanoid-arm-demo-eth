import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('myactuator_x6_test')

    urdf_file = os.path.join(pkg_dir, 'urdf', 'x6_test_effort.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers_effort.yaml')

    robot_description = Command(['xacro ', urdf_file, ' pkg_share:=', pkg_dir])

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

        # Effort Controller (forward command)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['effort_controller', '--controller-manager', '/controller_manager'],
            output='screen'
        ),

        # Virtual Wall Safety Node
        Node(
            package='myactuator_x6_test',
            executable='virtual_wall.py',
            name='virtual_wall',
            parameters=[{
                'position_limit': 1.5708,       # 90 degrees
                'soft_limit_margin': 0.01,    # 15 degrees margin (more gradual)
                'wall_stiffness': 2.0,         # Nm/rad (reduced - less aggressive)
                'wall_damping': 15.0,            # Nm/(rad/s) (increased - more damping)
                'max_wall_torque': 3.0,         # Nm (matches SDO limit)
                'joints': ['motor_joint']  # motor_2_joint disabled for testing
            }],
            output='screen'
        ),
    ])
