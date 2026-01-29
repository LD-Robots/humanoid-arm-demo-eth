#!/usr/bin/env python3
"""
Position Control Test Script

Sends position commands to individual joints for testing.
Keeps other joints at their current positions.

Usage:
    # Move joint 1 by +10 degrees (relative)
    ros2 run myactuator_arm test_position.py --joint 1 --angle 10

    # Move joint 2 to absolute position 0 degrees
    ros2 run myactuator_arm test_position.py --joint 2 --angle 0 --absolute

    # Move joint 1 by -5 degrees with 3 second duration
    ros2 run myactuator_arm test_position.py --joint 1 --angle -5 --duration 3.0

    # Show current joint positions
    ros2 run myactuator_arm test_position.py --status
"""

import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class PositionTestNode(Node):
    def __init__(self):
        super().__init__('position_test')

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.current_positions = None

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Action client for trajectory controller
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def joint_state_callback(self, msg: JointState):
        positions = {}
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                positions[name] = msg.position[i]
        if len(positions) == len(self.joint_names):
            self.current_positions = positions

    def wait_for_positions(self, timeout=5.0):
        """Wait until we have current joint positions."""
        start = self.get_clock().now()
        while self.current_positions is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().error('Timeout waiting for joint states')
                return False
        return True

    def print_status(self):
        """Print current joint positions."""
        if not self.wait_for_positions():
            return

        self.get_logger().info('Current joint positions:')
        for name in self.joint_names:
            pos_rad = self.current_positions[name]
            pos_deg = math.degrees(pos_rad)
            self.get_logger().info(f'  {name}: {pos_rad:.4f} rad ({pos_deg:.1f} deg)')

    def send_position(self, joint_num: int, angle_deg: float, absolute: bool, duration: float):
        """Send position command to a single joint."""
        if not self.wait_for_positions():
            return False

        if joint_num < 1 or joint_num > 5:
            self.get_logger().error(f'Invalid joint number: {joint_num}. Must be 1-5.')
            return False

        joint_name = f'joint{joint_num}'

        # Calculate target position
        if absolute:
            target_rad = math.radians(angle_deg)
        else:
            current_rad = self.current_positions[joint_name]
            target_rad = current_rad + math.radians(angle_deg)

        # Build trajectory with all joints (others stay at current position)
        target_positions = []
        for name in self.joint_names:
            if name == joint_name:
                target_positions.append(target_rad)
            else:
                target_positions.append(self.current_positions[name])

        # Log the command
        current_deg = math.degrees(self.current_positions[joint_name])
        target_deg = math.degrees(target_rad)
        self.get_logger().info(
            f'Moving {joint_name}: {current_deg:.1f} deg -> {target_deg:.1f} deg '
            f'(duration: {duration:.1f}s)'
        )

        # Wait for action server
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory action server not available')
            return False

        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * 5
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))

        goal.trajectory.points = [point]

        # Send goal
        self.get_logger().info('Sending trajectory...')
        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('Motion completed successfully!')
            return True
        else:
            self.get_logger().error(f'Motion failed with error code: {result.result.error_code}')
            return False


def main():
    parser = argparse.ArgumentParser(description='Test position control on individual joints')
    parser.add_argument('--joint', '-j', type=int, help='Joint number (1-5)')
    parser.add_argument('--angle', '-a', type=float, help='Angle in degrees')
    parser.add_argument('--absolute', action='store_true', help='Use absolute position (default: relative)')
    parser.add_argument('--duration', '-d', type=float, default=2.0, help='Motion duration in seconds (default: 2.0)')
    parser.add_argument('--status', '-s', action='store_true', help='Show current joint positions')

    args = parser.parse_args()

    rclpy.init()
    node = PositionTestNode()

    try:
        if args.status:
            node.print_status()
        elif args.joint is not None and args.angle is not None:
            node.send_position(args.joint, args.angle, args.absolute, args.duration)
        else:
            parser.print_help()
            print('\nExamples:')
            print('  ros2 run myactuator_arm test_position.py --status')
            print('  ros2 run myactuator_arm test_position.py --joint 1 --angle 10')
            print('  ros2 run myactuator_arm test_position.py --joint 2 --angle 0 --absolute')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
