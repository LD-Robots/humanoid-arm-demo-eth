#!/usr/bin/env python3
"""
Test Joint Motion Script

Sends position commands to test individual joints safely.

Usage:
    # Move joint 1 to 0.5 radians
    ros2 run myactuator_arm test_joint_motion.py --joint 1 --position 0.5

    # Move joint 2 to -0.3 radians with custom duration
    ros2 run myactuator_arm test_joint_motion.py --joint 2 --position -0.3 --duration 3.0

    # Return joint 3 to zero
    ros2 run myactuator_arm test_joint_motion.py --joint 3 --position 0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import argparse
import sys


class JointTester(Node):
    def __init__(self, joint_num, target_position, duration):
        super().__init__('joint_tester')

        self.joint_num = joint_num
        self.target_position = target_position
        self.duration = duration

        # All joint names
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        # Action client for trajectory controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info(f'Testing {self.joint_names[joint_num-1]} → {target_position:.3f} rad ({target_position * 57.3:.1f}°)')

    def send_goal(self):
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names

        # Create trajectory point (move only the specified joint, others stay at 0)
        point = JointTrajectoryPoint()
        point.positions = [0.0] * 5  # All joints at 0
        point.positions[self.joint_num - 1] = self.target_position  # Set target for test joint
        point.time_from_start.sec = int(self.duration)
        point.time_from_start.nanosec = int((self.duration - int(self.duration)) * 1e9)

        goal_msg.trajectory.points = [point]

        self.get_logger().info(f'Sending goal: {self.joint_names[self.joint_num-1]} = {self.target_position:.3f} rad')

        # Send goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, executing...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Print progress
        pass

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Motion completed! Error code: {result.error_code}')
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Test individual joint motion')
    parser.add_argument('--joint', type=int, required=True, choices=[1, 2, 3, 4, 5],
                        help='Joint number to test (1-5)')
    parser.add_argument('--position', type=float, required=True,
                        help='Target position in radians')
    parser.add_argument('--duration', type=float, default=2.0,
                        help='Motion duration in seconds (default: 2.0)')

    # Parse args (skip ROS args)
    args = parser.parse_args(sys.argv[1:])

    # Safety check on position limits
    limits = {
        1: (-1.57, 1.57),    # ±90°
        2: (-1.13, 2.01),    # -65° to 115°
        3: (-1.57, 1.57),    # ±90°
        4: (-1.57, 1.57),    # ±90°
        5: (-1.57, 1.57),    # ±90°
    }

    lower, upper = limits[args.joint]
    if args.position < lower or args.position > upper:
        print(f'⚠️  WARNING: Position {args.position:.3f} is outside safe limits [{lower:.3f}, {upper:.3f}]')
        response = input('Continue anyway? (yes/no): ')
        if response.lower() != 'yes':
            print('Aborted.')
            return

    rclpy.init()

    node = JointTester(args.joint, args.position, args.duration)

    if node.send_goal():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Interrupted by user')

    node.destroy_node()


if __name__ == '__main__':
    main()
