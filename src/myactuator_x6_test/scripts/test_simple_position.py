#!/usr/bin/env python3
"""
Simple position command with velocity and acceleration constraints.
Sends a single position command with trapezoidal velocity profile.
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import sys
import math


class SimplePositionCommand(Node):
    def __init__(self):
        super().__init__('simple_position_command')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def get_current_position(self, joint_name, timeout_sec=5.0):
        """Wait for a JointState message and return the position of the specified joint."""
        self.get_logger().info(f'Waiting for current position of joint: {joint_name}')

        position = [None]

        def callback(msg):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                position[0] = msg.position[idx]

        sub = self.create_subscription(JointState, '/joint_states', callback, 10)

        start_time = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sec)

        while position[0] is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().warn('Timeout waiting for joint_states. Defaulting to 0.0')
                position[0] = 0.0
                break

        self.destroy_subscription(sub)
        self.get_logger().info(f'Current position: {position[0]}')
        return position[0]

    def calculate_trapezoidal_duration(self, distance, max_velocity, acceleration, deceleration):
        """
        Calculate the duration for a trapezoidal velocity profile.

        Returns the total time needed to move the given distance with the specified
        velocity and acceleration constraints.
        """
        distance = abs(distance)

        # Time to accelerate to max velocity
        t_accel = max_velocity / acceleration
        # Time to decelerate from max velocity
        t_decel = max_velocity / deceleration

        # Distance covered during acceleration
        d_accel = 0.5 * acceleration * t_accel * t_accel
        # Distance covered during deceleration
        d_decel = 0.5 * deceleration * t_decel * t_decel

        # Check if we can reach max velocity (triangular vs trapezoidal profile)
        if d_accel + d_decel >= distance:
            # Triangular profile - can't reach max velocity
            # Solve for peak velocity: v_peak^2 / (2*a) + v_peak^2 / (2*d) = distance
            # v_peak^2 * (1/(2*a) + 1/(2*d)) = distance
            # v_peak = sqrt(distance / (1/(2*a) + 1/(2*d)))
            v_peak = math.sqrt(distance / (1/(2*acceleration) + 1/(2*deceleration)))
            t_accel = v_peak / acceleration
            t_decel = v_peak / deceleration
            total_time = t_accel + t_decel
            self.get_logger().info(f'Triangular profile: peak_velocity={v_peak:.2f}, duration={total_time:.2f}s')
        else:
            # Trapezoidal profile - cruise at max velocity
            d_cruise = distance - d_accel - d_decel
            t_cruise = d_cruise / max_velocity
            total_time = t_accel + t_cruise + t_decel
            self.get_logger().info(f'Trapezoidal profile: accel={t_accel:.2f}s, cruise={t_cruise:.2f}s, decel={t_decel:.2f}s')

        return total_time

    def send_position(self, target_position, max_velocity, acceleration, deceleration):
        """Send a single position command with velocity/acceleration constraints."""
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Get current position
        current_position = self.get_current_position('motor_joint')

        # Calculate distance and duration
        distance = target_position - current_position
        duration = self.calculate_trapezoidal_duration(abs(distance), max_velocity, acceleration, deceleration)

        # Add small buffer for stability
        duration += 0.1

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['motor_joint']

        # Start point - current position
        start_point = JointTrajectoryPoint()
        start_point.positions = [float(current_position)]
        start_point.velocities = [0.0]
        start_point.accelerations = [0.0]
        start_point.time_from_start = Duration(sec=0, nanosec=100000000)  # 0.1s

        # End point - target position
        end_point = JointTrajectoryPoint()
        end_point.positions = [float(target_position)]
        end_point.velocities = [0.0]
        end_point.accelerations = [0.0]
        duration_sec = int(duration)
        duration_nanosec = int((duration - duration_sec) * 1e9)
        end_point.time_from_start = Duration(sec=duration_sec, nanosec=duration_nanosec)

        goal_msg.trajectory.points = [start_point, end_point]

        self.get_logger().info(
            f'Sending position command:\n'
            f'  From: {current_position:.2f}\n'
            f'  To: {target_position:.2f}\n'
            f'  Distance: {distance:.2f}\n'
            f'  Max velocity: {max_velocity:.2f}\n'
            f'  Acceleration: {acceleration:.2f}\n'
            f'  Deceleration: {deceleration:.2f}\n'
            f'  Duration: {duration:.2f}s'
        )

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(f'Result: error_code={result.error_code}')


def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run myactuator_x6_test test_simple_position.py <position> [max_velocity] [acceleration] [deceleration]")
        print()
        print("Arguments:")
        print("  position      - Target position in encoder counts")
        print("  max_velocity  - Maximum velocity in counts/s (default: 10000)")
        print("  acceleration  - Acceleration in counts/s^2 (default: 5000)")
        print("  deceleration  - Deceleration in counts/s^2 (default: same as acceleration)")
        print()
        print("Examples:")
        print("  ros2 run myactuator_x6_test test_simple_position.py 50000")
        print("  ros2 run myactuator_x6_test test_simple_position.py 50000 10000 5000")
        print("  ros2 run myactuator_x6_test test_simple_position.py 50000 10000 5000 3000")
        return

    rclpy.init()

    target_position = float(sys.argv[1])
    max_velocity = float(sys.argv[2]) if len(sys.argv) > 2 else 10000.0
    acceleration = float(sys.argv[3]) if len(sys.argv) > 3 else 5000.0
    deceleration = float(sys.argv[4]) if len(sys.argv) > 4 else acceleration

    node = SimplePositionCommand()
    node.send_position(target_position, max_velocity, acceleration, deceleration)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
