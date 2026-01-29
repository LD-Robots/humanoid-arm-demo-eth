#!/usr/bin/env python3
"""
Speed and Position Test for MyActuator X6

Test position commands with specific velocity profiles, or pure velocity control.
Useful for tuning and validating motor performance before MoveIt integration.
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


class SpeedPositionTest(Node):
    def __init__(self):
        super().__init__('speed_position_test')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.current_position = None
        self.current_velocity = None
        self.joint_name = 'motor_joint'

        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_callback,
            10
        )

    def _joint_callback(self, msg):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]
            if idx < len(msg.velocity):
                self.current_velocity = msg.velocity[idx]

    def wait_for_position(self, timeout=5.0):
        import time
        start = time.time()
        while self.current_position is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_position is not None

    def move_to_position(self, target_pos, max_velocity, acceleration=None):
        """
        Move to target position with specified max velocity.
        Calculates appropriate trajectory timing based on velocity/acceleration.
        """
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        if not self.wait_for_position():
            self.get_logger().error('Could not get current position')
            return False

        start_pos = self.current_position
        distance = abs(target_pos - start_pos)

        if distance < 1:
            self.get_logger().info('Already at target position')
            return True

        # Calculate duration based on trapezoidal profile
        if acceleration is None:
            acceleration = max_velocity * 2  # Default: reach max vel in 0.5s

        # Time to accelerate/decelerate
        t_accel = max_velocity / acceleration
        d_accel = 0.5 * acceleration * t_accel * t_accel

        if 2 * d_accel >= distance:
            # Triangular profile
            t_total = 2 * math.sqrt(distance / acceleration)
            actual_max_vel = acceleration * (t_total / 2)
        else:
            # Trapezoidal profile
            d_cruise = distance - 2 * d_accel
            t_cruise = d_cruise / max_velocity
            t_total = 2 * t_accel + t_cruise
            actual_max_vel = max_velocity

        # Build trajectory
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [self.joint_name]

        # Start point
        p0 = JointTrajectoryPoint()
        p0.positions = [float(start_pos)]
        p0.velocities = [0.0]
        p0.accelerations = [0.0]
        p0.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms
        goal_msg.trajectory.points.append(p0)

        # End point
        p1 = JointTrajectoryPoint()
        p1.positions = [float(target_pos)]
        p1.velocities = [0.0]
        p1.accelerations = [0.0]
        sec = int(t_total)
        nsec = int((t_total - sec) * 1e9)
        p1.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal_msg.trajectory.points.append(p1)

        self.get_logger().info(
            f'Moving: {start_pos:.0f} -> {target_pos:.0f}\n'
            f'  Distance: {distance:.0f} counts\n'
            f'  Max velocity: {actual_max_vel:.0f} counts/s\n'
            f'  Duration: {t_total:.2f}s'
        )

        # Send and wait
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=t_total + 5)

        # Report result
        rclpy.spin_once(self, timeout_sec=0.1)
        error = abs(self.current_position - target_pos)
        self.get_logger().info(f'Reached: {self.current_position:.0f}, Error: {error:.0f} counts')

        return error < 200

    def velocity_move(self, velocity, duration):
        """
        Move at constant velocity for specified duration.
        """
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        if not self.wait_for_position():
            self.get_logger().error('Could not get current position')
            return False

        start_pos = self.current_position
        expected_end = start_pos + velocity * duration

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [self.joint_name]

        # Start - current position, zero velocity
        p0 = JointTrajectoryPoint()
        p0.positions = [float(start_pos)]
        p0.velocities = [0.0]
        p0.time_from_start = Duration(sec=0, nanosec=50000000)
        goal_msg.trajectory.points.append(p0)

        # Ramp up to velocity (0.2s)
        ramp_time = 0.2
        p1 = JointTrajectoryPoint()
        p1.positions = [float(start_pos + velocity * ramp_time * 0.5)]
        p1.velocities = [float(velocity)]
        sec = int(ramp_time)
        nsec = int((ramp_time - sec) * 1e9)
        p1.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal_msg.trajectory.points.append(p1)

        # Cruise at velocity
        cruise_end_time = ramp_time + duration
        cruise_end_pos = start_pos + velocity * ramp_time * 0.5 + velocity * duration
        p2 = JointTrajectoryPoint()
        p2.positions = [float(cruise_end_pos)]
        p2.velocities = [float(velocity)]
        sec = int(cruise_end_time)
        nsec = int((cruise_end_time - sec) * 1e9)
        p2.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal_msg.trajectory.points.append(p2)

        # Ramp down (0.2s)
        end_time = cruise_end_time + ramp_time
        end_pos = cruise_end_pos + velocity * ramp_time * 0.5
        p3 = JointTrajectoryPoint()
        p3.positions = [float(end_pos)]
        p3.velocities = [0.0]
        sec = int(end_time)
        nsec = int((end_time - sec) * 1e9)
        p3.time_from_start = Duration(sec=sec, nanosec=nsec)
        goal_msg.trajectory.points.append(p3)

        self.get_logger().info(
            f'Velocity move:\n'
            f'  Velocity: {velocity:.0f} counts/s\n'
            f'  Duration: {duration:.1f}s\n'
            f'  Expected travel: {end_pos - start_pos:.0f} counts'
        )

        # Send and wait
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=end_time + 5)

        rclpy.spin_once(self, timeout_sec=0.1)
        actual_travel = self.current_position - start_pos
        self.get_logger().info(
            f'Actual travel: {actual_travel:.0f} counts\n'
            f'Final position: {self.current_position:.0f}'
        )

        return True

    def speed_test(self, velocities):
        """Test multiple velocities and report performance."""
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('SPEED TEST')
        self.get_logger().info('='*50)

        if not self.wait_for_position():
            return

        start_pos = self.current_position
        results = []

        for vel in velocities:
            self.get_logger().info(f'\nTesting velocity: {vel} counts/s')

            # Move forward at this velocity for 1 second
            self.velocity_move(vel, 1.0)
            rclpy.spin_once(self, timeout_sec=0.2)

            # Return to start
            self.move_to_position(start_pos, abs(vel))
            rclpy.spin_once(self, timeout_sec=0.5)

            results.append(vel)

        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('Speed test complete')
        self.get_logger().info(f'Tested velocities: {results}')


def print_usage():
    print("""
Speed and Position Test
=======================

Usage:
  ros2 run myactuator_x6_test test_speed_position.py <command> [args...]

Commands:
  pos <target> <velocity> [accel]    Move to position with max velocity
  vel <velocity> <duration>          Move at constant velocity for duration
  test                               Run speed test at multiple velocities

Examples:
  # Move to position 50000 at max 10000 counts/s
  ros2 run myactuator_x6_test test_speed_position.py pos 50000 10000

  # Move to position with specific acceleration
  ros2 run myactuator_x6_test test_speed_position.py pos 50000 10000 5000

  # Move at 5000 counts/s for 2 seconds
  ros2 run myactuator_x6_test test_speed_position.py vel 5000 2.0

  # Run speed test
  ros2 run myactuator_x6_test test_speed_position.py test
""")


def main():
    if len(sys.argv) < 2:
        print_usage()
        return

    rclpy.init()
    node = SpeedPositionTest()

    try:
        cmd = sys.argv[1].lower()

        if cmd == 'pos':
            if len(sys.argv) < 4:
                print("Usage: pos <target> <velocity> [acceleration]")
                return
            target = float(sys.argv[2])
            velocity = float(sys.argv[3])
            accel = float(sys.argv[4]) if len(sys.argv) > 4 else None
            node.move_to_position(target, velocity, accel)

        elif cmd == 'vel':
            if len(sys.argv) < 4:
                print("Usage: vel <velocity> <duration>")
                return
            velocity = float(sys.argv[2])
            duration = float(sys.argv[3])
            node.velocity_move(velocity, duration)

        elif cmd == 'test':
            # Test various speeds
            velocities = [1000, 2000, 5000, 10000, 20000]
            node.speed_test(velocities)

        else:
            print_usage()

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
