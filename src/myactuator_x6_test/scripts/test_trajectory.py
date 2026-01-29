#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
from sensor_msgs.msg import JointState

class TrajectoryTest(Node):
    def __init__(self):
        super().__init__('trajectory_test')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

    def get_current_position(self, joint_name, timeout_sec=5.0):
        """Wait for a JointState message and return the position of the specified joint."""
        self.get_logger().info(f'Waiting for current position of joint: {joint_name}')
        
        position = [None]  # Use list to allow modification in callback
        
        def callback(msg):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                position[0] = msg.position[idx]
        
        sub = self.create_subscription(JointState, '/joint_states', callback, 10)
        
        # Spin until we get a message or timeout
        start_time = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=timeout_sec)
        
        while position[0] is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().warn(f'Timeout waiting for joint_states. Defaulting to 0.0')
                position[0] = 0.0
                break
        
        self.destroy_subscription(sub)
        self.get_logger().info(f'Current position of {joint_name}: {position[0]}')
        return position[0]

    def send_oscillating_goal(self, position1, position2, duration_sec=2.0, cycles=5):
        """Send oscillating trajectory between two positions."""
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        # Get current position
        start_position = self.get_current_position('motor_joint')

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['motor_joint']

        points = []
        current_time = 0.0

        # Start point
        point = JointTrajectoryPoint()
        point.positions = [float(start_position)]
        point.velocities = [0.0]
        point.accelerations = [0.0]
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5s
        points.append(point)
        current_time = 0.5

        # Oscillate between positions
        for i in range(cycles):
            # Move to position1
            current_time += duration_sec
            point = JointTrajectoryPoint()
            point.positions = [float(position1)]
            point.velocities = [0.0]
            point.accelerations = [0.0]
            point.time_from_start = Duration(sec=int(current_time), nanosec=int((current_time % 1) * 1e9))
            points.append(point)

            # Move to position2
            current_time += duration_sec
            point = JointTrajectoryPoint()
            point.positions = [float(position2)]
            point.velocities = [0.0]
            point.accelerations = [0.0]
            point.time_from_start = Duration(sec=int(current_time), nanosec=int((current_time % 1) * 1e9))
            points.append(point)

        goal_msg.trajectory.points = points

        self.get_logger().info(f'Sending oscillating trajectory: {position1} <-> {position2}, {cycles} cycles, {duration_sec}s per move')

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

    def send_goal(self, position, duration_sec=2.0, start_position=None):
        self.get_logger().info(f'Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['motor_joint']

        # Get current position
        if start_position is None:
            start_position = self.get_current_position('motor_joint')

        # Start point with small time offset to avoid aggressive start
        start_point = JointTrajectoryPoint()
        start_point.positions = [float(start_position)]
        start_point.velocities = [0.0]
        start_point.accelerations = [0.0]
        start_point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5s

        # End point
        end_point = JointTrajectoryPoint()
        end_point.positions = [float(position)]
        end_point.velocities = [0.0]
        end_point.accelerations = [0.0]
        end_point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))

        goal_msg.trajectory.points = [start_point, end_point]

        self.get_logger().info(f'Sending trajectory: from {start_position} to {position} in {duration_sec}s')

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
        print("Usage:")
        print("  Single position: ros2 run myactuator_x6_test test_trajectory.py <position> [duration_sec]")
        print("  Oscillating:     ros2 run myactuator_x6_test test_trajectory.py <pos1> <pos2> [duration_sec] [cycles]")
        print("Examples:")
        print("  ros2 run myactuator_x6_test test_trajectory.py 50000 2.0")
        print("  ros2 run myactuator_x6_test test_trajectory.py 10000 50000 2.0 5")
        return

    rclpy.init()
    node = TrajectoryTest()

    if len(sys.argv) >= 3 and sys.argv[2].replace('.', '').replace('-', '').isdigit() and float(sys.argv[2]) != 0:
        # Check if second arg could be a position (not a small duration)
        try:
            pos2 = float(sys.argv[2])
            if abs(pos2) > 10:  # Likely a position, not duration
                # Oscillating mode
                position1 = float(sys.argv[1])
                position2 = pos2
                duration = float(sys.argv[3]) if len(sys.argv) > 3 else 2.0
                cycles = int(sys.argv[4]) if len(sys.argv) > 4 else 5
                node.send_oscillating_goal(position1, position2, duration, cycles)
            else:
                # Single position mode
                position = float(sys.argv[1])
                duration = float(sys.argv[2])
                node.send_goal(position, duration)
        except:
            # Single position mode
            position = float(sys.argv[1])
            duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
            node.send_goal(position, duration)
    else:
        # Single position mode
        position = float(sys.argv[1])
        duration = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
        node.send_goal(position, duration)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
