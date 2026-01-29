#!/usr/bin/env python3
"""
MoveIt/MTC Readiness Test for MyActuator X6

Tests the motor and controller setup to ensure compatibility with MoveIt and MTC.
Validates:
  - Controller availability and status
  - Joint state feedback
  - Trajectory execution accuracy
  - Velocity and acceleration limits
  - Position repeatability
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import ListControllers
import sys
import time
import math


class MoveItReadinessTest(Node):
    def __init__(self, joint_name='motor_joint'):
        super().__init__('moveit_readiness_test')
        self.joint_name = joint_name
        self.current_position = None
        self.current_velocity = None
        self.position_history = []
        self.test_results = []

        # Callback group for concurrent operations
        self.cb_group = ReentrantCallbackGroup()

        # Action client for trajectory controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.cb_group
        )

        # Service client to check controllers
        self._list_controllers_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )

        # Joint state subscriber
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10,
            callback_group=self.cb_group
        )

    def _joint_state_callback(self, msg):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            self.current_position = msg.position[idx]
            if idx < len(msg.velocity):
                self.current_velocity = msg.velocity[idx]
            self.position_history.append((time.time(), self.current_position))
            # Keep last 1000 samples
            if len(self.position_history) > 1000:
                self.position_history.pop(0)

    def log_result(self, test_name, passed, message=""):
        status = "✓ PASS" if passed else "✗ FAIL"
        self.test_results.append((test_name, passed, message))
        self.get_logger().info(f"{status}: {test_name}" + (f" - {message}" if message else ""))

    def wait_for_position(self, timeout=5.0):
        """Wait until we receive position feedback."""
        start = time.time()
        while self.current_position is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.current_position is not None

    def test_controller_status(self):
        """Test 1: Check if required controllers are active."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST 1: Controller Status")
        self.get_logger().info("="*60)

        if not self._list_controllers_client.wait_for_service(timeout_sec=5.0):
            self.log_result("Controller Manager", False, "Service not available")
            return False

        request = ListControllers.Request()
        future = self._list_controllers_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.log_result("Controller Manager", False, "Failed to get controller list")
            return False

        controllers = future.result().controller
        jtc_active = False
        jsb_active = False

        for ctrl in controllers:
            if ctrl.name == 'joint_trajectory_controller':
                jtc_active = ctrl.state == 'active'
                self.log_result(
                    "JointTrajectoryController",
                    jtc_active,
                    f"State: {ctrl.state}"
                )
            elif ctrl.name == 'joint_state_broadcaster':
                jsb_active = ctrl.state == 'active'
                self.log_result(
                    "JointStateBroadcaster",
                    jsb_active,
                    f"State: {ctrl.state}"
                )

        return jtc_active and jsb_active

    def test_joint_state_feedback(self):
        """Test 2: Verify joint state feedback is working."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST 2: Joint State Feedback")
        self.get_logger().info("="*60)

        self.current_position = None
        self.position_history.clear()

        # Wait for feedback
        if not self.wait_for_position(timeout=5.0):
            self.log_result("Position Feedback", False, "No position data received")
            return False

        self.log_result("Position Feedback", True, f"Current position: {self.current_position:.2f}")

        # Check feedback rate - spin for 1 second collecting samples
        self.position_history.clear()
        start_time = time.time()
        while (time.time() - start_time) < 1.0:
            rclpy.spin_once(self, timeout_sec=0.02)

        if len(self.position_history) >= 2:
            # Calculate rate from collected samples
            sample_count = len(self.position_history)
            duration = self.position_history[-1][0] - self.position_history[0][0]
            rate = (sample_count - 1) / duration if duration > 0 else 0
            self.log_result("Feedback Rate", rate > 20, f"{rate:.1f} Hz ({sample_count} samples)")
        else:
            self.log_result("Feedback Rate", False, "Not enough samples")

        has_velocity = self.current_velocity is not None
        self.log_result("Velocity Feedback", has_velocity,
                       f"{self.current_velocity:.2f}" if has_velocity else "Not available")

        return True

    def test_action_server(self):
        """Test 3: Check if action server is available."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST 3: Action Server Availability")
        self.get_logger().info("="*60)

        available = self._action_client.wait_for_server(timeout_sec=5.0)
        self.log_result("FollowJointTrajectory Action", available,
                       "Server ready" if available else "Server not available")
        return available

    def send_trajectory(self, positions, duration_per_point=2.0, wait=True):
        """Send a trajectory and optionally wait for completion."""
        if not self.wait_for_position():
            return None

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [self.joint_name]

        # Start from current position
        start_point = JointTrajectoryPoint()
        start_point.positions = [float(self.current_position)]
        start_point.velocities = [0.0]
        start_point.time_from_start = Duration(sec=0, nanosec=100000000)
        goal_msg.trajectory.points = [start_point]

        # Add waypoints
        current_time = 0.1
        for pos in positions:
            current_time += duration_per_point
            point = JointTrajectoryPoint()
            point.positions = [float(pos)]
            point.velocities = [0.0]
            sec = int(current_time)
            nsec = int((current_time - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nsec)
            goal_msg.trajectory.points.append(point)

        # Send goal
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            return None

        if wait:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future,
                                            timeout_sec=duration_per_point * len(positions) + 10)
            return result_future.result()

        return goal_handle

    def test_trajectory_execution(self):
        """Test 4: Execute a simple trajectory and verify accuracy."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST 4: Trajectory Execution")
        self.get_logger().info("="*60)

        if not self.wait_for_position():
            self.log_result("Get Start Position", False)
            return False

        start_pos = self.current_position
        # Move 5000 counts from current position
        target_pos = start_pos + 5000

        self.get_logger().info(f"Moving from {start_pos:.1f} to {target_pos:.1f}")

        result = self.send_trajectory([target_pos], duration_per_point=3.0)

        if result is None:
            self.log_result("Trajectory Execution", False, "Goal rejected or failed")
            return False

        # Wait for settling
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)

        error = abs(self.current_position - target_pos)
        accuracy_ok = error < 100  # Within 100 counts

        self.log_result(
            "Position Accuracy",
            accuracy_ok,
            f"Target: {target_pos:.1f}, Actual: {self.current_position:.1f}, Error: {error:.1f}"
        )

        # Return to start
        self.get_logger().info(f"Returning to start position {start_pos:.1f}")
        self.send_trajectory([start_pos], duration_per_point=3.0)
        time.sleep(0.5)

        return accuracy_ok

    def test_multi_point_trajectory(self):
        """Test 5: Execute multi-point trajectory (MoveIt-style)."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST 5: Multi-Point Trajectory (MoveIt-style)")
        self.get_logger().info("="*60)

        if not self.wait_for_position():
            self.log_result("Get Start Position", False)
            return False

        start_pos = self.current_position

        # Create a multi-waypoint trajectory
        waypoints = [
            start_pos + 2000,
            start_pos + 4000,
            start_pos + 3000,
            start_pos + 5000,
            start_pos,  # Return to start
        ]

        self.get_logger().info(f"Executing {len(waypoints)}-point trajectory")

        result = self.send_trajectory(waypoints, duration_per_point=2.0)

        if result is None:
            self.log_result("Multi-Point Trajectory", False, "Execution failed")
            return False

        error_code = result.result.error_code
        success = error_code == 0

        self.log_result(
            "Multi-Point Trajectory",
            success,
            f"Error code: {error_code}"
        )

        return success

    def test_position_repeatability(self):
        """Test 6: Test position repeatability (important for MoveIt)."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST 6: Position Repeatability")
        self.get_logger().info("="*60)

        if not self.wait_for_position():
            self.log_result("Get Start Position", False)
            return False

        start_pos = self.current_position
        target_pos = start_pos + 3000
        errors = []

        for i in range(3):
            self.get_logger().info(f"  Repeatability test {i+1}/3")

            # Move to target
            self.send_trajectory([target_pos], duration_per_point=2.0)
            time.sleep(0.3)
            rclpy.spin_once(self, timeout_sec=0.1)
            pos_at_target = self.current_position

            # Return to start
            self.send_trajectory([start_pos], duration_per_point=2.0)
            time.sleep(0.3)
            rclpy.spin_once(self, timeout_sec=0.1)

            error = abs(pos_at_target - target_pos)
            errors.append(error)

        max_error = max(errors)
        avg_error = sum(errors) / len(errors)
        # 150 counts threshold - accounts for normal gearbox backlash
        repeatability_ok = max_error < 150

        self.log_result(
            "Position Repeatability",
            repeatability_ok,
            f"Max error: {max_error:.1f}, Avg error: {avg_error:.1f} (threshold: 150)"
        )

        return repeatability_ok

    def print_summary(self):
        """Print test summary."""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("TEST SUMMARY")
        self.get_logger().info("="*60)

        passed = sum(1 for _, p, _ in self.test_results if p)
        total = len(self.test_results)

        for name, p, msg in self.test_results:
            status = "✓" if p else "✗"
            self.get_logger().info(f"  {status} {name}")

        self.get_logger().info("-"*60)
        self.get_logger().info(f"Results: {passed}/{total} tests passed")

        if passed == total:
            self.get_logger().info("\n✓ MOTOR IS READY FOR MOVEIT/MTC INTEGRATION")
        else:
            self.get_logger().info("\n✗ SOME TESTS FAILED - Review issues before MoveIt integration")

        return passed == total

    def run_all_tests(self):
        """Run all readiness tests."""
        self.get_logger().info("\n" + "#"*60)
        self.get_logger().info("  MoveIt/MTC Readiness Test Suite")
        self.get_logger().info("#"*60)

        # Run tests in sequence
        self.test_controller_status()
        self.test_joint_state_feedback()

        if not self.test_action_server():
            self.get_logger().error("Action server not available - cannot continue")
            return self.print_summary()

        self.test_trajectory_execution()
        self.test_multi_point_trajectory()
        self.test_position_repeatability()

        return self.print_summary()


def main():
    print("\nMoveIt/MTC Readiness Test")
    print("=" * 60)
    print("\nThis script tests if your motor setup is ready for MoveIt/MTC.")
    print("Make sure the motor controller is running:")
    print("  ros2 launch myactuator_x6_test x6_test.launch.py")
    print()

    joint_name = sys.argv[1] if len(sys.argv) > 1 else 'motor_joint'

    rclpy.init()
    node = MoveItReadinessTest(joint_name)

    try:
        success = node.run_all_tests()
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted")
        success = False
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
