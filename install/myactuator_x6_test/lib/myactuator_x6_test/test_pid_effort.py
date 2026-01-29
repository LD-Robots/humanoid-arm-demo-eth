#!/usr/bin/env python3
"""
PID position control using effort (torque) commands.
Allows tuning of PID gains for smooth motor control.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import sys


class PIDController:
    def __init__(self, kp, ki, kd, output_limit=500):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_position = None  # For derivative on measurement
        self.prev_time = None

    def compute(self, setpoint, measurement, current_time):
        error = setpoint - measurement

        # Initialize on first call
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_position = measurement
            return 0.0, error, 0.0, 0.0, 0.0

        dt = current_time - self.prev_time
        if dt <= 0.001:  # Skip if dt too small (< 1ms)
            return None

        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup
        self.integral += error * dt
        # Clamp integral to prevent windup
        max_integral = self.output_limit / max(self.ki, 0.0001)
        self.integral = max(-max_integral, min(max_integral, self.integral))
        i_term = self.ki * self.integral

        # Derivative term on MEASUREMENT (not error) - prevents derivative kick
        d_measurement = (measurement - self.prev_position) / dt
        d_term = -self.kd * d_measurement  # Negative because we want to resist velocity

        self.prev_position = measurement
        self.prev_time = current_time

        # Total output with limiting
        output = p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))

        return output, error, p_term, i_term, d_term

    def reset(self):
        self.integral = 0.0
        self.prev_position = None
        self.prev_time = None


class PIDEffortController(Node):
    def __init__(self, target_position, kp, ki, kd, max_effort):
        super().__init__('pid_effort_controller')

        self.target_position = target_position
        self.current_position = None
        self.last_position_time = None
        self.pid = PIDController(kp, ki, kd, max_effort)

        # Publisher for effort commands
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )

        # Subscriber for joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self._log_counter = 0

        self.get_logger().info(
            f'PID Effort Controller started\n'
            f'  Target: {target_position}\n'
            f'  Kp={kp}, Ki={ki}, Kd={kd}\n'
            f'  Max effort: {max_effort}'
        )

    def joint_state_callback(self, msg):
        """Process joint state and compute control output."""
        if 'motor_joint' not in msg.name:
            return

        idx = msg.name.index('motor_joint')
        self.current_position = msg.position[idx]

        # Use message timestamp for accurate dt calculation
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if current_time == 0:  # Fallback if no timestamp
            current_time = self.get_clock().now().nanoseconds / 1e9

        # Compute PID
        result = self.pid.compute(self.target_position, self.current_position, current_time)

        if result is None:
            return  # Skip if dt too small

        effort, error, p, i, d = result

        # Publish effort command
        msg_out = Float64MultiArray()
        msg_out.data = [float(effort)]
        self.effort_pub.publish(msg_out)

        # Log periodically (every ~0.5s at 50Hz)
        self._log_counter += 1
        if self._log_counter % 25 == 0:
            self.get_logger().info(
                f'pos={self.current_position:.1f}, target={self.target_position:.1f}, '
                f'err={error:.1f}, effort={effort:.1f} (P={p:.1f}, I={i:.1f}, D={d:.1f})'
            )


def main():
    print("PID Effort Position Controller")
    print("=" * 50)
    print()
    print("Usage: ros2 run myactuator_x6_test test_pid_effort.py <position> [kp] [ki] [kd] [max_effort]")
    print()
    print("Arguments:")
    print("  position    - Target position in encoder counts")
    print("  kp          - Proportional gain (default: 0.01)")
    print("  ki          - Integral gain (default: 0.001)")
    print("  kd          - Derivative gain (default: 0.1)")
    print("  max_effort  - Maximum torque command (default: 100)")
    print()
    print("Examples:")
    print("  ros2 run myactuator_x6_test test_pid_effort.py 5000")
    print("  ros2 run myactuator_x6_test test_pid_effort.py 5000 0.01 0.001 0.1")
    print("  ros2 run myactuator_x6_test test_pid_effort.py 5000 0.02 0.0 0.2 150")
    print()
    print("Tips for tuning:")
    print("  - Start with VERY low gains: Kp=0.005, Ki=0, Kd=0")
    print("  - Increase Kp slowly until motor moves to target")
    print("  - Add Kd to dampen oscillations (acts like friction)")
    print("  - Add small Ki only if there's steady-state error")
    print()

    if len(sys.argv) < 2:
        print("Error: Please provide target position")
        return

    rclpy.init()

    target = float(sys.argv[1])
    kp = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
    ki = float(sys.argv[3]) if len(sys.argv) > 3 else 0.001
    kd = float(sys.argv[4]) if len(sys.argv) > 4 else 0.1
    max_effort = float(sys.argv[5]) if len(sys.argv) > 5 else 100.0

    node = PIDEffortController(target, kp, ki, kd, max_effort)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero effort on exit
        try:
            msg = Float64MultiArray()
            msg.data = [0.0]
            node.effort_pub.publish(msg)
            node.get_logger().info('Stopping - sent zero effort')
        except:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
