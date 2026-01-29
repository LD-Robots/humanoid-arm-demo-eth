#!/usr/bin/env python3
"""
Compliance/Damping Control for MyActuator X6

Provides various compliant control modes:
  - free: Zero torque (motor can be moved freely)
  - damping: Viscous damping (resists velocity)
  - spring: Spring-damper (holds position with compliance)
  - teach: Low damping for hand-guiding/teaching

Requires the effort controller to be running:
  ros2 launch myactuator_x6_test x6_effort.launch.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import sys
import math


class ComplianceController(Node):
    def __init__(self, mode, **params):
        super().__init__('compliance_controller')

        self.mode = mode
        self.params = params
        self.joint_name = 'motor_joint'

        # State
        self.current_position = None
        self.current_velocity = None
        self.filtered_velocity = 0.0  # Low-pass filtered velocity
        self.home_position = None  # For spring mode
        self.prev_position = None
        self.prev_time = None

        # Velocity filter coefficient (0-1, lower = more filtering)
        self.velocity_filter_alpha = params.get('filter', 0.1)  # Heavy filtering

        # Effort publisher
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )

        # Joint state subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self._log_counter = 0

        self.get_logger().info(f'Compliance Controller started in {mode.upper()} mode')
        self.get_logger().info(f'Parameters: {params}')

    def joint_callback(self, msg):
        if self.joint_name not in msg.name:
            return

        idx = msg.name.index(self.joint_name)
        self.current_position = msg.position[idx]

        # Get velocity from message or compute from position
        if idx < len(msg.velocity) and msg.velocity[idx] != 0:
            raw_velocity = msg.velocity[idx]
        else:
            # Compute velocity from position change
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if current_time == 0:
                current_time = self.get_clock().now().nanoseconds / 1e9

            if self.prev_position is not None and self.prev_time is not None:
                dt = current_time - self.prev_time
                if dt > 0.001:
                    raw_velocity = (self.current_position - self.prev_position) / dt
                else:
                    raw_velocity = 0.0
            else:
                raw_velocity = 0.0

            self.prev_position = self.current_position
            self.prev_time = current_time

        # Low-pass filter on velocity to reduce noise
        alpha = self.velocity_filter_alpha
        self.filtered_velocity = alpha * raw_velocity + (1 - alpha) * self.filtered_velocity
        self.current_velocity = self.filtered_velocity

        # Set home position on first reading (for spring mode)
        if self.home_position is None:
            self.home_position = self.current_position
            self.get_logger().info(f'Home position set to: {self.home_position:.1f}')

        # Compute and publish effort
        effort = self.compute_effort()

        msg_out = Float64MultiArray()
        msg_out.data = [float(effort)]
        self.effort_pub.publish(msg_out)

        # Log periodically
        self._log_counter += 1
        if self._log_counter % 25 == 0:
            self.log_status(effort)

    def compute_effort(self):
        """Compute effort based on current mode."""

        # Velocity deadband - ignore tiny velocities (noise)
        velocity = self.current_velocity if abs(self.current_velocity) > 5 else 0.0

        if self.mode == 'free':
            # Zero torque - motor is free to move
            return 0.0

        elif self.mode == 'damping':
            # Viscous damping: torque = -damping * velocity
            damping = self.params.get('damping', 0.5)
            effort = -damping * velocity
            max_effort = self.params.get('max_effort', 100)
            return max(-max_effort, min(max_effort, effort))

        elif self.mode == 'spring':
            # Spring-damper: torque = -k * (pos - home) - d * velocity
            stiffness = self.params.get('stiffness', 0.01)
            damping = self.params.get('damping', 0.1)
            max_effort = self.params.get('max_effort', 200)

            pos_error = self.current_position - self.home_position

            # Position deadband - ignore tiny errors
            if abs(pos_error) < 20:
                pos_error = 0.0

            effort = -stiffness * pos_error - damping * velocity

            return max(-max_effort, min(max_effort, effort))

        elif self.mode == 'teach':
            # Teaching mode: very low damping, feels like moving through air
            damping = self.params.get('damping', 0.1)
            friction_comp = self.params.get('friction_comp', 0.0)

            # Light damping
            effort = -damping * velocity

            # Optional: friction compensation (helps overcome static friction)
            if abs(velocity) > 10:
                effort += friction_comp * (1 if velocity > 0 else -1)

            max_effort = self.params.get('max_effort', 50)
            return max(-max_effort, min(max_effort, effort))

        elif self.mode == 'hold':
            # Hold current position with high stiffness
            stiffness = self.params.get('stiffness', 0.05)
            damping = self.params.get('damping', 0.2)
            max_effort = self.params.get('max_effort', 300)

            pos_error = self.current_position - self.home_position

            # Position deadband
            if abs(pos_error) < 20:
                pos_error = 0.0

            effort = -stiffness * pos_error - damping * velocity

            return max(-max_effort, min(max_effort, effort))

        elif self.mode == 'brake':
            # Friction brake: holds position until pushed beyond threshold
            hold_effort = self.params.get('hold_effort', 100)  # Effort to resist movement
            release_threshold = self.params.get('release_threshold', 200)  # Position error to yield

            pos_error = self.current_position - self.home_position

            # If pushed beyond threshold, yield (update home to current)
            if abs(pos_error) > release_threshold:
                self.home_position = self.current_position
                return 0.0

            # Inside threshold: resist movement with constant effort toward home
            if abs(pos_error) > 20:  # Small deadband
                # Resist in direction back toward home
                if pos_error > 0:
                    return -hold_effort  # Push negative to go back
                else:
                    return hold_effort   # Push positive to go back

            return 0.0  # At home position, no effort needed

        return 0.0

    def log_status(self, effort):
        vel_str = f'{self.current_velocity:.1f}' if self.current_velocity else 'N/A'

        if self.mode in ['spring', 'hold', 'brake']:
            error = self.current_position - self.home_position
            self.get_logger().info(
                f'pos={self.current_position:.1f}, home={self.home_position:.1f}, '
                f'error={error:.1f}, vel={vel_str}, effort={effort:.1f}'
            )
        else:
            self.get_logger().info(
                f'pos={self.current_position:.1f}, vel={vel_str}, effort={effort:.1f}'
            )

    def set_home(self):
        """Set current position as new home."""
        if self.current_position is not None:
            self.home_position = self.current_position
            self.get_logger().info(f'Home position updated to: {self.home_position:.1f}')


def print_usage():
    print("""
Compliance/Damping Controller
=============================

Usage:
  ros2 run myactuator_x6_test test_compliance.py <mode> [params...]

Modes:
  free                              Zero torque - motor moves freely
  damping [d] [max]                 Viscous damping (d=1.0, max=100)
  spring [k] [d] [max]              Spring-damper hold (k=0.005, d=0.5, max=150)
  teach [d] [friction] [max]        Teaching mode - light damping (d=0.3, max=50)
  hold [k] [d] [max]                Hold position firmly (k=0.02, d=1.0, max=200)
  brake [effort] [threshold]        Friction brake - holds until pushed hard (effort=100, threshold=200)

Examples:
  # Free mode - move motor by hand
  ros2 run myactuator_x6_test test_compliance.py free

  # Light damping for smooth feel
  ros2 run myactuator_x6_test test_compliance.py damping 0.3

  # Soft spring - holds position but can be pushed
  ros2 run myactuator_x6_test test_compliance.py spring 0.005 0.1

  # Teaching mode - easy to move by hand
  ros2 run myactuator_x6_test test_compliance.py teach 0.05

  # Firm hold - resists movement
  ros2 run myactuator_x6_test test_compliance.py hold 0.1 0.3

Parameters:
  k (stiffness)   - Position gain (higher = stiffer spring)
  d (damping)     - Velocity gain (higher = more resistance to motion)
  max             - Maximum effort/torque limit
  friction        - Friction compensation (for teach mode)

IMPORTANT: Run the effort controller first:
  ros2 launch myactuator_x6_test x6_effort.launch.py

Press Ctrl+C to stop.
""")


def main():
    if len(sys.argv) < 2:
        print_usage()
        return

    mode = sys.argv[1].lower()
    params = {}

    if mode == 'free':
        pass  # No params needed

    elif mode == 'damping':
        params['damping'] = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
        params['max_effort'] = float(sys.argv[3]) if len(sys.argv) > 3 else 100

    elif mode == 'spring':
        # Higher damping ratio for stability (overdamped)
        params['stiffness'] = float(sys.argv[2]) if len(sys.argv) > 2 else 0.005
        params['damping'] = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
        params['max_effort'] = float(sys.argv[4]) if len(sys.argv) > 4 else 150

    elif mode == 'teach':
        params['damping'] = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
        params['friction_comp'] = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
        params['max_effort'] = float(sys.argv[4]) if len(sys.argv) > 4 else 50

    elif mode == 'hold':
        # Higher damping for stability
        params['stiffness'] = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02
        params['damping'] = float(sys.argv[3]) if len(sys.argv) > 3 else 1.0
        params['max_effort'] = float(sys.argv[4]) if len(sys.argv) > 4 else 200

    elif mode == 'brake':
        # Friction brake - holds position until pushed hard enough
        params['hold_effort'] = float(sys.argv[2]) if len(sys.argv) > 2 else 100
        params['release_threshold'] = float(sys.argv[3]) if len(sys.argv) > 3 else 200

    else:
        print(f"Unknown mode: {mode}")
        print_usage()
        return

    rclpy.init()
    node = ComplianceController(mode, **params)

    print("\nControls:")
    print("  Ctrl+C  - Stop and exit")
    print("  (For spring/hold modes, restart to set new home position)")
    print()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping...')

    # Send zero effort multiple times to ensure motor stops
    try:
        msg = Float64MultiArray()
        msg.data = [0.0]
        for _ in range(10):
            node.effort_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info('Sent zero effort - motor should stop')
    except Exception as e:
        print(f'Warning: Could not send stop command: {e}')

    # Clean shutdown
    try:
        node.destroy_node()
    except:
        pass

    try:
        if rclpy.ok():
            rclpy.shutdown()
    except:
        pass


if __name__ == '__main__':
    main()
