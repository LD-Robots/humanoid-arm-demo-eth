#!/usr/bin/env python3
"""
Virtual Wall Safety Node

Monitors joint positions and applies opposing torque when approaching position limits.
Acts like a virtual spring/wall at the configured limits.

Features:
    - Spring-damper virtual wall at soft limits
    - Per-joint asymmetric limits (different lower/upper)
    - Automatic recovery mode: if motor starts outside limits, actively moves it back inside
    - Monitor-only mode: logs warnings without sending torque commands

Parameters:
    - joints: List of joint names
    - lower_limits: List of lower limits (rad) per joint
    - upper_limits: List of upper limits (rad) per joint
    - soft_limit_margin: Distance from limit where soft wall starts (radians)
    - wall_stiffness: Spring constant for virtual wall (Nm/rad)
    - wall_damping: Damping coefficient (Nm/(rad/s))
    - max_wall_torque: Maximum torque the wall can apply (Nm)
    - monitor_only: If true, only log warnings without sending torque commands
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class VirtualWallNode(Node):
    def __init__(self):
        super().__init__('virtual_wall')

        # Declare parameters
        self.declare_parameter('joints', ['joint1', 'joint2'])
        self.declare_parameter('lower_limits', [-1.5708, -1.5708])
        self.declare_parameter('upper_limits', [1.5708, 1.5708])
        self.declare_parameter('soft_limit_margin', 0.1)
        self.declare_parameter('wall_stiffness', 20.0)
        self.declare_parameter('wall_damping', 0.1)
        self.declare_parameter('max_wall_torque', 3.0)
        self.declare_parameter('monitor_only', False)

        # Get parameters
        self.joints = self.get_parameter('joints').value
        lower_limits = self.get_parameter('lower_limits').value
        upper_limits = self.get_parameter('upper_limits').value
        self.soft_limit_margin = self.get_parameter('soft_limit_margin').value
        self.wall_stiffness = self.get_parameter('wall_stiffness').value
        self.wall_damping = self.get_parameter('wall_damping').value
        self.max_wall_torque = self.get_parameter('max_wall_torque').value
        self.monitor_only = self.get_parameter('monitor_only').value

        # Build per-joint limits
        self.limits = {}
        for i, joint in enumerate(self.joints):
            lower = lower_limits[i] if i < len(lower_limits) else -1.5708
            upper = upper_limits[i] if i < len(upper_limits) else 1.5708
            self.limits[joint] = {
                'lower': lower,
                'upper': upper,
                'soft_lower': lower + self.soft_limit_margin,
                'soft_upper': upper - self.soft_limit_margin
            }

        # State
        self.positions = {joint: 0.0 for joint in self.joints}
        self.velocities = {joint: 0.0 for joint in self.joints}
        self.wall_active = {joint: False for joint in self.joints}
        self.external_torque = {joint: 0.0 for joint in self.joints}

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Subscriber for external torque commands (to pass through when not at limit)
        self.external_cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/effort_commands_raw',
            self.external_cmd_callback,
            10
        )

        # Publisher for effort commands
        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )

        # Timer for control loop (100 Hz)
        self.timer = self.create_timer(0.01, self.control_loop)

        # Log configuration
        mode_str = "MONITOR ONLY" if self.monitor_only else "ACTIVE"
        self.get_logger().info(f'Virtual wall mode: {mode_str}')
        for joint, lim in self.limits.items():
            self.get_logger().info(
                f'{joint}: limits=[{lim["lower"]:.3f}, {lim["upper"]:.3f}] rad '
                f'([{lim["lower"]*180/3.14159:.1f}, {lim["upper"]*180/3.14159:.1f}] deg), '
                f'soft=[{lim["soft_lower"]:.3f}, {lim["soft_upper"]:.3f}]'
            )

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joints:
                if i < len(msg.position):
                    self.positions[name] = msg.position[i]
                if i < len(msg.velocity):
                    self.velocities[name] = msg.velocity[i]

    def external_cmd_callback(self, msg: Float64MultiArray):
        for i, joint in enumerate(self.joints):
            if i < len(msg.data):
                self.external_torque[joint] = msg.data[i]

    def compute_wall_torque(self, joint: str, position: float, velocity: float) -> float:
        """Compute virtual wall torque based on position and velocity."""
        lim = self.limits[joint]
        torque = 0.0

        # Check upper limit
        if position > lim['soft_upper']:
            penetration = position - lim['soft_upper']
            # Spring force (opposing penetration - push negative)
            torque = -self.wall_stiffness * penetration
            # Damping (opposing velocity into wall)
            if velocity > 0:
                torque -= self.wall_damping * velocity
            return max(-self.max_wall_torque, torque)

        # Check lower limit
        if position < lim['soft_lower']:
            penetration = lim['soft_lower'] - position  # positive value
            # Spring force (opposing penetration - push positive)
            torque = self.wall_stiffness * penetration
            # Damping (opposing velocity into wall)
            if velocity < 0:
                torque -= self.wall_damping * velocity
            return min(self.max_wall_torque, torque)

        return 0.0

    def control_loop(self):
        torques = []

        for joint in self.joints:
            pos = self.positions[joint]
            vel = self.velocities[joint]
            external = self.external_torque[joint]

            wall_torque = self.compute_wall_torque(joint, pos, vel)

            # Combine external command with wall torque
            if wall_torque != 0.0:
                # Wall is active - add wall torque to external command
                # but prevent external torque from pushing further into wall
                if wall_torque < 0 and external > 0:
                    # At upper limit, don't allow positive external torque
                    external = min(0.0, external)
                elif wall_torque > 0 and external < 0:
                    # At lower limit, don't allow negative external torque
                    external = max(0.0, external)

                total_torque = external + wall_torque

                if not self.wall_active[joint]:
                    self.wall_active[joint] = True
                    self.get_logger().warn(
                        f'{joint}: Wall active at pos={pos:.3f} rad, '
                        f'wall_torque={wall_torque:.2f} Nm'
                    )
            else:
                total_torque = external
                if self.wall_active[joint]:
                    self.wall_active[joint] = False
                    self.get_logger().info(f'{joint}: Wall deactivated')

            # Clamp total torque
            total_torque = max(-self.max_wall_torque, min(self.max_wall_torque, total_torque))
            torques.append(total_torque)

        # Publish (skip in monitor-only mode)
        if not self.monitor_only:
            msg = Float64MultiArray()
            msg.data = torques
            self.effort_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualWallNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
