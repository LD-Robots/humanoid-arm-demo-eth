#!/usr/bin/env python3
"""
DualSense Controller for MoveIt Servo

Maps PS5 DualSense controller to MoveIt Servo commands for arm teleoperation.

Controller mapping:
- Left Stick X/Y: Translate X/Y (Cartesian space)
- Right Stick Y: Translate Z (up/down)
- Right Stick X: Rotate around Z (yaw)
- L2 Trigger: Rotate around Y (pitch)
- R2 Trigger: Rotate around X (roll)
- Cross (X): Switch to Joint control mode
- Circle: Switch to Cartesian control mode
- Triangle: Reset/Stop motion
- Square: Reduce speed
- L1: Increase speed
- R1: Decrease speed
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
import math


class DualSenseServo(Node):
    def __init__(self):
        super().__init__('dualsense_servo')

        # Publishers
        self.twist_pub = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        self.joint_pub = self.create_publisher(
            JointJog,
            '/servo_node/delta_joint_cmds',
            10
        )

        # Subscriber
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # State
        self.control_mode = 'cartesian'  # 'cartesian' or 'joint'
        self.speed_scale = 1.0
        self.deadzone = 0.1

        # DualSense button mapping
        self.BUTTON_CROSS = 0
        self.BUTTON_CIRCLE = 1
        self.BUTTON_TRIANGLE = 2
        self.BUTTON_SQUARE = 3
        self.BUTTON_L1 = 4
        self.BUTTON_R1 = 5
        self.BUTTON_L2 = 6
        self.BUTTON_R2 = 7

        # DualSense axis mapping
        self.AXIS_LEFT_X = 0
        self.AXIS_LEFT_Y = 1
        self.AXIS_RIGHT_X = 2
        self.AXIS_RIGHT_Y = 3
        self.AXIS_L2 = 4
        self.AXIS_R2 = 5

        self.get_logger().info('DualSense Servo node started')
        self.get_logger().info('Mode: CARTESIAN (press X for Joint mode)')

    def apply_deadzone(self, value, deadzone=0.1):
        """Apply deadzone to joystick input"""
        if abs(value) < deadzone:
            return 0.0
        # Scale to maintain smooth transition
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - deadzone) / (1.0 - deadzone)

    def joy_callback(self, msg):
        # Handle mode switching
        if len(msg.buttons) > self.BUTTON_CROSS and msg.buttons[self.BUTTON_CROSS]:
            if self.control_mode != 'joint':
                self.control_mode = 'joint'
                self.get_logger().info('Switched to JOINT control mode')

        if len(msg.buttons) > self.BUTTON_CIRCLE and msg.buttons[self.BUTTON_CIRCLE]:
            if self.control_mode != 'cartesian':
                self.control_mode = 'cartesian'
                self.get_logger().info('Switched to CARTESIAN control mode')

        # Handle speed scaling
        if len(msg.buttons) > self.BUTTON_L1 and msg.buttons[self.BUTTON_L1]:
            self.speed_scale = min(1.5, self.speed_scale + 0.1)
            self.get_logger().info(f'Speed scale: {self.speed_scale:.1f}x')

        if len(msg.buttons) > self.BUTTON_R1 and msg.buttons[self.BUTTON_R1]:
            self.speed_scale = max(0.1, self.speed_scale - 0.1)
            self.get_logger().info(f'Speed scale: {self.speed_scale:.1f}x')

        # Emergency stop
        if len(msg.buttons) > self.BUTTON_TRIANGLE and msg.buttons[self.BUTTON_TRIANGLE]:
            self.publish_zero_command()
            return

        # Publish commands based on mode
        if self.control_mode == 'cartesian':
            self.publish_cartesian_command(msg)
        else:
            self.publish_joint_command(msg)

    def publish_cartesian_command(self, joy_msg):
        """Publish Cartesian twist command"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'base_link'

        # Linear motion from left stick and right stick Y
        if len(joy_msg.axes) > self.AXIS_LEFT_X:
            twist.twist.linear.x = self.apply_deadzone(joy_msg.axes[self.AXIS_LEFT_Y]) * self.speed_scale
            twist.twist.linear.y = -self.apply_deadzone(joy_msg.axes[self.AXIS_LEFT_X]) * self.speed_scale

        if len(joy_msg.axes) > self.AXIS_RIGHT_Y:
            twist.twist.linear.z = self.apply_deadzone(joy_msg.axes[self.AXIS_RIGHT_Y]) * self.speed_scale

        # Rotational motion
        if len(joy_msg.axes) > self.AXIS_RIGHT_X:
            # Yaw from right stick X
            twist.twist.angular.z = -self.apply_deadzone(joy_msg.axes[self.AXIS_RIGHT_X]) * self.speed_scale

        # Pitch and Roll from L2/R2 triggers (range -1 to 1, at rest = -1)
        if len(joy_msg.axes) > self.AXIS_L2:
            # Convert trigger range from [-1, 1] to [0, 1]
            l2_val = (1.0 - joy_msg.axes[self.AXIS_L2]) / 2.0
            twist.twist.angular.y = self.apply_deadzone(l2_val, 0.05) * self.speed_scale

        if len(joy_msg.axes) > self.AXIS_R2:
            r2_val = (1.0 - joy_msg.axes[self.AXIS_R2]) / 2.0
            twist.twist.angular.x = self.apply_deadzone(r2_val, 0.05) * self.speed_scale

        self.twist_pub.publish(twist)

    def publish_joint_command(self, joy_msg):
        """Publish joint jog command"""
        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = 'base_link'
        jog.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        jog.velocities = [0.0] * 5

        # Map joystick to individual joints
        if len(joy_msg.axes) > self.AXIS_LEFT_X:
            # Left stick controls joint1 and joint2
            jog.velocities[0] = -self.apply_deadzone(joy_msg.axes[self.AXIS_LEFT_X]) * self.speed_scale
            jog.velocities[1] = self.apply_deadzone(joy_msg.axes[self.AXIS_LEFT_Y]) * self.speed_scale

        if len(joy_msg.axes) > self.AXIS_RIGHT_Y:
            # Right stick controls joint3 and joint4
            jog.velocities[2] = self.apply_deadzone(joy_msg.axes[self.AXIS_RIGHT_X]) * self.speed_scale
            jog.velocities[3] = self.apply_deadzone(joy_msg.axes[self.AXIS_RIGHT_Y]) * self.speed_scale

        # L2/R2 control joint5
        if len(joy_msg.axes) > self.AXIS_L2:
            l2_val = (1.0 - joy_msg.axes[self.AXIS_L2]) / 2.0
            jog.velocities[4] -= self.apply_deadzone(l2_val, 0.05) * self.speed_scale

        if len(joy_msg.axes) > self.AXIS_R2:
            r2_val = (1.0 - joy_msg.axes[self.AXIS_R2]) / 2.0
            jog.velocities[4] += self.apply_deadzone(r2_val, 0.05) * self.speed_scale

        self.joint_pub.publish(jog)

    def publish_zero_command(self):
        """Publish zero velocity to stop motion"""
        if self.control_mode == 'cartesian':
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = 'base_link'
            self.twist_pub.publish(twist)
        else:
            jog = JointJog()
            jog.header.stamp = self.get_clock().now().to_msg()
            jog.header.frame_id = 'base_link'
            jog.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
            jog.velocities = [0.0] * 5
            self.joint_pub.publish(jog)


def main(args=None):
    rclpy.init(args=args)
    node = DualSenseServo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
