#!/usr/bin/env python3
"""
PlayStation Controller Servo Control for MoveIt Servo

Maps PlayStation controller inputs to MoveIt Servo commands for real-time arm control.

Controller Mapping:
    LEFT STICK:
        X-axis: Move left/right (Y translation)
        Y-axis: Move forward/backward (X translation)

    RIGHT STICK:
        X-axis: Rotate yaw (Z rotation)
        Y-axis: Rotate pitch (Y rotation)

    TRIGGERS:
        L2: Move down (Z translation)
        R2: Move up (Z translation)

    BUMPERS:
        L1: Rotate roll negative (X rotation)
        R1: Rotate roll positive (X rotation)

    BUTTONS:
        Cross (X): Switch to Cartesian mode
        Circle (O): Switch to Joint mode
        Triangle: Increase speed
        Square: Decrease speed
        Share: Toggle reference frame (table/end-effector)
        Options: Exit

    D-PAD (in Joint mode):
        Up: Select next joint
        Down: Select previous joint
        Left: Rotate selected joint negative
        Right: Rotate selected joint positive

Requirements:
    sudo apt install ros-humble-joy
    ros2 run joy joy_node
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
import threading


class JoystickServoNode(Node):
    # Command types
    CMD_TYPE_TWIST = 1
    CMD_TYPE_JOINT = 2

    # PS4/PS5 Controller mapping (standard)
    AXIS_LEFT_X = 0
    AXIS_LEFT_Y = 1
    AXIS_L2 = 2
    AXIS_RIGHT_X = 3
    AXIS_RIGHT_Y = 4
    AXIS_R2 = 5

    BTN_CROSS = 0
    BTN_CIRCLE = 1
    BTN_TRIANGLE = 2
    BTN_SQUARE = 3
    BTN_L1 = 4
    BTN_R1 = 5
    BTN_SHARE = 8
    BTN_OPTIONS = 9
    BTN_DPAD_UP = 13
    BTN_DPAD_DOWN = 14
    BTN_DPAD_LEFT = 15
    BTN_DPAD_RIGHT = 16

    def __init__(self):
        super().__init__('joystick_servo_node')

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

        # Service client
        self.switch_cmd_type_client = self.create_client(
            ServoCommandType,
            '/servo_node/switch_command_type'
        )

        # State
        self.mode = 'cartesian'  # 'cartesian' or 'joint'
        self.selected_joint = 0  # 0-5
        self.speed = 0.5  # Speed multiplier
        self.command_frame = 'table'  # or 'urdf_l_wrist_assembly'
        self.joint_names = [
            'shoulder_pitch_joint',
            'shoulder_roll_joint',
            'shoulder_yaw_joint',
            'elbow_pitch_joint',
            'elbow_yaw_joint',
            'wrist_roll_joint'
        ]

        # Button state tracking (for edge detection)
        self.prev_buttons = []

        # Deadzone for joystick axes
        self.deadzone = 0.1

        # Timer for publishing commands at fixed rate
        self.timer = self.create_timer(0.02, self.publish_commands)  # 50 Hz

        # Current joy state
        self.current_joy = None
        self.command_type_ready = False

        self.get_logger().info('Joystick Servo Node started')
        self.get_logger().info('Waiting for /joy messages...')
        self.get_logger().info('Make sure to run: ros2 run joy joy_node')

        # Switch to cartesian mode by default
        self.switch_command_type(self.CMD_TYPE_TWIST)
        self.print_instructions()

    def apply_deadzone(self, value):
        """Apply deadzone to joystick axis value."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale to maintain smooth transition after deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)

    def joy_callback(self, msg):
        """Process joystick input and update state."""
        self.current_joy = msg

        # Ensure prev_buttons is initialized
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(msg.buttons)
            return

        # Detect button presses (rising edge)
        for i in range(min(len(msg.buttons), len(self.prev_buttons))):
            if msg.buttons[i] == 1 and self.prev_buttons[i] == 0:
                self.handle_button_press(i)

        # Update button state
        self.prev_buttons = list(msg.buttons)

    def print_mode_change(self, mode_name):
        """Print prominent mode change message."""
        banner = "=" * 60
        msg = f"\n{banner}\n   MODE CHANGED: {mode_name}\n{banner}"
        self.get_logger().info(msg)

    def handle_button_press(self, button):
        """Handle button press events."""
        if button == self.BTN_CROSS:
            # Switch to Cartesian mode
            self.mode = 'cartesian'
            self.switch_command_type(self.CMD_TYPE_TWIST)
            self.print_mode_change('CARTESIAN MODE ACTIVE')
            self.print_instructions()

        elif button == self.BTN_CIRCLE:
            # Switch to Joint mode
            self.mode = 'joint'
            self.switch_command_type(self.CMD_TYPE_JOINT)
            self.print_mode_change('JOINT MODE ACTIVE')
            self.print_instructions()

        elif button == self.BTN_TRIANGLE:
            # Increase speed
            self.speed = min(2.0, self.speed + 0.1)
            self.get_logger().info(f'Speed: {self.speed:.2f}')

        elif button == self.BTN_SQUARE:
            # Decrease speed
            self.speed = max(0.1, self.speed - 0.1)
            self.get_logger().info(f'Speed: {self.speed:.2f}')

        elif button == self.BTN_SHARE:
            # Toggle reference frame
            if self.command_frame == 'table':
                self.command_frame = 'urdf_l_wrist_assembly'
            else:
                self.command_frame = 'table'
            self.get_logger().info(f'Reference frame: {self.command_frame}')

        elif button == self.BTN_OPTIONS:
            # Exit
            self.get_logger().info('Exiting...')
            rclpy.shutdown()

        # Joint selection (D-pad)
        elif button == self.BTN_DPAD_UP:
            self.selected_joint = (self.selected_joint + 1) % 6
            self.get_logger().info(
                f'Selected joint {self.selected_joint + 1}: '
                f'{self.joint_names[self.selected_joint]}'
            )

        elif button == self.BTN_DPAD_DOWN:
            self.selected_joint = (self.selected_joint - 1) % 6
            self.get_logger().info(
                f'Selected joint {self.selected_joint + 1}: '
                f'{self.joint_names[self.selected_joint]}'
            )

    def publish_commands(self):
        """Publish servo commands based on joystick state."""
        if self.current_joy is None or not self.command_type_ready:
            return

        if self.mode == 'cartesian':
            self.publish_twist()
        elif self.mode == 'joint':
            self.publish_joint()

    def publish_twist(self):
        """Publish Cartesian twist command."""
        if self.current_joy is None:
            return

        joy = self.current_joy

        # Apply deadzone to all axes
        left_x = self.apply_deadzone(joy.axes[self.AXIS_LEFT_X])
        left_y = self.apply_deadzone(joy.axes[self.AXIS_LEFT_Y])
        right_x = self.apply_deadzone(joy.axes[self.AXIS_RIGHT_X])
        right_y = self.apply_deadzone(joy.axes[self.AXIS_RIGHT_Y])

        # L2/R2 triggers (typically 1.0 at rest, -1.0 when pressed)
        l2 = (1.0 - joy.axes[self.AXIS_L2]) / 2.0  # Convert to 0-1 range
        r2 = (1.0 - joy.axes[self.AXIS_R2]) / 2.0

        # L1/R1 bumpers for roll
        l1_pressed = joy.buttons[self.BTN_L1] if len(joy.buttons) > self.BTN_L1 else 0
        r1_pressed = joy.buttons[self.BTN_R1] if len(joy.buttons) > self.BTN_R1 else 0

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame

        # Linear motion (left stick + triggers)
        twist.twist.linear.x = -left_y * self.speed * 0.3  # Forward/back (inverted)
        twist.twist.linear.y = left_x * self.speed * 0.3   # Left/right
        twist.twist.linear.z = (r2 - l2) * self.speed * 0.3  # Up/down

        # Angular motion (right stick + bumpers)
        twist.twist.angular.x = (r1_pressed - l1_pressed) * self.speed * 0.5  # Roll
        twist.twist.angular.y = -right_y * self.speed * 0.5  # Pitch (inverted)
        twist.twist.angular.z = -right_x * self.speed * 0.5  # Yaw (inverted)

        self.twist_pub.publish(twist)

    def publish_joint(self):
        """Publish joint jog command."""
        if self.current_joy is None:
            return

        joy = self.current_joy

        # Use D-pad left/right for joint control
        dpad_left = joy.buttons[self.BTN_DPAD_LEFT] if len(joy.buttons) > self.BTN_DPAD_LEFT else 0
        dpad_right = joy.buttons[self.BTN_DPAD_RIGHT] if len(joy.buttons) > self.BTN_DPAD_RIGHT else 0

        joint_delta = (dpad_right - dpad_left) * self.speed * 0.5

        if abs(joint_delta) < 0.01:
            return

        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = 'table'
        jog.joint_names = self.joint_names
        jog.velocities = [0.0] * 6
        jog.velocities[self.selected_joint] = joint_delta

        self.joint_pub.publish(jog)

    def switch_command_type(self, cmd_type):
        """Switch servo command type."""
        if not self.switch_cmd_type_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Servo command type service not available')
            return

        request = ServoCommandType.Request()
        request.command_type = cmd_type

        future = self.switch_cmd_type_client.call_async(request)
        future.add_done_callback(self.command_type_callback)

    def command_type_callback(self, future):
        """Handle command type switch response."""
        try:
            response = future.result()
            if response.success:
                self.command_type_ready = True
            else:
                self.get_logger().warn('Failed to switch command type')
                self.command_type_ready = False
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            self.command_type_ready = False

    def print_instructions(self):
        """Print control instructions."""
        msg = """
══════════════════════════════════════════════════════════════
          PlayStation Controller Servo Control
══════════════════════════════════════════════════════════════
 Mode: {:^10} | Speed: {:.2f} | Frame: {:^20}
 Selected Joint: {} ({})
══════════════════════════════════════════════════════════════
 CARTESIAN MODE:
   Left Stick:  Forward/Back, Left/Right
   Right Stick: Yaw, Pitch
   L2/R2:       Down/Up
   L1/R1:       Roll Left/Right

 JOINT MODE:
   D-Pad Up/Down:    Select Joint
   D-Pad Left/Right: Rotate Joint

 CONTROLS:
   Cross (X):  Cartesian Mode
   Circle (O): Joint Mode
   Triangle:   Increase Speed
   Square:     Decrease Speed
   Share:      Toggle Frame
   Options:    Exit
══════════════════════════════════════════════════════════════
""".format(
            self.mode.upper(),
            self.speed,
            self.command_frame,
            self.selected_joint + 1,
            self.joint_names[self.selected_joint]
        )
        self.get_logger().info(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickServoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
