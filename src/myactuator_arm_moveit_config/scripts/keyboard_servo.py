#!/usr/bin/env python3
"""
Keyboard Teleop for MoveIt Servo

Control the robot arm using keyboard input with support for:
- Cartesian twist commands (move end-effector)
- Joint jog commands (move individual joints)
- Pose tracking (move to saved poses)

Controls:
    CARTESIAN MODE (default):
        W/S: Move forward/backward (X)
        A/D: Move left/right (Y)
        Q/E: Move up/down (Z)
        I/K: Rotate pitch (around Y)
        J/L: Rotate yaw (around Z)
        U/O: Rotate roll (around X)

    JOINT MODE:
        1-6: Select joint (1=shoulder_pitch, 2=shoulder_roll, etc.)
        W/S: Rotate selected joint +/-

    POSE MODE:
        H: Go to home pose
        R: Go to ready pose
        G: Save current pose as goal
        T: Track saved goal pose

    MODE SWITCHING:
        C: Switch to Cartesian mode
        V: Switch to Joint mode
        P: Switch to Pose mode

    SPEED/FRAME:
        +/-: Increase/decrease speed
        F: Toggle command frame (base_link / end_effector_link)

    EXIT:
        Ctrl+C or 'x'

Usage:
    ros2 run myactuator_arm_moveit_config keyboard_servo.py
"""
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from tf2_ros import TransformListener, Buffer


class KeyboardServo(Node):
    def __init__(self):
        super().__init__('keyboard_servo')

        # Publishers for different command types
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
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/servo_node/pose_target_cmds',
            10
        )

        # TF2 for getting current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Service client to switch servo command type
        self.switch_cmd_type_client = self.create_client(
            ServoCommandType,
            '/servo_node/switch_command_type'
        )

        # State
        self.mode = 'cartesian'  # 'cartesian', 'joint', or 'pose'
        self.selected_joint = 0  # 0-5 for shoulder_pitch through wrist_roll
        self.speed = 0.5  # Velocity scale
        self.command_frame = 'table'  # or 'urdf_l_wrist_assembly'
        self.joint_names = ['shoulder_pitch_joint', 'shoulder_roll_joint', 'shoulder_yaw_joint',
                            'elbow_pitch_joint', 'elbow_yaw_joint', 'wrist_roll_joint']

        # Predefined poses (in base_link frame)
        self.poses = {
            'home': {'position': [0.0, 0.0, 0.5], 'orientation': [0.0, 0.0, 0.0, 1.0]},
            'ready': {'position': [0.3, 0.0, 0.4], 'orientation': [0.0, 0.0, 0.0, 1.0]},
            'goal': None,  # User-saved goal
        }

        # Current command state
        self.linear = [0.0, 0.0, 0.0]  # x, y, z
        self.angular = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.joint_velocities = [0.0] * 6
        self.target_pose = None  # For pose tracking

        # Terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)

        # Command type mapping
        self.CMD_TYPE_JOINT_JOG = 0
        self.CMD_TYPE_TWIST = 1
        self.CMD_TYPE_POSE = 2

        # Flag to prevent publishing before command type is set
        self.command_type_ready = False

        # Wait for service and set initial command type
        self.get_logger().info('Waiting for servo_node switch_command_type service...')
        if self.switch_cmd_type_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Service available, setting initial command type to TWIST')
            self.switch_command_type(self.CMD_TYPE_TWIST)
        else:
            self.get_logger().warn('Service not available - servo commands may not work')

        # Timer for publishing commands (created AFTER command type is set)
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

        self.print_instructions()

    def switch_command_type(self, cmd_type):
        """Switch servo command type via service call."""
        request = ServoCommandType.Request()
        request.command_type = cmd_type
        future = self.switch_cmd_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is not None:
            if future.result().success:
                type_names = {0: 'JOINT_JOG', 1: 'TWIST', 2: 'POSE'}
                self.get_logger().info(f'Switched to {type_names.get(cmd_type, "UNKNOWN")} command type')
                self.command_type_ready = True
            else:
                self.get_logger().warn('Failed to switch command type')
                self.command_type_ready = False
        else:
            self.get_logger().warn('Service call timed out')
            self.command_type_ready = False

    def print_instructions(self):
        msg = """
══════════════════════════════════════════════════════════════
              MoveIt Servo Keyboard Teleop
══════════════════════════════════════════════════════════════
 Mode: {:^10} | Speed: {:.2f} | Frame: {:^20}
 Selected Joint: {} ({})
══════════════════════════════════════════════════════════════
 CARTESIAN:  W/S=X  A/D=Y  Q/E=Z  |  I/K=Pitch J/L=Yaw U/O=Roll
 JOINT:      1-6=Select  W/S=Rotate
 POSE:       H=Home  R=Ready  G=SaveGoal  T=TrackGoal
══════════════════════════════════════════════════════════════
 C=Cartesian  V=Joint  P=Pose  |  +/-=Speed  F=Frame  X=Exit
══════════════════════════════════════════════════════════════
""".format(
            self.mode.upper(),
            self.speed,
            self.command_frame,
            self.selected_joint + 1,
            self.joint_names[self.selected_joint]
        )
        self.get_logger().info(msg)

    def get_key(self, timeout=0.02):
        """Get keyboard input with timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key

    def get_current_ee_pose(self):
        """Get current end-effector pose from TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'table',
                'urdf_l_wrist_assembly',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return {
                'position': [
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ],
                'orientation': [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                ]
            }
        except Exception as e:
            self.get_logger().warn(f'Could not get EE pose: {e}')
            return None

    def process_key(self, key):
        """Process keyboard input and update command state."""
        key = key.lower()

        # Reset motion commands (they should be set fresh each iteration)
        self.linear = [0.0, 0.0, 0.0]
        self.angular = [0.0, 0.0, 0.0]
        self.joint_velocities = [0.0] * 6

        # Mode switching
        if key == 'c':
            self.mode = 'cartesian'
            self.target_pose = None
            self.switch_command_type(self.CMD_TYPE_TWIST)
            return True
        elif key == 'v':
            self.mode = 'joint'
            self.target_pose = None
            self.switch_command_type(self.CMD_TYPE_JOINT_JOG)
            return True
        elif key == 'p':
            self.mode = 'pose'
            self.switch_command_type(self.CMD_TYPE_POSE)
            self.get_logger().info('POSE mode - press H/R/T for poses')
            return True

        # Frame toggle
        elif key == 'f':
            if self.command_frame == 'table':
                self.command_frame = 'urdf_l_wrist_assembly'
            else:
                self.command_frame = 'table'
            self.get_logger().info(f'Command frame: {self.command_frame}')
            return True

        # Speed adjustment
        elif key == '+' or key == '=':
            self.speed = min(2.0, self.speed + 0.1)
            self.get_logger().info(f'Speed: {self.speed:.2f}')
            return True
        elif key == '-':
            self.speed = max(0.1, self.speed - 0.1)
            self.get_logger().info(f'Speed: {self.speed:.2f}')
            return True

        # Joint selection (in joint mode)
        elif key in '123456':
            self.selected_joint = int(key) - 1
            self.get_logger().info(f'Selected: {self.joint_names[self.selected_joint]}')
            return True

        # Pose commands
        elif key == 'h':  # Home pose
            self.target_pose = self.poses['home']
            self.mode = 'pose'
            self.switch_command_type(self.CMD_TYPE_POSE)
            self.get_logger().info('Tracking HOME pose')
            return True
        elif key == 'r':  # Ready pose
            self.target_pose = self.poses['ready']
            self.mode = 'pose'
            self.switch_command_type(self.CMD_TYPE_POSE)
            self.get_logger().info('Tracking READY pose')
            return True
        elif key == 'g':  # Save current pose as goal
            pose = self.get_current_ee_pose()
            if pose:
                self.poses['goal'] = pose
                self.get_logger().info(f'Saved goal pose: {pose["position"]}')
            return True
        elif key == 't':  # Track saved goal
            if self.poses['goal']:
                self.target_pose = self.poses['goal']
                self.mode = 'pose'
                self.switch_command_type(self.CMD_TYPE_POSE)
                self.get_logger().info('Tracking saved GOAL pose')
            else:
                self.get_logger().warn('No goal pose saved! Press G first.')
            return True

        # Exit
        elif key == 'x':
            return False

        # Movement commands
        if self.mode == 'cartesian':
            # Linear motion
            if key == 'w':
                self.linear[0] = self.speed
            elif key == 's':
                self.linear[0] = -self.speed
            elif key == 'a':
                self.linear[1] = self.speed
            elif key == 'd':
                self.linear[1] = -self.speed
            elif key == 'q':
                self.linear[2] = self.speed
            elif key == 'e':
                self.linear[2] = -self.speed
            # Angular motion
            elif key == 'u':
                self.angular[0] = self.speed
            elif key == 'o':
                self.angular[0] = -self.speed
            elif key == 'i':
                self.angular[1] = self.speed
            elif key == 'k':
                self.angular[1] = -self.speed
            elif key == 'j':
                self.angular[2] = self.speed
            elif key == 'l':
                self.angular[2] = -self.speed

        elif self.mode == 'joint':
            # Joint motion for selected joint
            if key == 'w':
                self.joint_velocities[self.selected_joint] = self.speed
            elif key == 's':
                self.joint_velocities[self.selected_joint] = -self.speed

        return True

    def timer_callback(self):
        """Publish commands at regular interval."""
        if not self.command_type_ready:
            return  # Don't publish until command type is set
        if self.mode == 'cartesian':
            self.publish_twist()
        elif self.mode == 'joint':
            self.publish_joint_jog()
        elif self.mode == 'pose' and self.target_pose:
            self.publish_pose()

    def publish_twist(self):
        """Publish Cartesian twist command."""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.command_frame
        twist.twist.linear.x = self.linear[0]
        twist.twist.linear.y = self.linear[1]
        twist.twist.linear.z = self.linear[2]
        twist.twist.angular.x = self.angular[0]
        twist.twist.angular.y = self.angular[1]
        twist.twist.angular.z = self.angular[2]
        self.twist_pub.publish(twist)

    def publish_joint_jog(self):
        """Publish joint jog command."""
        jog = JointJog()
        jog.header.stamp = self.get_clock().now().to_msg()
        jog.header.frame_id = self.command_frame
        jog.joint_names = self.joint_names
        jog.velocities = self.joint_velocities
        self.joint_pub.publish(jog)

    def publish_pose(self):
        """Publish pose tracking command."""
        if not self.target_pose:
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'table'
        pose.pose.position.x = self.target_pose['position'][0]
        pose.pose.position.y = self.target_pose['position'][1]
        pose.pose.position.z = self.target_pose['position'][2]
        pose.pose.orientation.x = self.target_pose['orientation'][0]
        pose.pose.orientation.y = self.target_pose['orientation'][1]
        pose.pose.orientation.z = self.target_pose['orientation'][2]
        pose.pose.orientation.w = self.target_pose['orientation'][3]
        self.pose_pub.publish(pose)

    def run(self):
        """Main loop."""
        try:
            while rclpy.ok():
                key = self.get_key(0.02)  # 50 Hz
                if key:
                    if not self.process_key(key):
                        break
                rclpy.spin_once(self, timeout_sec=0)
        except KeyboardInterrupt:
            pass
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            # Send zero commands to stop
            self.linear = [0.0, 0.0, 0.0]
            self.angular = [0.0, 0.0, 0.0]
            self.joint_velocities = [0.0] * 6
            self.target_pose = None
            self.publish_twist()
            self.publish_joint_jog()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardServo()

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
