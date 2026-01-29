#!/usr/bin/env python3
"""
Set Zero Position Script

Reads current joint position and calculates the offset needed to make it zero.
Outputs the offset value to add to the config file.

Usage:
    ros2 run myactuator_x6_test set_zero.py motor0_joint
    ros2 run myactuator_x6_test set_zero.py motor1_joint
    ros2 run myactuator_x6_test set_zero.py --all
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SetZeroNode(Node):
    def __init__(self, target_joints):
        super().__init__('set_zero')
        self.target_joints = target_joints
        self.positions = {}
        self.received = False

        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

        # Timeout after 3 seconds
        self.timer = self.create_timer(3.0, self.timeout)

    def callback(self, msg):
        if self.received:
            return

        for i, name in enumerate(msg.name):
            if name in self.target_joints and i < len(msg.position):
                self.positions[name] = msg.position[i]

        # Check if we have all joints
        if all(j in self.positions for j in self.target_joints):
            self.received = True
            self.print_results()
            rclpy.shutdown()

    def timeout(self):
        if not self.received:
            self.get_logger().error('Timeout waiting for /joint_states')
            self.get_logger().info('Make sure the robot is running (ros2 launch myactuator_x6_test x6_free.launch.py)')
            rclpy.shutdown()

    def print_results(self):
        print("\n" + "="*60)
        print("ZERO POSITION OFFSETS")
        print("="*60)

        for joint, position in self.positions.items():
            offset = -position  # Offset to make current position = 0
            print(f"\n{joint}:")
            print(f"  Current position: {position:.6f} rad ({position * 180/3.14159:.2f} deg)")
            print(f"  Offset to set:    {offset:.6f} rad ({offset * 180/3.14159:.2f} deg)")
            print(f"\n  Add to config YAML (tpdo position channel):")
            print(f"    offset: {offset:.6f}")

        print("\n" + "="*60)
        print("Copy the offset value to your motor config file:")
        print("  config/myactuator_x6.yaml (or motor-specific config)")
        print("  In tpdo -> channels -> position line, add: offset: <value>")
        print("  Then rebuild: colcon build --packages-select myactuator_x6_test")
        print("="*60 + "\n")


def main():
    args = sys.argv[1:]

    if not args:
        print("Usage:")
        print("  ros2 run myactuator_x6_test set_zero.py <joint_name>")
        print("  ros2 run myactuator_x6_test set_zero.py --all")
        print("\nExamples:")
        print("  ros2 run myactuator_x6_test set_zero.py motor0_joint")
        print("  ros2 run myactuator_x6_test set_zero.py motor1_joint")
        print("  ros2 run myactuator_x6_test set_zero.py --all")
        return

    if '--all' in args:
        target_joints = ['motor0_joint', 'motor1_joint']
    else:
        target_joints = args

    rclpy.init()
    node = SetZeroNode(target_joints)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
