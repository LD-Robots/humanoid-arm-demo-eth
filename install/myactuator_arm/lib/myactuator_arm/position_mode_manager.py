#!/usr/bin/env python3
"""
Position Mode Manager for MyActuator Arm

Manages controller switching for MoveIt integration.
Activates trajectory controller for position control with MoveIt.

Usage:
    ros2 run myactuator_arm position_mode_manager.py
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
import subprocess
import sys


class PositionModeManager(Node):
    def __init__(self):
        super().__init__('position_mode_manager')

        self.switch_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )

        # Wait for controller manager
        self.get_logger().info('Waiting for controller_manager...')
        while not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Still waiting...')

        # Activate trajectory controller for MoveIt
        self.get_logger().info('Activating trajectory controller for MoveIt...')
        if self.activate_trajectory_controller():
            self.get_logger().info('✓ Ready for MoveIt control!')
        else:
            self.get_logger().error('✗ Failed to activate controller')
            sys.exit(1)

        # Timer to keep alive
        self.timer = self.create_timer(1.0, lambda: None)

    def activate_trajectory_controller(self):
        """Activate trajectory controller for MoveIt."""
        req = SwitchController.Request()
        req.activate_controllers = ['joint_trajectory_controller']
        req.deactivate_controllers = []  # No controller active initially
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().ok:
            self.get_logger().info('SUCCESS: Trajectory controller active')
            return True
        return False


def main():
    rclpy.init()
    node = PositionModeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown: deactivating trajectory controller...')
    finally:
        # Deactivate controller to return to free-floating (safe)
        try:
            subprocess.Popen(
                ['ros2', 'service', 'call',
                 '/controller_manager/switch_controller',
                 'controller_manager_msgs/srv/SwitchController',
                 "{activate_controllers: [], deactivate_controllers: ['joint_trajectory_controller'], strictness: 1}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            node.get_logger().info('Trajectory controller deactivation initiated.')
        except:
            pass

        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
