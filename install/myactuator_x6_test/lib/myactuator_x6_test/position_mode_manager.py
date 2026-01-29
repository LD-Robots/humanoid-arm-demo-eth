#!/usr/bin/env python3
"""
Position Mode Manager

Keeps controller loaded but inactive for backdrivability.
Motor is free to move by hand. MoveIt will activate controller when needed.

Usage:
    ros2 run myactuator_x6_test position_mode_manager.py
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
        if self.switch_controller('position'):
            self.get_logger().info('✓ Ready for MoveIt control!')
        else:
            self.get_logger().error('✗ Failed to activate controller')
            sys.exit(1)

        # Timer to keep alive
        self.timer = self.create_timer(1.0, lambda: None)

    def switch_controller(self, mode):
        """Switch controllers."""
        if mode == 'position':
            activate = ['joint_trajectory_controller']
            deactivate = []  # No controller to deactivate - motor was free-floating
        else:
            activate = []
            deactivate = ['joint_trajectory_controller']  # Return to free-floating (safe)

        self.get_logger().info(f'Switching to {mode.upper()} mode...')

        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() and future.result().ok:
            self.get_logger().info(f'SUCCESS: {mode.upper()} mode active')
            return True
        return False


def main():
    rclpy.init()
    node = PositionModeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutdown requested, switching back to effort...')
    finally:
        # Use subprocess for clean shutdown - doesn't block
        try:
            subprocess.Popen(
                ['ros2', 'run', 'myactuator_x6_test', 'switch_controller.py', '--mode', 'effort'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            node.get_logger().info('Effort mode restore initiated.')
        except:
            pass

        node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
