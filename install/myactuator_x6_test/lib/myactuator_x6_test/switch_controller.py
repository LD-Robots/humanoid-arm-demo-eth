#!/usr/bin/env python3
"""
Controller Switcher for MyActuator X6

Switches between effort (safe) and trajectory (MoveIt) controllers.

Usage:
    ros2 run myactuator_x6_test switch_controller.py --mode position
    ros2 run myactuator_x6_test switch_controller.py --mode effort
"""

import argparse
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController, ListControllers


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')

        self.switch_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        self.list_client = self.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )

        while not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for controller_manager...')

    def list_controllers(self):
        """List active controllers."""
        req = ListControllers.Request()
        future = self.list_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            active = [c.name for c in future.result().controller if c.state == 'active']
            self.get_logger().info(f'Active controllers: {active}')
            return active
        return []

    def switch_to_position(self):
        """Switch from effort to trajectory controller."""
        self.get_logger().info('Switching to POSITION mode (trajectory controller)...')

        req = SwitchController.Request()
        req.activate_controllers = ['joint_trajectory_controller']
        req.deactivate_controllers = ['effort_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().ok:
            self.get_logger().info('SUCCESS: Now in POSITION mode')
            return True
        else:
            self.get_logger().error('FAILED to switch to position mode')
            return False

    def switch_to_effort(self):
        """Switch from trajectory to effort controller."""
        self.get_logger().info('Switching to EFFORT mode (safe default)...')

        req = SwitchController.Request()
        req.activate_controllers = ['effort_controller']
        req.deactivate_controllers = ['joint_trajectory_controller']
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().ok:
            self.get_logger().info('SUCCESS: Now in EFFORT mode (safe)')
            return True
        else:
            self.get_logger().error('FAILED to switch to effort mode')
            return False


def main():
    parser = argparse.ArgumentParser(description='Switch robot controllers')
    parser.add_argument('--mode', choices=['position', 'effort', 'list'],
                        required=True, help='Controller mode')
    args = parser.parse_args()

    rclpy.init()
    switcher = ControllerSwitcher()

    try:
        if args.mode == 'list':
            switcher.list_controllers()
        elif args.mode == 'position':
            switcher.switch_to_position()
        elif args.mode == 'effort':
            switcher.switch_to_effort()
    finally:
        switcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
