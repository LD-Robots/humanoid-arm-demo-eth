#!/usr/bin/env python3
"""
Interactive Controller Switcher

Switches between effort mode (backdrivable) and position mode (for MoveIt).

Usage:
    ros2 run myactuator_x6_test controller_switcher.py

Commands:
    e - Switch to Effort mode (backdrivable, safe)
    p - Switch to Position mode (for trajectory control)
    q - Quit
"""

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController
import sys
import select
import tty
import termios


class ControllerSwitcher(Node):
    def __init__(self):
        super().__init__('controller_switcher')
        
        self.switch_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        
        # Wait for controller manager
        self.get_logger().info('Waiting for controller_manager...')
        while not self.switch_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Still waiting...')
        
        self.get_logger().info('Controller Switcher Ready!')
        self.current_mode = None
        self.show_menu()
    
    def show_menu(self):
        print('\n' + '='*50)
        print('Controller Switcher')
        print('='*50)
        print('Commands:')
        print('  [e] Effort mode   - Backdrivable, safe, move by hand')
        print('  [p] Position mode - For MoveIt/trajectory control')
        print('  [q] Quit')
        print('='*50)
        if self.current_mode:
            print(f'Current mode: {self.current_mode.upper()}')
        print('> ', end='', flush=True)
    
    def switch_mode(self, mode):
        """Switch controller mode."""
        req = SwitchController.Request()
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        
        if mode == 'effort':
            req.activate_controllers = ['effort_controller']
            req.deactivate_controllers = ['joint_trajectory_controller']
            mode_name = 'EFFORT (backdrivable)'
        elif mode == 'position':
            req.activate_controllers = ['joint_trajectory_controller']
            req.deactivate_controllers = ['effort_controller']
            mode_name = 'POSITION (trajectory control)'
        else:
            return False
        
        self.get_logger().info(f'Switching to {mode_name}...')
        
        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().ok:
            self.get_logger().info(f'✓ Successfully switched to {mode_name}')
            self.current_mode = mode
            return True
        else:
            self.get_logger().error(f'✗ Failed to switch to {mode_name}')
            return False
    
    def run_interactive(self):
        """Run interactive mode with keyboard input."""
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while True:
                # Check for keyboard input
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1).lower()
                    
                    if key == 'e':
                        print('\n')
                        self.switch_mode('effort')
                        self.show_menu()
                    elif key == 'p':
                        print('\n')
                        self.switch_mode('position')
                        self.show_menu()
                    elif key == 'q':
                        print('\nExiting...')
                        break
                    else:
                        print(f'\nUnknown command: {key}')
                        self.show_menu()
                
                rclpy.spin_once(self, timeout_sec=0.01)
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    rclpy.init()
    switcher = ControllerSwitcher()
    
    try:
        switcher.run_interactive()
    except KeyboardInterrupt:
        print('\nInterrupted!')
    finally:
        switcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
