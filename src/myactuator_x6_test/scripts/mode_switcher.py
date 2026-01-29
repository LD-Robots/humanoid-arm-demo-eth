#!/usr/bin/env python3
"""
Motor Mode Switcher - Changes EtherCAT motor operation mode

Switches between:
  - Mode 8 (CSP - Position) - For trajectory control, not backdrivable
  - Mode 10 (CST - Torque) - For effort control, backdrivable

Usage:
    ros2 run myactuator_x6_test mode_switcher.py

Commands:
    t - Torque mode (backdrivable, move by hand)
    p - Position mode (for trajectory control)
    q - Quit
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import sys
import select
import tty
import termios


class ModeSwitcher(Node):
    def __init__(self, joints=None):
        super().__init__('mode_switcher')
        
        # Default to common joint names if not specified
        if joints is None:
            # Try to auto-detect from parameter server or use defaults
            joints = ['motor0_joint', 'motor1_joint']  # Dual motor default
        
        self.joints = joints
        
        # Service client to change motor mode
        self.param_client = self.create_client(
            SetParameters,
            '/controller_manager/set_parameters'
        )
        
        # Wait for controller manager
        self.get_logger().info('Waiting for controller_manager...')
        while not self.param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Still waiting...')
        
        self.get_logger().info(f'Mode Switcher Ready for joints: {self.joints}')
        self.current_mode = None
        self.show_menu()
    
    def show_menu(self):
        print('\n' + '='*60)
        print('Motor Mode Switcher')
        print('='*60)
        print('Commands:')
        print('  [t] Torque mode (CST)    - Backdrivable, move by hand')
        print('  [p] Position mode (CSP)  - For trajectory control')
        print('  [q] Quit')
        print('='*60)
        if self.current_mode:
            print(f'Current mode: {self.current_mode}')
        print('> ', end='', flush=True)
    
    def set_motor_mode(self, mode):
        """Set motor operation mode via parameter for all joints."""
        req = SetParameters.Request()
        
        # Create parameter for each joint
        params = []
        for joint_name in self.joints:
            param = Parameter()
            param.name = f'{joint_name}.mode_of_operation'
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = mode
            params.append(param)
        
        req.parameters = params
        
        mode_name = {8: 'Position (CSP)', 10: 'Torque (CST)'}.get(mode, f'Mode {mode}')
        self.get_logger().info(f'Setting {len(self.joints)} motor(s) to {mode_name}...')
        
        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result():
            results = future.result().results
            success_count = sum(1 for r in results if r.successful)
            
            if success_count == len(self.joints):
                self.get_logger().info(f'✓ Successfully set all {len(self.joints)} motor(s) to {mode_name}')
                self.current_mode = mode_name
                return True
            else:
                failed_joints = [self.joints[i] for i, r in enumerate(results) if not r.successful]
                self.get_logger().error(f'✗ Failed for joints: {failed_joints}')
                for i, result in enumerate(results):
                    if not result.successful:
                        self.get_logger().error(f'  {self.joints[i]}: {result.reason}')
                return False
        else:
            self.get_logger().error('✗ Service call failed')
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
                    
                    if key == 't':
                        print('\n')
                        self.set_motor_mode(10)  # CST - Torque
                        self.show_menu()
                    elif key == 'p':
                        print('\n')
                        self.set_motor_mode(8)   # CSP - Position
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
    
    # Parse command line arguments for joint names
    import sys
    joints = None
    if len(sys.argv) > 1:
        # Usage: mode_switcher.py motor0_joint motor1_joint
        joints = sys.argv[1:]
    
    switcher = ModeSwitcher(joints=joints)
    
    try:
        switcher.run_interactive()
    except KeyboardInterrupt:
        print('\nInterrupted!')
    finally:
        switcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
