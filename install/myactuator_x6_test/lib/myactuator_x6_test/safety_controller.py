#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from controller_manager_msgs.srv import SwitchController
import sys

class SafetyController(Node):
    def __init__(self, max_effort=5.0, joint_name='motor_joint'):
        super().__init__('safety_controller')
        
        self.max_effort = max_effort
        self.joint_name = joint_name
        self.effort_exceeded = False
        self.current_effort = 0.0
        self.controller_stopped = False
        
        # Subscribe to joint states to monitor effort
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for safety status
        self.safety_status_pub = self.create_publisher(
            Bool,
            '/safety_status',
            10
        )
        
        # Service client to stop/start controller
        self.switch_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        
        self.get_logger().info(f'Safety controller started - Max effort: {self.max_effort} for joint: {self.joint_name}')
        
        # Timer to publish status
        self.timer = self.create_timer(0.1, self.publish_status)
    
    def joint_state_callback(self, msg):
        """Monitor joint effort and stop controller if exceeded."""
        if self.joint_name not in msg.name:
            return
        
        idx = msg.name.index(self.joint_name)
        
        # Check if effort data is available
        if len(msg.effort) > idx:
            self.current_effort = abs(msg.effort[idx])
            
            if self.current_effort > self.max_effort:
                if not self.effort_exceeded:
                    self.get_logger().error(
                        f'SAFETY VIOLATION: Effort {self.current_effort:.2f} exceeds limit {self.max_effort:.2f}!'
                    )
                    self.effort_exceeded = True
                    self.stop_controller()
            else:
                if self.effort_exceeded and self.controller_stopped:
                    self.get_logger().info('Effort back within safe limits, restarting controller...')
                    self.effort_exceeded = False
                    self.start_controller()
    
    def stop_controller(self):
        """Stop the joint trajectory controller."""
        if self.controller_stopped:
            return
        
        self.get_logger().warn('Stopping joint_trajectory_controller for safety...')
        
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Controller manager service not available!')
            return
        
        request = SwitchController.Request()
        request.deactivate_controllers = ['joint_trajectory_controller']
        request.strictness = SwitchController.Request.STRICT
        request.timeout = rclpy.duration.Duration(seconds=0.0).to_msg()
        
        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(self.stop_controller_callback)
    
    def stop_controller_callback(self, future):
        """Callback for stop controller service call."""
        try:
            response = future.result()
            if response.ok:
                self.controller_stopped = True
                self.get_logger().warn('Controller stopped successfully')
            else:
                self.get_logger().error('Failed to stop controller!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def start_controller(self):
        """Restart the joint trajectory controller."""
        if not self.controller_stopped:
            return
        
        self.get_logger().info('Restarting joint_trajectory_controller...')
        
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Controller manager service not available!')
            return
        
        request = SwitchController.Request()
        request.activate_controllers = ['joint_trajectory_controller']
        request.strictness = SwitchController.Request.STRICT
        request.timeout = rclpy.duration.Duration(seconds=0.0).to_msg()
        
        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(self.start_controller_callback)
    
    def start_controller_callback(self, future):
        """Callback for start controller service call."""
        try:
            response = future.result()
            if response.ok:
                self.controller_stopped = False
                self.get_logger().info('Controller restarted successfully')
            else:
                self.get_logger().error('Failed to restart controller!')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def publish_status(self):
        """Publish safety status (true = safe, false = violation)."""
        msg = Bool()
        msg.data = not self.effort_exceeded
        self.safety_status_pub.publish(msg)
    
    def get_status_string(self):
        """Get a human-readable status string."""
        if self.effort_exceeded:
            return f'⚠️  UNSAFE - Effort: {self.current_effort:.2f}/{self.max_effort:.2f}'
        else:
            return f'✓ SAFE - Effort: {self.current_effort:.2f}/{self.max_effort:.2f}'

def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run myactuator_x6_test safety_controller.py <max_effort> [joint_name]")
        print("Example: ros2 run myactuator_x6_test safety_controller.py 5.0 motor_joint")
        print("\nStarts a safety monitor that cancels trajectories if effort exceeds the limit.")
        return
    
    max_effort = float(sys.argv[1])
    joint_name = sys.argv[2] if len(sys.argv) > 2 else 'motor_joint'
    
    rclpy.init()
    node = SafetyController(max_effort, joint_name)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
