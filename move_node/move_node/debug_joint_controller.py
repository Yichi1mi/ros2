#!/usr/bin/env python3
"""
Debug version to test joint state reception
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class DebugJointController(Node):
    def __init__(self):
        super().__init__('debug_joint_controller')
        self.get_logger().info('Starting debug joint controller')
        
        self._joint_state_received = False
        self._callback_count = 0
        
        # Subscribe to joint states
        self.get_logger().info('Creating joint state subscription...')
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.get_logger().info('Joint state subscription created')
        
        # Wait for joint states
        self._wait_for_joint_states()
    
    def _joint_state_callback(self, msg):
        """Debug callback for joint state updates"""
        self._callback_count += 1
        self.get_logger().info(f'Callback #{self._callback_count} - Received joint states')
        self.get_logger().info(f'Joint names: {msg.name}')
        self.get_logger().info(f'Positions: {[f"{p:.3f}" for p in msg.position]}')
        
        # Just set the flag
        self._joint_state_received = True
    
    def _wait_for_joint_states(self):
        """Wait for first joint state message"""
        self.get_logger().info('Waiting for joint states...')
        timeout = 10.0
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            # Spin once to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            if self._joint_state_received:
                self.get_logger().info('SUCCESS: Joint states received!')
                self.get_logger().info(f'Total callbacks received: {self._callback_count}')
                return True
                
            elapsed = time.time() - start_time
            if elapsed > 2.0 and self._callback_count == 0:
                self.get_logger().warn(f'No callbacks received after {elapsed:.1f}s')
        
        self.get_logger().error('TIMEOUT: Failed to receive joint states')
        self.get_logger().error(f'Total callbacks received: {self._callback_count}')
        return False

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = DebugJointController()
        
        # Keep spinning for a bit to see if more messages come
        self.get_logger().info('Spinning for 5 more seconds...')
        end_time = time.time() + 5.0
        while time.time() < end_time:
            rclpy.spin_once(controller, timeout_sec=0.1)
        
        controller.get_logger().info(f'Final callback count: {controller._callback_count}')
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()