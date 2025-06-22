#!/usr/bin/env python3
"""
gripper_controller.py - Franka gripper controller using official ROS2 actions
Based on franka_gripper actions: homing, move, grasp
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from franka_msgs.action import Homing, Move, Grasp
import time

class GripperController(Node):
    """
    Controller for Franka gripper using official franka_msgs actions.
    Supports homing, moving to width, and grasping operations.
    """
    
    def __init__(self, arm_id="fr3"):
        # Initialize ROS2 node
        super().__init__('gripper_controller')
        
        self._arm_id = arm_id
        self._gripper_namespace = f"{arm_id}_gripper"
        
        # Gripper parameters
        self._default_speed = 0.1  # m/s
        self._default_force = 50.0  # N
        self._default_epsilon_inner = 0.005  # m
        self._default_epsilon_outer = 0.005  # m
        self._max_width = 0.08  # Default max width, updated after homing
        
        # Track gripper state
        self._is_homed = False
        self._current_width = None
        self._connected = False
        
        # Initialize connection
        self._connected = self._initialize_connection()
    
    def _initialize_connection(self):
        """Initialize gripper action clients"""
        try:
            # Create action clients for gripper operations
            self._homing_client = ActionClient(self, Homing, f'/{self._gripper_namespace}/homing')
            self._move_client = ActionClient(self, Move, f'/{self._gripper_namespace}/move')
            self._grasp_client = ActionClient(self, Grasp, f'/{self._gripper_namespace}/grasp')
            
            # Wait for action servers
            self.get_logger().info('Waiting for gripper action servers...')
            
            if not self._homing_client.wait_for_server(timeout_sec=3.0):
                self.get_logger().info('Homing action server not available (normal in fake hardware mode)')
                return False
                
            if not self._move_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('Move action server not available (normal in fake hardware mode)')
                return False
                
            if not self._grasp_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().info('Grasp action server not available (normal in fake hardware mode)')
                return False
            
            self.get_logger().info('Gripper action servers connected')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize gripper: {e}')
            return False
    
    def is_connected(self):
        """Check if gripper is connected"""
        return self._connected
    
    def home_gripper(self, wait_for_completion=True):
        """
        Home the gripper and update maximum width
        
        Returns:
            bool: True if homing successful
        """
        if not self.is_connected():
            self.get_logger().error('Gripper not connected!')
            return False
        
        try:
            self.get_logger().info('Homing gripper...')
            
            # Create and send homing goal
            goal_msg = Homing.Goal()
            future = self._homing_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not future.done():
                self.get_logger().error('Homing goal submission timeout')
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Homing goal rejected')
                return False
            
            self.get_logger().info('Homing started...')
            
            if not wait_for_completion:
                return True
            
            # Wait for completion
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            
            while not result_future.done() and (time.time() - start_time) < 30.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if result_future.done():
                result = result_future.result()
                if result.result.success:
                    self._is_homed = True
                    self._max_width = result.result.max_width
                    self.get_logger().info(f'Homing successful! Max width: {self._max_width:.3f}m')
                    return True
                else:
                    self.get_logger().error(f'Homing failed: {result.result.error}')
                    return False
            else:
                self.get_logger().error('Homing timeout')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error during homing: {e}')
            return False
    
    def move_to_width(self, width, speed=None, wait_for_completion=True):
        """
        Move gripper to specified width
        
        Args:
            width: Target width in meters (0.0 = closed)
            speed: Movement speed in m/s (optional)
            wait_for_completion: If True, block until movement completes
            
        Returns:
            bool: True if movement successful
        """
        if not self.is_connected():
            self.get_logger().error('Gripper not connected!')
            return False
        
        if speed is None:
            speed = self._default_speed
            
        # Validate width
        if width < 0.0:
            self.get_logger().error(f'Invalid width: {width}. Must be >= 0.0')
            return False
            
        if self._is_homed and width > self._max_width:
            self.get_logger().warn(f'Width {width:.3f}m exceeds max width {self._max_width:.3f}m')
            width = self._max_width
        
        try:
            self.get_logger().info(f'Moving gripper to width: {width:.3f}m at speed: {speed:.3f}m/s')
            
            # Create and send move goal
            goal_msg = Move.Goal()
            goal_msg.width = width
            goal_msg.speed = speed
            
            future = self._move_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not future.done():
                self.get_logger().error('Move goal submission timeout')
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Move goal rejected')
                return False
            
            self.get_logger().info('Gripper movement started...')
            
            if not wait_for_completion:
                return True
            
            # Wait for completion
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            
            while not result_future.done() and (time.time() - start_time) < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if result_future.done():
                result = result_future.result()
                if result.result.success:
                    self._current_width = width
                    self.get_logger().info('Gripper movement completed successfully')
                    return True
                else:
                    self.get_logger().error(f'Gripper movement failed: {result.result.error}')
                    return False
            else:
                self.get_logger().error('Gripper movement timeout')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error during gripper movement: {e}')
            return False
    
    def grasp(self, width=0.0, speed=None, force=None, epsilon_inner=None, epsilon_outer=None, wait_for_completion=True):
        """
        Attempt to grasp an object
        
        Args:
            width: Target width to grasp at in meters
            speed: Closing speed in m/s (optional)
            force: Grasp force in N (optional)
            epsilon_inner: Inner tolerance for grasping (optional)
            epsilon_outer: Outer tolerance for grasping (optional)
            wait_for_completion: If True, block until grasp completes
            
        Returns:
            bool: True if grasp successful
        """
        if not self.is_connected():
            self.get_logger().error('Gripper not connected!')
            return False
        
        # Set defaults
        if speed is None:
            speed = self._default_speed
        if force is None:
            force = self._default_force
        if epsilon_inner is None:
            epsilon_inner = self._default_epsilon_inner
        if epsilon_outer is None:
            epsilon_outer = self._default_epsilon_outer
        
        try:
            self.get_logger().info(f'Grasping at width: {width:.3f}m, force: {force:.1f}N, speed: {speed:.3f}m/s')
            
            # Create and send grasp goal
            goal_msg = Grasp.Goal()
            goal_msg.width = width
            goal_msg.speed = speed
            goal_msg.force = force
            goal_msg.epsilon.inner = epsilon_inner
            goal_msg.epsilon.outer = epsilon_outer
            
            future = self._grasp_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not future.done():
                self.get_logger().error('Grasp goal submission timeout')
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Grasp goal rejected')
                return False
            
            self.get_logger().info('Grasping started...')
            
            if not wait_for_completion:
                return True
            
            # Wait for completion
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            
            while not result_future.done() and (time.time() - start_time) < 15.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if result_future.done():
                result = result_future.result()
                if result.result.success:
                    self.get_logger().info('Grasp completed successfully')
                    return True
                else:
                    self.get_logger().error(f'Grasp failed: {result.result.error}')
                    return False
            else:
                self.get_logger().error('Grasp timeout')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error during grasping: {e}')
            return False
    
    def open_gripper(self, wait_for_completion=True):
        """
        Open gripper to maximum width
        
        Returns:
            bool: True if successful
        """
        width = self._max_width if self._is_homed else 0.08  # Default max width
        self.get_logger().info('Opening gripper...')
        return self.move_to_width(width, wait_for_completion=wait_for_completion)
    
    def close_gripper(self, wait_for_completion=True):
        """
        Close gripper completely
        
        Returns:
            bool: True if successful
        """
        self.get_logger().info('Closing gripper...')
        return self.move_to_width(0.0, wait_for_completion=wait_for_completion)
    
    def get_max_width(self):
        """Get maximum gripper width"""
        return self._max_width if self._is_homed else None
    
    def is_homed(self):
        """Check if gripper is homed"""
        return self._is_homed


def main(args=None):
    """
    Unit test for GripperController
    """
    rclpy.init(args=args)
    try:
        controller = GripperController()
        if controller.is_connected():
            print("Testing gripper controller...")
            
            # Test homing
            print("1. Homing gripper...")
            if controller.home_gripper():
                print("Homing successful!")
                print(f"Max width: {controller.get_max_width():.3f}m")
                
                # Test opening
                print("2. Opening gripper...")
                if controller.open_gripper():
                    print("Gripper opened!")
                    time.sleep(1.0)
                    
                    # Test closing
                    print("3. Closing gripper...")
                    if controller.close_gripper():
                        print("Gripper closed!")
                        time.sleep(1.0)
                        
                        # Test specific width
                        print("4. Moving to 50% width...")
                        target_width = controller.get_max_width() * 0.5
                        if controller.move_to_width(target_width):
                            print(f"Moved to width: {target_width:.3f}m")
                        
                        print("Gripper test completed successfully!")
                    else:
                        print("Failed to close gripper")
                else:
                    print("Failed to open gripper")
            else:
                print("Homing failed")
        else:
            print("Failed to connect to gripper")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()