#!/usr/bin/env python3
"""
simple_vision_test.py - Simple test for vision components without camera
"""

import rclpy
from rclpy.node import Node
import time
import sys
import os

# Import robot controller
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'move_node'))
from move_node.robot_arm_controller import RobotArmController

class SimpleVisionTest(Node):
    """Simple test of vision-robot integration"""
    
    def __init__(self):
        super().__init__('simple_vision_test')
        
        self.get_logger().info('Starting simple vision test...')
        
        # Initialize robot controller
        self.robot = RobotArmController()
        
        # Wait for robot connection
        if not self._wait_for_robot_connection():
            self.get_logger().error('Failed to connect to robot')
            return
        
        self.get_logger().info('Robot connected successfully!')
        
        # Set robot parameters
        self.robot.set_speed(30)  # Slow speed for testing
        
        self.get_logger().info('Simple vision test ready')
    
    def _wait_for_robot_connection(self, timeout=10.0):
        """Wait for robot to be connected"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.robot.is_connected():
                return True
            time.sleep(0.5)
            self.get_logger().info('Waiting for robot connection...')
        return False
    
    def run_movement_test(self):
        """Test robot movement without vision"""
        self.get_logger().info('Starting movement test...')
        
        try:
            # Move to home position
            self.get_logger().info('Moving to home position...')
            self.robot.move_to_home()
            self.robot.wait_for_completion()
            
            # Get current pose
            current_pose = self.robot.get_current_cartesian_pose()
            if current_pose:
                x, y, z = current_pose.position.x, current_pose.position.y, current_pose.position.z
                self.get_logger().info(f'Current position: x={x:.3f}, y={y:.3f}, z={z:.3f}')
            
            # Test relative movements (simulating vision feedback)
            movements = [
                (0.05, 0, 0, "Move +5cm in X"),
                (0, 0.05, 0, "Move +5cm in Y"), 
                (0, 0, -0.03, "Move -3cm in Z"),
                (-0.05, -0.05, 0.03, "Return to start")
            ]
            
            for dx, dy, dz, description in movements:
                self.get_logger().info(f'Executing: {description}')
                self.robot.move_relative_xyz(dx, dy, dz, description)
                self.robot.wait_for_completion()
                time.sleep(1.0)  # Pause between movements
            
            self.get_logger().info('Movement test completed successfully!')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error in movement test: {e}')
            return False

def main(args=None):
    """Simple vision test main function"""
    rclpy.init(args=args)
    
    try:
        test_node = SimpleVisionTest()
        
        if not test_node.robot.is_connected():
            print("ERROR: Robot not connected!")
            print("Please make sure robot simulation is running:")
            print("  ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur5")
            return
        
        print("\n" + "="*50)
        print("    SIMPLE VISION SYSTEM TEST")
        print("="*50)
        print("Testing robot movement without camera...")
        print("This simulates vision-guided movement patterns.")
        print()
        
        # Run movement test
        success = test_node.run_movement_test()
        
        if success:
            print("\n✅ Test completed successfully!")
            print("\nNext steps:")
            print("1. Connect a USB camera")
            print("2. Run: ros2 run vision_node camera_node")
            print("3. Run: ros2 run vision_node vision_demo")
        else:
            print("\n❌ Test failed!")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()