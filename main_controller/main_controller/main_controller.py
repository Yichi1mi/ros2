#!/usr/bin/env python3
"""
main_controller.py - Simple main controller using robot API
"""

import sys
import os
import time
import rclpy

# Add the correct path to find robot_arm_controller
current_dir = os.path.dirname(os.path.abspath(__file__))
move_node_dir = os.path.join(current_dir, '..', '..', 'move_node', 'move_node')
sys.path.insert(0, move_node_dir)

from robot_arm_controller import RobotArmController

def main():
    rclpy.init()
    try:
        print("=" * 50)
        print("Starting main controller")
        print("=" * 50)
        
        # Create robot
        robot = RobotArmController()
        
        if not robot.is_connected():
            print("Failed to connect to robot arm")
            return
        
        print("Robot connected successfully!")
        
        try:
            # Set speed
            robot.set_speed(30)  # 30% speed
            
            # Queue some joint movements
            print("Queueing movements...")
            positions = [
                [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],      # Home
                [0.785, -1.57, 0.0, -1.57, 0.0, 0.0],    # 45 deg
                [1.57, -1.57, 0.0, -1.57, 0.0, 0.0],     # 90 deg
                [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],      # Back to Home
            ]
            for idx, pos in enumerate(positions):
                robot.move_to_joint_positions(pos, f"Move {idx+1}")

            # Start execution (no-op, for compatibility)
            print("Starting execution...")
            robot.start_execution()
            
            # Wait for completion
            print("Waiting for completion...")
            if robot.wait_for_completion():
                print("All movements completed!")
            else:
                print("Timeout waiting for completion")
            
        except KeyboardInterrupt:
            print("User interrupted")
            robot.stop_execution()
            robot.clear_queue()
        
        print("Main controller finished")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()