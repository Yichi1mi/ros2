#!/usr/bin/env python3
"""
main_controller.py - Main controller
Example of calling robot arm API
"""

import sys
import os
# Add my_moveit_planner to Python path to import RobotArmController
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'move_node', 'move_node'))

from robot_arm_controller import RobotArmController
import time

def main():
    print("=" * 50)
    print("Starting main controller")
    print("=" * 50)
    
    # Create robot arm controller
    arm = RobotArmController('main_controller')
    
    if not arm.is_connected():
        print("Failed to connect to robot arm, please check simulation environment")
        return
    
    print("Robot arm connected successfully!")
    
    # Execute a series of actions
    try:
        print("\nExecuting action sequence...")
        
        # Action 1: Ensure at Home position
        print("1. Moving to Home position")
        arm.move_to_home()
        time.sleep(2)
        
        # Action 2: Rotate base
        print("2. Rotating base 45 degrees")
        arm.rotate_base(0.785)  # 45 degrees
        time.sleep(2)
        
        # Action 3: Custom position
        print("3. Moving to custom position")
        custom_angles = [0.5, -1.2, -0.3, -1.5, 0.2, 0.0]
        arm.move_joints(custom_angles, duration=3.0)
        time.sleep(3)
        
        # Action 4: Back to Home
        print("4. Returning to Home position")
        arm.move_to_home()
        time.sleep(2)
        
        print("\nAll actions completed successfully!")
        
    except KeyboardInterrupt:
        print("\nUser interrupted")
    except Exception as e:
        print(f"\nError occurred: {e}")
    finally:
        print("\nShutting down controller...")
        arm.shutdown()
        print("Goodbye!")

if __name__ == '__main__':
    main()