#!/usr/bin/env python3
"""
main_controller.py - Simple main controller using robot API
"""

import sys
import os
import time

# Add move_node to path - 修改这里
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'move_node', 'move_node'))

# 直接 import，不需要从 robot_arm_controller import
import robot_arm_controller

def main():
    print("=" * 50)
    print("Starting main controller")
    print("=" * 50)
    
    # Create robot - 修改这里
    robot = robot_arm_controller.RobotArmController()
    
    # 其他代码保持不变
    if not robot.is_connected():
        print("Failed to connect to robot arm")
        return
    
    print("Robot connected successfully!")
    
    try:
        robot.set_speed(30)
        
        print("Queueing movements...")
        robot.move_to_home()
        robot.rotate_base_degrees(45)
        robot.rotate_base_degrees(90)
        robot.rotate_base_degrees(-45)
        robot.move_to_home()
        
        print("Starting execution...")
        robot.start_execution()
        
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

if __name__ == '__main__':
    main()