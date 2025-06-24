#!/usr/bin/env python3
"""
main_controller.py - Simple main controller using robot API
"""

import time
import rclpy

from move_node.robot_arm_controller import RobotArmController

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
            # Set speed and pause between movements
            robot.set_speed(50)  # 50% speed
            robot.set_pause_between_movements(0.5)  # 0.5 second pause between movements
            
            print("=== UR5 Robot Controller Ready ===")
            print("Robot is ready for commands!")
            print("See MOVEMENT_FEATURES.txt for detailed API usage tutorial")
            print("=" * 50)
            
            # ========================================
            # ========================================

            # Display basic workspace info
            workspace_info = robot.get_workspace_info()
            limits = workspace_info['limits']
            print(f"Workspace: R={limits['inner_radius']:.2f}-{limits['outer_radius']:.2f}m, H={limits['z_min']:.2f}-{limits['z_max']:.2f}m")
            print()

            # 0. First to initial position, then to home position
            print("Init -> Home")
            robot.move_to_initial_position()
            robot.pause(0.5)
            robot.move_to_home()
            robot.pause(0.5)   

            print("1. Joint motion tests")
            robot.move_to_joint_positions(0.0, -1.57, -1.0, -1.57, 0.0, 0.0)
            robot.pause(0.5)
            robot.move_to_joint_positions(0.5, -1.0, 4.0, -1.0, 0.0, 0.0)  # Test joint limits
            robot.pause(0.5)

            print("2. Relative motion")
            robot.move_relative(0.0, 0.0, -0.05)
            robot.pause(0.5)              

            print("3. Position control")
            robot.move_to_position(0.3, 0.3, 0.4, 0.0, 1.0, 0.0, 0.0)
            robot.pause(0.5)  

            print("4. Front position")
            robot.move_to_position(0.0, 0.5, 0.3, 0.0, 1.0, 0.0, 0.0)
            robot.pause(0.5)

            print("5. Test boundary: outer radius (should fail)")
            robot.move_to_position(0.8, 0.0, 0.4, 0.0, 1.0, 0.0, 0.0)
            robot.pause(0.5)

            print("6. Test boundary: height limit (should fail)")
            robot.move_to_position(0.3, 0.3, 0.9, 0.0, 1.0, 0.0, 0.0)
            robot.pause(0.5)

            print("7. Test boundary: inner radius (should fail)")
            robot.move_to_position(0.05, 0.05, 0.4, 0.0, 1.0, 0.0, 0.0)
            robot.pause(0.5)

            print("8. Test boundary: height lower (should fail)")
            robot.move_to_position(0.3, 0.3, -0.2, 0.0, 1.0, 0.0, 0.0)
            robot.pause(0.5)
            
            print("9. Return home -> init")
            robot.move_to_home()
            robot.pause(0.5)
            robot.move_to_initial_position()
            
            print("=== Tests completed ===")
            
        except KeyboardInterrupt:
            print("User interrupted")
        
        print("Main controller finished")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()