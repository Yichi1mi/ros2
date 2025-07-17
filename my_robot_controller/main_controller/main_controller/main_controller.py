#!/usr/bin/env python3
"""
main_controller.py - Main controller integrating vision and robot control
"""

import time
import rclpy
import sys
import os

from move_node.robot_arm_controller import RobotArmController

# Add vision_node to path for importing VisionAPI
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'vision_node'))
from vision_node.vision_api import VisionAPI

def test_basic_robot_functions(robot):
    """测试基本机器人功能"""
    print("\n=== Basic Robot Function Tests ===")
    
    # Display basic workspace info
    workspace_info = robot.get_workspace_info()
    limits = workspace_info['limits']
    print(f"Workspace: R={limits['inner_radius']:.2f}-{limits['outer_radius']:.2f}m, H={limits['z_min']:.2f}-{limits['z_max']:.2f}m")
    print()

    # 0. First to initial position, then to home position
    print("1. Init -> Home")
    robot.move_to_initial_position()
    robot.pause(0.5)
    robot.move_to_home()
    robot.pause(0.5)   

    print("2. Basic position test")
    robot.move_to_position(0.3, 0.3, 0.4, 0.0, 1.0, 0.0, 0.0)
    robot.pause(0.5)

    print("3. Return home")
    robot.move_to_home()
    robot.pause(0.5)

def test_vision_integration(robot, vision_api):
    """测试视觉系统集成"""
    print("\n=== Vision-Robot Integration Test ===")
    
    print("Initializing vision system...")
    if not vision_api.initialize():
        print("Failed to initialize vision API")
        return False
    
    print("Vision API initialized successfully")
    print("Looking for objects in the scene...")
    
    # 移动到观察位置
    print("Moving to observation position...")
    robot.move_to_position(0.0, 0.4, 0.3, 0.0, 1.0, 0.0, 0.0)
    robot.pause(1.0)
    
    # 尝试检测物体
    for attempt in range(5):
        print(f"\nDetection attempt {attempt + 1}/5...")
        
        positions = vision_api.get_object_positions(timeout=3.0)
        
        if positions:
            print(f"Found {len(positions)} objects:")
            for i, pos in enumerate(positions):
                print(f"  Object {i+1}: {pos.color} at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
                      f"confidence={pos.confidence:.2f}")
            
            # 移动到最大的物体位置
            largest_obj = max(positions, key=lambda p: p.area)
            print(f"\nMoving to largest object: {largest_obj.color}")
            
            # 计算安全的接近位置（在物体上方）
            target_x = largest_obj.x
            target_y = largest_obj.y  
            target_z = largest_obj.z + 0.1  # 在物体上方10cm
            
            print(f"Target position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
            
            success = robot.move_to_position(target_x, target_y, target_z, 0.0, 1.0, 0.0, 0.0)
            if success:
                print("Successfully moved to object position!")
                robot.pause(2.0)
                
                # 返回观察位置
                print("Returning to observation position...")
                robot.move_to_position(0.0, 0.4, 0.3, 0.0, 1.0, 0.0, 0.0)
                robot.pause(1.0)
                
                return True
            else:
                print("Failed to move to object position")
        else:
            print("No objects detected")
            if attempt < 4:
                print("Waiting 2 seconds before next attempt...")
                time.sleep(2.0)
    
    print("Vision test completed - no objects found after 5 attempts")
    print("Make sure to add colored objects to the Gazebo scene!")
    return False

def main():
    rclpy.init()
    try:
        print("=" * 60)
        print("Starting Integrated Vision-Robot Controller")
        print("=" * 60)
        
        # Create robot controller
        robot = RobotArmController()
        
        if not robot.is_connected():
            print("Failed to connect to robot arm")
            print("Make sure ur_simulation_gazebo is running!")
            return
        
        print("Robot connected successfully!")
        
        # Create vision API
        vision_api = VisionAPI()
        
        try:
            # Set speed and pause between movements
            robot.set_speed(30)  # Slower speed for vision work
            robot.set_pause_between_movements(0.5)
            
            print("=== System Overview ===")
            print("Robot: Connected and ready")
            print("Vision: Ready to initialize")
            print("Camera topic: /world_camera/image_raw")
            print("=" * 60)
            
            # Show menu
            while True:
                print("\n=== Main Controller Menu ===")
                print("1. Test basic robot functions")
                print("2. Test vision-robot integration")
                print("3. Quick vision test (check objects)")
                print("4. Exit")
                
                try:
                    choice = input("\nEnter your choice (1-4): ").strip()
                    
                    if choice == '1':
                        test_basic_robot_functions(robot)
                        
                    elif choice == '2':
                        test_vision_integration(robot, vision_api)
                        
                    elif choice == '3':
                        print("\nQuick vision test...")
                        if vision_api.initialize():
                            positions = vision_api.get_object_positions(timeout=3.0)
                            if positions:
                                print(f"Found {len(positions)} objects:")
                                for pos in positions:
                                    print(f"  {pos.color}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
                            else:
                                print("No objects detected")
                                print("Add colored objects to Gazebo scene for testing")
                        else:
                            print("Failed to initialize vision system")
                            
                    elif choice == '4':
                        break
                        
                    else:
                        print("Invalid choice. Please enter 1-4.")
                        
                except KeyboardInterrupt:
                    print("\nInterrupted by user")
                    break
                except Exception as e:
                    print(f"Error: {e}")
            
        except KeyboardInterrupt:
            print("\nUser interrupted")
        
        print("\nShutting down...")
        vision_api.shutdown()
        print("Main controller finished")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()