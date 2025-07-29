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

def initialize_robot(robot):
    """初始化机器人到初始位置"""
    print("\n=== Robot Initialization ===")
    
    # Display basic workspace info
    workspace_info = robot.get_workspace_info()
    limits = workspace_info['limits']
    print(f"Workspace: R={limits['inner_radius']:.2f}-{limits['outer_radius']:.2f}m, H={limits['z_min']:.2f}-{limits['z_max']:.2f}m")
    
    print("Moving to initial position...")
    robot.move_to_initial_position()
    robot.pause(1.0)
    print("Robot initialization completed")

def move_to_home_pose(robot):
    """移动到home位置"""
    print("\n=== Moving to Home Pose ===")
    print("Moving to home position...")
    robot.move_to_home()
    robot.pause(1.0)
    print("Reached home position")

def detect_objects(vision_api):
    """检测物体"""
    print("\n=== Object Detection ===")
    
    print("Initializing vision system...")
    if not vision_api.initialize():
        print("Failed to initialize vision API")
        return []
    
    print("Scanning for objects...")
    positions = vision_api.get_object_positions(timeout=3.0)
    
    if positions:
        print(f"Found {len(positions)} objects:")
        for i, pos in enumerate(positions):
            print(f"  {i+1}. {pos.color}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) confidence={pos.confidence:.2f}")
        return positions
    else:
        print("No objects detected")
        print("Make sure colored objects are in the camera view!")
        return []

def move_to_objects(robot, positions):
    """按距离从近到远移动到物体"""
    print("\n=== Move to Objects ===")
    
    if not positions:
        print("No objects to move to!")
        return
    
    # 获取当前机器人位置作为参考点（使用home位置）
    home_pos = [0.0, 0.4, 0.3]  # 机器人home位置
    
    # 计算每个物体到机器人的距离
    import math
    for pos in positions:
        distance = math.sqrt((pos.x - home_pos[0])**2 + (pos.y - home_pos[1])**2)
        pos.distance_to_robot = distance
    
    # 按距离排序（从近到远）
    sorted_positions = sorted(positions, key=lambda p: p.distance_to_robot)
    
    print(f"Moving to {len(sorted_positions)} objects in order (near to far):")
    for i, pos in enumerate(sorted_positions):
        print(f"  {i+1}. {pos.color} at distance {pos.distance_to_robot:.3f}m")
    
    # 依次移动到每个物体
    for i, obj in enumerate(sorted_positions):
        print(f"\n--- Moving to object {i+1}/{len(sorted_positions)}: {obj.color} ---")
        
        # 计算安全的接近位置（在物体上方）
        target_x = obj.x
        target_y = obj.y  
        target_z = obj.z + 0.2  # 在物体上方10cm
        
        print(f"Target position: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        
        success = robot.move_to_position(target_x, target_y, target_z, 0.0, 1.0, 0.0, 0.0)
        if success:
            print(f"Successfully reached {obj.color} object!")
            robot.pause(2.0)
        else:
            print(f"Failed to reach {obj.color} object")
    
    print("\nCompleted visiting all objects. Returning to home position...")
    robot.move_to_home()
    robot.pause(1.0)

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
            
            # Global variable to store detected objects
            detected_objects = []
            
            # Show menu
            while True:
                print("\n=== Main Controller Menu ===")
                print("1. Initialize robot")
                print("2. Object detection")
                print("3. Move to home pose")
                print("4. Move to objects (near to far)")
                print("5. Exit")
                
                try:
                    choice = input("\nEnter your choice (1-5): ").strip()
                    
                    if choice == '1':
                        initialize_robot(robot)
                        
                    elif choice == '2':
                        detected_objects = detect_objects(vision_api)
                        
                    elif choice == '3':
                        move_to_home_pose(robot)
                        
                    elif choice == '4':
                        if detected_objects:
                            move_to_objects(robot, detected_objects)
                        else:
                            print("No objects detected yet. Please run object detection first (option 2).")
                            
                    elif choice == '5':
                        break
                        
                    else:
                        print("Invalid choice. Please enter 1-5.")
                        
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