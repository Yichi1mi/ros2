#!/usr/bin/env python3
"""
main_controller.py - Main controller integrating vision and robot control
"""

import time
import math
import rclpy
import sys
import os

from move_node.robot_arm_controller import RobotArmController

# Add vision_node to path for importing VisionAPI
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'vision_node'))
from vision_node.vision_api import VisionAPI

def initialize_robot(robot):
    """初始化机器人到Initial Position（垂直向上）"""
    print("\n=== Initialize Robot ===")
    print("Moving to INITIAL position (vertical up)...")
    
    success = robot.move_to_initial_position()
    if success:
        print("✅ Robot initialized successfully")
        # 显示当前关节角度
        angles = robot.get_current_joint_angles_degrees()
        if angles:
            print(f"Current joint angles: {[f'{a:.1f}°' for a in angles]}")
    else:
        print("❌ Failed to initialize robot")
    return success

def move_to_ready_position(robot):
    """移动机器人到Ready Position（工作就绪位置）"""
    print("\n=== Move to Ready Position ===")
    print("Moving to HOME position (ready for work)...")
    
    success = robot.move_to_home()
    if success:
        print("✅ Robot ready for operation")
        # 显示工作空间信息
        workspace_info = robot.get_workspace_info()
        limits = workspace_info['limits']
        print(f"Workspace: R={limits['inner_radius']:.2f}-{limits['outer_radius']:.2f}m, H={limits['z_min']:.2f}-{limits['z_max']:.2f}m")
        
        # 显示当前位置
        current_pos = robot.get_current_position()
        if current_pos:
            x, y, z = current_pos[:3]
            print(f"Current position: ({x:.3f}, {y:.3f}, {z:.3f})")
    else:
        print("❌ Failed to move to ready position")
    return success

def detect_objects(vision_api):
    """检测物体并打印位置信息"""
    print("\n=== Detect Objects ===")
    
    if not vision_api.initialize():
        print("❌ Failed to initialize vision system")
        return []
    
    print("Scanning for objects...")
    positions = vision_api.get_object_positions(timeout=3.0)
    
    if positions:
        print(f"✅ Found {len(positions)} objects:")
        for i, pos in enumerate(positions):
            print(f"  {i+1}. {pos.color.upper()}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
                  f"area={pos.area:.0f} confidence={pos.confidence:.2f}")
        return positions
    else:
        print("❌ No objects detected")
        print("   Make sure colored objects are placed in camera view")
        return []

def move_to_objects(robot, positions):
    """按从近到远顺序移动到物体位置"""
    print("\n=== Move to Objects ===")
    
    if not positions:
        print("No objects to move to")
        return
    
    # 获取机器人当前位置
    current_pos = robot.get_current_position()
    if not current_pos:
        print("❌ Cannot get robot current position")
        return
    
    robot_x, robot_y, robot_z = current_pos[:3]
    print(f"Robot current position: ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})")
    
    # 计算距离并排序（从近到远）
    distances = []
    for pos in positions:
        # 计算水平距离（忽略Z轴差异）
        dist = math.sqrt((pos.x - robot_x)**2 + (pos.y - robot_y)**2)
        distances.append((dist, pos))
    
    # 按距离排序
    distances.sort(key=lambda x: x[0])
    
    print(f"Moving to {len(distances)} objects in order (near to far):")
    
    for i, (dist, pos) in enumerate(distances):
        print(f"\n--- Object {i+1}/{len(distances)} ---")
        print(f"Target: {pos.color.upper()} at ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f})")
        print(f"Distance: {dist:.3f}m")
        
        # 计算安全接近位置（在物体上方20cm）
        target_x = pos.x
        target_y = pos.y
        target_z = pos.z + 0.2  # 在物体上方20cm
        
        print(f"Moving to: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        
        # 使用向下的四元数方向
        success = robot.move_to_position(target_x, target_y, target_z, 0.0, 1.0, 0.0, 0.0)
        
        if success:
            print("✅ Successfully reached target position")
            robot.pause(2.0)  # 停留2秒
        else:
            print("❌ Failed to reach target position")
            print("   Continuing to next object...")
    
    print("\n=== Movement sequence completed ===")

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
            target_z = largest_obj.z + 0.2  # 在物体上方20cm
            
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
            
            # 用于存储检测到的物体
            detected_objects = []
            
            # Show menu
            while True:
                print("\n=== Main Controller Menu ===")
                print("1. Initialize            # Move to initial pose (vertical up)")
                print("2. Ready position        # Move to home pose (ready for work)")
                print("3. Detect objects        # Scan and print object positions")
                print("4. Move to objects       # Move to objects in near-to-far order")
                print("5. Exit")
                
                try:
                    choice = input("\nEnter your choice (1-5): ").strip()
                    
                    if choice == '1':
                        initialize_robot(robot)
                        
                    elif choice == '2':
                        move_to_ready_position(robot)
                        
                    elif choice == '3':
                        detected_objects = detect_objects(vision_api)
                        
                    elif choice == '4':
                        if detected_objects:
                            move_to_objects(robot, detected_objects)
                        else:
                            print("No objects detected. Please run 'Detect objects' first.")
                            
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