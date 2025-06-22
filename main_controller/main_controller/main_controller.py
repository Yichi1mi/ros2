#!/usr/bin/env python3
"""
main_controller.py - Simple main controller using robot API
"""

import time
import rclpy

from move_node.robot_arm_controller import RobotArmController
from scene_manager.scene_manager import SceneManager

def main():
    rclpy.init()
    try:
        print("=" * 50)
        print("Starting main controller")
        print("=" * 50)
        
        # Create robot and scene manager
        robot = RobotArmController()
        scene_manager = SceneManager()
        
        # Check component connections
        arm_connected = robot.is_connected()
        gripper_connected = robot.is_gripper_connected()
        
        print(f"🤖 Robot arm connected: {arm_connected}")
        print(f"🤏 Gripper connected: {gripper_connected}")
        
        if not arm_connected:
            print("❌ Failed to connect to robot arm")
            return
        
        if not gripper_connected:
            print("⚠️  Gripper not connected. Continuing with arm-only tests.")
        
        print("✅ Robot system ready!")
        
        # Setup default scene with table and objects
        print("🏗️ Setting up planning scene...")
        scene_manager.setup_default_scene()
        print("Scene objects:", scene_manager.get_object_list())
        time.sleep(2)  # Allow scene to be published
        
        try:
            # Set speed and pause between movements
            robot.set_speed(50)  # 50% speed
            robot.set_pause_between_movements(0.5)  # 0.5 second pause between movements
            
            print("=== Panda Robot Controller Ready ===")
            print("Robot is ready for commands!")
            print("Panda 7-DOF arm with gripper ready for control")
            print("=" * 50)
            
            # ========================================
            # YOUR CODE GOES HERE - 你的代码写在这里
            # ========================================

            # Display system information
            print("📋 System Information:")
            workspace_info = robot.get_workspace_info()
            limits = workspace_info['limits']
            print(f"   Safe X range: {limits['x_min']:.1f} to {limits['x_max']:.1f}m")
            print(f"   Safe Y range: {limits['y_min']:.1f} to {limits['y_max']:.1f}m") 
            print(f"   Safe Z range: {limits['z_min']:.1f} to {limits['z_max']:.1f}m")
            print(f"   Max reach: {workspace_info['max_reach']:.2f}m from origin")
            
            # Display gripper information
            if gripper_connected:
                gripper_info = robot.get_gripper_info()
                print(f"   Gripper available: ✅")
                if gripper_info['is_homed']:
                    print(f"   Gripper max width: {gripper_info['max_width_mm']:.1f}mm")
                else:
                    print(f"   Gripper status: Not homed")
            else:
                print(f"   Gripper available: ❌")
            print()

            # initial home position
            print("0. 回归home位置")
            robot.move_to_home()
            robot.pause(0.5)   

            print("1. 关节空间运动到位置1")
            robot.move_to_joint_positions(1.0, -0.5, 0.0, -1.5, 0.0, 1.0, 0.5)
            robot.pause(0.5)
            
            print("1.5. 测试关节限制处理")
            # 测试一个超出限制的角度 (例如 J3 超出限制)
            robot.move_to_joint_positions(0.5, -0.5, 4.0, -1.5, 0.0, 1.0, 0.5)  # J3=4.0 rad 超出限制
            robot.pause(0.5)

            print("2. 相对运动测试")
            robot.move_relative(0.0, 0.0, -0.1)  # 向下10cm
            robot.pause(0.5)              

            print("3. 绝对位置控制 (安全位置)")
            robot.move_to_position(0.4, 0.0, 0.5, 1.000, 0.000, 0.000, 0.000)
            robot.pause(0.5)  

            print("4. 绝对位置控制 (安全位置)")
            robot.move_to_position(0.5, 0.3, 0.7, 1.000, 0.000, 0.000, 0.000)
            robot.pause(0.5)

            print("5. 🧪 测试工作空间边界保护 - 尝试超出X边界")
            print("   尝试移动到 X=1.0m (超出前边界 0.8m)")
            robot.move_to_position(1.0, 0.0, 0.5, 1.000, 0.000, 0.000, 0.000)  # Should be rejected
            robot.pause(0.5)

            print("6. 🧪 测试工作空间边界保护 - 尝试超出Z下边界")
            print("   尝试移动到 Z=0.1m (低于下边界 0.2m)")
            robot.move_to_position(0.4, 0.0, 0.1, 1.000, 0.000, 0.000, 0.000)  # Should be rejected
            robot.pause(0.5)

            print("7. 🧪 测试工作空间边界保护 - 尝试超出最大reach")
            print("   尝试移动到距离原点1.0m的位置 (超出最大reach 0.855m)")
            robot.move_to_position(0.7, 0.5, 0.7, 1.000, 0.000, 0.000, 0.000)  # Should be rejected
            robot.pause(0.5)

            print("8. 回到接近home的位置")
            robot.move_to_position(0.4, 0.0, 0.6, 1.000, 0.000, 0.000, 0.000)
            robot.pause(0.5)
            
            print("9. 回到home位置")
            robot.move_to_home()
            
            print("=== 工作空间安全测试完成 ===")
            print()
            
            # ========================================
            # GRIPPER CONTROL TESTS - 抓手控制测试
            # ========================================
            if gripper_connected:
                print("=== GRIPPER CONTROL TESTS ===")
                print("开始抓手控制测试...")
                
                try:
                    # 1. Home gripper
                    print("1. 校准抓手 (Homing gripper)...")
                    if robot.home_gripper():
                        gripper_info = robot.get_gripper_info()
                        if gripper_info['max_width_mm']:
                            print(f"   抓手校准成功! 最大开口: {gripper_info['max_width_mm']:.1f}mm")
                    robot.pause(1.0)
                    
                    # 2. Basic open/close test
                    print("2. 基础开关测试...")
                    print("   打开抓手...")
                    robot.open_gripper()
                    
                    print("   关闭抓手...")
                    robot.close_gripper()
                    
                    # 3. Specific width control
                    print("3. 精确宽度控制测试...")
                    print("   设置到50mm...")
                    robot.set_gripper_width(50)
                    
                    print("   设置到30mm...")
                    robot.set_gripper_width(30)
                    
                    print("   设置到10mm...")
                    robot.set_gripper_width(10)
                    
                    print("   重新打开...")
                    robot.open_gripper()
                    
                    # 4. Grasp force test
                    print("4. 抓取力度测试...")
                    print("   轻力抓取测试 (20N)...")
                    robot.grasp_object(force_n=20.0)
                    robot.pause(1.0)
                    
                    print("   中力抓取测试 (40N)...")
                    robot.grasp_object(force_n=40.0)
                    robot.pause(1.0)
                    
                    # 5. Final open
                    print("5. 最终打开抓手...")
                    robot.open_gripper()
                    
                    print("=== 抓手测试完成 ===")
                    
                except Exception as e:
                    print(f"❌ 抓手测试出错: {e}")
                    print("继续其他测试...")
                
                print()
            else:
                print("=== GRIPPER TESTS SKIPPED ===")
                print("抓手不可用，跳过抓手测试")
                print()
            
            
        except KeyboardInterrupt:
            print("User interrupted")
        
        print("Main controller finished")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()