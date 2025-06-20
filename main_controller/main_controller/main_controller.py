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
            
            print("=== Panda Robot Controller Ready ===")
            print("Robot is ready for commands!")
            print("Panda 7-DOF arm with gripper ready for control")
            print("=" * 50)
            
            # ========================================
            # YOUR CODE GOES HERE - 你的代码写在这里
            # ========================================

            # Display workspace information
            print("📋 Workspace Information:")
            workspace_info = robot.get_workspace_info()
            limits = workspace_info['limits']
            print(f"   Safe X range: {limits['x_min']:.1f} to {limits['x_max']:.1f}m")
            print(f"   Safe Y range: {limits['y_min']:.1f} to {limits['y_max']:.1f}m") 
            print(f"   Safe Z range: {limits['z_min']:.1f} to {limits['z_max']:.1f}m")
            print(f"   Max reach: {workspace_info['max_reach']:.2f}m from origin")
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
            robot.move_to_position(0.3, 0.0, 0.5, 1.000, 0.000, 0.000, 0.000)
            robot.pause(0.5)  

            print("4. 绝对位置控制 (安全位置)")
            robot.move_to_position(0.4, 0.2, 0.6, 1.000, 0.000, 0.000, 0.000)
            robot.pause(0.5)

            print("5. 🧪 测试工作空间边界保护 - 尝试超出X边界")
            print("   尝试移动到 X=1.0m (超出右边界 0.7m)")
            robot.move_to_position(1.0, 0.0, 0.5, 1.000, 0.000, 0.000, 0.000)  # Should be rejected
            robot.pause(0.5)

            print("6. 🧪 测试工作空间边界保护 - 尝试超出Z下边界")
            print("   尝试移动到 Z=0.05m (低于下边界 0.1m)")
            robot.move_to_position(0.3, 0.0, 0.05, 1.000, 0.000, 0.000, 0.000)  # Should be rejected
            robot.pause(0.5)

            print("7. 🧪 测试工作空间边界保护 - 尝试超出最大reach")
            print("   尝试移动到距离原点1.0m的位置 (超出最大reach 0.855m)")
            robot.move_to_position(0.6, 0.6, 0.6, 1.000, 0.000, 0.000, 0.000)  # Should be rejected
            robot.pause(0.5)

            print("8. 回到接近home的位置")
            robot.move_to_position(0.3, 0.0, 0.6, 1.000, 0.000, 0.000, 0.000)
            robot.pause(0.5)
            
            print("9. 回到home位置")
            robot.move_to_home()
            
            print("=== 工作空间安全测试完成 ===")
            
        except KeyboardInterrupt:
            print("User interrupted")
        
        print("Main controller finished")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()