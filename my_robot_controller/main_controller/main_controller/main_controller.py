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

            # Display workspace information
            print("📋 圆柱形工作空间信息:")
            workspace_info = robot.get_workspace_info()
            limits = workspace_info['limits']
            print(f"   内半径: {limits['inner_radius']:.2f}m")
            print(f"   外半径: {limits['outer_radius']:.2f}m") 
            print(f"   高度范围: {limits['z_min']:.2f}m 到 {limits['z_max']:.2f}m")
            print(f"   UR5e最大臂展: {workspace_info['max_reach']:.2f}m")
            print(f"   推荐工作区域:")
            print(f"     • 水平距离: {limits['recommended_inner_radius']:.2f}m 到 {limits['recommended_outer_radius']:.2f}m")
            print(f"     • 高度: {limits['recommended_z_min']:.2f}m 到 {limits['recommended_z_max']:.2f}m")
            print()

            # 0. 先到初始化位置，然后到home位置
            print("0. 移动到初始化位置（竖直向上）")
            robot.move_to_initial_position()
            robot.pause(0.5)
            
            print("0.5. 移动到Home位置（工作空间内）")
            robot.move_to_home()
            robot.pause(0.5)   

            print("1. 关节空间运动测试 - 安全位置")
            robot.move_to_joint_positions(0.0, -1.57, -1.0, -1.57, 0.0, 0.0)
            robot.pause(0.5)
            
            print("1.5. 测试关节限制处理")
            # 测试一个超出限制的角度 (例如 J3 超过 180°)
            robot.move_to_joint_positions(0.5, -1.0, 4.0, -1.0, 0.0, 0.0)  # J3=4.0 rad ≈ 229°
            robot.pause(0.5)

            print("2. 相对运动测试")
            robot.move_relative(0.0, 0.0, -0.05)  # 向下5cm
            robot.pause(0.5)              

            print("3. 绝对位置控制 - 推荐工作区域内 (垂直向下)")
            robot.move_to_position(0.3, 0.3, 0.4, 0.0, 1.0, 0.0, 0.0)  # 45度位置，垂直向下
            robot.pause(0.5)  

            print("4. 绝对位置控制 - 正前方位置 (垂直向下)")
            robot.move_to_position(0.0, 0.5, 0.3, 0.0, 1.0, 0.0, 0.0)  # 正前方，垂直向下
            robot.pause(0.5)

            print("5. 🧪 测试圆柱形工作空间边界保护 - 尝试超出外半径")
            print("   尝试移动到水平距离0.8m的位置 (超出外半径0.75m)")
            robot.move_to_position(0.8, 0.0, 0.4, 0.0, 1.0, 0.0, 0.0)  # Should be rejected
            robot.pause(0.5)

            print("6. 🧪 测试工作空间边界保护 - 尝试超出高度上限")
            print("   尝试移动到 Z=0.9m (超出高度上限 0.8m)")
            robot.move_to_position(0.3, 0.3, 0.9, 0.0, 1.0, 0.0, 0.0)  # Should be rejected
            robot.pause(0.5)

            print("7. 🧪 测试工作空间边界保护 - 尝试进入内半径盲区")
            print("   尝试移动到水平距离0.1m的位置 (小于内半径0.15m)")
            robot.move_to_position(0.05, 0.05, 0.4, 0.0, 1.0, 0.0, 0.0)  # Should be rejected
            robot.pause(0.5)

            print("8. 🧪 测试高度下限")
            print("   尝试移动到 Z=-0.2m (低于下限 -0.1m)")
            robot.move_to_position(0.3, 0.3, -0.2, 0.0, 1.0, 0.0, 0.0)  # Should be rejected
            robot.pause(0.5)
            
            print("9. 回到Home位置")
            robot.move_to_home()
            robot.pause(0.5)
            
            print("10. 最后回到初始化位置")
            robot.move_to_initial_position()
            
            print("=== 圆柱形工作空间安全测试完成 ===")
            
        except KeyboardInterrupt:
            print("User interrupted")
        
        print("Main controller finished")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()