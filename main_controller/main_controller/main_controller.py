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
            # YOUR CODE GOES HERE - 你的代码写在这里
            # ========================================
            
            # 演示正确的机器人控制策略:
            print("=== 正确的机器人控制策略演示 ===")
            
            # 1. 关节空间运动 - 移动到已知安全位置
            print("1. 关节空间运动到home位置")
            robot.move_to_joint_positions([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], "Move to home")
            robot.wait_for_completion()
            
            # 2. 读取传感器数据 - 关节角度
            print("2. 读取当前关节角度 (传感器数据)")
            joint_angles = robot.get_current_joint_angles_degrees()
            if joint_angles:
                print(f"   关节角度(度): {[f'{j:.1f}°' for j in joint_angles]}")
            
            # 3. 正向运动学 - 计算末端位姿
            print("3. 通过正向运动学计算末端位姿")
            current_pose = robot.get_current_cartesian_pose()
            if current_pose:
                x, y, z = current_pose.position.x, current_pose.position.y, current_pose.position.z
                print(f"   末端位置: x={x:.3f}, y={y:.3f}, z={z:.3f}")
                print(f"   末端姿态: x={current_pose.orientation.x:.3f}, y={current_pose.orientation.y:.3f}, z={current_pose.orientation.z:.3f}, w={current_pose.orientation.w:.3f}")
            
            # 4. 笛卡尔空间相对运动 (基于FK计算的位姿)
            print("4. 笛卡尔空间相对运动")
            robot.move_relative_xyz(0, 0, -0.1, "向下移动10cm")
            robot.wait_for_completion()
            
            robot.move_relative_xyz(0.03, 0, 0, "向前移动3cm") 
            robot.wait_for_completion()
            
            robot.move_relative_xyz(0, 0, 0.02, "向上移动2cm")
            robot.wait_for_completion()
            
            robot.move_relative_xyz(-0.03, 0, 0, "向后移动3cm")
            robot.wait_for_completion()
            
            # 5. 关节空间运动 - 改变机器人构型
            print("5. 改变机器人构型 (关节空间)")
            robot.move_to_joint_positions([0.785, -1.0, 0.0, -1.5, 0.0, 0.0], "新的关节构型")
            robot.wait_for_completion()
            
            # 6. 验证新构型下的末端位姿
            print("6. 验证新构型下的末端位姿")
            new_pose = robot.get_current_cartesian_pose()
            if new_pose:
                x, y, z = new_pose.position.x, new_pose.position.y, new_pose.position.z
                print(f"   新末端位置: x={x:.3f}, y={y:.3f}, z={z:.3f}")
            
            print("=== 控制策略演示完成 ===")
            
            print("\nMain controller finished - ready for your custom code!")
            
        except KeyboardInterrupt:
            print("User interrupted")
            robot.stop_execution()
            robot.clear_queue()
        
        print("Main controller finished")
    
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()