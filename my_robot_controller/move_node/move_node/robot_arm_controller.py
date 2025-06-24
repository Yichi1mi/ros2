#!/usr/bin/env python3
"""
robot_arm_controller.py - Simplified robot arm control API.
Provides clean, minimal joint and Cartesian movement interfaces.
All movements are blocking and auto-wait for completion.
"""

import math
from .joint_controller import JointController
from .position_controller import PositionController
from robot_common import UR5ArmGeometry

class RobotArmController:
    """
    Simplified robot arm controller with direct execution (no queue).
    Provides clean, minimal API for joint and position control.
    """

    def __init__(self):
        self.joint_controller = JointController()
        self.position_controller = PositionController()
        
        # 创建几何处理器
        self.arm_geometry = UR5ArmGeometry()
        
        # 定义机械臂姿态
        self.INITIAL_POSITION = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # 竖直向上，初始化用
        self.HOME_POSITION = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]  # 工作空间内的家位置
        self._pause_between_movements = 0.5  # Default pause in seconds
        
        # Define safe workspace boundaries (in meters) - Cylindrical workspace for UR5e
        # Based on UR5e specs: 850mm max reach, base height ~163mm
        self.WORKSPACE_LIMITS = {
            # Cylindrical workspace parameters
            'inner_radius': 0.15,    # 内半径15cm - 避免基座附近盲区
            'outer_radius': 0.75,    # 外半径75cm - 安全的最大半径 (< 85cm max reach)
            'z_min': -0.10,          # 最低高度 - 基座下方10cm (桌面操作)
            'z_max': 0.80,           # 最高高度 - 基座上方80cm (避免过度伸展)
            
            # 推荐的高效工作区域
            'recommended_inner_radius': 0.20,  # 推荐内半径20cm
            'recommended_outer_radius': 0.65,  # 推荐外半径65cm
            'recommended_z_min': 0.10,         # 推荐最低高度10cm
            'recommended_z_max': 0.60,         # 推荐最高高度60cm
        }
        

    def is_connected(self):
        """Check if robot is connected."""
        return self.joint_controller.is_connected() and self.position_controller.is_connected()

    def _is_position_safe(self, x, y, z):
        """
        Check if target position is within safe cylindrical workspace boundaries.
        :param x, y, z: Target position coordinates in meters
        :return: (is_safe: bool, error_message: str)
        """
        errors = []
        
        # Calculate horizontal distance from base center (cylindrical coordinates)
        horizontal_distance = math.sqrt(x*x + y*y)
        
        # Check Z boundaries (height limits)
        if z < self.WORKSPACE_LIMITS['z_min']:
            errors.append(f"高度Z={z:.3f}m低于最低限制 ({self.WORKSPACE_LIMITS['z_min']:.2f}m)")
        elif z > self.WORKSPACE_LIMITS['z_max']:
            errors.append(f"高度Z={z:.3f}m超出最高限制 ({self.WORKSPACE_LIMITS['z_max']:.2f}m)")
        
        # Check cylindrical boundaries
        if horizontal_distance < self.WORKSPACE_LIMITS['inner_radius']:
            errors.append(f"水平距离{horizontal_distance:.3f}m小于内半径限制 ({self.WORKSPACE_LIMITS['inner_radius']:.2f}m)")
        elif horizontal_distance > self.WORKSPACE_LIMITS['outer_radius']:
            errors.append(f"水平距离{horizontal_distance:.3f}m超出外半径限制 ({self.WORKSPACE_LIMITS['outer_radius']:.2f}m)")
        
        # Additional 3D distance check (total reach limit)
        total_distance = math.sqrt(x*x + y*y + z*z)
        absolute_max_reach = 0.85  # UR5e absolute maximum reach
        if total_distance > absolute_max_reach:
            errors.append(f"总距离{total_distance:.3f}m超出UR5e最大臂展 ({absolute_max_reach:.2f}m)")
        
        # Check if in recommended zone (warning, not error)
        in_recommended_zone = (
            self.WORKSPACE_LIMITS['recommended_inner_radius'] <= horizontal_distance <= self.WORKSPACE_LIMITS['recommended_outer_radius'] and
            self.WORKSPACE_LIMITS['recommended_z_min'] <= z <= self.WORKSPACE_LIMITS['recommended_z_max']
        )
        
        if errors:
            error_msg = "🚫 圆柱形工作空间安全违规:\n" + "\n".join(f"   • {error}" for error in errors)
            return False, error_msg
        elif not in_recommended_zone:
            # Position is safe but outside recommended zone
            print(f"⚠️  位置在安全区域但超出推荐工作区域 (水平距离: {horizontal_distance:.3f}m, 高度: {z:.3f}m)")
        
        return True, ""

    def get_workspace_info(self):
        """
        Get cylindrical workspace boundary information.
        :return: dict with workspace limits and current position info
        """
        info = {
            'limits': self.WORKSPACE_LIMITS.copy(),
            'max_reach': 0.85,
            'workspace_type': 'cylindrical'
        }
        
        # Add current position if available
        current_pos = self.get_current_position()
        if current_pos:
            x, y, z = current_pos[:3]
            horizontal_distance = math.sqrt(x*x + y*y)
            total_distance = math.sqrt(x*x + y*y + z*z)
            
            info['current_position'] = {'x': x, 'y': y, 'z': z}
            info['current_horizontal_distance'] = horizontal_distance
            info['current_total_distance'] = total_distance
            is_safe, _ = self._is_position_safe(x, y, z)
            info['current_position_safe'] = is_safe
        
        return info

    def move_to_joint_positions(self, j1, j2, j3, j4, j5, j6):
        """
        Move to joint positions (blocking, auto-wait for completion).
        :param j1-j6: Joint angles in radians
        :return: True if successful
        """
        angles = [j1, j2, j3, j4, j5, j6]
        
        # 检查是否是初始化位置，如果是则跳过工作空间检查
        if angles == self.INITIAL_POSITION:
            print(f"🔧 移动到初始化位置，跳过工作空间检查")
            success = self.joint_controller.move_to_joint_positions(angles)
            if success and self._pause_between_movements > 0:
                import time
                time.sleep(self._pause_between_movements)
            return success
        
        # 使用几何处理器进行角度标准化和限制检查
        normalized_angles = self.arm_geometry.normalize_joint_angles(angles)
        
        print(f"Moving to joint positions: {[f'{a:.3f}' for a in normalized_angles]}")
        print(f"Joint angles (degrees): {[f'{math.degrees(a):.1f}°' for a in normalized_angles]}")
        
        success = self.joint_controller.move_to_joint_positions(normalized_angles)
        if success and self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success

    def move_to_position(self, x, y, z, qx, qy, qz, qw):
        """
        Move to Cartesian position with orientation (blocking, auto-wait for completion).
        :param x, y, z: Position coordinates in meters
        :param qx, qy, qz, qw: Orientation quaternion
        :return: True if successful
        """
        # Safety check: validate cylindrical workspace boundaries
        is_safe, error_msg = self._is_position_safe(x, y, z)
        if not is_safe:
            print(error_msg)
            print(f"💡 圆柱形安全工作空间:")
            print(f"   内半径: {self.WORKSPACE_LIMITS['inner_radius']:.2f}m")
            print(f"   外半径: {self.WORKSPACE_LIMITS['outer_radius']:.2f}m") 
            print(f"   高度范围: {self.WORKSPACE_LIMITS['z_min']:.2f}m 到 {self.WORKSPACE_LIMITS['z_max']:.2f}m")
            print(f"   UR5e最大臂展: 0.85m")
            print(f"   推荐工作区域:")
            print(f"     • 水平距离: {self.WORKSPACE_LIMITS['recommended_inner_radius']:.2f}m 到 {self.WORKSPACE_LIMITS['recommended_outer_radius']:.2f}m")
            print(f"     • 高度: {self.WORKSPACE_LIMITS['recommended_z_min']:.2f}m 到 {self.WORKSPACE_LIMITS['recommended_z_max']:.2f}m")
            return False
        
        print(f"✅ Position within safe workspace")
        print(f"Moving to position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        print(f"With orientation: qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}")
        
        pose = {
            'position': {'x': x, 'y': y, 'z': z},
            'orientation': {'x': qx, 'y': qy, 'z': qz, 'w': qw}
        }
        success = self.position_controller.move_to_cartesian_position(pose)
        if success and self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    def move_relative(self, dx, dy, dz):
        """
        Move relative to current position (blocking, auto-wait for completion).
        Uses current position + offset, maintains current orientation.
        :param dx, dy, dz: Relative movement in meters
        :return: True if successful
        """
        print(f"Moving relative: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
        
        # Get current position (x, y, z, qx, qy, qz, qw)
        current_position = self.get_current_position()
        if current_position is None:
            print("❌ Cannot get current position for relative move")
            return False
        
        x, y, z, qx, qy, qz, qw = current_position
        print(f"Current position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        # Calculate target position: current + relative offset
        target_x = x + dx
        target_y = y + dy
        target_z = z + dz
        
        print(f"Target position: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")
        print(f"Maintaining orientation: qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f}")
        
        # Use move_to_position with current orientation (includes workspace validation)
        success = self.move_to_position(target_x, target_y, target_z, qx, qy, qz, qw)
        
        if success and self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
            
        return success

    def move_to_initial_position(self):
        """
        移动到初始化位置（竖直向上）
        这个方法用于机械臂启动时的初始化，跳过工作空间检查
        """
        print("🔧 移动到初始化位置（竖直向上）")
        return self.move_to_joint_positions(*self.INITIAL_POSITION)

    def move_to_home(self):
        """
        移动到Home位置（工作空间内的安全位置）
        这个位置在安全工作空间内，适合作为工作的起始点
        """
        print("🏠 移动到Home位置")
        return self.move_to_joint_positions(*self.HOME_POSITION)
    
    def pause(self, duration):
        """
        Pause for specified duration.
        :param duration: Pause duration in seconds
        """
        print(f"Pausing for {duration:.1f}s")
        import time
        time.sleep(duration)

    def set_speed(self, percentage):
        """Set movement speed percentage."""
        self.joint_controller.set_velocity_percentage(percentage)
        self.position_controller.set_velocity_percentage(percentage)
        print(f"Speed set to {percentage}%")
    
    def set_pause_between_movements(self, pause_seconds):
        """
        Set pause duration between movements.
        :param pause_seconds: Pause duration in seconds (float).
        """
        self._pause_between_movements = max(0.0, pause_seconds)
        print(f"Pause between movements set to {self._pause_between_movements:.1f}s")

    def get_current_joint_angles_radians(self):
        """Get current joint angles in radians (raw sensor data)."""
        return self.joint_controller.get_current_joint_positions()
    
    def get_current_joint_angles_degrees(self):
        """Get current joint angles in degrees."""
        pos = self.get_current_joint_angles_radians()
        if pos:
            return [math.degrees(p) for p in pos]
        return None
    
    def get_current_cartesian_pose(self):
        """
        Get current end-effector pose using forward kinematics.
        This is computed from current joint angles via FK.
        """
        return self.position_controller.get_current_pose()
    
    def get_current_position(self):
        """
        Get current position as tuple (x, y, z, qx, qy, qz, qw).
        Returns None if pose cannot be obtained.
        """
        # Ensure we wait for latest state
        import time
        time.sleep(0.1)
        
        pose = self.get_current_cartesian_pose()
        if pose:
            return (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
            )
        return None


def main(args=None):
    """Test function for robot arm controller."""
    import rclpy
    
    rclpy.init(args=args)
    
    try:
        print("🤖 Initializing UR5 Robot Arm Controller...")
        robot = RobotArmController()
        
        # Check connections
        print(f"Robot arm connected: {robot.is_connected()}")
        
        if not robot.is_connected():
            print("❌ Robot arm not connected. Make sure simulation is running.")
            return
        
        # Test basic movements
        print("\n=== BASIC ARM TESTS ===")
        
        print("1. Moving to home position...")
        robot.move_to_home()
        
        current_pos = robot.get_current_position()
        if current_pos:
            x, y, z, _, _, _, _ = current_pos
            print(f"Current position: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        print("\n✅ Robot controller test completed!")
        print("Use this controller in your applications for arm control.")
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Error in robot controller test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()