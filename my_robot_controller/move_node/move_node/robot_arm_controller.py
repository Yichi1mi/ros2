#!/usr/bin/env python3
"""
robot_arm_controller.py - Simplified robot arm control API.
Provides clean, minimal joint and Cartesian movement interfaces.
All movements are blocking and auto-wait for completion.
"""

import math
from .joint_controller import JointController
from .position_controller import PositionController

class RobotArmController:
    """
    Simplified robot arm controller with direct execution (no queue).
    Provides clean, minimal API for joint and position control.
    """

    def __init__(self):
        self.joint_controller = JointController()
        self.position_controller = PositionController()
        self.HOME_POSITION = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self._pause_between_movements = 0.5  # Default pause in seconds
        
        # Define safe workspace boundaries (in meters)
        # Based on UR5 specs: 850mm reach, home at (0.001, 0.191, 1.001)
        self.WORKSPACE_LIMITS = {
            'x_min': -0.6,    # Left limit
            'x_max': +0.6,    # Right limit  
            'y_min': -0.3,    # Back limit (closer to base)
            'y_max': +0.8,    # Front limit (further from base)
            'z_min': 0.3,     # Lower limit (avoid table collision)
            'z_max': 1.2      # Upper limit (avoid overextension)
        }

    def is_connected(self):
        """Check if robot is connected."""
        return self.joint_controller.is_connected() and self.position_controller.is_connected()

    def _is_position_safe(self, x, y, z):
        """
        Check if target position is within safe workspace boundaries.
        :param x, y, z: Target position coordinates in meters
        :return: (is_safe: bool, error_message: str)
        """
        errors = []
        
        # Check X boundaries
        if x < self.WORKSPACE_LIMITS['x_min']:
            errors.append(f"X={x:.3f}m exceeds left limit ({self.WORKSPACE_LIMITS['x_min']:.1f}m)")
        elif x > self.WORKSPACE_LIMITS['x_max']:
            errors.append(f"X={x:.3f}m exceeds right limit ({self.WORKSPACE_LIMITS['x_max']:.1f}m)")
            
        # Check Y boundaries  
        if y < self.WORKSPACE_LIMITS['y_min']:
            errors.append(f"Y={y:.3f}m exceeds back limit ({self.WORKSPACE_LIMITS['y_min']:.1f}m)")
        elif y > self.WORKSPACE_LIMITS['y_max']:
            errors.append(f"Y={y:.3f}m exceeds front limit ({self.WORKSPACE_LIMITS['y_max']:.1f}m)")
            
        # Check Z boundaries
        if z < self.WORKSPACE_LIMITS['z_min']:
            errors.append(f"Z={z:.3f}m exceeds lower limit ({self.WORKSPACE_LIMITS['z_min']:.1f}m)")
        elif z > self.WORKSPACE_LIMITS['z_max']:
            errors.append(f"Z={z:.3f}m exceeds upper limit ({self.WORKSPACE_LIMITS['z_max']:.1f}m)")
        
        # Check distance from origin (additional safety check)
        distance_from_origin = math.sqrt(x*x + y*y + z*z)
        max_safe_distance = 0.85  # UR5 max reach is 850mm, use full range
        if distance_from_origin > max_safe_distance:
            errors.append(f"Distance from origin {distance_from_origin:.3f}m exceeds max reach ({max_safe_distance:.2f}m)")
        
        if errors:
            error_msg = "🚫 WORKSPACE SAFETY VIOLATION:\n" + "\n".join(f"   • {error}" for error in errors)
            return False, error_msg
        
        return True, ""

    def get_workspace_info(self):
        """
        Get workspace boundary information.
        :return: dict with workspace limits and current position info
        """
        info = {
            'limits': self.WORKSPACE_LIMITS.copy(),
            'max_reach': 0.85
        }
        
        # Add current position if available
        current_pos = self.get_current_position()
        if current_pos:
            x, y, z = current_pos[:3]
            info['current_position'] = {'x': x, 'y': y, 'z': z}
            info['current_distance_from_origin'] = math.sqrt(x*x + y*y + z*z)
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
        
        # Check joint limits for UR5 (in radians)
        # UR5 joint limits: ±360° for most joints, some have different limits
        joint_limits = [
            (-2*math.pi, 2*math.pi),     # J1: ±360°
            (-2*math.pi, 2*math.pi),     # J2: ±360°  
            (-math.pi, math.pi),         # J3: ±180°
            (-2*math.pi, 2*math.pi),     # J4: ±360°
            (-2*math.pi, 2*math.pi),     # J5: ±360°
            (-2*math.pi, 2*math.pi),     # J6: ±360°
        ]
        
        # Normalize angles to shortest path and check limits
        normalized_angles = []
        for i, angle in enumerate(angles):
            # Normalize to -π to π range first
            normalized = ((angle + math.pi) % (2*math.pi)) - math.pi
            
            # Check if within joint limits
            min_limit, max_limit = joint_limits[i]
            if normalized < min_limit or normalized > max_limit:
                print(f"⚠️  Warning: Joint {i+1} angle {math.degrees(normalized):.1f}° exceeds limits ({math.degrees(min_limit):.1f}° to {math.degrees(max_limit):.1f}°)")
                # Clamp to limits
                normalized = max(min_limit, min(max_limit, normalized))
                print(f"   Clamped to: {math.degrees(normalized):.1f}°")
            
            normalized_angles.append(normalized)
        
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
        # Safety check: validate workspace boundaries
        is_safe, error_msg = self._is_position_safe(x, y, z)
        if not is_safe:
            print(error_msg)
            print(f"💡 Safe workspace limits:")
            print(f"   X: {self.WORKSPACE_LIMITS['x_min']:.1f} to {self.WORKSPACE_LIMITS['x_max']:.1f}m")
            print(f"   Y: {self.WORKSPACE_LIMITS['y_min']:.1f} to {self.WORKSPACE_LIMITS['y_max']:.1f}m")
            print(f"   Z: {self.WORKSPACE_LIMITS['z_min']:.1f} to {self.WORKSPACE_LIMITS['z_max']:.1f}m")
            print(f"   Max reach: 0.85m from origin")
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

    def move_to_home(self):
        """Move to home position."""
        print("Moving to home position")
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