#!/usr/bin/env python3
"""
robot_arm_controller.py - Simplified robot arm control API.
Provides clean, minimal joint and Cartesian movement interfaces.
All movements are blocking and auto-wait for completion.
"""

import math
from .joint_controller import JointController
from .position_controller import PositionController
from .gripper_controller import GripperController

class RobotArmController:
    """
    Simplified robot arm controller with direct execution (no queue).
    Provides clean, minimal API for joint and position control.
    """

    def __init__(self):
        self.joint_controller = JointController()
        self.position_controller = PositionController()
        self.gripper_controller = GripperController()
        self.HOME_POSITION = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # Panda ready position
        self._pause_between_movements = 0.5  # Default pause in seconds
        
        # Define safe workspace boundaries (in meters)
        # Based on Panda specs: 855mm reach, mounted at table edge
        # Coordinate system: x=forward, y=left, z=up from base
        self.WORKSPACE_LIMITS = {
            'x_min': -0.6,    # Behind base limit
            'x_max': +0.8,    # Forward reach limit  
            'y_min': -0.6,    # Right limit
            'y_max': +0.6,    # Left limit
            'z_min': 0.2,     # Above table surface
            'z_max': 1.0      # Safe upper limit
        }

    def is_connected(self):
        """Check if robot is connected."""
        return (self.joint_controller.is_connected() and 
                self.position_controller.is_connected() and 
                self.gripper_controller.is_connected())

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
        max_safe_distance = 0.855  # Panda max reach is 855mm
        if distance_from_origin > max_safe_distance:
            errors.append(f"Distance from origin {distance_from_origin:.3f}m exceeds max reach ({max_safe_distance:.3f}m)")
        
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
            'max_reach': 0.855
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

    def move_to_joint_positions(self, j1, j2, j3, j4, j5, j6, j7):
        """
        Move to joint positions (blocking, auto-wait for completion).
        :param j1-j7: Joint angles in radians for Panda
        :return: True if successful
        """
        angles = [j1, j2, j3, j4, j5, j6, j7]
        
        # Check joint limits for Panda (in radians) - from URDF
        joint_limits = [
            (-2.9671, 2.9671),     # J1: ±170°
            (-1.8326, 1.8326),     # J2: ±105°
            (-2.9671, 2.9671),     # J3: ±170°
            (-3.1416, 0.0873),     # J4: -180° to +5°
            (-2.9671, 2.9671),     # J5: ±170°
            (-0.0873, 3.8223),     # J6: -5° to +219°
            (-2.9671, 2.9671),     # J7: ±170°
        ]
        
        # Get current joint positions for shortest path calculation
        current_positions = self.get_current_joint_angles_radians()
        if current_positions is None:
            current_positions = [0.0] * 7  # Default if no current position available
        
        # Normalize angles considering shortest path and joint limits
        normalized_angles = []
        for i, angle in enumerate(angles):
            min_limit, max_limit = joint_limits[i]
            current_angle = current_positions[i] if i < len(current_positions) else 0.0
            
            # Find the equivalent angle closest to current position
            # Consider multiple wraps: angle, angle±2π, angle±4π
            candidates = [
                angle,
                angle + 2*math.pi,
                angle - 2*math.pi,
                angle + 4*math.pi,
                angle - 4*math.pi
            ]
            
            # Filter candidates that are within joint limits
            valid_candidates = [a for a in candidates if min_limit <= a <= max_limit]
            
            if valid_candidates:
                # Choose the candidate closest to current position
                normalized = min(valid_candidates, key=lambda a: abs(a - current_angle))
            else:
                # If no valid candidate, clamp to limits
                normalized = max(min_limit, min(max_limit, angle))
                print(f"⚠️  Warning: Joint {i+1} angle {math.degrees(angle):.1f}° cannot be reached within limits")
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

    # ==================== GRIPPER CONTROL API ====================
    
    def open_gripper(self):
        """
        Open gripper to maximum safe width.
        :return: True if successful
        """
        print("Opening gripper")
        success = self.gripper_controller.open_gripper()
        if self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    def close_gripper(self):
        """
        Close gripper completely.
        :return: True if successful
        """
        print("Closing gripper")
        success = self.gripper_controller.close_gripper()
        if self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    def grasp_with_force(self, max_force=30.0, target_width=0.01):
        """
        Attempt to grasp an object with specified maximum force.
        :param max_force: Maximum grasping force in Newtons
        :param target_width: Minimum width to maintain in meters
        :return: True if grasp attempt completed
        """
        print(f"Attempting to grasp with force: {max_force:.1f}N")
        success = self.gripper_controller.grasp_object(
            target_width=target_width, 
            max_effort=max_force
        )
        if self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    def move_gripper_to_width(self, width, max_force=50.0):
        """
        Move gripper to specific width.
        :param width: Target width in meters (0.0 = closed, 0.08 = fully open)
        :param max_force: Maximum force in Newtons
        :return: True if successful
        """
        print(f"Moving gripper to width: {width*1000:.1f}mm")
        success = self.gripper_controller.move_to_width(width, max_force)
        if self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    # ==================== GRIPPER STATUS API ====================
    
    def get_gripper_width(self):
        """
        Get current gripper width in meters.
        :return: Width in meters, or None if unavailable
        """
        return self.gripper_controller.get_current_width()
    
    def is_gripper_open(self):
        """
        Check if gripper is nearly fully open.
        :return: True if gripper is open (> 60mm)
        """
        width = self.get_gripper_width()
        return width is not None and width > 0.06
    
    def is_gripper_closed(self):
        """
        Check if gripper is nearly fully closed.
        :return: True if gripper is closed (< 5mm)
        """
        width = self.get_gripper_width()
        return width is not None and width < 0.005
    
    def is_grasping_object(self):
        """
        Intelligent detection of whether gripper is grasping an object.
        Based on position, force, and stall detection.
        :return: True if likely grasping an object
        """
        return self.gripper_controller.is_grasping()
    
    def get_gripper_info(self):
        """
        Get comprehensive gripper status information.
        :return: Dictionary with gripper state information
        """
        width = self.get_gripper_width()
        state = self.gripper_controller.get_gripper_state()
        
        info = {
            'width_mm': width * 1000 if width is not None else None,
            'width_m': width,
            'is_open': self.is_gripper_open(),
            'is_closed': self.is_gripper_closed(),
            'is_grasping': self.is_grasping_object()
        }
        
        if state:
            info.update({
                'force_N': state.effort,
                'is_stalled': state.stalled,
                'reached_goal': state.reached_goal
            })
        
        return info


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