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
from .robot_config import get_config, validate_mode

class RobotArmController:
    """
    Dual-mode robot arm controller supporting MoveIt and Gazebo simulation.
    Provides clean, minimal API for joint and position control.
    """

    def __init__(self, mode="moveit"):
        """
        Initialize robot arm controller
        
        Args:
            mode: "moveit" or "gazebo" - simulation mode to use
        """
        if not validate_mode(mode):
            raise ValueError(f"Invalid mode: {mode}. Use 'moveit' or 'gazebo'")
            
        self.mode = mode
        self.config = get_config(mode)
        
        print(f"🤖 初始化机器人控制器: {self.config['name']}")
        print(f"   {self.config['description']}")
        
        # Initialize controllers based on mode
        if mode == "moveit":
            self.joint_controller = JointController()
            self.position_controller = PositionController()
            self.gripper_controller = GripperController() if self.config['gripper_available'] else None
        elif mode == "gazebo":
            # TODO: Implement Gazebo controllers
            self.joint_controller = JointController()  # Will be replaced with GazeboJointController
            self.position_controller = PositionController()  # Will be replaced with GazeboPositionController
            self.gripper_controller = GripperController() if self.config['gripper_available'] else None
        self.HOME_POSITION = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]  # FR3 ready position
        self._pause_between_movements = 0.5  # Default pause in seconds
        
        # Define safe workspace boundaries (in meters)
        # Based on FR3 specs: 855mm reach, mounted at table edge
        # Coordinate system: x=forward, y=left, z=up from base
        self.WORKSPACE_LIMITS = {
            'x_min': -0.6,    # Behind base limit
            'x_max': +0.8,    # Forward reach limit  
            'y_min': -0.6,    # Right limit
            'y_max': +0.6,    # Left limit
            'z_min': 0.15,    # Above elevated table surface (20cm + safety)
            'z_max': 1.0      # Safe upper limit
        }

    def is_connected(self):
        """Check if robot arm is connected (gripper optional)."""
        return (self.joint_controller.is_connected() and 
                self.position_controller.is_connected())
    
    def is_gripper_connected(self):
        """Check if gripper is connected."""
        if self.gripper_controller is None:
            return False
        return self.gripper_controller.is_connected()
    
    def is_gripper_available(self):
        """Check if gripper functionality is available in current mode."""
        return self.config['gripper_available'] and self.gripper_controller is not None

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
        max_safe_distance = 0.855  # FR3 max reach is 855mm
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
        :param j1-j7: Joint angles in radians for FR3
        :return: True if successful
        """
        angles = [j1, j2, j3, j4, j5, j6, j7]
        
        # Check joint limits for FR3 (in radians) - from URDF
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

    # ========================================
    # Gripper Control Methods
    # ========================================
    
    def home_gripper(self):
        """Home the gripper and determine maximum width"""
        if not self.is_gripper_available():
            print(f"❌ Gripper not available in {self.mode} mode")
            return False
            
        print("Homing gripper...")
        success = self.gripper_controller.home_gripper()
        if success:
            max_width = self.gripper_controller.get_max_width()
            print(f"Gripper homed successfully! Max width: {max_width*1000:.1f}mm")
        else:
            print("❌ Failed to home gripper")
        return success
    
    def open_gripper(self):
        """Open gripper to maximum width"""
        if not self.is_gripper_available():
            print(f"❌ Gripper not available in {self.mode} mode")
            return False
            
        print("Opening gripper...")
        success = self.gripper_controller.open_gripper()
        if success:
            print("✅ Gripper opened")
        else:
            print("❌ Failed to open gripper")
        
        if success and self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    def close_gripper(self):
        """Close gripper completely"""
        if not self.is_gripper_available():
            print(f"❌ Gripper not available in {self.mode} mode")
            return False
            
        print("Closing gripper...")
        success = self.gripper_controller.close_gripper()
        if success:
            print("✅ Gripper closed")
        else:
            print("❌ Failed to close gripper")
        
        if success and self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    def set_gripper_width(self, width_mm):
        """
        Set gripper to specific width
        
        Args:
            width_mm: Target width in millimeters (0-80mm typically)
        """
        if not self.is_gripper_available():
            print(f"❌ Gripper not available in {self.mode} mode")
            return False
            
        width_m = width_mm / 1000.0  # Convert to meters
        print(f"Setting gripper width to {width_mm:.1f}mm...")
        success = self.gripper_controller.move_to_width(width_m)
        if success:
            print(f"✅ Gripper set to {width_mm:.1f}mm")
        else:
            print(f"❌ Failed to set gripper to {width_mm:.1f}mm")
        
        if success and self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    def grasp_object(self, force_n=30.0):
        """
        Attempt to grasp an object with specified force
        
        Args:
            force_n: Grasping force in Newtons
        """
        if not self.is_gripper_available():
            print(f"❌ Gripper not available in {self.mode} mode")
            return False
            
        print(f"Attempting to grasp object with {force_n:.1f}N force...")
        success = self.gripper_controller.grasp(width=0.01, force=force_n)
        if success:
            print("✅ Grasp attempt completed")
        else:
            print("❌ Grasp attempt failed")
        
        if success and self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
        return success
    
    def get_gripper_info(self):
        """Get gripper status information"""
        info = {
            'available': self.is_gripper_available(),
            'mode': self.mode,
            'is_homed': False,
            'max_width_mm': None,
            'current_width_mm': None
        }
        
        if self.is_gripper_available():
            info['is_homed'] = self.gripper_controller.is_homed()
            if info['is_homed']:
                max_width = self.gripper_controller.get_max_width()
                if max_width:
                    info['max_width_mm'] = max_width * 1000.0
        
        return info



def main(args=None):
    """Test function for robot arm controller."""
    import rclpy
    
    rclpy.init(args=args)
    
    try:
        print("🤖 Initializing FR3 Robot Arm Controller...")
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