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
        
        # Create geometry handler
        self.arm_geometry = UR5ArmGeometry()
        
        # Define arm poses
        self.INITIAL_POSITION = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]  # Vertical up, for initialization
        self.HOME_POSITION = [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0]  # Home position within workspace
        self._pause_between_movements = 0.5  # Default pause in seconds
        
        # Define safe workspace boundaries (in meters) - Cylindrical workspace for UR5e
        # Based on UR5e specs: 850mm max reach, base height ~163mm
        self.WORKSPACE_LIMITS = {
            # Cylindrical workspace parameters
            'inner_radius': 0.15,    # Inner radius 15cm - avoid base blind zone
            'outer_radius': 0.75,    # Outer radius 75cm - safe max radius (< 85cm max reach)
            'z_min': -0.10,          # Min height - 10cm below base (desktop operations)
            'z_max': 0.80,           # Max height - 80cm above base (avoid overextension)
            
            # Recommended efficient workspace
            'recommended_inner_radius': 0.20,  # Recommended inner radius 20cm
            'recommended_outer_radius': 0.65,  # Recommended outer radius 65cm
            'recommended_z_min': 0.10,         # Recommended min height 10cm
            'recommended_z_max': 0.60,         # Recommended max height 60cm
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
            errors.append(f"Height Z={z:.3f}m below minimum limit ({self.WORKSPACE_LIMITS['z_min']:.2f}m)")
        elif z > self.WORKSPACE_LIMITS['z_max']:
            errors.append(f"Height Z={z:.3f}m exceeds maximum limit ({self.WORKSPACE_LIMITS['z_max']:.2f}m)")
        
        # Check cylindrical boundaries
        if horizontal_distance < self.WORKSPACE_LIMITS['inner_radius']:
            errors.append(f"Horizontal distance {horizontal_distance:.3f}m below inner radius limit ({self.WORKSPACE_LIMITS['inner_radius']:.2f}m)")
        elif horizontal_distance > self.WORKSPACE_LIMITS['outer_radius']:
            errors.append(f"Horizontal distance {horizontal_distance:.3f}m exceeds outer radius limit ({self.WORKSPACE_LIMITS['outer_radius']:.2f}m)")
        
        # Additional 3D distance check (total reach limit)
        total_distance = math.sqrt(x*x + y*y + z*z)
        absolute_max_reach = 0.85  # UR5e absolute maximum reach
        if total_distance > absolute_max_reach:
            errors.append(f"Total distance {total_distance:.3f}m exceeds UR5e max reach ({absolute_max_reach:.2f}m)")
        
        # Check if in recommended zone (warning, not error)
        in_recommended_zone = (
            self.WORKSPACE_LIMITS['recommended_inner_radius'] <= horizontal_distance <= self.WORKSPACE_LIMITS['recommended_outer_radius'] and
            self.WORKSPACE_LIMITS['recommended_z_min'] <= z <= self.WORKSPACE_LIMITS['recommended_z_max']
        )
        
        if errors:
            error_msg = "Cylindrical workspace safety violations:\n" + "\n".join(f"   • {error}" for error in errors)
            return False, error_msg
        elif not in_recommended_zone:
            # Position is safe but outside recommended zone
            pass  # Reduce verbose output
        
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
        
        # Check if initial position, skip workspace validation if so
        if angles == self.INITIAL_POSITION:
            pass  # Moving to initial position
            success = self.joint_controller.move_to_joint_positions(angles)
            if success and self._pause_between_movements > 0:
                import time
                time.sleep(self._pause_between_movements)
            return success
        
        # Use geometry handler for angle normalization and limit checking
        normalized_angles = self.arm_geometry.normalize_joint_angles(angles)
        
        # Simplified joint position logging
        self.get_logger().debug(f"Moving to joint positions: {[f'{math.degrees(a):.1f}°' for a in normalized_angles]}")
        
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
            print("Cylindrical safe workspace:")
            print(f"   Inner radius: {self.WORKSPACE_LIMITS['inner_radius']:.2f}m")
            print(f"   Outer radius: {self.WORKSPACE_LIMITS['outer_radius']:.2f}m") 
            print(f"   Height range: {self.WORKSPACE_LIMITS['z_min']:.2f}m to {self.WORKSPACE_LIMITS['z_max']:.2f}m")
            print(f"   UR5e max reach: 0.85m")
            print(f"   Recommended workspace:")
            print(f"     • Horizontal distance: {self.WORKSPACE_LIMITS['recommended_inner_radius']:.2f}m to {self.WORKSPACE_LIMITS['recommended_outer_radius']:.2f}m")
            print(f"     • Height: {self.WORKSPACE_LIMITS['recommended_z_min']:.2f}m to {self.WORKSPACE_LIMITS['recommended_z_max']:.2f}m")
            return False
        
        print(f"Moving to position: ({x:.3f}, {y:.3f}, {z:.3f})")
        
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
        # Relative movement
        
        # Get current position (x, y, z, qx, qy, qz, qw)
        current_position = self.get_current_position()
        if current_position is None:
            print("Cannot get current position for relative move")
            return False
        
        x, y, z, qx, qy, qz, qw = current_position
        # Current position logged for debug
        
        # Calculate target position: current + relative offset
        target_x = x + dx
        target_y = y + dy
        target_z = z + dz
        
        print(f"Moving relative to: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        
        # Use move_to_position with current orientation (includes workspace validation)
        success = self.move_to_position(target_x, target_y, target_z, qx, qy, qz, qw)
        
        if success and self._pause_between_movements > 0:
            import time
            time.sleep(self._pause_between_movements)
            
        return success

    def move_to_initial_position(self):
        """
        Move to initial position (vertical up)
        Used for arm initialization on startup, skips workspace validation
        """
        print("Moving to initial position (vertical up)")
        return self.move_to_joint_positions(*self.INITIAL_POSITION)

    def move_to_home(self):
        """
        Move to home position (safe position within workspace)
        This position is within safe workspace, suitable as work starting point
        """
        print("Moving to home position")
        return self.move_to_joint_positions(*self.HOME_POSITION)
    
    def pause(self, duration):
        """
        Pause for specified duration.
        :param duration: Pause duration in seconds
        """
        # Pausing (simplified output)
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
        print("Initializing UR5 Robot Arm Controller...")
        robot = RobotArmController()
        
        # Check connections
        print(f"Robot arm connected: {robot.is_connected()}")
        
        if not robot.is_connected():
            print("Robot arm not connected. Make sure simulation is running.")
            return
        
        # Test basic movements
        print("\n=== BASIC ARM TESTS ===")
        
        print("1. Moving to home position...")
        robot.move_to_home()
        
        current_pos = robot.get_current_position()
        if current_pos:
            x, y, z, _, _, _, _ = current_pos
            print(f"Current position: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        print("\nRobot controller test completed!")
        print("Use this controller in your applications for arm control.")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error in robot controller test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()