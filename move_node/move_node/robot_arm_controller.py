#!/usr/bin/env python3
"""
robot_arm_controller.py - Simple wrapper for existing joint_controller
Just provides easy functions for main_controller to use
"""

import sys
import os
import math

# Import existing joint controller
sys.path.append(os.path.join(os.path.dirname(__file__)))
from joint_controller import JointController

class RobotArmController:
    """Simple wrapper around existing JointController"""
    
    def __init__(self):
        self.controller = JointController()
        
        # Predefined positions
        self.HOME_POSITION = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
    def is_connected(self):
        """Check if robot is connected"""
        return self.controller.is_connected()
    
    def move_to_home(self):
        """Move to home position"""
        self.controller.queue_joint_movement(self.HOME_POSITION, "Move to home")
    
    def move_joint_angles(self, angles, description="Joint movement"):
        """Move to joint angles (radians)"""
        self.controller.queue_joint_movement(angles, description)
    
    def move_joint_angles_degrees(self, angles_deg, description="Joint movement (degrees)"):
        """Move to joint angles (degrees)"""
        angles_rad = [math.radians(a) for a in angles_deg]
        self.controller.queue_joint_movement(angles_rad, description)
    
    def rotate_base_degrees(self, angle_deg):
        """Rotate base to angle in degrees"""
        position = [math.radians(angle_deg), -1.57, 0.0, -1.57, 0.0, 0.0]
        self.controller.queue_joint_movement(position, f"Rotate base to {angle_deg} degrees")
    
    def set_speed(self, percentage):
        """Set movement speed percentage"""
        self.controller.set_velocity_scaling(velocity_percentage=percentage)
    
    def start_execution(self):
        """Start executing queued commands"""
        self.controller.set_queued_cmd_start_exec()
    
    def stop_execution(self):
        """Stop executing commands"""
        self.controller.set_queued_cmd_stop_exec()
    
    def clear_queue(self):
        """Clear all queued commands"""
        self.controller.set_queued_cmd_clear()
    
    def wait_for_completion(self, timeout=60):
        """Wait for all commands to complete"""
        return self.controller.wait_for_queue_complete(timeout)
    
    def get_current_position_degrees(self):
        """Get current joint positions in degrees"""
        pos = self.controller.get_current_joint_positions()
        if pos:
            return [math.degrees(p) for p in pos]
        return None