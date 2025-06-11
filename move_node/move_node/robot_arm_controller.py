#!/usr/bin/env python3
"""
robot_arm_controller.py - Unified API for robot arm control with queue system.
Provides joint and Cartesian movement interfaces for main_controller.
"""

import sys
import os
import math
import threading
import queue
from .joint_controller import JointController
from .position_controller import PositionController

class RobotArmController:
    """
    Robot arm controller with command queue system.
    Only exposes joint and position movement APIs for main_controller.
    """

    def __init__(self):
        self.joint_controller = JointController()
        self.position_controller = PositionController()
        self.HOME_POSITION = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self._cmd_queue = queue.Queue()
        self._running = True
        self._pause_between_movements = 0.5  # Default pause in seconds
        self._worker_thread = threading.Thread(target=self._worker, daemon=True)
        self._worker_thread.start()

    def is_connected(self):
        """Check if robot is connected."""
        return self.joint_controller.is_connected() and self.position_controller.is_connected()

    def move_to_joint_positions(self, angles, description="Joint movement"):
        """
        Queue a joint space movement command.
        :param angles: List of 6 joint angles in radians.
        :param description: Optional description for the command.
        """
        self._cmd_queue.put(('joint', angles, description))

    def move_to_cartesian_position(self, pose, description="Cartesian movement"):
        """
        Queue a Cartesian space movement command.
        :param pose: Target pose (dict with 'position' and 'orientation' or geometry_msgs.msg.Pose).
        :param description: Optional description for the command.
        """
        self._cmd_queue.put(('cartesian', pose, description))
    
    def move_to_position_xyz(self, x, y, z, orientation=None, description="XYZ movement"):
        """
        Queue a movement to specific XYZ coordinates.
        :param x, y, z: Target position coordinates.
        :param orientation: Optional orientation dict or None to keep current.
        :param description: Optional description for the command.
        """
        pose_data = {'x': x, 'y': y, 'z': z, 'orientation': orientation}
        self._cmd_queue.put(('xyz', pose_data, description))
    
    def add_pause(self, duration, description="Pause"):
        """
        Add a pause command to the queue.
        :param duration: Pause duration in seconds.
        :param description: Optional description for the pause.
        """
        self._cmd_queue.put(('pause', duration, description))
    
    def move_relative_xyz(self, dx, dy, dz, description="Relative move"):
        """
        Queue a relative movement command.
        :param dx, dy, dz: Relative movement in meters.
        :param description: Optional description for the command.
        """
        relative_data = {'dx': dx, 'dy': dy, 'dz': dz}
        self._cmd_queue.put(('relative', relative_data, description))

    def move_to_home(self):
        """Queue a command to move to the home position."""
        self.move_to_joint_positions(self.HOME_POSITION, "Move to home")

    def set_speed(self, percentage):
        """Set movement speed percentage."""
        self.joint_controller.set_velocity_percentage(percentage)
        self.position_controller.set_velocity_percentage(percentage)
    
    def set_pause_between_movements(self, pause_seconds):
        """
        Set pause duration between movements.
        :param pause_seconds: Pause duration in seconds (float).
        """
        self._pause_between_movements = max(0.0, pause_seconds)
        print(f"Pause between movements set to {self._pause_between_movements:.1f}s")

    def clear_queue(self):
        """Clear all queued commands."""
        while not self._cmd_queue.empty():
            try:
                self._cmd_queue.get_nowait()
                self._cmd_queue.task_done()
            except queue.Empty:
                break

    def wait_for_completion(self, timeout=60):
        """Wait for all queued commands to complete."""
        self._cmd_queue.join()
        return True

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
    
    # 保持向后兼容
    def get_current_position_degrees(self):
        """DEPRECATED: Use get_current_joint_angles_degrees() instead."""
        return self.get_current_joint_angles_degrees()

    def stop(self):
        """Stop the worker thread."""
        self._running = False
        self._worker_thread.join()

    def start_execution(self):
        """For compatibility: execution starts automatically, so this is a no-op."""
        pass

    def stop_execution(self):
        """Stop the worker thread."""
        self.stop()

    def _worker(self):
        """Worker thread to execute queued commands in order."""
        import time
        movement_count = 0
        
        while self._running:
            try:
                cmd_type, data, desc = self._cmd_queue.get(timeout=0.1)
                movement_count += 1
                
                print(f"\n[{movement_count}] Starting: {desc}")
                start_time = time.time()
                success = False
                
                if cmd_type == 'joint':
                    success = self.joint_controller.move_to_joint_positions(data)
                elif cmd_type == 'cartesian':
                    success = self.position_controller.move_to_cartesian_position(data)
                elif cmd_type == 'xyz':
                    x, y, z = data['x'], data['y'], data['z']
                    orientation = data.get('orientation')
                    success = self.position_controller.move_to_position_xyz(x, y, z, orientation)
                elif cmd_type == 'pause':
                    # Handle pause command
                    pause_duration = data
                    print(f"[{movement_count}] Pausing for {pause_duration:.1f}s...")
                    time.sleep(pause_duration)
                    success = True
                elif cmd_type == 'relative':
                    # Handle relative movement
                    dx, dy, dz = data['dx'], data['dy'], data['dz']
                    success = self.position_controller.move_relative_xyz(dx, dy, dz, desc)
                else:
                    print(f"Unknown command type: {cmd_type}")
                    success = False
                
                # Show completion status
                duration = time.time() - start_time
                status = "✓ COMPLETED" if success else "✗ FAILED"
                print(f"[{movement_count}] {status}: {desc} (took {duration:.1f}s)")
                
                # Add pause between movements if specified
                if success and self._pause_between_movements > 0:
                    print(f"[{movement_count}] Pausing for {self._pause_between_movements:.1f}s...")
                    time.sleep(self._pause_between_movements)
                
                self._cmd_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Exception in worker thread: {e}")
                self._cmd_queue.task_done()