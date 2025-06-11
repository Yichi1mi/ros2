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
from joint_controller import JointController

class RobotArmController:
    """
    Robot arm controller with command queue system.
    Only exposes joint and position movement APIs for main_controller.
    """

    def __init__(self):
        self.controller = JointController()
        self.HOME_POSITION = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self._cmd_queue = queue.Queue()
        self._running = True
        self._worker_thread = threading.Thread(target=self._worker, daemon=True)
        self._worker_thread.start()

    def is_connected(self):
        """Check if robot is connected."""
        return self.controller.is_connected()

    def move_to_joint_positions(self, angles, description="Joint movement"):
        """
        Queue a joint space movement command.
        :param angles: List of 6 joint angles in radians.
        :param description: Optional description for the command.
        """
        self._cmd_queue.put(('joint', angles, description))

    def move_to_cartesian_position(self, pose, description="Cartesian movement"):
        """
        Queue a Cartesian space movement command (not implemented).
        :param pose: Target pose (custom type or dict).
        :param description: Optional description for the command.
        """
        self._cmd_queue.put(('cartesian', pose, description))

    def move_to_home(self):
        """Queue a command to move to the home position."""
        self.move_to_joint_positions(self.HOME_POSITION, "Move to home")

    def set_speed(self, percentage):
        """Set movement speed percentage."""
        self.controller.set_velocity_percentage(percentage)

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

    def get_current_position_degrees(self):
        """Get current joint positions in degrees."""
        pos = self.controller.get_current_joint_positions()
        if pos:
            return [math.degrees(p) for p in pos]
        return None

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
        while self._running:
            try:
                cmd_type, data, desc = self._cmd_queue.get(timeout=0.1)
                if cmd_type == 'joint':
                    self.controller.move_to_joint_positions(data)
                elif cmd_type == 'cartesian':
                    # Cartesian movement not implemented yet
                    raise NotImplementedError("Cartesian movement not implemented yet")
                self._cmd_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Exception in worker thread: {e}")
                self._cmd_queue.task_done()