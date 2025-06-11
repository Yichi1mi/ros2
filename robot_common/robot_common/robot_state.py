#!/usr/bin/env python3
"""
robot_state.py - Robot state management (joint states, etc.)
"""

import rclpy
from sensor_msgs.msg import JointState
import threading
import time

class RobotStateManager:
    """Manages robot state subscriptions and data"""
    
    def __init__(self, node, joint_names):
        """
        Initialize robot state manager
        
        Args:
            node: ROS2 node instance
            joint_names: List of joint names in order
        """
        self._node = node
        self._joint_names = joint_names
        
        # Current state
        self._current_joint_positions = None
        self._current_joint_velocities = None
        self._joint_state_lock = threading.Lock()
        
        # Subscribe to joint states
        self._joint_state_sub = node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        self._node.get_logger().info('Robot state manager initialized')
    
    def _joint_state_callback(self, msg):
        """Callback for joint state updates"""
        with self._joint_state_lock:
            try:
                # Reorder joint positions to match our joint names
                positions = []
                velocities = []
                
                for joint_name in self._joint_names:
                    if joint_name in msg.name:
                        idx = msg.name.index(joint_name)
                        positions.append(msg.position[idx])
                        if msg.velocity and len(msg.velocity) > idx:
                            velocities.append(msg.velocity[idx])
                    else:
                        # Joint not found in message
                        return
                
                if len(positions) == len(self._joint_names):
                    self._current_joint_positions = positions
                    self._current_joint_velocities = velocities if velocities else None
                
            except (ValueError, IndexError) as e:
                self._node.get_logger().error(f'Error processing joint state: {e}')
    
    def wait_for_joint_states(self, timeout=10.0):
        """
        Wait for first joint state message
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            bool: True if joint states received
        """
        self._node.get_logger().info('Waiting for joint states...')
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            # Spin the node to process callbacks
            rclpy.spin_once(self._node, timeout_sec=0.1)
            
            with self._joint_state_lock:
                if self._current_joint_positions is not None:
                    self._node.get_logger().info('Joint states received!')
                    self._node.get_logger().info(
                        f'Current positions: {[f"{p:.3f}" for p in self._current_joint_positions]}'
                    )
                    return True
            
            time.sleep(0.1)
        
        self._node.get_logger().error('Timeout waiting for joint states')
        return False
    
    def get_current_joint_positions(self):
        """
        Get current joint positions
        
        Returns:
            list: Copy of current joint positions, or None if not available
        """
        # Spin once to get latest joint states
        rclpy.spin_once(self._node, timeout_sec=0.01)
        
        with self._joint_state_lock:
            return self._current_joint_positions.copy() if self._current_joint_positions else None
    
    def get_current_joint_velocities(self):
        """
        Get current joint velocities
        
        Returns:
            list: Copy of current joint velocities, or None if not available
        """
        with self._joint_state_lock:
            return self._current_joint_velocities.copy() if self._current_joint_velocities else None
    
    def has_joint_states(self):
        """Check if joint states have been received"""
        with self._joint_state_lock:
            return self._current_joint_positions is not None
    
    def calculate_joint_displacement(self, target_positions):
        """
        Calculate displacement from current to target positions
        
        Args:
            target_positions: Target joint positions
            
        Returns:
            tuple: (max_displacement, per_joint_displacements) or (None, None) if no current position
        """
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return None, None
        
        displacements = [abs(t - c) for t, c in zip(target_positions, current_positions)]
        return max(displacements), displacements
    
    def check_position_reached(self, target_positions, tolerance):
        """
        Check if current position is within tolerance of target
        
        Args:
            target_positions: Target joint positions
            tolerance: Position tolerance in radians
            
        Returns:
            bool: True if position reached within tolerance
        """
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return False
        
        for i, (target, current) in enumerate(zip(target_positions, current_positions)):
            error = abs(target - current)
            if error > tolerance:
                return False
        
        return True
    
    def get_position_errors(self, target_positions):
        """
        Get position errors for each joint
        
        Args:
            target_positions: Target joint positions
            
        Returns:
            list: Position errors for each joint, or None if no current position
        """
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            return None
        
        return [abs(t - c) for t, c in zip(target_positions, current_positions)]