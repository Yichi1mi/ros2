#!/usr/bin/env python3
"""
connection_base.py - Base class for robot connection management
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
from abc import ABC, abstractmethod

class RobotConnectionBase(Node, ABC):
    """Base class for robot controllers with connection management"""
    
    def __init__(self, node_name, action_name, action_type):
        """
        Initialize robot connection base
        
        Args:
            node_name: Name of the ROS2 node
            action_name: Name of the action server (e.g., '/joint_trajectory_controller/follow_joint_trajectory')
            action_type: Action type class (e.g., FollowJointTrajectory)
        """
        super().__init__(node_name)
        
        self._action_name = action_name
        self._action_type = action_type
        self._connected = False
        
        # Movement parameters
        self._velocity_percentage = 50
        self._base_angular_velocity = 2  # rad/s
        self._position_tolerance = 0.01    # rad (更精确的位置容差)
        self._goal_time_tolerance = 10.0    # seconds (更长的超时时间)
        
        # Create action client
        self._action_client = ActionClient(self, action_type, action_name)
        
        # Initialize connection
        self._connected = self._initialize_connection()
    
    def _initialize_connection(self):
        """Initialize connection to robot action server"""
        self.get_logger().info(f'Waiting for action server: {self._action_name}...')
        
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return False
        
        self.get_logger().info('Action server connected!')
        
        # Call child class initialization
        if not self._on_connection_established():
            return False
        
        self.get_logger().info(f'{self.__class__.__name__} ready')
        return True
    
    @abstractmethod
    def _on_connection_established(self):
        """
        Called when action server connection is established
        Child classes should implement initialization logic here
        
        Returns:
            bool: True if initialization successful
        """
        pass
    
    def set_velocity_percentage(self, percentage):
        """Set velocity as percentage of maximum (1-100)"""
        self._velocity_percentage = max(1, min(100, percentage))
        self.get_logger().info(f'Velocity set to {self._velocity_percentage}%')
    
    def set_position_tolerance(self, tolerance):
        """Set position tolerance in radians"""
        self._position_tolerance = max(0.001, tolerance)
        self.get_logger().info(f'Position tolerance set to {self._position_tolerance:.4f} rad')
    
    def get_current_velocity(self):
        """Get current velocity in rad/s"""
        return self._base_angular_velocity * (self._velocity_percentage / 100.0)
    
    def is_connected(self):
        """Check if controller is connected and ready"""
        return self._connected
    
    def calculate_movement_duration(self, max_displacement):
        """
        Calculate movement duration based on displacement and velocity
        
        Args:
            max_displacement: Maximum joint displacement in radians
            
        Returns:
            float: Duration in seconds
        """
        velocity = self.get_current_velocity()
        duration = max_displacement / velocity * 1.2  # 20% buffer
        return max(duration, 1.0)  # Minimum 1 second