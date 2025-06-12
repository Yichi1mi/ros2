#!/usr/bin/env python3
"""
vision_robot_controller.py - Integration of vision system with robot control
"""

import rclpy
from rclpy.node import Node
import time
import threading
import math
from geometry_msgs.msg import Point
from std_msgs.msg import String
import json

# Import robot controller
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', 'move_node'))
from move_node.robot_arm_controller import RobotArmController

class VisionRobotController(Node):
    """Controller that integrates vision feedback with robot movement"""
    
    def __init__(self):
        super().__init__('vision_robot_controller')
        
        # Initialize robot controller
        self.robot = RobotArmController()
        
        # Vision data
        self._target_position = None
        self._detected_objects = []
        self._vision_lock = threading.Lock()
        
        # Control parameters
        self._control_enabled = False
        self._following_target = False
        self._last_target_time = 0
        self._target_timeout = 2.0  # seconds
        
        # Movement parameters
        self._movement_scale = 0.1  # Scale factor for vision-to-robot movement
        self._min_movement_threshold = 0.1  # Minimum target offset to trigger movement
        self._approach_distance = 0.05  # How close to get to target (meters)
        
        # Camera-to-robot coordinate transformation
        self._camera_to_robot_transform = {
            'scale_x': 0.3,  # meters per normalized coordinate
            'scale_y': 0.3,
            'offset_x': 0.0,
            'offset_y': 0.0,
            'invert_x': False,
            'invert_y': True  # Camera Y is typically inverted relative to robot
        }
        
        # Subscribe to vision topics
        self._target_sub = self.create_subscription(
            Point,
            '/vision/target_position',
            self._target_callback,
            10
        )
        
        self._objects_sub = self.create_subscription(
            String,
            '/vision/detected_objects',
            self._objects_callback,
            10
        )
        
        # Wait for robot connection
        if not self._wait_for_robot_connection():
            self.get_logger().error('Failed to connect to robot')
            return
        
        self.get_logger().info('Vision-robot controller initialized')
    
    def _wait_for_robot_connection(self, timeout=10.0):
        """Wait for robot to be connected"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.robot.is_connected():
                self.get_logger().info('Robot connected successfully')
                return True
            time.sleep(0.5)
        return False
    
    def _target_callback(self, msg):
        """Handle target position updates from vision"""
        with self._vision_lock:
            self._target_position = {
                'x': msg.x,  # Normalized coordinates [-1, 1]
                'y': msg.y,
                'confidence': msg.z,  # Using z for confidence/area
                'timestamp': time.time()
            }
            self._last_target_time = time.time()
    
    def _objects_callback(self, msg):
        """Handle detected objects updates"""
        try:
            data = json.loads(msg.data)
            with self._vision_lock:
                self._detected_objects = data.get('objects', [])
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Error parsing objects data: {e}')
    
    def enable_control(self, enabled=True):
        """Enable/disable vision-based control"""
        self._control_enabled = enabled
        if enabled:
            self.robot.set_speed(30)  # Slower speed for vision control
            self.get_logger().info('Vision control enabled')
        else:
            self._following_target = False
            self.get_logger().info('Vision control disabled')
    
    def start_target_following(self):
        """Start following the detected target"""
        if not self._control_enabled:
            self.get_logger().warn('Control not enabled')
            return False
        
        if not self._has_valid_target():
            self.get_logger().warn('No valid target detected')
            return False
        
        self._following_target = True
        self.get_logger().info('Started target following')
        
        # Start following thread
        follow_thread = threading.Thread(target=self._follow_target_loop, daemon=True)
        follow_thread.start()
        
        return True
    
    def stop_target_following(self):
        """Stop following the target"""
        self._following_target = False
        self.robot.clear_queue()
        self.get_logger().info('Stopped target following')
    
    def _follow_target_loop(self):
        """Main loop for target following"""
        self.get_logger().info('Target following loop started')
        
        while self._following_target and rclpy.ok():
            try:
                if not self._has_valid_target():
                    self.get_logger().info('Target lost, stopping following')
                    break
                
                # Get current target
                with self._vision_lock:
                    target = self._target_position.copy()
                
                # Calculate target offset from center
                offset_x = target['x']  # Normalized [-1, 1]
                offset_y = target['y']
                
                # Check if movement is needed
                total_offset = math.sqrt(offset_x**2 + offset_y**2)
                if total_offset < self._min_movement_threshold:
                    self.get_logger().info('Target centered, no movement needed')
                    time.sleep(0.5)
                    continue
                
                # Transform vision coordinates to robot coordinates
                robot_dx, robot_dy = self._vision_to_robot_coords(offset_x, offset_y)
                
                self.get_logger().info(
                    f'Target offset: vision=({offset_x:.3f}, {offset_y:.3f}), '
                    f'robot=({robot_dx:.3f}, {robot_dy:.3f})'
                )
                
                # Execute movement
                success = self._move_towards_target(robot_dx, robot_dy)
                
                if success:
                    self.get_logger().info('Movement completed')
                    time.sleep(0.5)  # Pause between movements
                else:
                    self.get_logger().warn('Movement failed')
                    break
                
            except Exception as e:
                self.get_logger().error(f'Error in following loop: {e}')
                break
            
            time.sleep(0.1)  # Control loop rate
        
        self._following_target = False
        self.get_logger().info('Target following loop ended')
    
    def _vision_to_robot_coords(self, norm_x, norm_y):
        """Transform normalized vision coordinates to robot coordinates"""
        transform = self._camera_to_robot_transform
        
        # Apply scaling
        robot_x = norm_x * transform['scale_x']
        robot_y = norm_y * transform['scale_y']
        
        # Apply inversion if needed
        if transform['invert_x']:
            robot_x = -robot_x
        if transform['invert_y']:
            robot_y = -robot_y
        
        # Apply offsets
        robot_x += transform['offset_x']
        robot_y += transform['offset_y']
        
        return robot_x, robot_y
    
    def _move_towards_target(self, dx, dy):
        """Move robot towards target"""
        try:
            # Limit movement distance for safety
            max_movement = self._approach_distance
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance > max_movement:
                scale = max_movement / distance
                dx *= scale
                dy *= scale
            
            # Execute relative movement
            description = f"Vision-guided move: dx={dx:.3f}, dy={dy:.3f}"
            self.robot.move_relative_xyz(dx, dy, 0, description)
            self.robot.wait_for_completion()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error moving towards target: {e}')
            return False
    
    def _has_valid_target(self):
        """Check if we have a valid target"""
        with self._vision_lock:
            if self._target_position is None:
                return False
            
            # Check if target is recent
            age = time.time() - self._target_position['timestamp']
            return age < self._target_timeout
    
    def get_robot_status(self):
        """Get current robot status"""
        return {
            'connected': self.robot.is_connected(),
            'control_enabled': self._control_enabled,
            'following_target': self._following_target,
            'has_target': self._has_valid_target(),
            'target_position': self._target_position,
            'detected_objects_count': len(self._detected_objects)
        }
    
    def set_camera_transform(self, scale_x=None, scale_y=None, offset_x=None, offset_y=None, 
                           invert_x=None, invert_y=None):
        """Update camera-to-robot coordinate transformation"""
        transform = self._camera_to_robot_transform
        
        if scale_x is not None:
            transform['scale_x'] = scale_x
        if scale_y is not None:
            transform['scale_y'] = scale_y
        if offset_x is not None:
            transform['offset_x'] = offset_x
        if offset_y is not None:
            transform['offset_y'] = offset_y
        if invert_x is not None:
            transform['invert_x'] = invert_x
        if invert_y is not None:
            transform['invert_y'] = invert_y
        
        self.get_logger().info(f'Camera transform updated: {transform}')
    
    def move_to_home(self):
        """Move robot to home position"""
        if self._control_enabled:
            self.robot.move_to_home()
            self.robot.wait_for_completion()
            return True
        return False

def main(args=None):
    """Vision-robot controller main function"""
    rclpy.init(args=args)
    
    try:
        controller = VisionRobotController()
        
        if not controller.robot.is_connected():
            print("Robot not connected!")
            return
        
        print("Vision-robot controller ready!")
        print("Commands:")
        print("  - Enable control and start following")
        
        # Enable control
        controller.enable_control(True)
        
        # Wait a bit for vision data
        time.sleep(2.0)
        
        # Start following if target is available
        if controller._has_valid_target():
            print("Target detected, starting following...")
            controller.start_target_following()
        else:
            print("No target detected. Waiting for target...")
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        print("\nVision-robot controller interrupted by user")
        if 'controller' in locals():
            controller.stop_target_following()
    except Exception as e:
        print(f"Error in vision-robot controller: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()