#!/usr/bin/env python3
"""
vision_base.py - Base class for vision processing components
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time
from abc import ABC, abstractmethod

class VisionBase(Node, ABC):
    """Base class for vision processing nodes"""
    
    def __init__(self, node_name):
        super().__init__(node_name)
        
        # OpenCV bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Current image data
        self._current_image = None
        self._image_lock = threading.Lock()
        self._image_timestamp = None
        
        # Processing parameters
        self._processing_enabled = True
        self._debug_mode = False
        
        self.get_logger().info(f'{node_name} vision base initialized')
    
    def set_debug_mode(self, enabled):
        """Enable/disable debug visualization"""
        self._debug_mode = enabled
        self.get_logger().info(f'Debug mode: {"enabled" if enabled else "disabled"}')
    
    def enable_processing(self, enabled=True):
        """Enable/disable image processing"""
        self._processing_enabled = enabled
        self.get_logger().info(f'Processing: {"enabled" if enabled else "disabled"}')
    
    def get_current_image(self):
        """
        Get the current image as OpenCV format
        
        Returns:
            numpy.ndarray: Current image or None if not available
        """
        with self._image_lock:
            return self._current_image.copy() if self._current_image is not None else None
    
    def get_image_timestamp(self):
        """Get timestamp of current image"""
        return self._image_timestamp
    
    def has_image(self):
        """Check if image data is available"""
        with self._image_lock:
            return self._current_image is not None
    
    def _image_callback(self, msg):
        """Process incoming image messages"""
        if not self._processing_enabled:
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self._image_lock:
                self._current_image = cv_image
                self._image_timestamp = time.time()
            
            # Call child class processing
            self._process_image(cv_image, msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    @abstractmethod
    def _process_image(self, cv_image, timestamp):
        """
        Process the incoming image - must be implemented by child classes
        
        Args:
            cv_image: OpenCV image (numpy array)
            timestamp: ROS timestamp
        """
        pass
    
    def _create_image_subscriber(self, topic_name, queue_size=1):
        """Create image subscriber"""
        return self.create_subscription(
            Image,
            topic_name,
            self._image_callback,
            queue_size
        )
    
    def _create_image_publisher(self, topic_name, queue_size=1):
        """Create image publisher"""
        return self.create_publisher(Image, topic_name, queue_size)
    
    def _publish_image(self, publisher, cv_image, frame_id="camera"):
        """Publish OpenCV image as ROS message"""
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = frame_id
            publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {e}')
    
    def draw_text(self, image, text, position, color=(0, 255, 0), font_scale=0.7, thickness=2):
        """Draw text on image"""
        cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale, color, thickness)
    
    def draw_circle(self, image, center, radius, color=(0, 255, 0), thickness=2):
        """Draw circle on image"""
        cv2.circle(image, center, radius, color, thickness)
    
    def draw_rectangle(self, image, pt1, pt2, color=(0, 255, 0), thickness=2):
        """Draw rectangle on image"""
        cv2.rectangle(image, pt1, pt2, color, thickness)
    
    def calculate_image_center(self, image):
        """Calculate center point of image"""
        h, w = image.shape[:2]
        return (w // 2, h // 2)
    
    def pixel_to_normalized(self, pixel_coords, image_shape):
        """
        Convert pixel coordinates to normalized coordinates [-1, 1]
        
        Args:
            pixel_coords: (x, y) pixel coordinates
            image_shape: (height, width) of image
            
        Returns:
            tuple: (normalized_x, normalized_y)
        """
        h, w = image_shape[:2]
        x, y = pixel_coords
        norm_x = (x - w/2) / (w/2)
        norm_y = (y - h/2) / (h/2)
        return (norm_x, norm_y)
    
    def distance_between_points(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)