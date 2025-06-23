#!/usr/bin/env python3
"""
object_detector.py - Object detection using color-based and contour-based methods
"""

import rclpy
import cv2
import numpy as np
import time
import json
from .vision_base import VisionBase
from geometry_msgs.msg import Point
from std_msgs.msg import String

class ObjectDetector(VisionBase):
    """Object detector using computer vision techniques"""
    
    def __init__(self):
        super().__init__('object_detector')
        
        # Subscribe to camera images
        self._image_sub = self._create_image_subscriber('/camera/image_raw')
        
        # Publishers
        self._detection_pub = self._create_image_publisher('/vision/detection_image')
        self._object_pub = self.create_publisher(String, '/vision/detected_objects', 10)
        self._target_pub = self.create_publisher(Point, '/vision/target_position', 10)
        
        # Detection parameters
        self._min_area = 500  # Minimum contour area
        self._max_area = 50000  # Maximum contour area
        
        # Color ranges for different objects (HSV)
        self._color_ranges = {
            'red': {
                'lower1': np.array([0, 120, 70]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 120, 70]),
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 150, 0]),
                'upper': np.array([140, 255, 255])
            },
            'green': {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255])
            },
            'yellow': {
                'lower': np.array([20, 100, 100]),
                'upper': np.array([30, 255, 255])
            }
        }
        
        # Detection state
        self._detected_objects = []
        self._target_object = None
        
        self.get_logger().info('Object detector initialized')
    
    def _process_image(self, cv_image, timestamp):
        """Process image for object detection"""
        try:
            # Convert to HSV for better color detection
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Create result image for visualization
            result_image = cv_image.copy()
            
            # Detect objects of different colors
            detected_objects = []
            
            for color_name, color_range in self._color_ranges.items():
                objects = self._detect_color_objects(hsv, cv_image, color_name, color_range)
                detected_objects.extend(objects)
                
                # Draw detections on result image
                for obj in objects:
                    self._draw_detection(result_image, obj)
            
            # Update detected objects
            self._detected_objects = detected_objects
            
            # Find and publish target object
            self._update_target_object()
            
            # Publish detection results
            self._publish_detections(detected_objects)
            
            # Publish result image
            if self._debug_mode:
                self._add_debug_info(result_image)
                self._publish_image(self._detection_pub, result_image, "camera")
            
        except Exception as e:
            self.get_logger().error(f'Error in object detection: {e}')
    
    def _detect_color_objects(self, hsv_image, original_image, color_name, color_range):
        """Detect objects of specific color"""
        # Create color mask
        if 'lower1' in color_range:  # Handle red color (wraps around HSV)
            mask1 = cv2.inRange(hsv_image, color_range['lower1'], color_range['upper1'])
            mask2 = cv2.inRange(hsv_image, color_range['lower2'], color_range['upper2'])
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = cv2.inRange(hsv_image, color_range['lower'], color_range['upper'])
        
        # Morphological operations to clean up mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if self._min_area < area < self._max_area:
                # Calculate object properties
                moments = cv2.moments(contour)
                if moments['m00'] != 0:
                    cx = int(moments['m10'] / moments['m00'])
                    cy = int(moments['m01'] / moments['m00'])
                    
                    # Bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Create object info
                    obj_info = {
                        'color': color_name,
                        'center': (cx, cy),
                        'area': area,
                        'bounding_box': (x, y, w, h),
                        'contour': contour
                    }
                    
                    objects.append(obj_info)
        
        return objects
    
    def _draw_detection(self, image, obj_info):
        """Draw detection on image"""
        color_map = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'yellow': (0, 255, 255)
        }
        
        color = color_map.get(obj_info['color'], (255, 255, 255))
        center = obj_info['center']
        x, y, w, h = obj_info['bounding_box']
        
        # Draw bounding rectangle
        self.draw_rectangle(image, (x, y), (x + w, y + h), color, 2)
        
        # Draw center point
        self.draw_circle(image, center, 5, color, -1)
        
        # Draw label
        label = f"{obj_info['color']} ({obj_info['area']:.0f})"
        self.draw_text(image, label, (x, y - 10), color, 0.6, 2)
    
    def _add_debug_info(self, image):
        """Add debug information to image"""
        h, w = image.shape[:2]
        
        # Draw image center
        center = self.calculate_image_center(image)
        self.draw_circle(image, center, 3, (255, 255, 255), 2)
        cv2.line(image, (center[0]-15, center[1]), (center[0]+15, center[1]), (255, 255, 255), 1)
        cv2.line(image, (center[0], center[1]-15), (center[0], center[1]+15), (255, 255, 255), 1)
        
        # Show detection count
        count_text = f"Objects: {len(self._detected_objects)}"
        self.draw_text(image, count_text, (10, h - 60), (255, 255, 255), 0.7, 2)
        
        # Show target info
        if self._target_object:
            target_text = f"Target: {self._target_object['color']} at {self._target_object['center']}"
            self.draw_text(image, target_text, (10, h - 30), (0, 255, 255), 0.7, 2)
    
    def _update_target_object(self):
        """Update target object (largest detected object)"""
        if not self._detected_objects:
            self._target_object = None
            return
        
        # Find largest object
        largest_obj = max(self._detected_objects, key=lambda obj: obj['area'])
        self._target_object = largest_obj
        
        # Publish target position
        target_msg = Point()
        
        # Convert to normalized coordinates [-1, 1]
        if self.has_image():
            image_shape = self.get_current_image().shape
            norm_x, norm_y = self.pixel_to_normalized(largest_obj['center'], image_shape)
            target_msg.x = norm_x
            target_msg.y = norm_y
            target_msg.z = largest_obj['area']  # Use area as confidence
        
        self._target_pub.publish(target_msg)
    
    def _publish_detections(self, objects):
        """Publish detection results as JSON"""
        detection_data = {
            'timestamp': time.time(),
            'object_count': len(objects),
            'objects': []
        }
        
        for obj in objects:
            obj_data = {
                'color': obj['color'],
                'center_x': obj['center'][0],
                'center_y': obj['center'][1],
                'area': obj['area'],
                'bounding_box': obj['bounding_box']
            }
            detection_data['objects'].append(obj_data)
        
        # Publish as JSON string
        msg = String()
        msg.data = json.dumps(detection_data)
        self._object_pub.publish(msg)
    
    def set_color_range(self, color_name, lower, upper, lower2=None, upper2=None):
        """Update color detection range"""
        color_range = {
            'lower': np.array(lower),
            'upper': np.array(upper)
        }
        
        if lower2 is not None and upper2 is not None:
            color_range['lower1'] = color_range['lower']
            color_range['upper1'] = color_range['upper']
            color_range['lower2'] = np.array(lower2)
            color_range['upper2'] = np.array(upper2)
            del color_range['lower']
            del color_range['upper']
        
        self._color_ranges[color_name] = color_range
        self.get_logger().info(f'Updated color range for {color_name}')
    
    def set_area_range(self, min_area, max_area):
        """Set area range for object detection"""
        self._min_area = min_area
        self._max_area = max_area
        self.get_logger().info(f'Area range set to {min_area} - {max_area}')
    
    def get_detected_objects(self):
        """Get list of currently detected objects"""
        return self._detected_objects.copy()
    
    def get_target_object(self):
        """Get current target object"""
        return self._target_object

def main(args=None):
    """Object detector main function"""
    rclpy.init(args=args)
    
    try:
        detector = ObjectDetector()
        detector.set_debug_mode(True)
        
        print("Object detector running. Press Ctrl+C to stop.")
        print("Detecting: red, blue, green, yellow objects")
        
        rclpy.spin(detector)
        
    except KeyboardInterrupt:
        print("\nObject detector interrupted by user")
    except Exception as e:
        print(f"Error in object detector: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()