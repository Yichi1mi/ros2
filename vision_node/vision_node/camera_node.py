#!/usr/bin/env python3
"""
camera_node.py - Camera capture and streaming node
"""

import rclpy
import cv2
import threading
import time
from .vision_base import VisionBase

class CameraNode(VisionBase):
    """Camera capture node with USB camera support"""
    
    def __init__(self, camera_id=0):
        super().__init__('camera_node')
        
        self._camera_id = camera_id
        self._cap = None
        self._capture_thread = None
        self._running = False
        
        # Camera parameters
        self._fps = 30
        self._width = 640
        self._height = 480
        
        # Publishers
        self._image_pub = self._create_image_publisher('/camera/image_raw')
        self._debug_pub = self._create_image_publisher('/camera/image_debug')
        
        # Initialize camera
        self._initialize_camera()
        
        # Start capture thread
        self._start_capture()
        
        self.get_logger().info(f'Camera node initialized (camera_id: {camera_id})')
    
    def _initialize_camera(self):
        """Initialize USB camera"""
        try:
            self._cap = cv2.VideoCapture(self._camera_id)
            
            if not self._cap.isOpened():
                self.get_logger().error(f'Cannot open camera {self._camera_id}')
                return False
            
            # Set camera properties
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)
            
            # Verify settings
            actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f'Camera initialized: {actual_width}x{actual_height} @ {actual_fps:.1f}fps')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error initializing camera: {e}')
            return False
    
    def _start_capture(self):
        """Start camera capture thread"""
        if self._cap is None or not self._cap.isOpened():
            self.get_logger().error('Camera not initialized')
            return
        
        self._running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self.get_logger().info('Camera capture started')
    
    def _capture_loop(self):
        """Main camera capture loop"""
        frame_count = 0
        last_fps_time = time.time()
        
        while self._running and rclpy.ok():
            try:
                ret, frame = self._cap.read()
                
                if not ret:
                    self.get_logger().warn('Failed to capture frame')
                    time.sleep(0.1)
                    continue
                
                # Update current image
                with self._image_lock:
                    self._current_image = frame.copy()
                    self._image_timestamp = time.time()
                
                # Publish raw image
                self._publish_image(self._image_pub, frame, "camera")
                
                # Process image (calls _process_image)
                ros_timestamp = self.get_clock().now()
                self._process_image(frame, ros_timestamp)
                
                # FPS calculation
                frame_count += 1
                current_time = time.time()
                if current_time - last_fps_time >= 5.0:  # Every 5 seconds
                    fps = frame_count / (current_time - last_fps_time)
                    self.get_logger().info(f'Camera FPS: {fps:.1f}')
                    frame_count = 0
                    last_fps_time = current_time
                
                # Control frame rate
                time.sleep(1.0 / self._fps)
                
            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {e}')
                time.sleep(0.1)
    
    def _process_image(self, cv_image, timestamp):
        """Process captured image"""
        if not self._debug_mode:
            return
        
        # Create debug image with basic info
        debug_image = cv_image.copy()
        
        # Draw timestamp and frame info
        h, w = debug_image.shape[:2]
        timestamp_str = f'Time: {time.strftime("%H:%M:%S")}'
        resolution_str = f'Size: {w}x{h}'
        
        self.draw_text(debug_image, timestamp_str, (10, 30))
        self.draw_text(debug_image, resolution_str, (10, 60))
        
        # Draw center crosshair
        center = self.calculate_image_center(debug_image)
        self.draw_circle(debug_image, center, 5, (0, 255, 0), 2)
        cv2.line(debug_image, (center[0]-20, center[1]), (center[0]+20, center[1]), (0, 255, 0), 2)
        cv2.line(debug_image, (center[0], center[1]-20), (center[0], center[1]+20), (0, 255, 0), 2)
        
        # Publish debug image
        self._publish_image(self._debug_pub, debug_image, "camera")
    
    def set_camera_params(self, width=None, height=None, fps=None):
        """Update camera parameters"""
        if width:
            self._width = width
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height:
            self._height = height
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if fps:
            self._fps = fps
            self._cap.set(cv2.CAP_PROP_FPS, fps)
        
        self.get_logger().info(f'Camera params updated: {self._width}x{self._height} @ {self._fps}fps')
    
    def stop_capture(self):
        """Stop camera capture"""
        self._running = False
        if self._capture_thread:
            self._capture_thread.join(timeout=2.0)
        
        if self._cap:
            self._cap.release()
        
        self.get_logger().info('Camera capture stopped')
    
    def __del__(self):
        """Cleanup when node is destroyed"""
        self.stop_capture()

def main(args=None):
    """Camera node main function"""
    rclpy.init(args=args)
    
    try:
        # Try different camera IDs
        camera_node = None
        for camera_id in [0, 1, 2]:
            try:
                camera_node = CameraNode(camera_id)
                break
            except Exception as e:
                print(f"Failed to initialize camera {camera_id}: {e}")
                continue
        
        if camera_node is None:
            print("No camera found!")
            return
        
        # Enable debug mode for testing
        camera_node.set_debug_mode(True)
        
        print("Camera node running. Press Ctrl+C to stop.")
        rclpy.spin(camera_node)
        
    except KeyboardInterrupt:
        print("\nCamera node interrupted by user")
    except Exception as e:
        print(f"Error in camera node: {e}")
    finally:
        if 'camera_node' in locals():
            camera_node.stop_capture()
        rclpy.shutdown()

if __name__ == '__main__':
    main()