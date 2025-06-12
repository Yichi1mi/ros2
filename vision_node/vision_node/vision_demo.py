#!/usr/bin/env python3
"""
vision_demo.py - Complete vision-guided robot demonstration
"""

import rclpy
from rclpy.node import Node
import time
import threading
from .camera_node import CameraNode
from .object_detector import ObjectDetector
from .vision_robot_controller import VisionRobotController

class VisionDemo(Node):
    """Complete vision system demonstration"""
    
    def __init__(self):
        super().__init__('vision_demo')
        
        self.get_logger().info('Initializing vision demo...')
        
        # Initialize all components
        self._camera_node = None
        self._detector_node = None
        self._controller_node = None
        
        # Component threads
        self._threads = []
        self._running = True
        
        self.get_logger().info('Vision demo node created')
    
    def start_demo(self):
        """Start the complete vision demonstration"""
        try:
            self.get_logger().info('Starting vision demo components...')
            
            # Start camera node
            self.get_logger().info('1. Starting camera node...')
            self._start_camera()
            
            # Wait for camera to initialize
            time.sleep(2.0)
            
            # Start object detector
            self.get_logger().info('2. Starting object detector...')
            self._start_detector()
            
            # Wait for detector to initialize
            time.sleep(2.0)
            
            # Start robot controller
            self.get_logger().info('3. Starting robot controller...')
            self._start_controller()
            
            # Wait for controller to initialize
            time.sleep(3.0)
            
            self.get_logger().info('All components started successfully!')
            self._print_demo_instructions()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error starting demo: {e}')
            return False
    
    def _start_camera(self):
        """Start camera node in separate thread"""
        def camera_thread():
            try:
                # Try different camera IDs
                camera_found = False
                for camera_id in [0, 1, 2]:
                    try:
                        self._camera_node = CameraNode(camera_id)
                        self._camera_node.set_debug_mode(True)
                        camera_found = True
                        self.get_logger().info(f'Camera {camera_id} initialized successfully')
                        break
                    except Exception as e:
                        self.get_logger().warn(f'Failed to initialize camera {camera_id}: {e}')
                        continue
                
                if not camera_found:
                    self.get_logger().error('No camera found! Running without camera.')
                    return
                
                while self._running and rclpy.ok():
                    rclpy.spin_once(self._camera_node, timeout_sec=0.1)
                    
            except Exception as e:
                self.get_logger().error(f'Camera thread error: {e}')
        
        thread = threading.Thread(target=camera_thread, daemon=True)
        thread.start()
        self._threads.append(thread)
    
    def _start_detector(self):
        """Start object detector in separate thread"""
        def detector_thread():
            try:
                self._detector_node = ObjectDetector()
                self._detector_node.set_debug_mode(True)
                
                while self._running and rclpy.ok():
                    rclpy.spin_once(self._detector_node, timeout_sec=0.1)
                    
            except Exception as e:
                self.get_logger().error(f'Detector thread error: {e}')
        
        thread = threading.Thread(target=detector_thread, daemon=True)
        thread.start()
        self._threads.append(thread)
    
    def _start_controller(self):
        """Start robot controller in separate thread"""
        def controller_thread():
            try:
                self._controller_node = VisionRobotController()
                
                # Wait for robot connection
                if not self._controller_node.robot.is_connected():
                    self.get_logger().error('Robot not connected!')
                    return
                
                # Enable control
                self._controller_node.enable_control(True)
                
                while self._running and rclpy.ok():
                    rclpy.spin_once(self._controller_node, timeout_sec=0.1)
                    
            except Exception as e:
                self.get_logger().error(f'Controller thread error: {e}')
        
        thread = threading.Thread(target=controller_thread, daemon=True)
        thread.start()
        self._threads.append(thread)
    
    def _print_demo_instructions(self):
        """Print demo instructions"""
        print("\n" + "="*60)
        print("    VISION-GUIDED ROBOT DEMONSTRATION")
        print("="*60)
        print()
        print("SETUP:")
        print("  1. Make sure camera is connected and can see objects")
        print("  2. Place colored objects (red, blue, green, yellow) in view")
        print("  3. Robot will detect and track the largest object")
        print()
        print("CONTROLS:")
        print("  - The robot will automatically follow detected objects")
        print("  - Move objects slowly for best tracking")
        print("  - Press Ctrl+C to stop the demo")
        print()
        print("WHAT TO EXPECT:")
        print("  - Camera feed with object detection overlays")
        print("  - Robot movement to center objects in camera view")
        print("  - Console output showing detection and movement info")
        print()
        print("SAFETY:")
        print("  - Keep movements slow and within robot workspace")
        print("  - Emergency stop: Ctrl+C")
        print("="*60)
        print()
    
    def run_interactive_demo(self):
        """Run interactive demonstration with user commands"""
        if not self.start_demo():
            return
        
        try:
            while self._running and rclpy.ok():
                # Get status
                if self._controller_node:
                    status = self._controller_node.get_robot_status()
                    
                    print(f"\nSTATUS:")
                    print(f"  Robot connected: {status['connected']}")
                    print(f"  Objects detected: {status['detected_objects_count']}")
                    print(f"  Has target: {status['has_target']}")
                    print(f"  Following: {status['following_target']}")
                    
                    # Auto-start following if target detected
                    if (status['has_target'] and not status['following_target'] and 
                        status['control_enabled']):
                        print("\n>>> Target detected! Starting to follow...")
                        self._controller_node.start_target_following()
                
                time.sleep(2.0)  # Status update rate
                
        except KeyboardInterrupt:
            print("\nDemo interrupted by user")
            self.stop_demo()
    
    def stop_demo(self):
        """Stop all demo components"""
        self.get_logger().info('Stopping vision demo...')
        
        self._running = False
        
        # Stop robot controller
        if self._controller_node:
            self._controller_node.stop_target_following()
        
        # Stop camera
        if self._camera_node:
            self._camera_node.stop_capture()
        
        # Wait for threads to finish
        for thread in self._threads:
            thread.join(timeout=2.0)
        
        self.get_logger().info('Vision demo stopped')

def main(args=None):
    """Main function for vision demo"""
    rclpy.init(args=args)
    
    try:
        demo = VisionDemo()
        
        print("Starting vision-guided robot demonstration...")
        demo.run_interactive_demo()
        
    except KeyboardInterrupt:
        print("\nDemo interrupted")
    except Exception as e:
        print(f"Demo error: {e}")
    finally:
        if 'demo' in locals():
            demo.stop_demo()
        rclpy.shutdown()

if __name__ == '__main__':
    main()