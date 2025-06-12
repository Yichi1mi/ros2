#!/usr/bin/env python3
"""
gripper_controller.py - Robotiq 2F-85 gripper control interface.
Provides simple open/close and position control for grasping tasks.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
import threading
import time


class GripperController(Node):
    """
    Robotiq 2F-85 gripper controller with action-based interface.
    Provides blocking and non-blocking gripper control methods.
    """

    def __init__(self):
        super().__init__('gripper_controller')
        
        # Gripper parameters (Robotiq 2F-85 specifications)
        self.GRIPPER_OPEN_POSITION = 0.0      # Fully open (0.0m)
        self.GRIPPER_CLOSED_POSITION = 0.042  # Fully closed (42mm)
        self.GRIPPER_MAX_EFFORT = 100.0       # Maximum force in Newtons
        self.GRIPPER_SPEED = 0.1              # Default speed (m/s)
        
        # State tracking
        self._current_position = 0.0
        self._current_effort = 0.0
        self._is_moving = False
        self._connection_established = False
        
        # Thread safety
        self._state_lock = threading.Lock()
        
        # Action client for gripper control
        self._action_client = ActionClient(
            self, 
            GripperCommand, 
            '/gripper_controller/gripper_cmd'
        )
        
        # Subscribe to joint states for feedback
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # Wait for action server
        self.get_logger().info("Waiting for gripper action server...")
        if self._action_client.wait_for_server(timeout_sec=5.0):
            self._connection_established = True
            self.get_logger().info("✅ Gripper controller connected")
        else:
            self.get_logger().warn("⚠️  Gripper action server not available")
        
    def _joint_state_callback(self, msg):
        """Process joint state updates for gripper feedback."""
        try:
            # Look for left_finger_joint (main gripper joint)
            if 'left_finger_joint' in msg.name:
                idx = msg.name.index('left_finger_joint')
                with self._state_lock:
                    self._current_position = msg.position[idx]
                    if len(msg.effort) > idx:
                        self._current_effort = msg.effort[idx]
        except (ValueError, IndexError) as e:
            # Joint not found or index error
            pass
    
    def is_connected(self):
        """Check if gripper controller is connected."""
        return self._connection_established
    
    def get_current_position(self):
        """
        Get current gripper position.
        :return: Current position in meters (0.0 = open, 0.042 = closed)
        """
        with self._state_lock:
            return self._current_position
    
    def get_current_effort(self):
        """
        Get current gripper effort/force.
        :return: Current effort in Newtons
        """
        with self._state_lock:
            return self._current_effort
    
    def is_moving(self):
        """Check if gripper is currently moving."""
        return self._is_moving
    
    def get_gripper_state(self):
        """
        Get comprehensive gripper state.
        :return: Dictionary with position, effort, and status
        """
        with self._state_lock:
            pos = self._current_position
            effort = self._current_effort
        
        # Determine state
        if pos < 0.005:
            state = "open"
        elif pos > 0.037:
            state = "closed" 
        else:
            state = "partial"
        
        return {
            'position': pos,
            'position_mm': pos * 1000.0,
            'effort': effort,
            'state': state,
            'is_moving': self._is_moving,
            'is_open': pos < 0.005,
            'is_closed': pos > 0.037
        }
    
    def _send_gripper_command(self, position, max_effort=None, blocking=True):
        """
        Send gripper command via action interface.
        :param position: Target position (0.0 to 0.042 meters)
        :param max_effort: Maximum effort (Newtons)
        :param blocking: Wait for completion if True
        :return: True if successful
        """
        if not self._connection_established:
            self.get_logger().error("Gripper not connected")
            return False
        
        # Clamp position to valid range
        position = max(self.GRIPPER_OPEN_POSITION, 
                      min(self.GRIPPER_CLOSED_POSITION, position))
        
        if max_effort is None:
            max_effort = self.GRIPPER_MAX_EFFORT
        else:
            max_effort = max(0.0, min(self.GRIPPER_MAX_EFFORT, max_effort))
        
        # Create goal
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        
        self.get_logger().info(f"Sending gripper command: pos={position:.3f}m, effort={max_effort:.1f}N")
        
        # Send goal
        self._is_moving = True
        future = self._action_client.send_goal_async(goal)
        
        if blocking:
            # Wait for goal to be accepted
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if not future.done():
                self.get_logger().error("Gripper goal submission timed out")
                self._is_moving = False
                return False
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error("Gripper goal rejected")
                self._is_moving = False
                return False
            
            # Wait for completion
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)
            
            self._is_moving = False
            
            if result_future.done():
                result = result_future.result()
                if result.result.reached_goal:
                    self.get_logger().info(f"✅ Gripper reached target position")
                    return True
                else:
                    self.get_logger().warn(f"⚠️  Gripper stopped before reaching target (likely contact)")
                    return True  # Still consider success for grasping
            else:
                self.get_logger().error("Gripper command timed out")
                return False
        else:
            # Non-blocking mode
            def done_callback(future):
                self._is_moving = False
                try:
                    goal_handle = future.result()
                    if goal_handle.accepted:
                        self.get_logger().info("Gripper goal accepted")
                    else:
                        self.get_logger().error("Gripper goal rejected")
                except Exception as e:
                    self.get_logger().error(f"Gripper goal error: {e}")
            
            future.add_done_callback(done_callback)
            return True
    
    def open_gripper(self, blocking=True):
        """
        Open gripper fully.
        :param blocking: Wait for completion if True
        :return: True if successful
        """
        self.get_logger().info("🖐️  Opening gripper")
        return self._send_gripper_command(self.GRIPPER_OPEN_POSITION, blocking=blocking)
    
    def close_gripper(self, max_effort=None, blocking=True):
        """
        Close gripper fully.
        :param max_effort: Maximum closing force (Newtons)
        :param blocking: Wait for completion if True
        :return: True if successful
        """
        if max_effort is None:
            max_effort = self.GRIPPER_MAX_EFFORT * 0.8  # Use 80% of max force by default
        
        self.get_logger().info(f"✊ Closing gripper (max effort: {max_effort:.1f}N)")
        return self._send_gripper_command(self.GRIPPER_CLOSED_POSITION, max_effort, blocking=blocking)
    
    def set_gripper_position(self, position_mm, max_effort=None, blocking=True):
        """
        Set gripper to specific position.
        :param position_mm: Target position in millimeters (0-42mm)
        :param max_effort: Maximum effort (Newtons)
        :param blocking: Wait for completion if True
        :return: True if successful
        """
        # Convert mm to meters
        position_m = position_mm / 1000.0
        
        self.get_logger().info(f"🤏 Setting gripper position: {position_mm:.1f}mm")
        return self._send_gripper_command(position_m, max_effort, blocking=blocking)
    
    def grasp_object(self, max_effort=None, position_mm=None):
        """
        Attempt to grasp an object with controlled force.
        :param max_effort: Maximum grasping force (Newtons)
        :param position_mm: Target position in mm (if None, close fully)
        :return: True if successful
        """
        if max_effort is None:
            max_effort = self.GRIPPER_MAX_EFFORT * 0.6  # Gentle grasp by default
        
        if position_mm is None:
            position_m = self.GRIPPER_CLOSED_POSITION
        else:
            position_m = position_mm / 1000.0
        
        self.get_logger().info(f"🫴 Attempting to grasp object (effort: {max_effort:.1f}N)")
        
        success = self._send_gripper_command(position_m, max_effort, blocking=True)
        
        if success:
            # Check if we actually grasped something
            final_pos = self.get_current_position()
            if final_pos < (position_m - 0.005):  # Gripper stopped before target
                self.get_logger().info("✅ Object detected - grasp successful")
                return True
            else:
                self.get_logger().warn("⚠️  No object detected - gripper reached target position")
                return False
        
        return False
    
    def release_object(self):
        """
        Release grasped object by opening gripper.
        :return: True if successful
        """
        self.get_logger().info("🫳 Releasing object")
        return self.open_gripper(blocking=True)
    
    def emergency_stop(self):
        """Emergency stop - cancel current gripper action."""
        if self._action_client.server_is_ready():
            self._action_client.cancel_all_goals()
            self.get_logger().warn("🛑 Gripper emergency stop - all goals cancelled")
        self._is_moving = False


def main(args=None):
    """Gripper controller test function."""
    rclpy.init(args=args)
    
    try:
        gripper = GripperController()
        
        if not gripper.is_connected():
            print("❌ Gripper not connected. Make sure simulation is running.")
            return
        
        print("🤖 Gripper controller ready!")
        print("Testing gripper functionality...")
        
        # Test sequence
        time.sleep(1.0)
        
        print("\n1. Opening gripper...")
        gripper.open_gripper()
        time.sleep(2.0)
        print(f"   Status: {gripper.get_gripper_state()}")
        
        print("\n2. Closing gripper...")
        gripper.close_gripper()
        time.sleep(2.0)
        print(f"   Status: {gripper.get_gripper_state()}")
        
        print("\n3. Setting to 20mm...")
        gripper.set_gripper_position(20.0)
        time.sleep(2.0)
        print(f"   Status: {gripper.get_gripper_state()}")
        
        print("\n✅ Gripper test completed successfully!")
        
        # Keep running for manual testing
        print("\nGripper controller running. Press Ctrl+C to stop.")
        rclpy.spin(gripper)
        
    except KeyboardInterrupt:
        print("\n🛑 Gripper controller interrupted by user")
    except Exception as e:
        print(f"❌ Error in gripper controller: {e}")
    finally:
        if 'gripper' in locals():
            gripper.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()