#!/usr/bin/env python3
"""
gripper_controller.py - Panda gripper control using GripperCommand action
Provides position and force control for the Panda gripper
"""

import rclpy
from control_msgs.action import GripperCommand
import time

# Import from robot_common package
from robot_common import RobotConnectionBase

class GripperController(RobotConnectionBase):
    """
    Controller for Panda gripper using GripperCommand action.
    Supports position control with force limiting.
    """
    
    def __init__(self):
        # Gripper specifications for Panda
        self.GRIPPER_MAX_WIDTH = 0.08    # Maximum opening width (80mm)
        self.GRIPPER_MIN_WIDTH = 0.0     # Fully closed
        self.DEFAULT_SPEED = 0.03        # Default closing/opening speed
        self.DEFAULT_MAX_EFFORT = 50.0   # Default max effort (Newtons)
        self.ANIMATION_STEP_SIZE = 0.005 # 5mm step size for smooth animation
        
        # Initialize base class
        super().__init__(
            node_name='gripper_controller',
            action_name='/panda_hand_controller/gripper_cmd',
            action_type=GripperCommand
        )
        
        # Current gripper state
        self._last_result = None
        self._current_position = None
    
    def _on_connection_established(self):
        """Initialize after connection - no additional setup needed for gripper"""
        self.get_logger().info('Gripper controller ready')
        return True
    
    def move_to_width(self, target_width, max_effort=None, wait_for_completion=True):
        """
        Move gripper to target width with optional force limiting.
        
        Args:
            target_width: Target gripper width in meters (0.0 = closed, 0.08 = fully open)
            max_effort: Maximum force in Newtons (default: 50.0)
            wait_for_completion: If True, block until movement completes
            
        Returns:
            bool: True if movement was successful
        """
        if not self.is_connected():
            self.get_logger().error('Gripper controller not connected!')
            return False
        
        # Validate and clamp target width
        target_width = max(self.GRIPPER_MIN_WIDTH, min(self.GRIPPER_MAX_WIDTH, target_width))
        
        if max_effort is None:
            max_effort = self.DEFAULT_MAX_EFFORT
        
        self.get_logger().info(f'Moving gripper to width: {target_width*1000:.1f}mm, max_effort: {max_effort:.1f}N')
        
        # Create goal message
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = target_width
        goal_msg.command.max_effort = max_effort
        
        # Send goal
        future = self._action_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not future.done():
            self.get_logger().error('Goal submission timeout')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected by action server')
            return False
        
        self.get_logger().info('Gripper goal accepted')
        
        if not wait_for_completion:
            return True
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        timeout = 10.0  # 10 second timeout for gripper movements
        start_time = time.time()
        
        while not result_future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Check result
        if result_future.done():
            result = result_future.result()
            self._last_result = result.result
            
            self.get_logger().info(
                f'Gripper movement completed: '
                f'position={result.result.position*1000:.1f}mm, '
                f'effort={result.result.effort:.1f}N, '
                f'stalled={result.result.stalled}, '
                f'reached_goal={result.result.reached_goal}'
            )
            
            # Update current position tracking
            self._current_position = result.result.position
            return result.result.reached_goal
        else:
            self.get_logger().error('Gripper movement timeout')
            return False
    
    def open_gripper(self, wait_for_completion=True, smooth_animation=True):
        """
        Open gripper to maximum width.
        
        Args:
            wait_for_completion: If True, block until movement completes
            smooth_animation: If True, use smooth interpolated animation
            
        Returns:
            bool: True if successful
        """
        self.get_logger().info('Opening gripper')
        target_width = self.GRIPPER_MAX_WIDTH * 0.9  # 90% of max width for safety
        
        if smooth_animation and wait_for_completion:
            return self._animate_to_width(target_width, self.DEFAULT_MAX_EFFORT)
        else:
            return self.move_to_width(
                target_width=target_width,
                max_effort=self.DEFAULT_MAX_EFFORT,
                wait_for_completion=wait_for_completion
            )
    
    def close_gripper(self, wait_for_completion=True, smooth_animation=True):
        """
        Close gripper completely.
        
        Args:
            wait_for_completion: If True, block until movement completes
            smooth_animation: If True, use smooth interpolated animation
            
        Returns:
            bool: True if successful
        """
        self.get_logger().info('Closing gripper')
        
        if smooth_animation and wait_for_completion:
            return self._animate_to_width(self.GRIPPER_MIN_WIDTH, self.DEFAULT_MAX_EFFORT)
        else:
            return self.move_to_width(
                target_width=self.GRIPPER_MIN_WIDTH,
                max_effort=self.DEFAULT_MAX_EFFORT,
                wait_for_completion=wait_for_completion
            )
    
    def grasp_object(self, target_width=0.01, max_effort=30.0, wait_for_completion=True, smooth_animation=True):
        """
        Attempt to grasp an object with specified force.
        
        Args:
            target_width: Minimum width to maintain (meters)
            max_effort: Maximum grasping force (Newtons)
            wait_for_completion: If True, block until movement completes
            smooth_animation: If True, use smooth interpolated animation
            
        Returns:
            bool: True if grasp attempt completed
        """
        self.get_logger().info(f'Attempting to grasp object with force: {max_effort:.1f}N')
        
        if smooth_animation and wait_for_completion:
            return self._animate_to_width(target_width, max_effort)
        else:
            return self.move_to_width(
                target_width=target_width,
                max_effort=max_effort,
                wait_for_completion=wait_for_completion
            )
    
    def get_gripper_state(self):
        """
        Get current gripper state from last action result.
        
        Returns:
            GripperCommandResult or None if no recent action
        """
        return self._last_result
    
    def get_current_width(self):
        """
        Get current gripper width in meters.
        
        Returns:
            float: Current width in meters, or None if unavailable
        """
        if self._last_result:
            return self._last_result.position
        return None
    
    def _animate_to_width(self, target_width, max_effort=None):
        """
        Smoothly animate gripper to target width using interpolation.
        
        Args:
            target_width: Target gripper width in meters
            max_effort: Maximum force in Newtons
            
        Returns:
            bool: True if animation completed successfully
        """
        if not self.is_connected():
            return False
        
        # Get current position
        current_width = self._current_position if self._current_position is not None else 0.04
        
        # Calculate step direction and count
        width_diff = target_width - current_width
        if abs(width_diff) <= self.ANIMATION_STEP_SIZE:
            # Close enough, move directly
            return self.move_to_width(target_width, max_effort, wait_for_completion=True)
        
        # Calculate number of steps
        step_direction = 1 if width_diff > 0 else -1
        num_steps = int(abs(width_diff) / self.ANIMATION_STEP_SIZE)
        
        self.get_logger().info(f'Animating gripper from {current_width*1000:.1f}mm to {target_width*1000:.1f}mm in {num_steps} steps')
        
        # Perform interpolated movement
        for i in range(num_steps):
            intermediate_width = current_width + (i + 1) * step_direction * self.ANIMATION_STEP_SIZE
            
            # For intermediate steps, use lower effort to allow smooth motion
            intermediate_effort = max_effort * 0.7 if max_effort else self.DEFAULT_MAX_EFFORT * 0.7
            
            if not self.move_to_width(intermediate_width, intermediate_effort, wait_for_completion=True):
                self.get_logger().warning(f'Animation step {i+1} failed')
                return False
            
            # Small delay between steps for smooth animation
            time.sleep(0.1)
        
        # Final movement to exact target with full effort
        return self.move_to_width(target_width, max_effort, wait_for_completion=True)
    
    def is_grasping(self):
        """
        Check if gripper is currently grasping an object.
        Based on effort and stall detection.
        
        Returns:
            bool: True if likely grasping an object
        """
        if not self._last_result:
            return False
        
        # Consider grasping if:
        # 1. Gripper is not fully closed (position > 0.005m)
        # 2. Effort is above threshold (indicating contact)
        # 3. Gripper is stalled (can't move further)
        return (self._last_result.position > 0.005 and 
                self._last_result.effort > 5.0 and 
                self._last_result.stalled)


def main(args=None):
    """
    Unit test for GripperController
    """
    rclpy.init(args=args)
    try:
        controller = GripperController()
        if controller.is_connected():
            print("🤖 Gripper Controller Test")
            
            # Test sequence with smooth animation
            test_actions = [
                ("Opening gripper (smooth)", lambda: controller.open_gripper(smooth_animation=True)),
                ("Closing gripper (smooth)", lambda: controller.close_gripper(smooth_animation=True)),
                ("Partial open (30mm, smooth)", lambda: controller._animate_to_width(0.03)),
                ("Grasp test (smooth)", lambda: controller.grasp_object(max_effort=20.0, smooth_animation=True)),
            ]
            
            for description, action in test_actions:
                print(f"\n{description}...")
                if action():
                    state = controller.get_gripper_state()
                    if state:
                        print(f"  Position: {state.position*1000:.1f}mm")
                        print(f"  Effort: {state.effort:.1f}N")
                        print(f"  Grasping: {controller.is_grasping()}")
                    time.sleep(1.0)
                else:
                    print(f"  ❌ {description} failed")
            
            print("\n✅ Gripper controller test completed!")
        else:
            print("❌ Failed to connect to gripper controller")
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()