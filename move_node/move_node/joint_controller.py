#!/usr/bin/env python3
"""
joint_controller.py - Joint control using robot_common base classes
"""

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

# Import from robot_common package
from robot_common import RobotConnectionBase, RobotStateManager

class JointController(RobotConnectionBase):
    """Controller for joint-space movements"""
    
    def __init__(self):
        # Joint configuration
        self._joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Initialize base class
        super().__init__(
            node_name='joint_controller',
            action_name='/joint_trajectory_controller/follow_joint_trajectory',
            action_type=FollowJointTrajectory
        )
    
    def _on_connection_established(self):
        """Initialize robot state manager after connection"""
        # Create state manager
        self._state_manager = RobotStateManager(self, self._joint_names)
        
        # Wait for joint states
        if not self._state_manager.wait_for_joint_states():
            return False
        
        return True
    
    def is_connected(self):
        """Check if controller is connected and has joint states"""
        return super().is_connected() and self._state_manager.has_joint_states()
    
    def get_current_joint_positions(self):
        """Get current joint positions"""
        return self._state_manager.get_current_joint_positions()
    
    def move_to_joint_positions(self, target_positions, wait_for_completion=True):
        """
        Execute movement to target joint positions
        
        Args:
            target_positions: List of 6 joint angles in radians
            wait_for_completion: If True, block until movement completes
            
        Returns:
            bool: True if movement was successful
        """
        if not self.is_connected():
            self.get_logger().error('Controller not connected!')
            return False
        
        if len(target_positions) != 6:
            self.get_logger().error(f'Expected 6 joint positions, got {len(target_positions)}')
            return False
        
        # Calculate movement duration
        max_displacement, _ = self._state_manager.calculate_joint_displacement(target_positions)
        if max_displacement is None:
            self.get_logger().error('Cannot calculate displacement')
            return False
        
        duration = self.calculate_movement_duration(max_displacement)
        
        # Log movement info
        current_pos = self.get_current_joint_positions()
        self.get_logger().info(f'Current: {[f"{p:.3f}" for p in current_pos]}')
        self.get_logger().info(f'Target:  {[f"{p:.3f}" for p in target_positions]}')
        self.get_logger().info(f'Duration: {duration:.1f}s at {self._velocity_percentage}% speed')
        
        # Create trajectory message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self._joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
        goal_msg.trajectory.points = [point]
        
        # Set tolerances
        goal_tolerances = []
        for _ in self._joint_names:
            tolerance = JointTolerance()
            tolerance.position = self._position_tolerance
            tolerance.velocity = 0.1
            goal_tolerances.append(tolerance)
        goal_msg.goal_tolerance = goal_tolerances
        goal_msg.goal_time_tolerance = Duration(sec=int(self._goal_time_tolerance))
        
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
            self.get_logger().error('Goal rejected by action server')
            return False
        
        self.get_logger().info('Goal accepted')
        
        if not wait_for_completion:
            return True
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        timeout = duration + self._goal_time_tolerance + 5.0
        start_time = time.time()
        last_log_time = 0
        
        while not result_future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # Periodic position check
            current_time = time.time()
            if current_time - last_log_time > 1.0:
                if self._state_manager.check_position_reached(target_positions, self._position_tolerance):
                    self.get_logger().info('Target position reached')
                last_log_time = current_time
        
        # Check result
        if result_future.done():
            result = result_future.result()
            if result.result.error_code == 0:
                self.get_logger().info('Movement completed successfully')
                return True
            else:
                self.get_logger().error(f'Movement failed with error code: {result.result.error_code}')
                return False
        else:
            self.get_logger().error('Movement timeout')
            # Check if we actually reached the position
            if self._state_manager.check_position_reached(target_positions, self._position_tolerance * 2):
                self.get_logger().info('Position reached despite timeout')
                return True
            return False

def main(args=None):
    """Test the joint controller"""
    rclpy.init(args=args)
    
    try:
        controller = JointController()
        
        if controller.is_connected():
            # Test movements
            controller.set_velocity_percentage(30)
            controller.set_position_tolerance(0.02)
            
            # Define test positions
            home_pos = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            pos1 = [0.785, -1.57, 0.0, -1.57, 0.0, 0.0]
            pos2 = [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]
            
            # Execute movements
            print("Moving to home...")
            if controller.move_to_joint_positions(home_pos):
                print("✓ Home position reached")
            
            print("\nMoving to 45 degrees...")
            if controller.move_to_joint_positions(pos1):
                print("✓ 45 degrees position reached")
            
            print("\nMoving to 90 degrees...")
            if controller.move_to_joint_positions(pos2):
                print("✓ 90 degrees position reached")
            
            print("\nReturning home...")
            if controller.move_to_joint_positions(home_pos):
                print("✓ Returned to home")
            
            print("\nTest completed successfully!")
        else:
            print("Failed to connect to robot")
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()