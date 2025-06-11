#!/usr/bin/env python3
"""
position_controller.py - Cartesian space controller for point-to-point movement
Uses MoveIt2 actions and services (ROS2 native approach)
"""

import rclpy
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import Header
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, 
    OrientationConstraint, WorkspaceParameters, RobotState,
    PlanningOptions, MoveItErrorCodes
)
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
import time
import math

from robot_common import RobotConnectionBase, RobotStateManager

class PositionController(RobotConnectionBase):
    """
    Controller for Cartesian space movement using MoveIt2 actions and services.
    Handles point-to-point movement with inverse kinematics.
    """
    
    def __init__(self, planning_group="ur_manipulator"):
        # Joint configuration
        self._joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        self._planning_group = planning_group
        self._planning_frame = "base_link"
        self._end_effector_link = "tool0"
        
        # Initialize base class
        super().__init__(
            node_name='position_controller',
            action_name='/joint_trajectory_controller/follow_joint_trajectory',
            action_type=FollowJointTrajectory
        )
    
    def _on_connection_established(self):
        """Initialize MoveIt2 action clients and services after connection"""
        try:
            # Create MoveIt2 action client for motion planning
            self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
            
            # Create service clients for kinematics
            self._ik_client = self.create_client(GetPositionIK, '/compute_ik')
            self._fk_client = self.create_client(GetPositionFK, '/compute_fk')
            
            # Wait for MoveIt2 services
            self.get_logger().info('Waiting for MoveIt2 services...')
            if not self._move_group_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error('MoveGroup action server not available!')
                return False
            
            if not self._ik_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('IK service not available!')
                return False
            
            # Create state manager
            self._state_manager = RobotStateManager(self, self._joint_names)
            
            # Wait for joint states
            if not self._state_manager.wait_for_joint_states():
                self.get_logger().error('Failed to get joint states')
                return False
            
            self.get_logger().info('MoveIt2 position controller ready')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize MoveIt2: {e}')
            return False
    
    def is_connected(self):
        """Check if controller is connected and has joint states"""
        return super().is_connected() and self._state_manager.has_joint_states()
    
    def get_current_pose(self):
        """
        Get current end-effector pose using forward kinematics
        
        Returns:
            geometry_msgs.msg.Pose: Current pose or None if unavailable
        """
        try:
            # Get current joint positions
            current_joints = self._state_manager.get_current_joint_positions()
            if current_joints is None:
                return None
            
            # Create FK request
            request = GetPositionFK.Request()
            request.header.frame_id = self._planning_frame
            request.header.stamp = self.get_clock().now().to_msg()
            request.fk_link_names = [self._end_effector_link]
            
            # Set robot state
            request.robot_state.joint_state.header = request.header
            request.robot_state.joint_state.name = self._joint_names
            request.robot_state.joint_state.position = current_joints
            
            # Call FK service
            future = self._fk_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.result() is not None:
                response = future.result()
                if response.error_code.val == MoveItErrorCodes.SUCCESS:
                    return response.pose_stamped[0].pose
                else:
                    self.get_logger().error(f'FK failed with error: {response.error_code.val}')
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'Failed to get current pose: {e}')
            return None
    
    def get_current_joint_positions(self):
        """Get current joint positions"""
        return self._state_manager.get_current_joint_positions()
    
    def move_to_cartesian_position(self, target_pose, wait_for_completion=True):
        """
        Move to target Cartesian position using MoveIt2 planning
        
        Args:
            target_pose: geometry_msgs.msg.Pose or dict with position/orientation
            wait_for_completion: If True, block until movement completes
            
        Returns:
            bool: True if movement was successful
        """
        if not self.is_connected():
            self.get_logger().error('Controller not connected!')
            return False
        
        try:
            # Convert dict to Pose if necessary
            if isinstance(target_pose, dict):
                pose = Pose()
                pose.position = Point(**target_pose.get('position', {'x': 0, 'y': 0, 'z': 0}))
                pose.orientation = Quaternion(**target_pose.get('orientation', {'x': 0, 'y': 0, 'z': 0, 'w': 1}))
                target_pose = pose
            
            # Log current and target poses
            current_pose = self.get_current_pose()
            if current_pose:
                self.get_logger().info(f'Current pose: x={current_pose.position.x:.3f}, '
                                     f'y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}')
            
            self.get_logger().info(f'Target pose: x={target_pose.position.x:.3f}, '
                                 f'y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}')
            
            # First, try inverse kinematics to get joint solution
            joint_solution = self._solve_ik(target_pose)
            if joint_solution is None:
                self.get_logger().error('No IK solution found for target pose')
                return False
            
            # Execute the joint trajectory
            return self._execute_joint_solution(joint_solution, wait_for_completion)
            
        except Exception as e:
            self.get_logger().error(f'Error in Cartesian movement: {e}')
            return False
    
    def move_to_position_xyz(self, x, y, z, orientation=None, wait_for_completion=True):
        """
        Move to target position with optional orientation
        
        Args:
            x, y, z: Target position coordinates
            orientation: dict with 'x', 'y', 'z', 'w' quaternion or None to keep current
            wait_for_completion: If True, block until movement completes
            
        Returns:
            bool: True if movement was successful
        """
        # Always get fresh current pose for consistency
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error('Cannot get current pose')
            return False
            
        # Use current orientation if not provided
        if orientation is None:
            orientation = {
                'x': current_pose.orientation.x,
                'y': current_pose.orientation.y,
                'z': current_pose.orientation.z,
                'w': current_pose.orientation.w
            }
        
        # Log positions for debugging
        self.get_logger().info(f'Internal current pose: x={current_pose.position.x:.3f}, '
                             f'y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}')
        self.get_logger().info(f'Requested target XYZ: x={x:.3f}, y={y:.3f}, z={z:.3f}')
        
        # Use the internally obtained current position as reference and apply relative movement
        # This ensures consistency between what we think is current and what we're targeting
        target_pose = {
            'position': {'x': x, 'y': y, 'z': z},
            'orientation': orientation
        }
        
        return self.move_to_cartesian_position(target_pose, wait_for_completion)
    
    def move_relative_xyz(self, dx, dy, dz, description="Relative move", wait_for_completion=True):
        """
        Move relative to current position using forward kinematics
        
        Args:
            dx, dy, dz: Relative movement in meters
            description: Movement description  
            wait_for_completion: If True, block until movement completes
            
        Returns:
            bool: True if movement was successful
        """
        # Get current joint angles (raw sensor data)
        current_joints = self._state_manager.get_current_joint_positions()
        if current_joints is None:
            self.get_logger().error('Cannot get current joint angles')
            return False
        
        # Get current pose via FK (computed from joint angles)
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().error('Cannot compute current pose via FK')
            return False
        
        # Calculate target position
        target_x = current_pose.position.x + dx
        target_y = current_pose.position.y + dy  
        target_z = current_pose.position.z + dz
        
        # Log joint angles and computed pose for debugging
        joint_degrees = [math.degrees(j) for j in current_joints]
        self.get_logger().info(f'Current joint angles (deg): {[f"{j:.1f}" for j in joint_degrees]}')
        self.get_logger().info(f'FK computed pose: x={current_pose.position.x:.3f}, y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}')
        self.get_logger().info(f'Relative movement: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}')
        self.get_logger().info(f'Target pose: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}')
        
        # Use current orientation (maintain end-effector orientation)
        orientation = {
            'x': current_pose.orientation.x,
            'y': current_pose.orientation.y,
            'z': current_pose.orientation.z,
            'w': current_pose.orientation.w
        }
        
        target_pose = {
            'position': {'x': target_x, 'y': target_y, 'z': target_z},
            'orientation': orientation
        }
        
        return self.move_to_cartesian_position(target_pose, wait_for_completion)
    
    def _solve_ik(self, target_pose):
        """
        Solve inverse kinematics for target pose
        
        Args:
            target_pose: geometry_msgs.msg.Pose
            
        Returns:
            list: Joint positions or None if no solution
        """
        try:
            # Create IK request
            request = GetPositionIK.Request()
            
            # Set up the IK request
            request.ik_request.group_name = self._planning_group
            request.ik_request.ik_link_name = self._end_effector_link
            request.ik_request.avoid_collisions = True
            request.ik_request.timeout = Duration(sec=5)
            
            # Set target pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self._planning_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = target_pose
            request.ik_request.pose_stamped = pose_stamped
            
            # Set current robot state as starting point
            current_joints = self._state_manager.get_current_joint_positions()
            if current_joints is None:
                return None
            
            request.ik_request.robot_state.joint_state.header = pose_stamped.header
            request.ik_request.robot_state.joint_state.name = self._joint_names
            request.ik_request.robot_state.joint_state.position = current_joints
            
            # Call IK service
            future = self._ik_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                response = future.result()
                if response.error_code.val == MoveItErrorCodes.SUCCESS:
                    # Extract joint positions from solution
                    solution_joints = []
                    for joint_name in self._joint_names:
                        if joint_name in response.solution.joint_state.name:
                            idx = response.solution.joint_state.name.index(joint_name)
                            solution_joints.append(response.solution.joint_state.position[idx])
                        else:
                            self.get_logger().error(f'Joint {joint_name} not found in IK solution')
                            return None
                    
                    self.get_logger().info('IK solution found')
                    return solution_joints
                else:
                    self.get_logger().error(f'IK failed with error: {response.error_code.val}')
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'Error in IK solving: {e}')
            return None
    
    def _execute_joint_solution(self, joint_positions, wait_for_completion=True):
        """
        Execute joint trajectory to reach the IK solution
        
        Args:
            joint_positions: List of joint positions
            wait_for_completion: If True, block until movement completes
            
        Returns:
            bool: True if execution was successful
        """
        try:
            # Calculate movement duration
            max_displacement, _ = self._state_manager.calculate_joint_displacement(joint_positions)
            if max_displacement is None:
                self.get_logger().error('Cannot calculate displacement')
                return False
            
            duration = self.calculate_movement_duration(max_displacement)
            
            # Create trajectory message
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self._joint_names
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = joint_positions
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
            
            self.get_logger().info('Cartesian trajectory execution started')
            
            if not wait_for_completion:
                return True
            
            # Wait for completion
            result_future = goal_handle.get_result_async()
            timeout = duration + self._goal_time_tolerance + 5.0
            start_time = time.time()
            
            while not result_future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Check result
            if result_future.done():
                result = result_future.result()
                if result.result.error_code == 0:
                    self.get_logger().info('Cartesian movement completed successfully')
                    return True
                else:
                    self.get_logger().error(f'Movement failed with error code: {result.result.error_code}')
                    return False
            else:
                self.get_logger().error('Movement timeout')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error executing joint solution: {e}')
            return False

def main(args=None):
    """
    Unit test for PositionController
    """
    rclpy.init(args=args)
    try:
        controller = PositionController()
        if controller.is_connected():
            controller.set_velocity_percentage(30)
            
            # Get current pose
            current_pose = controller.get_current_pose()
            if current_pose:
                print(f"Current position: x={current_pose.position.x:.3f}, "
                      f"y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}")
                
                # Test movement: move 10cm in +X direction
                target_x = current_pose.position.x + 0.1
                target_y = current_pose.position.y
                target_z = current_pose.position.z
                
                print(f"Moving to: x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}")
                
                if controller.move_to_position_xyz(target_x, target_y, target_z):
                    print("Movement completed successfully!")
                    time.sleep(1.0)
                    
                    # Move back
                    print("Moving back to original position...")
                    if controller.move_to_position_xyz(
                        current_pose.position.x,
                        current_pose.position.y,
                        current_pose.position.z
                    ):
                        print("Returned to original position!")
                else:
                    print("Movement failed")
            else:
                print("Failed to get current pose")
        else:
            print("Failed to connect to robot")
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()