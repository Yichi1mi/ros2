#!/usr/bin/env python3
"""
joint_controller.py - Joint control with proper position feedback and tolerance checking
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import threading
import queue
import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.get_logger().info('Starting joint controller with position feedback')
        
        # Current joint state
        self._current_joint_positions = None
        self._current_joint_velocities = None
        self._joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self._joint_state_lock = threading.Lock()
        
        # Subscribe to joint states for position feedback FIRST
        self.get_logger().info('Creating joint state subscription...')
        self._joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        self.get_logger().info('Joint state subscription created')
        
        # Action client
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Speed control parameters
        self._base_angular_velocity = 1.0  # rad/s
        self._velocity_percentage = 50
        self._current_velocity = self._base_angular_velocity * (self._velocity_percentage / 100.0)
        
        # Tolerance settings
        self._position_tolerance = 0.02  # 0.02 rad tolerance
        self._velocity_tolerance = 0.1   # 0.1 rad/s tolerance
        self._goal_time_tolerance = 2.0  # 2 seconds tolerance
        
        # Command queue system
        self._command_queue = queue.Queue()
        self._queue_stopped = True
        self._queue_executing = False
        self._current_command_index = 0
        self._command_counter = 0
        self._queue_lock = threading.Lock()
        
        # Executor thread
        self._executor_thread = threading.Thread(target=self._queue_executor, daemon=True)
        self._shutdown_flag = False
        self._executor_thread.start()
        
        # Connect to robot
        self.get_logger().info('Waiting for action server...')
        if self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Action server connected!')
            self._connected = True
            # Wait for first joint state
            self._wait_for_joint_states()
            self._initialize_robot()
        else:
            self.get_logger().error('Action server not available!')
            self._connected = False
    
    def _joint_state_callback(self, msg):
        """Callback for joint state updates"""
        with self._joint_state_lock:
            try:
                # Reorder joint positions to match our joint names
                positions = []
                velocities = []
                for joint_name in self._joint_names:
                    if joint_name in msg.name:
                        idx = msg.name.index(joint_name)
                        positions.append(msg.position[idx])
                        if msg.velocity and len(msg.velocity) > idx:
                            velocities.append(msg.velocity[idx])
                        else:
                            velocities.append(0.0)
                    else:
                        self.get_logger().error(f'Joint {joint_name} not found in joint states')
                        return
                
                if len(positions) == len(self._joint_names):
                    self._current_joint_positions = positions
                    self._current_joint_velocities = velocities if velocities else None
                
            except (ValueError, IndexError) as e:
                self.get_logger().error(f'Error processing joint state: {e}')
                self.get_logger().error(f'Available joints: {msg.name}')
                self.get_logger().error(f'Expected joints: {self._joint_names}')
    
    def _wait_for_joint_states(self):
        """Wait for first joint state message"""
        self.get_logger().info('Waiting for joint states...')
        timeout = 10.0
        start_time = time.time()
        
        while (time.time() - start_time) < timeout:
            # CRITICAL: Must spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
            
            with self._joint_state_lock:
                if self._current_joint_positions is not None:
                    self.get_logger().info('Joint states received!')
                    self.get_logger().info(f'Current positions: {[f"{p:.3f}" for p in self._current_joint_positions]}')
                    return True
            
            time.sleep(0.1)
        
        self.get_logger().error('Timeout waiting for joint states')
        self.get_logger().error('Check if joint_state_publisher or robot is running')
        return False
    
    def get_current_joint_positions(self):
        """Get current joint positions thread-safely with real-time update"""
        # Spin once to get latest joint states
        rclpy.spin_once(self, timeout_sec=0.01)
        
        with self._joint_state_lock:
            return self._current_joint_positions.copy() if self._current_joint_positions else None
    
    def _check_position_reached(self, target_positions, tolerance=None):
        """Check if robot reached target position within tolerance"""
        if tolerance is None:
            tolerance = self._position_tolerance
        
        current_positions = self.get_current_joint_positions()
        if current_positions is None:
            self.get_logger().warn('No current position available for checking')
            return False
        
        max_error = 0.0
        errors = []
        
        for i, (target, current) in enumerate(zip(target_positions, current_positions)):
            error = abs(target - current)
            errors.append(error)
            max_error = max(max_error, error)
            
            if error > tolerance:
                self.get_logger().debug(f'Joint {self._joint_names[i]} error: {error:.4f} rad > tolerance {tolerance:.4f} rad')
        
        is_reached = max_error <= tolerance
        
        if is_reached:
            self.get_logger().info(f'Position reached! Max error: {max_error:.4f} rad')
        else:
            self.get_logger().info(f'Position not reached. Max error: {max_error:.4f} rad, tolerance: {tolerance:.4f} rad')
            for i, error in enumerate(errors):
                if error > tolerance:
                    self.get_logger().info(f'  {self._joint_names[i]}: target={target_positions[i]:.3f}, current={current_positions[i]:.3f}, error={error:.4f}')
        
        return is_reached
    
    def _initialize_robot(self):
        """Initialize robot"""
        self.get_logger().info('Initializing robot...')
        self.set_queued_cmd_stop_exec()
        self.set_queued_cmd_clear()
        self.set_velocity_scaling(velocity_percentage=50)
        self.set_position_tolerance(tolerance=0.02)
        self.get_logger().info('Robot initialized')
    
    def set_velocity_scaling(self, velocity_percentage=50):
        """Set velocity scaling percentage"""
        self._velocity_percentage = max(1, min(100, velocity_percentage))
        self._current_velocity = self._base_angular_velocity * (self._velocity_percentage / 100.0)
        self.get_logger().info(f'Set velocity to {self._velocity_percentage}% ({self._current_velocity:.3f} rad/s)')
    
    def set_position_tolerance(self, tolerance=0.02):
        """Set position tolerance for goal reaching"""
        self._position_tolerance = max(tolerance, 0.01)
        self.get_logger().info(f'Set position tolerance to {self._position_tolerance:.4f} rad ({self._position_tolerance*57.3:.2f} deg)')
    
    def set_queued_cmd_stop_exec(self):
        """Stop queue execution"""
        with self._queue_lock:
            self._queue_stopped = True
        self.get_logger().info('Queue execution stopped')
    
    def set_queued_cmd_clear(self):
        """Clear command queue"""
        with self._queue_lock:
            cleared_count = 0
            try:
                while True:
                    self._command_queue.get_nowait()
                    cleared_count += 1
            except queue.Empty:
                pass
            self._command_counter = 0
            self._current_command_index = 0
        self.get_logger().info(f'Cleared {cleared_count} commands from queue')
    
    def set_queued_cmd_start_exec(self):
        """Start queue execution"""
        with self._queue_lock:
            self._queue_stopped = False
        self.get_logger().info('Queue execution started')
    
    def get_queued_cmd_current_index(self):
        """Get current command index"""
        return self._current_command_index
    
    def _calculate_duration(self, target_angles):
        """Calculate movement duration based on velocity"""
        current_angles = self.get_current_joint_positions()
        if current_angles is None:
            self.get_logger().warn('No current position available, using default duration')
            return 3.0
        
        max_angle_change = max(abs(t - c) for t, c in zip(target_angles, current_angles))
        
        # Base duration calculation with acceleration buffer
        base_duration = max_angle_change / self._current_velocity
        duration_with_buffer = base_duration * 1.2
        final_duration = max(duration_with_buffer, 1.5)
        
        self.get_logger().debug(f'Duration calculation: max_change={max_angle_change:.3f}rad, '
                               f'velocity={self._current_velocity:.3f}rad/s, '
                               f'base_duration={base_duration:.2f}s, '
                               f'final_duration={final_duration:.2f}s')
        
        return final_duration
    
    def _queue_executor(self):
        """Background queue executor"""
        while not self._shutdown_flag:
            try:
                if self._queue_stopped:
                    time.sleep(0.1)
                    continue
                
                # Get next command
                cmd_data = self._command_queue.get(timeout=1.0)
                joint_angles, description = cmd_data
                
                # Execute command
                with self._queue_lock:
                    self._queue_executing = True
                
                self._execute_joint_movement(joint_angles, description)
                
                with self._queue_lock:
                    self._queue_executing = False
                    self._current_command_index += 1
                
                self._command_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Error in queue executor: {e}')
                with self._queue_lock:
                    self._queue_executing = False
    
    def _execute_joint_movement(self, joint_angles, description):
        """Execute joint movement with proper feedback checking"""
        self.get_logger().info(f'Executing: {description}')
        
        # Show current and target positions
        current_pos = self.get_current_joint_positions()
        if current_pos:
            self.get_logger().info(f'Current: {[f"{p:.3f}" for p in current_pos]}')
        self.get_logger().info(f'Target:  {[f"{p:.3f}" for p in joint_angles]}')
        
        try:
            duration = self._calculate_duration(joint_angles)
            self.get_logger().info(f'Calculated duration: {duration:.2f}s at {self._velocity_percentage}% speed')
            
            # Create action goal with tolerances
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = self._joint_names
            
            # Create trajectory point
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in joint_angles]
            point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            goal_msg.trajectory.points = [point]
            
            # Set goal tolerances
            goal_tolerances = []
            for _ in self._joint_names:
                tolerance = JointTolerance()
                tolerance.position = max(self._position_tolerance, 0.02)
                tolerance.velocity = self._velocity_tolerance
                goal_tolerances.append(tolerance)
            goal_msg.goal_tolerance = goal_tolerances
            goal_msg.goal_time_tolerance = Duration(sec=int(self._goal_time_tolerance))
            
            # Send goal
            future = self._action_client.send_goal_async(goal_msg)
            
            # Wait for acceptance with proper spinning
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not future.done():
                raise Exception("Goal acceptance timeout")
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                raise Exception("Goal rejected by action server")
            
            self.get_logger().info('Goal accepted, monitoring execution...')
            
            # Monitor execution with position feedback
            result_future = goal_handle.get_result_async()
            start_time = time.time()
            max_wait = duration + self._goal_time_tolerance + 5.0
            
            last_check_time = 0
            position_check_interval = 1.0
            
            while not result_future.done() and (time.time() - start_time) < max_wait:
                current_time = time.time()
                
                # Spin to process callbacks
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # Periodic position checking
                if current_time - last_check_time > position_check_interval:
                    if self._check_position_reached(joint_angles, tolerance=max(self._position_tolerance, 0.02)):
                        self.get_logger().info('Target position reached during execution')
                    last_check_time = current_time
            
            # Check final result
            if result_future.done():
                result = result_future.result()
                if result.result.error_code == 0:
                    # Final position check
                    final_check = self._check_position_reached(joint_angles, tolerance=max(self._position_tolerance, 0.02))
                    if final_check:
                        self.get_logger().info(f'Movement completed successfully: {description}')
                        # Wait 0.5s before next movement
                        time.sleep(0.5)
                        self.get_logger().info('Ready for next movement')
                    else:
                        self.get_logger().warn(f'Action reports success but final position check shows deviation: {description}')
                        self.get_logger().info('Accepting result based on action server feedback')
                        # Still wait even if position check shows deviation
                        time.sleep(0.5)
                else:
                    error_codes = {
                        -1: "INVALID_GOAL",
                        -2: "INVALID_JOINTS", 
                        -3: "OLD_HEADER_TIMESTAMP",
                        -4: "PATH_TOLERANCE_VIOLATED",
                        -5: "GOAL_TOLERANCE_VIOLATED"
                    }
                    error_name = error_codes.get(result.result.error_code, f"UNKNOWN({result.result.error_code})")
                    self.get_logger().error(f'Movement failed with error: {error_name}')
            else:
                self.get_logger().error(f'Movement timeout after {max_wait:.1f}s')
                # Check position even on timeout
                if self._check_position_reached(joint_angles, tolerance=0.05):
                    self.get_logger().info('Position appears to be reached despite timeout')
                
        except Exception as e:
            self.get_logger().error(f'Movement execution failed: {e}')
    
    def queue_joint_movement(self, joint_angles, description="Joint movement"):
        """Queue a joint movement"""
        if not self._connected:
            self.get_logger().error('Not connected to robot!')
            return -1
        
        if len(joint_angles) != 6:
            self.get_logger().error(f'Need 6 joint angles, got {len(joint_angles)}')
            return -1
        
        self._command_counter += 1
        self._command_queue.put((joint_angles, description))
        self.get_logger().info(f'Queued command {self._command_counter}: {description}')
        return self._command_counter
    
    def wait_for_queue_complete(self, timeout=60.0):
        """Wait for all queued commands to complete"""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self._command_queue.empty() and not self._queue_executing:
                return True
            time.sleep(0.1)
        return False
    
    def is_connected(self):
        return self._connected and self._current_joint_positions is not None
    
    def shutdown(self):
        """Shutdown controller"""
        self.get_logger().info('Shutting down joint controller')
        self.set_queued_cmd_stop_exec()
        self.set_queued_cmd_clear()
        self._shutdown_flag = True

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = JointController()
        
        if controller.is_connected():
            controller.get_logger().info('Joint controller ready - starting test')
            
            # Set speed and tolerance
            controller.set_velocity_scaling(velocity_percentage=30)
            controller.set_position_tolerance(tolerance=0.02)
            
            # Test movements
            home_pos = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            pos1 = [0.785, -1.57, 0.0, -1.57, 0.0, 0.0]   # 45 degrees
            pos2 = [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]    # 90 degrees
            pos3 = [-0.785, -1.57, 0.0, -1.57, 0.0, 0.0]  # -45 degrees
            
            # Queue commands
            controller.queue_joint_movement(home_pos, "Move to home")
            controller.queue_joint_movement(pos1, "Rotate base +45°")
            controller.queue_joint_movement(pos2, "Rotate base +90°")
            controller.queue_joint_movement(pos3, "Rotate base -45°")
            controller.queue_joint_movement(home_pos, "Return home")
            
            # Start execution
            controller.set_queued_cmd_start_exec()
            
            # Wait for completion
            controller.get_logger().info('Waiting for all commands to complete...')
            if controller.wait_for_queue_complete(timeout=180.0):
                controller.get_logger().info('All commands completed successfully!')
            else:
                controller.get_logger().error('Timeout waiting for commands')
            
            controller.get_logger().info('Test completed')
        
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()