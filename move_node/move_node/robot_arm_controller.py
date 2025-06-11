#!/usr/bin/env python3
"""
RobotArmController - Thread-safe robot arm controller with command queue
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import threading
import queue
import time
from enum import Enum

class CommandStatus(Enum):
    PENDING = "pending"
    EXECUTING = "executing" 
    COMPLETED = "completed"
    FAILED = "failed"

class Command:
    def __init__(self, cmd_id, joint_angles, duration, description):
        self.cmd_id = cmd_id
        self.joint_angles = joint_angles
        self.duration = duration
        self.description = description
        self.status = CommandStatus.PENDING
        self.result = None
        self.completion_event = threading.Event()

class RobotArmController:
    """
    Thread-safe robot arm controller with command queue and resource locking
    """
    
    def __init__(self, node_name='robot_arm_controller'):
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        self._node = Node(node_name)
        self._logger = self._node.get_logger()
        
        # Command queue and synchronization
        self._command_queue = queue.Queue()
        self._resource_lock = threading.Lock()  # Resource lock for arm control
        self._is_executing = False
        self._current_command = None
        self._command_counter = 0
        self._shutdown_flag = False
        
        # Action client
        self._action_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Start background threads
        self._ros_thread = threading.Thread(target=self._ros_spin_loop, daemon=True)
        self._executor_thread = threading.Thread(target=self._command_executor_loop, daemon=True)
        
        self._ros_thread.start()
        self._executor_thread.start()
        
        # Wait for connection
        self._logger.info('Connecting to robot arm controller...')
        if self._action_client.wait_for_server(timeout_sec=10.0):
            self._logger.info('Robot arm controller connected!')
            self._connected = True
        else:
            self._logger.error('Failed to connect to robot arm controller!')
            self._connected = False
    
    def _ros_spin_loop(self):
        """Background ROS2 spin loop"""
        while rclpy.ok() and not self._shutdown_flag:
            rclpy.spin_once(self._node, timeout_sec=0.1)
    
    def _command_executor_loop(self):
        """Background command executor loop"""
        while not self._shutdown_flag:
            try:
                # Get next command from queue (blocking with timeout)
                command = self._command_queue.get(timeout=1.0)
                
                # Acquire resource lock
                with self._resource_lock:
                    self._execute_command(command)
                
                self._command_queue.task_done()
                
            except queue.Empty:
                continue  # No commands, keep waiting
            except Exception as e:
                self._logger.error(f'Error in command executor: {e}')
    
    def _execute_command(self, command):
        """Execute a single command with the resource lock held"""
        self._current_command = command
        self._is_executing = True
        command.status = CommandStatus.EXECUTING
        
        self._logger.info(f'Executing command {command.cmd_id}: {command.description}')
        
        try:
            # Create and send action goal
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            
            point = JointTrajectoryPoint()
            point.positions = [float(x) for x in command.joint_angles]
            point.time_from_start = Duration(sec=int(command.duration))
            goal_msg.trajectory.points = [point]
            
            # Send goal and wait for completion
            future = self._action_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 10.0:
                time.sleep(0.1)
            
            if not future.done():
                raise Exception("Timeout waiting for goal acceptance")
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                raise Exception("Goal rejected by action server")
            
            # Wait for execution completion
            result_future = goal_handle.get_result_async()
            while not result_future.done():
                time.sleep(0.1)
            
            result = result_future.result()
            if result.result.error_code == 0:
                command.status = CommandStatus.COMPLETED
                command.result = "SUCCESS"
                self._logger.info(f'Command {command.cmd_id} completed successfully')
            else:
                command.status = CommandStatus.FAILED
                command.result = f"ERROR_{result.result.error_code}"
                self._logger.error(f'Command {command.cmd_id} failed with error {result.result.error_code}')
                
        except Exception as e:
            command.status = CommandStatus.FAILED
            command.result = str(e)
            self._logger.error(f'Command {command.cmd_id} failed: {e}')
        
        finally:
            self._is_executing = False
            self._current_command = None
            command.completion_event.set()  # Signal completion
    
    def move_joints(self, joint_angles, duration=3.0, description="Joint movement", wait=True):
        """
        Queue a joint movement command
        
        Args:
            joint_angles: list[float] - 6 joint angles in radians
            duration: float - movement duration in seconds
            description: str - command description
            wait: bool - whether to wait for completion
            
        Returns:
            Command object for tracking status
        """
        if not self._connected:
            self._logger.error('Robot arm not connected!')
            return None
        
        if len(joint_angles) != 6:
            self._logger.error(f'Need 6 joint angles, got {len(joint_angles)}')
            return None
        
        # Create command
        self._command_counter += 1
        command = Command(self._command_counter, joint_angles, duration, description)
        
        # Add to queue
        self._command_queue.put(command)
        self._logger.info(f'Queued command {command.cmd_id}: {description} (Queue size: {self._command_queue.qsize()})')
        
        if wait:
            # Wait for completion
            command.completion_event.wait()
            return command.status == CommandStatus.COMPLETED
        else:
            return command
    
    def get_queue_status(self):
        """Get current queue and execution status"""
        return {
            'queue_size': self._command_queue.qsize(),
            'is_executing': self._is_executing,
            'current_command': self._current_command.description if self._current_command else None
        }
    
    def clear_queue(self):
        """Clear all pending commands (emergency stop)"""
        cleared_count = 0
        try:
            while True:
                self._command_queue.get_nowait()
                cleared_count += 1
        except queue.Empty:
            pass
        
        self._logger.warning(f'Cleared {cleared_count} pending commands from queue')
        return cleared_count
    
    def wait_for_idle(self, timeout=30.0):
        """Wait until all commands are completed and arm is idle"""
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            if self._command_queue.empty() and not self._is_executing:
                return True
            time.sleep(0.1)
        return False
    
    def is_connected(self):
        return self._connected
    
    # Convenience methods
    def move_to_home(self):
        """Move to home position"""
        home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        return self.move_joints(home_position, duration=3.0, description="Move to home")
    
    def rotate_base(self, angle):
        """Rotate base to specified angle"""
        position = [angle, -1.57, 0.0, -1.57, 0.0, 0.0]
        return self.move_joints(position, duration=2.0, description=f"Rotate base to {angle:.2f} rad")
    
    def shutdown(self):
        """Shutdown controller gracefully"""
        self._logger.info('Shutting down robot arm controller...')
        self._shutdown_flag = True
        
        # Clear queue
        self.clear_queue()
        
        # Wait for current command to finish (max 10 seconds)
        if self._is_executing:
            self._logger.info('Waiting for current command to finish...')
            time.sleep(min(10.0, 5.0))
        
        self._node.destroy_node()

def demo_usage():
    """Demonstrate the thread-safe API with queue"""
    print("Creating thread-safe robot arm controller...")
    arm = RobotArmController()
    
    if not arm.is_connected():
        print("Unable to connect to robot arm!")
        return
    
    print("Testing queued movements...")
    
    # Queue multiple commands without waiting
    print("Queueing multiple commands...")
    arm.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], duration=2.0, description="Home", wait=False)
    arm.move_joints([0.5, -1.57, 0.0, -1.57, 0.0, 0.0], duration=2.0, description="Base 30°", wait=False)
    arm.move_joints([1.0, -1.57, 0.0, -1.57, 0.0, 0.0], duration=2.0, description="Base 60°", wait=False)
    arm.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0], duration=2.0, description="Return home", wait=False)
    
    print(f"Queue status: {arm.get_queue_status()}")
    
    # Wait for all commands to complete
    print("Waiting for all commands to complete...")
    if arm.wait_for_idle(timeout=30.0):
        print("All commands completed!")
    else:
        print("Timeout waiting for commands to complete")
    
    print("Demo completed!")
    arm.shutdown()

if __name__ == '__main__':
    demo_usage()