#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class ActionTrajectoryController(Node):
    def __init__(self):
        super().__init__('action_trajectory_controller')
        self.get_logger().info('Starting ACTION trajectory controller')
        
        # Create ACTION client instead of publisher
        self._action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        if self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info('Action server found!')
            self.test_movement()
        else:
            self.get_logger().error('Action server not available!')
    
    def send_joint_trajectory_action(self, positions, description, duration_sec=3.0):
        """Send joint trajectory using ACTION interface"""
        self.get_logger().info(f'Sending ACTION: {description}')
        self.get_logger().info(f'Positions: {positions}')
        
        # Create goal message
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set joint names
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [float(x) for x in positions]
        point.time_from_start = Duration(sec=int(duration_sec))
        
        goal_msg.trajectory.points = [point]
        
        # Send action goal
        self.get_logger().info('Sending action goal...')
        future = self._action_client.send_goal_async(goal_msg)
        
        return future
    
    def test_movement(self):
        """Test small safe movement using ACTION interface"""
        self.get_logger().info('Testing small safe movement with ACTION...')
        
        # First, stay at current position
        current = [0, -1.57, 0, -1.57, 0, 0]
        future1 = self.send_joint_trajectory_action(current, "Stay at current", 1.0)
        
        time.sleep(2.0)
        
        # Then, small base rotation (0.1 radians = 5.7 degrees)
        small_move = [1, -1.57, 0, -1.57, 0, 0]
        future2 = self.send_joint_trajectory_action(small_move, "Small base rotation", 2.0)
        
        time.sleep(3.0)
        
        # Back to original
        future3 = self.send_joint_trajectory_action(current, "Back to original", 2.0)
        
        self.get_logger().info('Safe movement test completed!')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = ActionTrajectoryController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()