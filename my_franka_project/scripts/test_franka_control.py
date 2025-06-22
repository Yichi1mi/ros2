#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math

class FrankaTestController(Node):
    def __init__(self):
        super().__init__('franka_test_controller')
        
        # Action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available!')
        
    def send_goal(self, positions, duration=3.0):
        """Send a trajectory goal to the robot"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f'Sending goal: {positions}')
        
        # Send goal
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f'Result: {result.error_code}')
        
    def demo_movements(self):
        """Perform some demo movements"""
        self.get_logger().info('Starting demo movements...')
        
        # Home position
        home_pos = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self.send_goal(home_pos, 3.0)
        
        # Wait a bit
        import time
        time.sleep(1.0)
        
        # Another position
        pos2 = [0.5, -0.5, 0.3, -2.0, 0.2, 1.2, 0.5]
        self.send_goal(pos2, 3.0)
        
        time.sleep(1.0)
        
        # Back to home
        self.send_goal(home_pos, 3.0)
        
        self.get_logger().info('Demo completed!')

def main(args=None):
    rclpy.init(args=args)
    
    controller = FrankaTestController()
    
    try:
        controller.demo_movements()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()