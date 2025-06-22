#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import math

class FrankaActionController(Node):
    def __init__(self):
        super().__init__('franka_action_controller')
        
        # Action server for joint trajectory control
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        # Publisher for joint trajectory commands
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/fr3_arm_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info('Franka Action Controller started')
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Get the trajectory from the goal
        trajectory = goal_handle.request.trajectory
        
        # Publish the trajectory
        self.trajectory_pub.publish(trajectory)
        
        # Simulate execution (in real implementation, monitor joint states)
        import time
        time.sleep(2.0)  # Simulate trajectory execution time
        
        goal_handle.succeed()
        
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result
    
    def create_simple_trajectory(self, positions, duration=3.0):
        """Helper function to create a simple trajectory"""
        trajectory = JointTrajectory()
        trajectory.header = Header()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points.append(point)
        return trajectory

def main(args=None):
    rclpy.init(args=args)
    
    controller = FrankaActionController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()