#!/usr/bin/env python3

"""
Simple test script to verify MoveIt + Gazebo integration
This script will test basic planning functionality without using the interactive markers
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory
from sensor_msgs.msg import JointState
import time

class MoveItGazeboTest(Node):
    def __init__(self):
        super().__init__('moveit_gazebo_test')
        
        # Subscribe to joint states to verify communication
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to planned trajectories
        self.trajectory_sub = self.create_subscription(
            DisplayTrajectory,
            '/move_group/display_planned_path',
            self.trajectory_callback,
            10
        )
        
        self.joint_states_received = False
        self.latest_joint_states = None
        
        self.get_logger().info("MoveIt Gazebo Test Node Started")
        self.get_logger().info("Checking for joint states and trajectory planning...")

    def joint_state_callback(self, msg):
        if not self.joint_states_received:
            self.get_logger().info(f"✅ Joint states received! Joints: {msg.name}")
            self.joint_states_received = True
        self.latest_joint_states = msg

    def trajectory_callback(self, msg):
        self.get_logger().info("✅ Trajectory received from MoveIt!")
        if msg.trajectory:
            self.get_logger().info(f"Trajectory has {len(msg.trajectory)} segments")

def main():
    rclpy.init()
    
    test_node = MoveItGazeboTest()
    
    print("\n" + "="*60)
    print("🤖 FRANKA FR3 MOVEIT + GAZEBO INTEGRATION TEST")
    print("="*60)
    print("This test will:")
    print("1. Check if joint states are being published")
    print("2. Monitor for MoveIt trajectory messages")
    print("3. Verify the integration is working")
    print("\nRunning for 30 seconds...")
    print("="*60 + "\n")
    
    start_time = time.time()
    
    try:
        while rclpy.ok() and (time.time() - start_time) < 30:
            rclpy.spin_once(test_node, timeout_sec=1.0)
            
            # Print status every 5 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0 and int(elapsed) > 0:
                status = "✅ WORKING" if test_node.joint_states_received else "❌ NO DATA"
                test_node.get_logger().info(f"Status after {int(elapsed)}s: {status}")
    
    except KeyboardInterrupt:
        pass
    
    print("\n" + "="*60)
    print("🏁 TEST RESULTS:")
    print("="*60)
    
    if test_node.joint_states_received:
        print("✅ Joint states communication: WORKING")
        if test_node.latest_joint_states:
            print(f"✅ Active joints: {len(test_node.latest_joint_states.name)}")
    else:
        print("❌ Joint states communication: FAILED")
    
    print("\nTo test planning:")
    print("1. In RViz, avoid checking 'MotionPlanning' display initially")
    print("2. Use the Planning tab to set goal states")
    print("3. Try 'Plan' and 'Execute' buttons")
    print("4. Monitor this terminal for trajectory messages")
    
    print("\n" + "="*60)
    
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()