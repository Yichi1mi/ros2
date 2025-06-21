#!/usr/bin/env python3
"""
scene_manager.py - MoveIt Planning Scene management for adding objects and environments
Handles table, objects, and obstacles in the robot workspace
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from std_msgs.msg import Header

class SceneManager(Node):
    """
    Manager for MoveIt Planning Scene objects.
    Adds tables, objects, and obstacles to the robot's environment.
    """
    
    def __init__(self):
        # Initialize ROS2 node
        super().__init__('scene_manager')
        
        # Planning scene publisher
        self._scene_publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )
        
        # Current scene objects
        self._scene_objects = {}
        
        # Wait for publisher to be ready
        import time
        time.sleep(1.0)
        
        self.get_logger().info('Scene manager initialized')
    
    def clear_scene(self):
        """
        Clear all objects from the planning scene.
        """
        self.get_logger().info('Clearing planning scene')
        
        # Create planning scene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        
        # Remove all known objects
        for object_id in self._scene_objects.keys():
            collision_object = CollisionObject()
            collision_object.header.frame_id = "panda_link0"
            collision_object.id = object_id
            collision_object.operation = CollisionObject.REMOVE
            scene_msg.world.collision_objects.append(collision_object)
        
        # Publish scene update
        self._scene_publisher.publish(scene_msg)
        self._scene_objects.clear()
        
        self.get_logger().info('Planning scene cleared')
    
    def add_table(self, x=0.5, y=0.0, z=-0.01, 
                  length=1.2, width=0.8, height=0.02, 
                  object_id="table"):
        """
        Add a table to the planning scene.
        
        Args:
            x, y, z: Position of table center (meters)
            length: Table length in X direction (meters)
            width: Table width in Y direction (meters) 
            height: Table thickness (meters)
            object_id: Unique identifier for the table
        """
        self.get_logger().info(f'Adding table: {length:.1f}m x {width:.1f}m at ({x:.1f}, {y:.1f}, {z:.1f})')
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_id
        
        # Define box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [length, width, height]
        collision_object.primitives.append(box)
        
        # Set pose
        pose = PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        collision_object.primitive_poses.append(pose.pose)
        
        collision_object.operation = CollisionObject.ADD
        
        # Create planning scene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(collision_object)
        
        # Publish scene update
        self._scene_publisher.publish(scene_msg)
        self._scene_objects[object_id] = collision_object
        
        self.get_logger().info(f'Table "{object_id}" added to scene')
    
    def add_cylinder(self, x=0.3, y=0.2, z=0.05, 
                     radius=0.03, height=0.1, 
                     object_id="cylinder"):
        """
        Add a cylinder (e.g., soda can) to the planning scene.
        
        Args:
            x, y, z: Position of cylinder center (meters)
            radius: Cylinder radius (meters)
            height: Cylinder height (meters)
            object_id: Unique identifier for the cylinder
        """
        self.get_logger().info(f'Adding cylinder: r={radius*1000:.0f}mm, h={height*1000:.0f}mm at ({x:.1f}, {y:.1f}, {z:.1f})')
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_id
        
        # Define cylinder primitive
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = [height, radius]  # [height, radius] for cylinder
        collision_object.primitives.append(cylinder)
        
        # Set pose
        pose = PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        collision_object.primitive_poses.append(pose.pose)
        
        collision_object.operation = CollisionObject.ADD
        
        # Create planning scene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(collision_object)
        
        # Publish scene update
        self._scene_publisher.publish(scene_msg)
        self._scene_objects[object_id] = collision_object
        
        self.get_logger().info(f'Cylinder "{object_id}" added to scene')
    
    def add_cube(self, x=0.4, y=-0.2, z=0.025, 
                 size=0.05, 
                 object_id="cube"):
        """
        Add a cube (e.g., small box) to the planning scene.
        
        Args:
            x, y, z: Position of cube center (meters)
            size: Cube side length (meters)
            object_id: Unique identifier for the cube
        """
        self.get_logger().info(f'Adding cube: {size*1000:.0f}mm sides at ({x:.1f}, {y:.1f}, {z:.1f})')
        
        # Create collision object
        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_id
        
        # Define box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size, size, size]
        collision_object.primitives.append(box)
        
        # Set pose
        pose = PoseStamped()
        pose.header.frame_id = "panda_link0"
        pose.pose.position = Point(x=x, y=y, z=z)
        pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        collision_object.primitive_poses.append(pose.pose)
        
        collision_object.operation = CollisionObject.ADD
        
        # Create planning scene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(collision_object)
        
        # Publish scene update
        self._scene_publisher.publish(scene_msg)
        self._scene_objects[object_id] = collision_object
        
        self.get_logger().info(f'Cube "{object_id}" added to scene')
    
    def remove_object(self, object_id):
        """
        Remove a specific object from the planning scene.
        
        Args:
            object_id: Identifier of object to remove
        """
        if object_id not in self._scene_objects:
            self.get_logger().warning(f'Object "{object_id}" not found in scene')
            return
        
        self.get_logger().info(f'Removing object "{object_id}" from scene')
        
        # Create collision object for removal
        collision_object = CollisionObject()
        collision_object.header.frame_id = "panda_link0"
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE
        
        # Create planning scene message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.world.collision_objects.append(collision_object)
        
        # Publish scene update
        self._scene_publisher.publish(scene_msg)
        del self._scene_objects[object_id]
        
        self.get_logger().info(f'Object "{object_id}" removed from scene')
    
    def setup_default_scene(self):
        """
        Set up a default scene with elevated table and objects for pick-and-place testing.
        """
        self.get_logger().info('Setting up elevated scene for pick-and-place testing')
        
        # Clear existing scene
        self.clear_scene()
        
        # Add elevated smaller table (20cm higher, smaller size)
        self.add_table(x=0.4, y=0.0, z=0.19, 
                      length=0.6, width=0.4, height=0.02, 
                      object_id="elevated_table")
        
        # Add cylinder object (soda can) on table surface
        self.add_cylinder(x=0.3, y=0.1, z=0.25, 
                         radius=0.03, height=0.1, 
                         object_id="soda_can")
        
        # Add cube object (small box) on table surface
        self.add_cube(x=0.35, y=-0.15, z=0.225, 
                     size=0.05, 
                     object_id="small_box")
        
        self.get_logger().info('Elevated scene setup complete - table at 20cm height')
    
    def get_object_list(self):
        """
        Get list of objects currently in the scene.
        
        Returns:
            list: Object IDs in the scene
        """
        return list(self._scene_objects.keys())


def main(args=None):
    """
    Unit test for SceneManager
    """
    rclpy.init(args=args)
    try:
        scene_manager = SceneManager()
        
        print("🏗️ Scene Manager Test")
        print("\nSetting up default scene...")
        scene_manager.setup_default_scene()
        
        print(f"\nCurrent objects in scene: {scene_manager.get_object_list()}")
        
        import time
        time.sleep(3)
        
        print("\nTesting object removal...")
        scene_manager.remove_object("soda_can")
        print(f"Objects after removal: {scene_manager.get_object_list()}")
        
        time.sleep(2)
        
        print("\nClearing scene...")
        scene_manager.clear_scene()
        print(f"Objects after clear: {scene_manager.get_object_list()}")
        
        print("\n✅ Scene manager test completed!")
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()