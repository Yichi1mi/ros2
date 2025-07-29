#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    try:
        from robot_common.camera_config import get_camera_config
        camera_config = get_camera_config()
        spawn_args = [
            '-file', os.path.join(get_package_share_directory('ur_vision_system'), 'models', 'world_camera', 'model.sdf'),
            '-entity', 'world_camera'
        ] + camera_config.get_spawn_args()
    except ImportError:
        package_dir = get_package_share_directory('ur_vision_system')
        model_path = os.path.join(package_dir, 'models', 'world_camera', 'model.sdf')
        spawn_args = [
            '-file', model_path,
            '-entity', 'world_camera',
            '-x', '0.5',
            '-y', '0.0', 
            '-z', '1.5',
            '-P', '1.5708'
        ]
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=spawn_args,
        output='screen'
    )

    return LaunchDescription([
        spawn_entity
    ])