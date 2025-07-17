#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取SDF模型文件路径
    package_dir = get_package_share_directory('ur_vision_system')
    model_path = os.path.join(package_dir, 'models', 'world_camera', 'model.sdf')
    
    # spawn实体 - 使用SDF模型（static）
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', model_path,
            '-entity', 'world_camera',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '1.5',
            '-P', '1.57'
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_entity
    ])