#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取模型文件路径
    package_dir = get_package_share_directory('ur_vision_system')
    model_path = os.path.join(package_dir, 'models', 'test_cylinder', 'model.sdf')
    
    # spawn圆柱体 - 放在相机能看到的位置
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', model_path,
            '-entity', 'test_cylinder',
            '-x', '0.0',      # 世界原点
            '-y', '0.0',      # 世界原点
            '-z', '0.0',      # 地面上
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_entity
    ])