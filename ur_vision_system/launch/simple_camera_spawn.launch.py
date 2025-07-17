#!/usr/bin/env python3
import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# 添加robot_common到路径以导入相机配置
sys.path.append(os.path.join(get_package_share_directory('robot_common'), '..', '..', 'src', 'my_robot_controller', 'robot_common'))

def generate_launch_description():
    # 导入统一的相机配置
    try:
        from robot_common.camera_config import get_camera_config
        camera_config = get_camera_config()
        spawn_args = camera_config.get_spawn_args()
        print(f"Using unified camera config: position=({camera_config.position_x}, {camera_config.position_y}, {camera_config.position_z})")
    except ImportError:
        print("Warning: Could not import camera_config, using fallback values")
        spawn_args = ['-x', '0.0', '-y', '0.0', '-z', '1.5', '-R', '0.0', '-P', '1.5708', '-Y', '0.0']
    
    # 获取SDF模型文件路径
    package_dir = get_package_share_directory('ur_vision_system')
    model_path = os.path.join(package_dir, 'models', 'world_camera', 'model.sdf')
    
    # spawn实体 - 使用统一配置
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', model_path,
            '-entity', 'world_camera'
        ] + spawn_args,
        output='screen'
    )

    return LaunchDescription([
        spawn_entity
    ])