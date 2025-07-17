#!/usr/bin/env python3
import os
import sys
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# 添加robot_common到路径以导入相机配置
sys.path.append(os.path.join(get_package_share_directory('robot_common'), '..', '..', 'src', 'my_robot_controller', 'robot_common'))

def generate_launch_description():
    package_dir = get_package_share_directory('ur_vision_system')
    
    # 导入统一的相机配置来计算物体生成范围
    try:
        from robot_common.camera_config import get_camera_config
        camera_config = get_camera_config()
        
        # 基于相机配置计算合适的物体生成范围
        # 使用相机能看到的区域内的合理范围（避免边缘）
        ground_coverage = camera_config.get_ground_coverage()
        # max_range_x = ground_coverage * 0.3  # 使用30%的覆盖范围，确保在视野中心
        # max_range_y = ground_coverage * 0.2  # Y方向稍微小一点
        max_range_x = 0.4
        max_range_y = 0.2
        
        # 相机位置作为中心点
        center_x = camera_config.position_x
        center_y = camera_config.position_y
        
        print(f"Using camera config: center=({center_x}, {center_y}), ground_coverage={ground_coverage:.1f}m, spawn_range=±({max_range_x:.1f}, {max_range_y:.1f})m")
        
    except ImportError:
        print("Warning: Could not import camera_config, using fallback spawn range")
        center_x, center_y = 0.0, 0.0
        max_range_x = 0.4
        max_range_y = 0.2
    
    # 定义物体模型路径
    models = {
        'red_cylinder': os.path.join(package_dir, 'models', 'test_cylinder', 'model.sdf'),
        'green_box': os.path.join(package_dir, 'models', 'test_box', 'model.sdf'),
        'blue_prism': os.path.join(package_dir, 'models', 'test_prism', 'model.sdf')
    }
    
    # 基于相机配置随机生成位置
    positions = []
    for i in range(3):
        x = center_x + round(random.uniform(-max_range_x, max_range_x), 2)
        y = center_y + 0.15 + round(random.uniform(-max_range_y, max_range_y), 2)
        positions.append((x, y))
    
    print(f"Spawning objects at positions: {positions}")
    
    # 创建spawn节点
    spawn_nodes = []
    
    # 红色圆柱体
    spawn_nodes.append(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', models['red_cylinder'],
            '-entity', 'red_cylinder',
            '-x', str(positions[0][0]),
            '-y', str(positions[0][1]),
            '-z', '0.0'
        ],
        output='screen'
    ))
    
    # 绿色长方体
    spawn_nodes.append(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', models['green_box'],
            '-entity', 'green_box',
            '-x', str(positions[1][0]),
            '-y', str(positions[1][1]),
            '-z', '0.0'
        ],
        output='screen'
    ))
    
    # 蓝色三棱柱
    spawn_nodes.append(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', models['blue_prism'],
            '-entity', 'blue_prism',
            '-x', str(positions[2][0]),
            '-y', str(positions[2][1]),
            '-z', '0.0'
        ],
        output='screen'
    ))

    return LaunchDescription(spawn_nodes)