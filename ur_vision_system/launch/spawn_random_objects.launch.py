#!/usr/bin/env python3
import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('ur_vision_system')

    try:
        from robot_common.camera_config import get_camera_config
        camera_config = get_camera_config()
        camera_x = camera_config.position_x
        camera_y = camera_config.position_y
    except ImportError:
        camera_x = 0.5
        camera_y = 0.0
    
    models = {
        'red_cylinder': os.path.join(package_dir, 'models', 'test_cylinder', 'model.sdf'),
        'green_box': os.path.join(package_dir, 'models', 'test_box', 'model.sdf'),
        'blue_prism': os.path.join(package_dir, 'models', 'test_prism', 'model.sdf')
    }
    
    # 以相机位置为中心，在机器人工作范围内随机生成位置
    positions = []
    for i in range(3):
        x = round(camera_x + random.uniform(-0.3, 0.2), 2)
        y = round(camera_y - 0.1 + random.uniform(-0.15, 0.15), 2)
        positions.append((x, y))
    
    print(f"Spawning objects at positions: {positions}")
    
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