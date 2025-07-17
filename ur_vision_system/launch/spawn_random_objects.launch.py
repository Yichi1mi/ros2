#!/usr/bin/env python3
import os
import random
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('ur_vision_system')
    
    # 定义物体模型路径
    models = {
        'red_cylinder': os.path.join(package_dir, 'models', 'test_cylinder', 'model.sdf'),
        'green_box': os.path.join(package_dir, 'models', 'test_box', 'model.sdf'),
        'blue_prism': os.path.join(package_dir, 'models', 'test_prism', 'model.sdf')
    }
    
    # 随机生成位置 (-1到1米范围)
    positions = []
    for i in range(3):
        x = round(random.uniform(-0.4, 0.4), 2)
        y = round(random.uniform(-0.4, 0.4), 2)
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