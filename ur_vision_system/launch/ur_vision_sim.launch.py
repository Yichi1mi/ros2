#!/usr/bin/env python3
"""
ur_vision_sim.launch.py - 启动官方UR仿真环境并添加摄像头
集成了官方ur_simulation_gazebo包和环境摄像头
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='Type of UR robot'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )
    
    # Include the official UR simulation launch (不修改任何官方参数)
    ur_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ur_simulation_gazebo'),
            '/launch/ur_sim_control.launch.py'
        ]),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
        }.items()
    )

    # Get the camera URDF path and process xacro
    camera_urdf_path = PathJoinSubstitution([
        FindPackageShare('ur_vision_system'),
        'urdf',
        'world_camera.urdf.xacro'
    ])
    
    # Process xacro to generate URDF
    from launch.substitutions import Command
    camera_urdf_content = Command(['xacro ', camera_urdf_path])

    # Spawn the world camera in Gazebo (delayed to ensure Gazebo is ready)
    spawn_camera = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to be ready
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_world_camera',
                arguments=[
                    '-entity', 'world_camera',
                    '-topic', 'robot_description_camera',
                    '-x', '1.0',    # 机器人前方1米
                    '-y', '0.5',    # 稍微偏右
                    '-z', '1.5',    # 高1.5米
                    '-R', '0.0',    # 无滚转
                    '-P', '0.3',    # 向下倾斜
                    '-Y', '3.14159' # 面向机器人（180度）
                ],
                output='screen'
            )
        ]
    )
    
    # Robot state publisher for camera
    camera_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='camera_state_publisher',
        parameters=[{'robot_description': camera_urdf_content}],
        remappings=[('/robot_description', '/robot_description_camera')]
    )

    return LaunchDescription([
        ur_type_arg,
        launch_rviz_arg,
        ur_sim_launch,
        camera_state_publisher,
        spawn_camera
    ])