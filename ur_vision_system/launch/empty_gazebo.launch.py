#!/usr/bin/env python3
"""
empty_gazebo.launch.py - 启动空的Gazebo环境（配置与官方UR仿真一致）
用于独立测试相机功能，不包含机械臂
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    gazebo_gui_arg = DeclareLaunchArgument(
        'gazebo_gui',
        default_value='true',
        description='Start gazebo with GUI?'
    )
    
    # Gazebo launch - 使用与官方UR仿真相同的配置
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "gui": LaunchConfiguration('gazebo_gui'),
        }.items(),
    )

    return LaunchDescription([
        gazebo_gui_arg,
        gazebo
    ])