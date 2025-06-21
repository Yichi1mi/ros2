#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Command-line arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='pick_place.world',
        description='Gazebo world file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="gazebo_ros2_control",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, gazebo_ros2_control]",
    )
    
    # Paths
    gazebo_worlds_path = get_package_share_directory('gazebo_worlds')
    world_file_path = PathJoinSubstitution([
        FindPackageShare('gazebo_worlds'),
        'worlds',
        LaunchConfiguration('world')
    ])
    
    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            world_file_path
        ],
        output='screen'
    )
    
    # Gazebo client
    gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindPackageShare("xacro"), "xacro"]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("moveit_resources_panda_moveit_config"),
            "config",
            "panda_gazebo.urdf.xacro"
        ]),
        " ros2_control_hardware_type:=",
        LaunchConfiguration("ros2_control_hardware_type"),
    ])
    
    robot_description = {"robot_description": robot_description_content}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ],
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'panda',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0'
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output='screen'
    )
    
    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        ros2_control_hardware_type,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_entity,
    ])