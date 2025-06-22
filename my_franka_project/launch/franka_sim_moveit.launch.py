#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui",
            default_value="true",
            description="Start gazebo with GUI?",
        )
    )

    # Initialize Arguments
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    # Include Gazebo base layer (robot spawning + basic control)
    gazebo_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("my_franka_project"),
                "launch",
                "franka_sim_gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",  # Don't launch RViz from base
        }.items(),
    )

    # Include Franka MoveIt launch with simulation parameters
    franka_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("franka_fr3_moveit_config"),
                "launch", 
                "moveit.launch.py"
            ])
        ]),
        launch_arguments={
            "use_fake_hardware": "true",  # Use fake hardware for simulation
            "robot_ip": "dont-care",      # Not used in simulation
        }.items(),
    )

    nodes_to_start = [
        gazebo_base,
        franka_moveit,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)