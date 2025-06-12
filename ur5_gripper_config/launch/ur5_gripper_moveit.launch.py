#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    prefix = LaunchConfiguration("prefix")

    # Include UR5 with gripper simulation
    ur5_gripper_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur5_gripper_config"), "/launch", "/ur5_gripper_simulation.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "prefix": prefix,
            "launch_rviz": "false",  # We'll launch RViz with MoveIt
        }.items(),
    )

    # Include MoveIt configuration for UR5
    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch", "/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": safety_limits,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "true",
            "use_fake_hardware": "true",
        }.items(),
    )

    nodes_to_launch = [
        ur5_gripper_control_launch,
        ur_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=[
                "ur3",
                "ur3e", 
                "ur5",
                "ur5e",
                "ur7e",
                "ur10",
                "ur12e",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
            default_value="ur5e",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])