#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, arm_id, hand, ee_id):
    """参考Franka官方方式生成robot_description"""
    arm_id_str = context.perform_substitution(arm_id)
    hand_str = context.perform_substitution(hand)
    ee_id_str = context.perform_substitution(ee_id)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': hand_str,
            'ros2_control': 'true',
            'gazebo': 'false',  # 不使用Ignition Gazebo
            'ee_id': ee_id_str,
            'use_fake_hardware': 'true'  # 关键：使用fake hardware
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    return [robot_state_publisher]


def generate_launch_description():
    # Arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "arm_id",
            default_value="fr3",
            description="Name of the robot arm",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand",
            default_value="true",
            description="Use Franka Hand",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ee_id",
            default_value="franka_hand",
            description="End effector ID",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false", description="Launch RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gazebo_gui", default_value="true", description="Start gazebo with GUI?"
        )
    )

    # Initialize Arguments
    arm_id = LaunchConfiguration("arm_id")
    hand = LaunchConfiguration("hand")
    ee_id = LaunchConfiguration("ee_id")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    # Robot description using Franka official method
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, hand, ee_id])

    # Controller configuration 
    controllers_file_path = os.path.join(
        get_package_share_directory('my_franka_project'),
        'config',
        'gazebo_ros2_controllers.yaml'
    )

    # ROS2 Control node with controller config
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_file_path],
        output="both",
    )

    # Gazebo launch (经典Gazebo)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "gui": gazebo_gui,
        }.items(),
    )

    # Spawn robot in Gazebo
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_franka",
        arguments=["-entity", "fr3", "-topic", "robot_description"],
        output="screen",
    )

    # Controllers (按照UR方式用ExecuteProcess)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_fr3_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fr3_arm_controller'],
        output='screen'
    )

    # RViz (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
    )

    # Event sequencing
    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo_spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    delay_arm_controller_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_fr3_arm_controller],
        )
    )

    nodes_to_start = [
        robot_state_publisher,
        ros2_control_node,
        gazebo,
        gazebo_spawn_robot,
        delay_controllers_after_spawn,
        delay_arm_controller_after_joint_state,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)