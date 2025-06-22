#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        DeclareLaunchArgument(
            "controllers_file",
            default_value="gazebo_ros2_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="franka_description",
            description="Description package with robot URDF/XACRO files.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="fr3.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
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
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")

    # Controller configuration file path
    controllers_file_path = PathJoinSubstitution(
        [FindPackageShare("my_franka_project"), "config", controllers_file]
    )

    # Robot description - 按照UR的方式，使用官方franka URDF + gazebo参数
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "robots", "fr3", description_file]
            ),
            " ",
            "arm_id:=",
            arm_id,
            " ",
            "hand:=",
            hand,
            " ",
            "ee_id:=",
            ee_id,
            " ",
            "description_pkg:=",
            description_package,
            " ",
            "gazebo:=true",  # 关键：启用gazebo模式
            " ",
            "ros2_control:=true",  # 关键：启用ros2_control
            " ",
            "use_fake_hardware:=false",  # 关键：不使用fake hardware
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # Joint state broadcaster spawner - 按照UR的方式使用spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Arm controller spawner - use fr3_arm_controller to match MoveIt expectations
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", "-c", "/controller_manager"],
    )

    # Gripper controller spawner  
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_gripper_controller", "-c", "/controller_manager"],
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]
        ),
        launch_arguments={
            "gui": gazebo_gui,
        }.items(),
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_franka",
        arguments=["-entity", "fr3", "-topic", "robot_description"],
        output="screen",
    )

    # RViz node (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(launch_rviz),
    )

    # Event handlers for proper sequencing
    delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner],
        )
    )

    delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_rviz_after_joint_state_broadcaster,
        gazebo,
        gazebo_spawn_robot,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)