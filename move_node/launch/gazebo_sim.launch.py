#!/usr/bin/env python3
"""
gazebo_sim.launch.py - Franka FR3 Gazebo simulation launch file
Based on Universal Robots and Panda ROS2 Gazebo patterns
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def get_robot_description(context, arm_id, load_gripper):
    """Generate robot description for Gazebo simulation"""
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)

    # Find the FR3 URDF file
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 
        arm_id_str,
        arm_id_str + '.urdf.xacro'
    )

    # Process URDF with ros2_control and gazebo enabled
    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'arm_id': arm_id_str,
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'franka_hand'
        }
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    return [robot_state_publisher]


def generate_launch_description():
    """Generate launch description for FR3 Gazebo simulation"""
    
    # Launch arguments
    arm_id_arg = DeclareLaunchArgument(
        'arm_id',
        default_value='fr3',
        description='Name of the robot arm'
    )
    
    load_gripper_arg = DeclareLaunchArgument(
        'load_gripper',
        default_value='true',
        description='Load gripper in simulation'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Launch configuration
    arm_id = LaunchConfiguration('arm_id')
    load_gripper = LaunchConfiguration('load_gripper')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Robot description
    robot_description_nodes = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper]
    )

    # Gazebo simulation
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description')
    )
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Controller manager
    controller_config_file = os.path.join(
        get_package_share_directory('move_node'),
        'config',
        'fr3_gazebo_controllers.yaml'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config_file, {'use_sim_time': use_sim_time}],
        output='both',
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ]
    )

    # Load joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Load joint trajectory controller  
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('franka_description'),
        'rviz',
        'visualize_franka.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        arm_id_arg,
        load_gripper_arg,
        use_sim_time_arg,
        
        # Launch Gazebo
        gazebo_world_launch,
        
        # Robot description and state publisher
        robot_description_nodes,
        
        # Spawn robot
        spawn_robot,
        
        # Controller manager
        controller_manager,
        
        # RViz
        rviz,
        
        # Controller loading sequence
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
    ])