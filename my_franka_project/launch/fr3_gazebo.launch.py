#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    pkg_share = launch_ros.substitutions.FindPackageShare(package='my_franka_project').find('my_franka_project')
    xacro_file = os.path.join(pkg_share, "description", "fr3_gazebo.urdf.xacro")
    urdf_file = '/tmp/fr3_gazebo.urdf'
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'fr3.rviz')
    
    # Generate URDF file from xacro (ROS2_PickandPlace style)
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    
    # Gazebo server and client
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"verbose": "true"}.items(),
    )
    
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
        launch_arguments={"verbose": "true"}.items(),
    )
    
    # Robot state publisher (ROS2_PickandPlace style)
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': use_sim_time, 
             'robot_description': open(urdf_file).read()}
        ]
    )
    
    # Joint state publisher (backup, usually not needed with controllers)
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # RViz
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition=launch.conditions.IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Spawn robot in Gazebo (ROS2_PickandPlace style)
    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "fr3", "-file", urdf_file],
        output="screen",
    )
    
    # Controller configuration
    controller_config = os.path.join(
        get_package_share_directory("my_franka_project"), "config", "controllers.yaml"
    )
    
    # ROS2 Control node (needs robot_description)
    ros2_control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': open(urdf_file).read()}, controller_config],
        output="both",
    )
    
    # Spawn joint state broadcaster
    spawn_joint_state_broadcaster = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    # Spawn FR3 arm controller 
    spawn_fr3_controller = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller"],
        output="screen",
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            name='gui', 
            default_value='false',
            description='Flag to enable joint_state_publisher_gui'
        ),
        DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        ),
        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Launch RViz'
        ),
        
        # Launch sequence
        robot_state_publisher_node,
        ros2_control_node,
        gazebo_server,
        gazebo_client, 
        spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_fr3_controller,
        rviz_node,
        joint_state_publisher_node,
    ])