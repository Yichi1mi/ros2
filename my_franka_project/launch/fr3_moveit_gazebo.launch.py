#  Copyright (c) 2024 Custom Project
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# Launch file combining official Franka Gazebo with MoveIt
# Based on official franka_gazebo_bringup architecture

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch import LaunchContext
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def get_robot_description(context: LaunchContext, arm_id, load_gripper, franka_hand):
    """Generate robot description exactly like official Gazebo launch"""
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

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
            'hand': load_gripper_str,
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': franka_hand_str
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    return [robot_state_publisher]


def generate_launch_description():
    # Launch arguments - same as official
    load_gripper_name = 'load_gripper'
    franka_hand_name = 'franka_hand'
    arm_id_name = 'arm_id'
    namespace_name = 'namespace'

    load_gripper = LaunchConfiguration(load_gripper_name)
    franka_hand = LaunchConfiguration(franka_hand_name)
    arm_id = LaunchConfiguration(arm_id_name)
    namespace = LaunchConfiguration(namespace_name)

    load_gripper_launch_argument = DeclareLaunchArgument(
        load_gripper_name,
        default_value='true',
        description='true/false for activating the gripper')

    franka_hand_launch_argument = DeclareLaunchArgument(
        franka_hand_name,
        default_value='franka_hand',
        description='Default value: franka_hand')

    arm_id_launch_argument = DeclareLaunchArgument(
        arm_id_name,
        default_value='fr3',
        description='Available values: fr3, fp3 and fer')

    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot.')

    # === GAZEBO SETUP (Official Architecture) ===
    
    # Set resource path exactly like official
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(get_package_share_directory('franka_description'))
    
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # Robot state publisher (official way)
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[arm_id, load_gripper, franka_hand])

    # Spawn robot (official way)
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Load joint state broadcaster (official way)
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load controller parameters
    controller_config_file = os.path.join(
        get_package_share_directory('my_franka_project'),
        'config',
        'fr3_gazebo_moveit_controllers.yaml'
    )
    
    load_controller_params = ExecuteProcess(
        cmd=['ros2', 'param', 'load', '/controller_manager', controller_config_file],
        output='screen'
    )

    # Load fr3_arm_controller for MoveIt trajectory execution
    load_fr3_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'fr3_arm_controller'],
        output='screen'
    )

    # Joint state publisher (official way)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=namespace,
        parameters=[
            {'source_list': ['joint_states'], 'rate': 30}],
    )

    # === MOVEIT SETUP ===

    # Robot description for MoveIt (without gazebo=true to avoid conflicts)
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )

    robot_description_moveit = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file,
         ' hand:=', load_gripper, ' arm_id:=', arm_id,
         ' ros2_control:=true', ' gazebo:=false'])

    robot_description_param = {'robot_description': ParameterValue(
        robot_description_moveit, value_type=str)}

    # Robot semantic description
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ',
         os.path.join(get_package_share_directory('franka_fr3_moveit_config'),
                     'srdf', 'fr3_arm.srdf.xacro'),
         ' hand:=true', ' arm_id:=', arm_id]
    )

    robot_description_semantic = {'robot_description_semantic': ParameterValue(
        robot_description_semantic_config, value_type=str)}

    # Load MoveIt configuration
    kinematics_yaml = load_yaml('franka_fr3_moveit_config', 'config/kinematics.yaml')

    # Planning pipeline
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml')
    if ompl_planning_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # MoveIt controllers - connect to Gazebo's fr3_arm_controller  
    moveit_controllers = {
        'moveit_simple_controller_manager': {
            'controller_names': ['fr3_arm_controller'],
            'fr3_arm_controller': {
                'type': 'FollowJointTrajectory',
                'action_ns': 'follow_joint_trajectory',
                'default': True,
                'joints': [
                    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
                    'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
                ]
            }
        },
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': False,  # Let Gazebo manage controllers
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Move group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=namespace,
        output='screen',
        parameters=[
            robot_description_param,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz with MoveIt configuration
    rviz_config_file = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'rviz', 'moveit.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description_param,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Delay MoveIt to start after Gazebo is fully loaded
    delayed_moveit_launch = TimerAction(
        period=8.0,  # Wait for Gazebo to fully initialize
        actions=[move_group_node, rviz_node]
    )

    return LaunchDescription([
        # Arguments
        load_gripper_launch_argument,
        franka_hand_launch_argument,
        arm_id_launch_argument,
        namespace_launch_argument,
        
        # Start Gazebo (official way)
        gazebo_empty_world,
        
        # Robot in Gazebo (official way)
        robot_state_publisher,
        spawn,
        joint_state_publisher,
        
        # Load Gazebo controllers (official way)
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        
        # Load controller parameters after joint state broadcaster
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_controller_params],
            )
        ),
        
        # Load trajectory controller after parameters
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_controller_params,
                on_exit=[load_fr3_arm_controller],
            )
        ),
        
        # Start MoveIt after delay
        delayed_moveit_launch,
    ])