import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

def generate_launch_description():
    # Package and file names
    moveit_config_package = "my_moveit_config"
    description_package = "my_ur_description"
    description_file = "ur_robotiq_85.urdf.xacro"
    
    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
        " ",
        "use_fake_hardware:=true",
        " ",
        "generate_ros2_control_tag:=true",
    ])
    robot_description = {"robot_description": robot_description_content}
    
    # Robot description semantic (SRDF)
    robot_description_semantic_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(moveit_config_package), "config", "ur5e_with_robotiq.srdf"]),
    ])
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    
    # Kinematics
    robot_description_kinematics = PathJoinSubstitution([
        FindPackageShare(moveit_config_package), "config", "kinematics.yaml"
    ])
    
    # Joint limits
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            moveit_config_package, "config/joint_limits.yaml"
        )
    }
    
    # OMPL planning
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    
    # Controllers configuration  
    controllers_yaml = load_yaml(moveit_config_package, "config/moveit_controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    
    # Trajectory execution
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    
    # Planning scene monitor
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    
    # ros2_control configuration
    ros2_controllers_config = load_yaml(moveit_config_package, "config/ros2_controllers.yaml")

    # Move group node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": False},
        ],
    )
    
    # ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_config],
        output="screen",
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare(moveit_config_package), "config", "moveit.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    # Controller spawners
    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[controller_name, "-c", "/controller_manager"],
            output="screen",
        )
        for controller_name in [
            "ur_manipulator_controller",
            "robotiq_gripper_controller", 
            "joint_state_broadcaster",
        ]
    ]
    
    nodes_to_start = [
        move_group_node,
        ros2_control_node,
        rviz_node,
    ] + controller_spawners
    
    return LaunchDescription(nodes_to_start)