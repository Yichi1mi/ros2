#!/usr/bin/env python3
"""
robot_config.py - Configuration management for different simulation modes
"""

# Controller configurations for different modes
CONTROLLER_CONFIGS = {
    "moveit": {
        "name": "MoveIt仿真模式",
        "description": "机械臂运动正常，gripper不可用 (fake hardware)",
        "arm_controller": "/fr3_arm_controller/follow_joint_trajectory",
        "gripper_available": False,
        "gripper_controllers": {},
        "planning_group": "fr3_manipulator",
        "planning_frame": "fr3_link0",
        "end_effector_link": "fr3_hand_tcp",
        "joint_names": [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3',
            'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]
    },
    "gazebo": {
        "name": "Gazebo仿真模式", 
        "description": "完整arm+gripper仿真 (需要调试controller名称)",
        "arm_controller": "/arm_controller/follow_joint_trajectory",  # 待确认
        "gripper_available": True,
        "gripper_controllers": {
            "homing": "/fr3_gripper/homing",
            "move": "/fr3_gripper/move", 
            "grasp": "/fr3_gripper/grasp"
        },
        "planning_group": "panda_arm",  # 待确认
        "planning_frame": "panda_link0",  # 待确认
        "end_effector_link": "panda_hand_tcp",  # 待确认
        "joint_names": [
            'panda_joint1', 'panda_joint2', 'panda_joint3',
            'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
    }
}

def get_config(mode):
    """
    Get configuration for specified mode
    
    Args:
        mode: "moveit" or "gazebo"
        
    Returns:
        dict: Configuration parameters for the mode
    """
    if mode not in CONTROLLER_CONFIGS:
        raise ValueError(f"Unknown mode: {mode}. Available modes: {list(CONTROLLER_CONFIGS.keys())}")
    
    return CONTROLLER_CONFIGS[mode].copy()

def list_available_modes():
    """
    List all available simulation modes with descriptions
    
    Returns:
        dict: mode_name -> description mapping
    """
    return {
        mode: config["name"] + " - " + config["description"] 
        for mode, config in CONTROLLER_CONFIGS.items()
    }

def validate_mode(mode):
    """
    Validate if mode is supported
    
    Args:
        mode: Mode string to validate
        
    Returns:
        bool: True if mode is valid
    """
    return mode in CONTROLLER_CONFIGS