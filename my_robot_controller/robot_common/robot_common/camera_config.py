#!/usr/bin/env python3
"""
camera_config.py - 统一的相机配置管理

为整个机器人系统提供一致的相机参数配置，
包括ur_vision_system和vision_node都使用相同的配置
"""

import math
from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class CameraConfig:
    """相机配置数据类"""
    
    # == 物理位置 ==
    position_x: float = 0.0      # 相机世界坐标X (米)
    position_y: float = 0.5      # 相机世界坐标Y (米)  
    position_z: float = 1.5      # 相机世界坐标Z (米)
    
    # == 姿态角度 ==
    roll: float = 1.5708         # Roll
    pitch: float = 1.5708        # Pitch
    yaw: float = 0.0             # Yaw
    
    # == 图像参数 ==
    image_width: int = 640       # 图像宽度 (像素)
    image_height: int = 480      # 图像高度 (像素)
    image_format: str = "R8G8B8" # 图像格式
    
    # == 光学参数 ==
    horizontal_fov: float = 1.047198  # 水平视野角 (弧度，60度)
    focal_length: float = 320    # 焦距 (像素，基于640宽度)
    
    # == 深度相机参数 ==
    near_clip: float = 0.05      # 近裁剪距离 (米)
    far_clip: float = 10.0       # 远裁剪距离 (米)
    update_rate: float = 30.0    # 更新频率 (Hz)
    
    # == ROS话题 ==
    namespace: str = "world_camera"
    image_topic: str = "/world_camera/world_camera/image_raw"
    depth_topic: str = "/world_camera/world_camera/depth/image_raw"
    camera_info_topic: str = "/world_camera/world_camera/camera_info"
    depth_info_topic: str = "/world_camera/world_camera/depth/camera_info"
    points_topic: str = "/world_camera/world_camera/points"
    
    # == 工作空间参数 ==
    table_height: float = 0.0    # 工作台面高度 (米)
    
    def get_spawn_args(self) -> list:
        """获取Gazebo spawn_entity.py的参数列表"""
        return [
            '-x', str(self.position_x),
            '-y', str(self.position_y),
            '-z', str(self.position_z),
            '-R', str(self.roll),
            '-P', str(self.pitch),
            '-Y', str(self.yaw)
        ]
    
    def get_sdf_camera_params(self) -> dict:
        """获取SDF相机参数字典"""
        return {
            'horizontal_fov': self.horizontal_fov,
            'image_width': self.image_width,
            'image_height': self.image_height,
            'image_format': self.image_format,
            'near_clip': self.near_clip,
            'far_clip': self.far_clip,
            'update_rate': self.update_rate,
            'namespace': self.namespace
        }
    
    def get_vision_api_params(self) -> dict:
        """获取vision_api使用的参数字典"""
        return {
            'camera_x': self.position_x,
            'camera_y': self.position_y,
            'camera_z': self.position_z,
            'camera_pitch': self.pitch,
            'camera_yaw': self.yaw,
            'focal_length': self.focal_length,
            'image_width': self.image_width,
            'image_height': self.image_height,
            'horizontal_fov': self.horizontal_fov,
            'table_height': self.table_height
        }
    
    def get_ground_coverage(self) -> float:
        """计算地面覆盖范围直径 (米)"""
        return 2 * self.position_z * math.tan(self.horizontal_fov / 2)
    
    def get_pixel_to_meter_ratio(self) -> float:
        """计算像素到米的转换比例"""
        ground_coverage = self.get_ground_coverage()
        return ground_coverage / self.image_width

# 全局默认相机配置实例
DEFAULT_CAMERA_CONFIG = CameraConfig()

def get_camera_config() -> CameraConfig:
    """获取当前相机配置实例"""
    return DEFAULT_CAMERA_CONFIG

def update_camera_config(**kwargs) -> None:
    """更新相机配置参数"""
    global DEFAULT_CAMERA_CONFIG
    for key, value in kwargs.items():
        if hasattr(DEFAULT_CAMERA_CONFIG, key):
            setattr(DEFAULT_CAMERA_CONFIG, key, value)
        else:
            raise ValueError(f"Unknown camera config parameter: {key}")

# 预定义配置
class CameraConfigs:
    """预定义的相机配置"""
    
    @staticmethod
    def low_height_config() -> CameraConfig:
        """低高度配置 (1.0米)"""
        return CameraConfig(position_z=1.0)
    
    @staticmethod
    def high_height_config() -> CameraConfig:
        """高高度配置 (2.0米)"""
        return CameraConfig(position_z=2.0)
    
    @staticmethod
    def wide_fov_config() -> CameraConfig:
        """宽视野配置 (90度)"""
        return CameraConfig(horizontal_fov=1.5708, focal_length=240)
    
    @staticmethod
    def narrow_fov_config() -> CameraConfig:
        """窄视野配置 (30度)"""
        return CameraConfig(horizontal_fov=0.5236, focal_length=640)