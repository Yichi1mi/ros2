#!/usr/bin/env python3
"""
vision_api.py - Vision API for getting object positions from camera
提供标准化的视觉检测接口，供main_controller调用
"""

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
import math
from dataclasses import dataclass
from typing import List, Optional
from .object_detector import ObjectDetector
from robot_common.camera_config import get_camera_config

@dataclass
class ObjectPosition:
    """物体位置信息"""
    color: str          # 物体颜色
    x: float           # 世界坐标X (米)
    y: float           # 世界坐标Y (米)  
    z: float           # 世界坐标Z (米)
    confidence: float  # 置信度 (0-1)
    pixel_x: int       # 像素坐标X
    pixel_y: int       # 像素坐标Y
    area: float        # 检测区域面积

class VisionAPI:
    """
    视觉API类 - 提供物体检测和位置估算功能
    订阅ur_vision_system提供的摄像头数据，输出物体3D位置信息
    """
    
    def __init__(self, camera_topic=None):
        """
        初始化视觉API
        
        Args:
            camera_topic: 摄像头图像话题名称，None则使用配置中的默认值
        """
        # 获取统一相机配置
        self.camera_config = get_camera_config()
        
        # 设置话题名称
        if camera_topic is None:
            self.camera_topic = self.camera_config.image_topic
        else:
            self.camera_topic = camera_topic
            
        self.detector = None
        self.is_initialized = False
        
        # 获取相机参数（保持向后兼容）
        self.camera_params = self.camera_config.get_vision_api_params()
        self.table_height = self.camera_config.table_height
        
        # 最近检测结果缓存
        self._last_detection_time = 0
        self._cached_positions = []
        self._cache_timeout = 1.0  # 缓存超时时间(秒)
    
    def initialize(self):
        """初始化视觉系统"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            # 创建物体检测器（话题已经在object_detector中修改为正确的）
            self.detector = ObjectDetector()
            
            self.is_initialized = True
            self.detector.get_logger().info(f'VisionAPI initialized, subscribing to {self.camera_topic}')
            return True
            
        except Exception as e:
            print(f"Error initializing VisionAPI: {e}")
            return False
    
    def get_object_positions(self, timeout=5.0) -> List[ObjectPosition]:
        """
        获取检测到的物体位置列表
        
        Args:
            timeout: 获取数据的超时时间(秒)
            
        Returns:
            List[ObjectPosition]: 检测到的物体位置列表
        """
        if not self.is_initialized:
            if not self.initialize():
                return []
        
        # 检查缓存
        current_time = time.time()
        if (current_time - self._last_detection_time) < self._cache_timeout:
            return self._cached_positions.copy()
        
        # 等待新的检测数据
        start_time = time.time()
        while (time.time() - start_time) < timeout:
            try:
                # 短暂处理ROS回调
                rclpy.spin_once(self.detector, timeout_sec=0.1)
                
                # 获取检测结果
                detected_objects = self.detector.get_detected_objects()
                if detected_objects:
                    # 转换为3D位置
                    positions = self._convert_to_3d_positions(detected_objects)
                    
                    # 更新缓存
                    self._cached_positions = positions
                    self._last_detection_time = current_time
                    
                    return positions
                    
            except Exception as e:
                self.detector.get_logger().error(f'Error in get_object_positions: {e}')
                break
        
        # 超时或出错，返回空列表
        return []
    
    def get_largest_object_position(self, color_filter=None) -> Optional[ObjectPosition]:
        """
        获取最大的物体位置（通常是最感兴趣的目标）
        
        Args:
            color_filter: 颜色过滤器，如'red', 'blue'等，None表示不过滤
            
        Returns:
            ObjectPosition or None: 最大物体的位置信息
        """
        positions = self.get_object_positions()
        
        if not positions:
            return None
        
        # 应用颜色过滤
        if color_filter:
            positions = [pos for pos in positions if pos.color == color_filter]
            if not positions:
                return None
        
        # 返回面积最大的物体
        return max(positions, key=lambda p: p.area)
    
    def _convert_to_3d_positions(self, detected_objects) -> List[ObjectPosition]:
        """
        将2D检测结果转换为3D世界坐标
        
        Args:
            detected_objects: 2D检测结果列表
            
        Returns:
            List[ObjectPosition]: 3D位置列表
        """
        positions = []
        
        for obj in detected_objects:
            try:
                # 获取像素坐标
                pixel_x, pixel_y = obj['center']
                
                # 转换为3D世界坐标
                world_x, world_y, world_z = self._pixel_to_world_coordinates(
                    pixel_x, pixel_y
                )
                
                # 计算置信度（基于检测区域大小）
                confidence = min(1.0, obj['area'] / 10000.0)  # 归一化到0-1
                
                # 创建位置对象
                position = ObjectPosition(
                    color=obj['color'],
                    x=world_x,
                    y=world_y,
                    z=world_z,
                    confidence=confidence,
                    pixel_x=pixel_x,
                    pixel_y=pixel_y,
                    area=obj['area']
                )
                
                positions.append(position)
                
            except Exception as e:
                if self.detector:
                    self.detector.get_logger().error(f'Error converting object to 3D: {e}')
        
        return positions
    
    def _pixel_to_world_coordinates(self, pixel_x, pixel_y):
        """
        将像素坐标转换为世界坐标 - 专为垂直向下相机设计
        
        相机在(0,0,1.5)垂直向下看，物体在z=0平面
        
        Args:
            pixel_x, pixel_y: 像素坐标
            
        Returns:
            tuple: (world_x, world_y, world_z) 世界坐标
        """
        # 图像中心
        cx = self.camera_params['image_width'] / 2   # 320
        cy = self.camera_params['image_height'] / 2  # 240
        
        # 相机高度
        cam_height = self.camera_params['camera_z']  # 1.5米
        
        # 使用配置中的参数计算地面覆盖范围
        ground_coverage = self.camera_config.get_ground_coverage()
        pixel_to_meter_ratio = self.camera_config.get_pixel_to_meter_ratio()
        
        # 转换坐标（原点在图像中心对应世界原点）
        world_x = (pixel_x - cx) * pixel_to_meter_ratio
        world_y = (pixel_y - cy) * pixel_to_meter_ratio  
        world_z = self.table_height  # 物体在地面
        
        return world_x, world_y, world_z
    
    def shutdown(self):
        """关闭视觉API"""
        if self.detector:
            self.detector.destroy_node()
        self.is_initialized = False

def main(args=None):
    """测试VisionAPI"""
    print("Testing VisionAPI...")
    
    api = VisionAPI()
    
    try:
        if api.initialize():
            print("VisionAPI initialized successfully")
            
            # 测试获取物体位置
            for i in range(10):
                positions = api.get_object_positions(timeout=2.0)
                if positions:
                    print(f"\nDetected {len(positions)} objects:")
                    for pos in positions:
                        print(f"  {pos.color}: ({pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}) "
                              f"confidence={pos.confidence:.2f}")
                else:
                    print(f"No objects detected (attempt {i+1})")
                
                time.sleep(1.0)
        
    except KeyboardInterrupt:
        print("\nVisionAPI test interrupted")
    finally:
        api.shutdown()

if __name__ == '__main__':
    main()