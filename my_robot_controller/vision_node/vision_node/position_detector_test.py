#!/usr/bin/env python3
"""
position_detector_test.py - 测试物体位置检测系统
实时显示检测到的物体位置信息
"""

import rclpy
from rclpy.node import Node
import time
from .vision_api import VisionAPI, ObjectPosition

class PositionDetectorTest(Node):
    """位置检测测试节点"""
    
    def __init__(self):
        super().__init__('position_detector_test')
        
        # 初始化视觉API
        self.vision_api = VisionAPI()
        self.vision_api.initialize()
        
        # 创建定时器，每秒检测一次
        self.timer = self.create_timer(1.0, self.detect_and_display)
        
        self.get_logger().info('Position detector test started')
        self.get_logger().info('Looking for red cylinder, green box, blue prism...')
    
    def detect_and_display(self):
        """检测并显示物体位置"""
        try:
            # 获取所有检测到的物体位置
            positions = self.vision_api.get_object_positions()
            
            if not positions:
                self.get_logger().info('No objects detected')
                return
            
            # 按颜色分类显示
            detected_colors = []
            
            for pos in positions:
                # 格式化输出
                pos_str = f"{pos.color}: ({pos.x:.2f}, {pos.y:.2f}) conf:{pos.confidence:.2f}"
                detected_colors.append(pos_str)
            
            # 输出检测结果
            result = f"Detected {len(positions)} objects: " + " | ".join(detected_colors)
            self.get_logger().info(result)
            
            # 检查是否检测到所有三种颜色
            colors_found = {pos.color for pos in positions}
            expected_colors = {'red', 'green', 'blue'}
            missing_colors = expected_colors - colors_found
            
            if missing_colors:
                self.get_logger().warn(f"Missing colors: {missing_colors}")
                
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        detector_test = PositionDetectorTest()
        
        # 等待系统初始化
        time.sleep(2.0)
        
        rclpy.spin(detector_test)
        
    except KeyboardInterrupt:
        print("\nShutting down position detector test...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()