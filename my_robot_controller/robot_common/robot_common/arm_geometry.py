#!/usr/bin/env python3
"""
arm_geometry.py - UR5机械臂几何判断和变换模块
集中管理机械臂的几何形状判断、关节限制和配置变换逻辑
"""

import math
import logging

class UR5ArmGeometry:
    """
    UR5机械臂几何学处理类
    负责机械臂的几何形状判断、关节限制和配置变换
    """
    
    def __init__(self, logger=None):
        """
        初始化UR5机械臂几何处理器
        
        Args:
            logger: 日志记录器，如果为None则使用标准logging
        """
        self.logger = logger if logger else logging.getLogger(__name__)
        
        # UR5机械臂几何参数
        self.L2 = 0.425     # 上臂长度 (Link2)  
        self.L3 = 0.39225   # 前臂长度 (Link3)
        
        # UR5关节限制 (弧度)
        self.joint_limits = {
            'j1': (-math.pi, math.pi),            # J1: ±180
            'j2': (-math.pi, 0),                  # J2: -180° 到 0°
            'j3': (-math.pi, math.pi),            # J3: ±180°
            'j4': (-1.5*math.pi, 0.5*math.pi),      # J4: -270° 到 90°
            'j5': (-math.pi, math.pi),            # J5: ±180
            'j6': (-math.pi, math.pi),            # J6: ±180
        }
    
    def get_joint_limits(self):
        """
        获取UR5关节限制
        
        Returns:
            dict: 关节限制字典
        """
        return self.joint_limits.copy()
    
    def get_joint_limits_list(self):
        """
        获取UR5关节限制（列表格式，用于robot_arm_controller）
        
        Returns:
            list: [(min, max), ...] 格式的关节限制列表
        """
        return [
            self.joint_limits['j1'],  # J1
            self.joint_limits['j2'],  # J2
            self.joint_limits['j3'],  # J3
            self.joint_limits['j4'],  # J4
            self.joint_limits['j5'],  # J5
            self.joint_limits['j6'],  # J6
        ]
    
    def is_arm_concave_down(self, j2_angle, j3_angle):
        """
        判断机械臂是否"向下凹"（肘部下垂）
        简单判断标准：Link2相对地面的角度应该大于Link3相对地面的角度
        - Link2角度 = min(|j2|, |j2+π|) 
        - Link3角度 = min(|j2+j3|, |j2+j3+π|)
        - 如果 Link2角度 <= Link3角度 → 向下凹
        
        Args:
            j2_angle: J2关节角度（弧度）
            j3_angle: J3关节角度（弧度）
            
        Returns:
            bool: True表示机械臂向下凹，需要重新计算
        """
        j2_deg = math.degrees(j2_angle)
        j3_deg = math.degrees(j3_angle)
        self.logger.info(f'判断机械臂形状: J2={j2_deg:.1f}°, J3={j3_deg:.1f}°')

        # 特例：Link2 竖直向上
        if abs(j2_angle + math.pi/2) < math.radians(3):  # ±3°
            self.logger.info('J2 ≈ -90°，Link2 竖直向上 → 视为向上')
            return False

        # 计算Link2相对地面的角度（取较小值）
        link2_ground_angle = min(abs(j2_angle), abs(j2_angle + math.pi))
        
        # 计算Link3相对地面的角度（取较小值）
        link3_absolute = j2_angle + j3_angle
        link3_ground_angle = min(abs(link3_absolute), abs(link3_absolute + math.pi))
        
        link2_deg = math.degrees(link2_ground_angle)
        link3_deg = math.degrees(link3_ground_angle)
        
        self.logger.info(f'Link2相对地面角度: {link2_deg:.1f}°')
        self.logger.info(f'Link3相对地面角度: {link3_deg:.1f}°')

        # 判断：Link2角度应该大于Link3角度
        if link2_ground_angle <= link3_ground_angle:
            self.logger.info(f'Link2角度({link2_deg:.1f}°) <= Link3角度({link3_deg:.1f}°) → 向下凹')
            return True
        else:
            self.logger.info(f'Link2角度({link2_deg:.1f}°) > Link3角度({link3_deg:.1f}°) → 向上凸出')
            return False
    
    def correct_joint_configuration(self, joint_solution):
        """
        通过几何变换修正关节配置
        正确计算alpha角（在J2点处，J4-J2-J3三点形成的角度）
        
        Args:
            joint_solution: 原始关节角度解 [J1, J2, J3, J4, J5, J6]
            
        Returns:
            list: 修正后的关节角度，或None如果无法修正
        """
        try:
            j1, j2, j3, j4, j5, j6 = joint_solution.copy()
            
            # 默认值（如果不需要变换）
            new_j1, new_j2, new_j3, new_j4, new_j5, new_j6 = j1, j2, j3, j4, j5, j6
            
            # 计算Link2和Link3的夹角（内角）
            angle_between_links = math.pi - abs(j3)
            
            # 计算J2到J4的距离（使用余弦定理）
            J2_to_J4_distance = math.sqrt(self.L2*self.L2 + self.L3*self.L3 - 2*self.L2*self.L3*math.cos(angle_between_links))
            
            # 计算alpha角：在J2点处，J4-J2-J3三点形成的角度
            cos_alpha = (self.L2*self.L2 + J2_to_J4_distance*J2_to_J4_distance - self.L3*self.L3) / (2*self.L2*J2_to_J4_distance)
            # 确保在有效范围内
            cos_alpha = max(-1.0, min(1.0, cos_alpha))
            alpha = math.acos(cos_alpha)
            
            self.logger.info(f'几何计算: Link夹角={math.degrees(angle_between_links):.1f}°, J2到J4距离={J2_to_J4_distance:.3f}m')
            self.logger.info(f'alpha角(J4-J2-J3): {math.degrees(alpha):.1f}°')
            self.logger.info(f'详细alpha计算: cos_alpha={cos_alpha:.6f}, alpha_rad={alpha:.6f}, alpha_deg={math.degrees(alpha):.3f}°')
            
            # 保存Link4相对地面的角度（保持不变）
            link4_ground_angle = j2 + j3 + j4
            
            # 基于α角的配置转换
            new_j1 = j1      # 基座不变
            new_j5 = j5      # 腕关节2保持不变
            new_j6 = j6      # 腕关节3保持不变
            
            # 肘关节的两种配置 - 简单翻转
            new_j3 = -j3  # J3就是简单翻转
            
            # J2根据α角调整
            if j3 < 0:  # 当前是肘下配置 → 肘上
                new_j2 = j2 - 2*alpha       # J2相应调整
                self.logger.info(f'肘下→肘上配置转换')
            else:  # 当前是肘上配置 → 肘下
                new_j2 = j2 + 2*alpha       # J2相应调整
                self.logger.info(f'肘上→肘下配置转换')
            
            # J4计算：保持Link4相对地面角度不变
            new_j4 = link4_ground_angle - new_j2 - new_j3
            
            self.logger.info(f'  Link4地面角度保持: {math.degrees(link4_ground_angle):.1f}°')
            self.logger.info(f'  J2: {math.degrees(j2):.1f}°→{math.degrees(new_j2):.1f}°')
            self.logger.info(f'  J3: {math.degrees(j3):.1f}°→{math.degrees(new_j3):.1f}°')
            self.logger.info(f'  J4: {math.degrees(j4):.1f}°→{math.degrees(new_j4):.1f}°')
            
            # 调整J4到限制范围内（360°关节）
            j4_min, j4_max = self.joint_limits['j4']
            
            # 将J4调整到等效角度范围内
            while new_j4 > j4_max:
                new_j4 -= 2 * math.pi  # 减去360°
            while new_j4 < j4_min:
                new_j4 += 2 * math.pi  # 加上360°
            
            self.logger.info(f'  J4调整后: {math.degrees(new_j4):.1f}°')
            
            # 检查调整后是否在范围内
            if j4_min <= new_j4 <= j4_max:
                corrected_solution = [new_j1, new_j2, new_j3, new_j4, new_j5, new_j6]
                self.logger.info(f'✅ UR完整配置转换成功！肘上↔肘下转换完成')
                return corrected_solution
            else:
                self.logger.warning(f'J4调整后仍超出限制范围: {math.degrees(new_j4):.1f}°')
                return None
                
        except Exception as e:
            self.logger.error(f'几何变换出错: {e}')
            return None
    
    def normalize_joint_angles(self, joint_angles):
        """
        标准化关节角度到合理范围
        
        Args:
            joint_angles: 关节角度列表 [J1, J2, J3, J4, J5, J6]
            
        Returns:
            list: 标准化后的关节角度
        """
        normalized_angles = []
        
        for i, angle in enumerate(joint_angles):
            # 标准化到 -π 到 π 范围
            normalized = ((angle + math.pi) % (2*math.pi)) - math.pi
            
            # 检查是否在关节限制范围内
            joint_key = f'j{i+1}'
            min_limit, max_limit = self.joint_limits[joint_key]
            
            if normalized < min_limit or normalized > max_limit:
                self.logger.warning(f'Joint {i+1} angle {math.degrees(normalized):.1f}° exceeds limits ({math.degrees(min_limit):.1f}° to {math.degrees(max_limit):.1f}°)')
                # 钳制到限制范围内
                normalized = max(min_limit, min(max_limit, normalized))
                self.logger.info(f'   Clamped to: {math.degrees(normalized):.1f}°')
            
            normalized_angles.append(normalized)
        
        return normalized_angles
    
    def validate_joint_solution(self, joint_solution):
        """
        验证关节解是否有效（在限制范围内且几何形状合理）
        
        Args:
            joint_solution: 关节角度解 [J1, J2, J3, J4, J5, J6]
            
        Returns:
            tuple: (is_valid: bool, error_message: str)
        """
        if len(joint_solution) != 6:
            return False, "关节解长度不正确，应为6个关节"
        
        # 检查关节限制
        for i, angle in enumerate(joint_solution):
            joint_key = f'j{i+1}'
            min_limit, max_limit = self.joint_limits[joint_key]
            
            if angle < min_limit or angle > max_limit:
                return False, f"Joint {i+1} 角度 {math.degrees(angle):.1f}° 超出限制范围 ({math.degrees(min_limit):.1f}° to {math.degrees(max_limit):.1f}°)"
        
        # 检查几何形状
        j2_angle = joint_solution[1]
        j3_angle = joint_solution[2]
        
        if self.is_arm_concave_down(j2_angle, j3_angle):
            return False, f"机械臂配置向下凹 (J2={math.degrees(j2_angle):.1f}°, J3={math.degrees(j3_angle):.1f}°)"
        
        return True, "关节解有效"


def main():
    """测试函数"""
    # 设置日志
    logging.basicConfig(level=logging.INFO)
    
    # 创建几何处理器
    geometry = UR5ArmGeometry()
    
    # 测试关节限制
    print("=== 关节限制测试 ===")
    limits = geometry.get_joint_limits()
    for joint, (min_val, max_val) in limits.items():
        print(f"{joint}: {math.degrees(min_val):.1f}° to {math.degrees(max_val):.1f}°")
    
    # 测试几何形状判断
    print("\n=== 几何形状判断测试 ===")
    test_cases = [
        (-1.57, 0.5),   # J2=-90°, J3=+30° → 正常向上
        (-1.57, -0.5),  # J2=-90°, J3=-30° → 向下凹
        (-0.79, 0.0),   # J2=-45°, J3=0°   → 边界情况
    ]
    
    for j2, j3 in test_cases:
        is_concave = geometry.is_arm_concave_down(j2, j3)
        print(f"J2={math.degrees(j2):.1f}°, J3={math.degrees(j3):.1f}° → {'向下凹' if is_concave else '正常'}")
    
    print("\n=== 几何变换测试 ===")
    # 测试配置转换
    test_solution = [0.0, -1.57, -0.5, -1.57, 0.0, 0.0]  # 向下凹的配置
    print(f"原始解: {[f'{math.degrees(a):.1f}°' for a in test_solution]}")
    
    corrected = geometry.correct_joint_configuration(test_solution)
    if corrected:
        print(f"修正解: {[f'{math.degrees(a):.1f}°' for a in corrected]}")
    else:
        print("无法修正")


if __name__ == '__main__':
    main()