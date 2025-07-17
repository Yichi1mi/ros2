#!/usr/bin/env python3
"""
arm_geometry.py - UR5 robot arm geometry module
Manages arm geometry validation, joint limits and configuration transformations
"""

import math
import logging

class UR5ArmGeometry:
    
    def __init__(self, logger=None):

        self.logger = logger if logger else logging.getLogger(__name__)
        
        # UR5 arm geometry parameters
        self.L2 = 0.425     # Link2
        self.L3 = 0.39225   # Link3
        
        # UR5 joint limits
        self.joint_limits = {
            'j1': (-math.pi, math.pi),            # J1: ±180
            'j2': (-math.pi, 0),                  # J2: -180° 到 0°
            'j3': (-math.pi, math.pi),            # J3: ±180°
            # 'j4': (-1.5*math.pi, 0.5*math.pi),    # J4: -270° 到 90°
            'j4': (-math.pi, 0),                  # J4: -180° 到 0°
            'j5': (-math.pi, math.pi),            # J5: ±180
            'j6': (-math.pi, math.pi),            # J6: ±180
        }
    
    def get_joint_limits(self):

        return self.joint_limits.copy()
    
    def get_joint_limits_list(self):

        return [
            self.joint_limits['j1'],
            self.joint_limits['j2'],
            self.joint_limits['j3'],
            self.joint_limits['j4'],
            self.joint_limits['j5'],
            self.joint_limits['j6'],
        ]
    
    def is_arm_concave_down(self, j2_angle, j3_angle):
        """
        Check if arm is in concave-down configuration (elbow drooping)
        Simple criteria: Link2 ground angle should be greater than Link3 ground angle
        - Link2 angle = min(|j2|, |j2+π|) 
        - Link3 angle = min(|j2+j3|, |j2+j3+π|)
        - If Link2 angle <= Link3 angle → concave down
            
        """
        j2_deg = math.degrees(j2_angle)
        j3_deg = math.degrees(j3_angle)
        self.logger.debug(f'Checking arm configuration: J2={j2_deg:.1f}°, J3={j3_deg:.1f}°')

        # Special case: Link2 vertical up
        if abs(j2_angle + math.pi/2) < math.radians(3):  # ±3°
            self.logger.debug('J2 ≈ -90°, Link2 vertical up → considered upward')
            return False

        # Calculate Link2 ground angle (take smaller value)
        link2_ground_angle = min(abs(j2_angle), abs(j2_angle + math.pi))
        
        # Calculate Link3 ground angle (take smaller value)
        link3_absolute = j2_angle + j3_angle
        link3_ground_angle = min(abs(link3_absolute), abs(link3_absolute + math.pi))
        
        link2_deg = math.degrees(link2_ground_angle)
        link3_deg = math.degrees(link3_ground_angle)
        
        self.logger.debug(f'Link2 ground angle: {link2_deg:.1f}°')
        self.logger.debug(f'Link3 ground angle: {link3_deg:.1f}°')

        # Check: Link2 angle should be greater than Link3 angle
        if link2_ground_angle <= link3_ground_angle:
            self.logger.debug(f'Link2({link2_deg:.1f}°) <= Link3({link3_deg:.1f}°) → concave down')
            return True
        else:
            self.logger.debug(f'Link2({link2_deg:.1f}°) > Link3({link3_deg:.1f}°) → upward')
            return False
    
    def correct_joint_configuration(self, joint_solution):
        """
        Correct joint configuration through geometric transformation
        Calculate alpha angle (angle formed by J4-J2-J3 at J2 point)
        
        Args:
            joint_solution: Original joint angle solution [J1, J2, J3, J4, J5, J6]
            
        Returns:
            list: Corrected joint angles, or None if correction fails
        """
        try:
            j1, j2, j3, j4, j5, j6 = joint_solution.copy()
            
            # Default values (if no transformation needed)
            new_j1, new_j2, new_j3, new_j4, new_j5, new_j6 = j1, j2, j3, j4, j5, j6
            
            # Calculate angle between Link2 and Link3 (interior angle)
            angle_between_links = math.pi - abs(j3)
            
            # Calculate J2 to J4 distance (using cosine law)
            J2_to_J4_distance = math.sqrt(self.L2*self.L2 + self.L3*self.L3 - 2*self.L2*self.L3*math.cos(angle_between_links))
            
            # Calculate alpha angle: angle formed by J4-J2-J3 at J2 point
            cos_alpha = (self.L2*self.L2 + J2_to_J4_distance*J2_to_J4_distance - self.L3*self.L3) / (2*self.L2*J2_to_J4_distance)
            # Ensure within valid range
            cos_alpha = max(-1.0, min(1.0, cos_alpha))
            alpha = math.acos(cos_alpha)
            
            self.logger.debug(f'Geometry calc: Link angle={math.degrees(angle_between_links):.1f}°')
            self.logger.debug(f'Alpha angle(J4-J2-J3): {math.degrees(alpha):.1f}°')
            
            # Save Link4 ground angle (keep unchanged)
            link4_ground_angle = j2 + j3 + j4
            
            # Configuration conversion based on alpha angle
            new_j1 = j1
            new_j5 = j5
            new_j6 = j6
            
            # J3 simple flip
            new_j3 = -j3
            
            # Adjust J2 based on alpha angle
            if j3 < 0:
                new_j2 = j2 - 2*alpha
                self.logger.debug('Elbow-down → elbow-up conversion')
            else:
                new_j2 = j2 + 2*alpha
                self.logger.debug('Elbow-up → elbow-down conversion')
            
            # J4 calculation: keep Link4 ground angle unchanged
            new_j4 = link4_ground_angle - new_j2 - new_j3
            
            self.logger.debug(f'Link4 ground angle: {math.degrees(link4_ground_angle):.1f}°')
            self.logger.debug(f'J2: {math.degrees(j2):.1f}°→{math.degrees(new_j2):.1f}°')
            self.logger.debug(f'J3: {math.degrees(j3):.1f}°→{math.degrees(new_j3):.1f}°')
            self.logger.debug(f'J4: {math.degrees(j4):.1f}°→{math.degrees(new_j4):.1f}°')
            
            # Adjust J4 to limit range (360° joint)
            j4_min, j4_max = self.joint_limits['j4']
            
            # Adjust J4 to equivalent angle range
            while new_j4 > j4_max:
                new_j4 -= 2 * math.pi
            while new_j4 < j4_min:
                new_j4 += 2 * math.pi
            
            self.logger.debug(f'J4 adjusted: {math.degrees(new_j4):.1f}°')
            
            # Check if within range after adjustment
            if j4_min <= new_j4 <= j4_max:
                corrected_solution = [new_j1, new_j2, new_j3, new_j4, new_j5, new_j6]
                self.logger.debug('Configuration conversion successful: elbow up/down switched')
                return corrected_solution
            else:
                self.logger.warning(f'J4 still exceeds limits after adjustment: {math.degrees(new_j4):.1f}°')
                return None
                
        except Exception as e:
            self.logger.error(f'Geometry transformation error: {e}')
            return None
    
    def normalize_joint_angles(self, joint_angles):
        """
        Normalize joint angles to reasonable range
        
        Args:
            joint_angles: Joint angle list [J1, J2, J3, J4, J5, J6]
            
        Returns:
            list: Normalized joint angles
        """
        normalized_angles = []
        
        for i, angle in enumerate(joint_angles):
            # Normalize to -π to π range
            normalized = ((angle + math.pi) % (2*math.pi)) - math.pi
            
            # Check if within joint limit range
            joint_key = f'j{i+1}'
            min_limit, max_limit = self.joint_limits[joint_key]
            
            if normalized < min_limit or normalized > max_limit:
                self.logger.warning(f'Joint {i+1} angle {math.degrees(normalized):.1f}° exceeds limits ({math.degrees(min_limit):.1f}° to {math.degrees(max_limit):.1f}°)')
                # Clamp to limit range
                normalized = max(min_limit, min(max_limit, normalized))
                self.logger.info(f'   Clamped to: {math.degrees(normalized):.1f}°')
            
            normalized_angles.append(normalized)
        
        return normalized_angles
    
    def validate_joint_solution(self, joint_solution):
        """
        Validate if joint solution is valid (within limits and reasonable geometry)
        
        Args:
            joint_solution: Joint angle solution [J1, J2, J3, J4, J5, J6]
            
        Returns:
            tuple: (is_valid: bool, error_message: str)
        """
        if len(joint_solution) != 6:
            return False,
        
        # Check joint limits
        for i, angle in enumerate(joint_solution):
            joint_key = f'j{i+1}'
            min_limit, max_limit = self.joint_limits[joint_key]
            
            if angle < min_limit or angle > max_limit:
                return False, f"Joint {i+1} Angle {math.degrees(angle):.1f}° over limit ({math.degrees(min_limit):.1f}° to {math.degrees(max_limit):.1f}°)"
        
        # Check geometry
        j2_angle = joint_solution[1]
        j3_angle = joint_solution[2]
        
        if self.is_arm_concave_down(j2_angle, j3_angle):
            return False, f"Arm configuration concave down (J2={math.degrees(j2_angle):.1f}°, J3={math.degrees(j3_angle):.1f}°)"
        
        return True, "Joint solution valid"


def main():
    """Test function"""
    # Setup logging
    logging.basicConfig(level=logging.INFO)
    
    # Create geometry handler
    geometry = UR5ArmGeometry()
    
    # Test joint limits
    print("=== Joint Limits Test ===")
    limits = geometry.get_joint_limits()
    for joint, (min_val, max_val) in limits.items():
        print(f"{joint}: {math.degrees(min_val):.1f}° to {math.degrees(max_val):.1f}°")
    
    # Test geometry validation
    print("\n=== Geometry Validation Test ===")
    test_cases = [
        (-1.57, 0.5),   # J2=-90°, J3=+30° → normal upward
        (-1.57, -0.5),  # J2=-90°, J3=-30° → concave down
        (-0.79, 0.0),   # J2=-45°, J3=0°   → boundary case
    ]
    
    for j2, j3 in test_cases:
        is_concave = geometry.is_arm_concave_down(j2, j3)
        print(f"J2={math.degrees(j2):.1f}°, J3={math.degrees(j3):.1f}° → {'concave down' if is_concave else 'normal'}")
    
    print("\n=== Geometry Transform Test ===")
    # Test configuration conversion
    test_solution = [0.0, -1.57, -0.5, -1.57, 0.0, 0.0]  # concave down config
    print(f"Original: {[f'{math.degrees(a):.1f}°' for a in test_solution]}")
    
    corrected = geometry.correct_joint_configuration(test_solution)
    if corrected:
        print(f"Corrected: {[f'{math.degrees(a):.1f}°' for a in corrected]}")
    else:
        print("Cannot correct")


if __name__ == '__main__':
    main()