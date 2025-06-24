#!/usr/bin/env python3
"""
Analysis of the concave-down detection algorithm for robot arm configurations.

This script analyzes the geometric logic for detecting when a robot arm 
has an "elbow drooping" configuration that points toward the ground.

Algorithm under analysis:
1. Uses Y-Z plane (Y: horizontal, Z: vertical-up)
2. Link-2 direction φ2 = -J2
3. Link-3 direction φ3 = -(J2 + J3)
4. Creates unit vectors v2, v3 from these angles
5. Calculates angle-bisector vector vb ∝ v2 + v3
6. If Z-component of vb < 0, then bisector points downward → concave-down
7. Special rule: when |J2 + π/2| ≤ tolerance (≈ J2 = -90°), always treat as UP
"""

import numpy as np
import matplotlib.pyplot as plt
from math import pi, cos, sin, sqrt, degrees, radians

class ConcaveDownAnalyzer:
    def __init__(self, tolerance=0.1):
        """
        Initialize the analyzer with tolerance for special case detection.
        
        Args:
            tolerance: Tolerance for detecting J2 ≈ -90° case (radians)
        """
        self.tolerance = tolerance
    
    def calculate_link_directions(self, J2, J3):
        """
        Calculate the link direction angles according to the algorithm.
        
        Args:
            J2: Joint 2 angle (radians)
            J3: Joint 3 angle (radians)
            
        Returns:
            tuple: (φ2, φ3) - Link direction angles
        """
        phi2 = -J2
        phi3 = -(J2 + J3)
        return phi2, phi3
    
    def calculate_unit_vectors(self, phi2, phi3):
        """
        Calculate unit vectors for link directions in Y-Z plane.
        
        Args:
            phi2: Link-2 direction angle (radians)
            phi3: Link-3 direction angle (radians)
            
        Returns:
            tuple: (v2, v3) - Unit vectors as numpy arrays [y, z]
        """
        v2 = np.array([cos(phi2), sin(phi2)])
        v3 = np.array([cos(phi3), sin(phi3)])
        return v2, v3
    
    def calculate_bisector(self, v2, v3):
        """
        Calculate the angle bisector vector.
        
        Args:
            v2: Unit vector for link-2 direction
            v3: Unit vector for link-3 direction
            
        Returns:
            numpy.array: Bisector vector (not necessarily unit length)
        """
        vb = v2 + v3
        return vb
    
    def is_concave_down(self, J2, J3):
        """
        Determine if the arm configuration is concave down.
        
        Args:
            J2: Joint 2 angle (radians)
            J3: Joint 3 angle (radians)
            
        Returns:
            tuple: (is_concave_down, analysis_data)
        """
        # Special case: J2 ≈ -90°
        if abs(J2 + pi/2) <= self.tolerance:
            return False, {
                'special_case': True,
                'reason': 'J2 ≈ -90° (vertical up)',
                'J2_deg': degrees(J2),
                'J3_deg': degrees(J3)
            }
        
        # Calculate link directions
        phi2, phi3 = self.calculate_link_directions(J2, J3)
        
        # Calculate unit vectors
        v2, v3 = self.calculate_unit_vectors(phi2, phi3)
        
        # Calculate bisector
        vb = self.calculate_bisector(v2, v3)
        
        # Check if bisector points downward
        is_down = vb[1] < 0  # Z-component (index 1) < 0
        
        analysis_data = {
            'special_case': False,
            'J2_deg': degrees(J2),
            'J3_deg': degrees(J3),
            'phi2_deg': degrees(phi2),
            'phi3_deg': degrees(phi3),
            'v2': v2,
            'v3': v3,
            'vb': vb,
            'vb_z': vb[1],
            'is_concave_down': is_down
        }
        
        return is_down, analysis_data
    
    def analyze_test_cases(self):
        """Analyze the specific test cases mentioned."""
        test_cases = [
            (-90, 0, "J2 = -90°, J3 = 0° (should be UP)"),
            (-45, -90, "J2 = -45°, J3 = -90° (might be concave down)"),
            (-58.4, -136.8, "J2 = -58.4°, J3 = -136.8° (problematic case)")
        ]
        
        results = []
        for J2_deg, J3_deg, description in test_cases:
            J2_rad = radians(J2_deg)
            J3_rad = radians(J3_deg)
            
            is_down, data = self.is_concave_down(J2_rad, J3_rad)
            
            result = {
                'description': description,
                'J2_deg': J2_deg,
                'J3_deg': J3_deg,
                'is_concave_down': is_down,
                'data': data
            }
            results.append(result)
        
        return results
    
    def print_analysis(self, results):
        """Print detailed analysis of test cases."""
        print("=" * 80)
        print("CONCAVE DOWN ALGORITHM ANALYSIS")
        print("=" * 80)
        
        for i, result in enumerate(results, 1):
            print(f"\nTest Case {i}: {result['description']}")
            print("-" * 60)
            
            data = result['data']
            
            if data['special_case']:
                print(f"SPECIAL CASE: {data['reason']}")
                print(f"Result: NOT concave down (forced UP)")
            else:
                print(f"J2 = {data['J2_deg']:.1f}°, J3 = {data['J3_deg']:.1f}°")
                print(f"φ2 = -J2 = {data['phi2_deg']:.1f}°")
                print(f"φ3 = -(J2+J3) = {data['phi3_deg']:.1f}°")
                print(f"v2 = [{data['v2'][0]:.3f}, {data['v2'][1]:.3f}] (Y,Z)")
                print(f"v3 = [{data['v3'][0]:.3f}, {data['v3'][1]:.3f}] (Y,Z)")
                print(f"vb = v2 + v3 = [{data['vb'][0]:.3f}, {data['vb'][1]:.3f}]")
                print(f"vb_z = {data['vb_z']:.3f}")
                print(f"vb_z < 0? {data['vb_z'] < 0}")
            
            print(f"RESULT: {'CONCAVE DOWN' if result['is_concave_down'] else 'NOT CONCAVE DOWN'}")
    
    def visualize_configuration(self, J2, J3, title=""):
        """
        Visualize the arm configuration and bisector.
        
        Args:
            J2: Joint 2 angle (radians)
            J3: Joint 3 angle (radians)
            title: Plot title
        """
        is_down, data = self.is_concave_down(J2, J3)
        
        if data['special_case']:
            # For special case, create a simple visualization
            fig, ax = plt.subplots(1, 1, figsize=(8, 6))
            ax.text(0.5, 0.5, f"SPECIAL CASE\n{data['reason']}\nForced: NOT CONCAVE DOWN", 
                   ha='center', va='center', fontsize=14, 
                   bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow"))
            ax.set_xlim(-1, 1)
            ax.set_ylim(-1, 1)
            ax.set_title(title)
            ax.axis('off')
            return fig
        
        # Calculate link directions
        phi2, phi3 = self.calculate_link_directions(J2, J3)
        v2, v3 = self.calculate_unit_vectors(phi2, phi3)
        vb = self.calculate_bisector(v2, v3)
        
        # Normalize bisector for visualization
        vb_norm = vb / np.linalg.norm(vb) if np.linalg.norm(vb) > 0 else vb
        
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        
        # Plot coordinate system
        ax.arrow(0, 0, 1, 0, head_width=0.05, head_length=0.05, fc='gray', ec='gray')
        ax.text(1.1, 0, 'Y (horizontal)', ha='left', va='center')
        ax.arrow(0, 0, 0, 1, head_width=0.05, head_length=0.05, fc='gray', ec='gray')
        ax.text(0, 1.1, 'Z (vertical up)', ha='center', va='bottom')
        
        # Plot link direction vectors
        ax.arrow(0, 0, v2[0], v2[1], head_width=0.05, head_length=0.05, 
                fc='blue', ec='blue', linewidth=2)
        ax.text(v2[0]*1.2, v2[1]*1.2, f'v2\n(φ2={degrees(phi2):.1f}°)', 
               ha='center', va='center', color='blue', fontweight='bold')
        
        ax.arrow(0, 0, v3[0], v3[1], head_width=0.05, head_length=0.05, 
                fc='green', ec='green', linewidth=2)
        ax.text(v3[0]*1.2, v3[1]*1.2, f'v3\n(φ3={degrees(phi3):.1f}°)', 
               ha='center', va='center', color='green', fontweight='bold')
        
        # Plot bisector vector
        color = 'red' if vb[1] < 0 else 'orange'
        ax.arrow(0, 0, vb_norm[0], vb_norm[1], head_width=0.05, head_length=0.05, 
                fc=color, ec=color, linewidth=3)
        ax.text(vb_norm[0]*1.3, vb_norm[1]*1.3, f'vb (bisector)\nvb_z={vb[1]:.3f}', 
               ha='center', va='center', color=color, fontweight='bold')
        
        # Add ground line
        ax.axhline(y=0, color='brown', linestyle='--', alpha=0.7, linewidth=2)
        ax.text(1.2, -0.05, 'Ground level', ha='left', va='top', color='brown')
        
        # Formatting
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(f"{title}\nJ2={degrees(J2):.1f}°, J3={degrees(J3):.1f}°\n"
                    f"Result: {'CONCAVE DOWN' if is_down else 'NOT CONCAVE DOWN'}")
        
        return fig

def main():
    """Main analysis function."""
    analyzer = ConcaveDownAnalyzer(tolerance=0.1)
    
    # Analyze test cases
    results = analyzer.analyze_test_cases()
    analyzer.print_analysis(results)
    
    print("\n" + "=" * 80)
    print("ALGORITHM VALIDATION")
    print("=" * 80)
    
    # Validate the algorithm logic
    print("\n1. ANGLE CALCULATIONS:")
    print("   φ2 = -J2: This maps J2 to standard mathematical angles")
    print("   φ3 = -(J2 + J3): This represents the absolute orientation of link-3")
    print("   For UR5: J2=0° means horizontal, J2=-90° means vertical up")
    print("   The negative signs convert from joint angles to geometric angles")
    
    print("\n2. UNIT VECTOR CALCULATIONS:")
    print("   v2 = [cos(φ2), sin(φ2)]: Standard unit vector in Y-Z plane")
    print("   v3 = [cos(φ3), sin(φ3)]: Standard unit vector in Y-Z plane")
    print("   These correctly represent link directions")
    
    print("\n3. BISECTOR CALCULATION:")
    print("   vb = v2 + v3: Vector addition gives bisector direction")
    print("   This is mathematically sound for angle bisectors")
    print("   The resulting vector points in the bisector direction")
    
    print("\n4. CONCAVE DOWN DETECTION:")
    print("   vb_z < 0: Checks if bisector points below horizontal")
    print("   This correctly identifies when the 'elbow' points downward")
    
    print("\n5. SPECIAL CASE HANDLING:")
    print("   |J2 + π/2| ≤ tolerance: Detects when J2 ≈ -90°")
    print("   This handles the vertical configuration appropriately")
    
    # Create visualizations
    print("\n" + "=" * 80)
    print("CREATING VISUALIZATIONS...")
    print("=" * 80)
    
    test_cases = [
        (-90, 0, "Case 1: J2=-90°, J3=0°"),
        (-45, -90, "Case 2: J2=-45°, J3=-90°"),
        (-58.4, -136.8, "Case 3: J2=-58.4°, J3=-136.8°")
    ]
    
    for J2_deg, J3_deg, title in test_cases:
        J2_rad = radians(J2_deg)
        J3_rad = radians(J3_deg)
        
        fig = analyzer.visualize_configuration(J2_rad, J3_rad, title)
        plt.tight_layout()
        plt.show()
        plt.close()

if __name__ == "__main__":
    main()