import pybullet as p
import numpy as np
from force_analysis import ForceAnalyzer

def test_force_analyzer():
    # Initialize analyzer
    analyzer = ForceAnalyzer()
    
    # Reset robot to starting position
    stance_angle = analyzer.reset_robot()
    
    # Step simulation a few times to let forces develop
    for _ in range(10):
        p.stepSimulation()
        
        # Get and print forces
        left_force, right_force = analyzer.get_contact_forces()
        print("\nStep forces:")
        print(f"Left leg forces (N): {left_force}")
        print(f"Right leg forces (N): {right_force}")
        
        # Get COM position
        com_pos = analyzer.get_com_position()
        print(f"COM position (m): {com_pos}")

if __name__ == "__main__":
    test_force_analyzer()
