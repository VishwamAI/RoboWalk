import pybullet as p
import numpy as np
import time
from force_analysis import ForceAnalyzer

def debug_simulation():
    # Initialize with GUI mode for visualization
    analyzer = ForceAnalyzer(connection_mode=p.DIRECT)
    
    # Reset robot and get initial stance
    stance_angle = analyzer.reset_robot()
    
    print("\nStarting debug simulation...")
    print(f"Initial stance angle: {np.degrees(stance_angle):.2f} degrees")
    
    # Run simulation with detailed debugging
    for step in range(500):  # Run for 500 steps
        # Get robot state
        pos, orientation = p.getBasePositionAndOrientation(analyzer.robot_id)
        euler = p.getEulerFromQuaternion(orientation)
        
        # Get all contact points
        contacts = p.getContactPoints(analyzer.robot_id)
        
        # Print detailed state every 50 steps
        if step % 50 == 0:
            print(f"\nStep {step}:")
            print(f"Position (x,y,z): {pos}")
            print(f"Orientation (roll,pitch,yaw): {[np.degrees(angle) for angle in euler]}")
            print(f"Number of contact points: {len(contacts)}")
            
            # Detailed contact information
            if contacts:
                for i, contact in enumerate(contacts):
                    print(f"\nContact {i + 1}:")
                    print(f"Body A: {contact[1]}, Body B: {contact[2]}")
                    print(f"Link index: {contact[3]}")
                    print(f"Position on A: {contact[5]}")
                    print(f"Position on B: {contact[6]}")
                    print(f"Normal force: {contact[9]}")
                    if len(contact) >= 13:
                        print(f"Lateral force 1: {contact[11]}")
                        print(f"Lateral force 2: {contact[12]}")
            else:
                print("No contact points detected!")
                # Get joint states for debugging
                for joint in range(p.getNumJoints(analyzer.robot_id)):
                    joint_state = p.getJointState(analyzer.robot_id, joint)
                    print(f"Joint {joint} position: {np.degrees(joint_state[0]):.2f} degrees")
        
        # Get and print contact forces
        left_force, right_force = analyzer.get_contact_forces()
        total_force = np.linalg.norm(left_force) + np.linalg.norm(right_force)
        
        if step % 50 == 0:
            print(f"Total ground force: {total_force:.2f}N")
            print(f"Left force: {np.linalg.norm(left_force):.2f}N")
            print(f"Right force: {np.linalg.norm(right_force):.2f}N")
        
        # Step simulation
        p.stepSimulation()
    
    print("\nSimulation completed.")
    p.disconnect()

if __name__ == "__main__":
    debug_simulation()
