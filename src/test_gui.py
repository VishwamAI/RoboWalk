import argparse
import pybullet as p
import pybullet_data
import time
from force_analysis import ForceAnalyzer
from walking_controller_fixed import WalkingController

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Bipedal Robot Walking Controller')
    parser.add_argument('--gui', action='store_true', help='Enable GUI visualization (run locally)')
    args = parser.parse_args()

    # Initialize PyBullet in DIRECT mode for force analysis
    try:
        physicsClient = p.connect(p.DIRECT)
        print("\nRunning in DIRECT mode for force analysis")
        print("NOTE: To see GUI visualization, run this script locally with --gui flag")
        print("The GUI cannot be displayed in a remote/headless environment\n")
    except Exception as e:
        print(f"Error connecting to PyBullet: {str(e)}")
        return

    # Set up environment
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        p.setRealTimeSimulation(0)  # Use stepSimulation for better control
    except Exception as e:
        print(f"Error setting up environment: {str(e)}")
        p.disconnect()
        return

    try:
        # Initialize force analyzer and controller
        force_analyzer = ForceAnalyzer(connection_mode=p.DIRECT)
        controller = WalkingController(force_analyzer)

        # Step simulation to let forces settle
        print("Letting forces settle...")
        for _ in range(100):
            p.stepSimulation()
            time.sleep(0.01)

        # Get initial force analysis
        left_force, right_force = force_analyzer.get_contact_forces()
        total_force = left_force[2] + right_force[2]
        force_vector = [
            left_force[0] + right_force[0],
            left_force[1] + right_force[1],
            left_force[2] + right_force[2]
        ]
        force_magnitude = (force_vector[0]**2 + force_vector[1]**2 + force_vector[2]**2)**0.5

        # Print force analysis in required format
        print("\nForce Analysis:")
        print(f"Total normal force: {total_force:.3f}N")
        print(f"Total force vector: [{force_vector[0]:.3f}, {force_vector[1]:.3f}, {force_vector[2]:.3f}]N")
        print(f"Force vector magnitude: {force_magnitude:.3f}N")
        print(f"Expected force (mass * g): {9.81:.3f}N\n")

        # Get and print contact points
        contact_points = p.getContactPoints(force_analyzer.robot_id)
        if contact_points:
            for i, point in enumerate(contact_points, 1):
                print(f"Contact point {i} details:")
                print(f"Position on box: [{point[5][0]:.3f}, {point[5][1]:.3f}, {point[5][2]:.3f}]")
                print(f"Position on ground: [{point[6][0]:.3f}, {point[6][1]:.3f}, {point[6][2]:.3f}]")
                print(f"Contact distance: {point[8]:.6f}\n")
        else:
            print("Warning: No contact points detected\n")

        # Verify stability
        if abs(force_magnitude - 9.81) < 0.5:
            print("Force magnitude within expected range")
            print("Box has settled!")
        else:
            print(f"Warning: Force magnitude ({force_magnitude:.3f}N) differs from expected value (9.81N)")

    except Exception as e:
        print(f"Error during simulation: {str(e)}")
        import traceback
        print(traceback.format_exc())
    finally:
        # Cleanup
        p.disconnect()
        print("\nSimulation completed")

if __name__ == "__main__":
    main()
