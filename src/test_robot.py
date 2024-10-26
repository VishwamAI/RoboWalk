import pybullet as p
import time
import numpy as np
from models.urdf_walker import URDFWalker
from force_analysis import ForceAnalyzer

def test_robot_stability():
    print("Initializing robot stability test...")

    try:
        # Initialize PyBullet in DIRECT mode
        analyzer = ForceAnalyzer(connection_mode=p.DIRECT)

        # Reset robot and get initial stance
        initial_stance = analyzer.reset_robot()
        print(f"Initial stance angle: {initial_stance}")

        # Get initial position and orientation
        pos, orn = p.getBasePositionAndOrientation(analyzer.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        print(f"\nInitial state:")
        print(f"Position (x,y,z): {pos}")
        print(f"Orientation (roll,pitch,yaw): {[np.degrees(a) for a in euler]} degrees")

        # Run simulation for 5 seconds (500 steps at 100Hz)
        print("\nRunning stability test...")
        print("Time(s) | Left Force(N) | Right Force(N) | Total Force(N) | Height(m) | Stable")

        for i in range(500):
            # Get force readings
            left_force, right_force = analyzer.get_contact_forces()
            total_force = np.sum(left_force) + np.sum(right_force)

            # Get current height
            pos, _ = p.getBasePositionAndOrientation(analyzer.robot_id)
            current_height = pos[2]

            # Check stability
            is_stable = analyzer._check_stability()

            # Print results every 50 steps (0.5 seconds)
            if i % 50 == 0:
                time_s = i / 100.0
                print(f"{time_s:6.2f} | {np.linalg.norm(left_force):11.2f} | "
                      f"{np.linalg.norm(right_force):12.2f} | {total_force:11.2f} | "
                      f"{current_height:8.3f} | {is_stable}")

            # Step simulation
            p.stepSimulation()
            time.sleep(0.01)

        # Final measurements
        pos, orn = p.getBasePositionAndOrientation(analyzer.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        left_force, right_force = analyzer.get_contact_forces()
        total_force = np.sum(left_force) + np.sum(right_force)

        print("\nFinal measurements:")
        print(f"Position (x,y,z): {pos}")
        print(f"Orientation (roll,pitch,yaw): {[np.degrees(a) for a in euler]} degrees")
        print(f"Total ground force: {total_force:.2f}N")
        print(f"Stability status: {'Stable' if analyzer._check_stability() else 'Unstable'}")

    except Exception as e:
        print(f"Error during stability test: {str(e)}")
        raise
    finally:
        # Cleanup
        if p.isConnected():
            p.disconnect()

if __name__ == "__main__":
    test_robot_stability()
