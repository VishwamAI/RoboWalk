import pybullet as p
import time
import numpy as np
import os
from force_analysis import ForceAnalyzer
from walking_controller import WalkingController

def test_stable_pose():
    try:
        # Initialize PyBullet in DIRECT mode for headless operation
        if p.isConnected():
            p.disconnect()
        physicsClient = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0/240.0)  # Match ForceAnalyzer timestep
        p.setRealTimeSimulation(0)

        # Initialize force analyzer with the stable URDF model
        print("\nInitializing ForceAnalyzer...")
        analyzer = ForceAnalyzer(connection_mode=p.DIRECT)  # ForceAnalyzer creates its own ground plane

        # Create walking controller
        print("\nInitializing WalkingController...")
        controller = WalkingController(analyzer)

        print("\nInitial reset to stable pose...")
        controller.reset_to_stable_pose()

        # Run simulation with detailed monitoring
        print("\nRunning stability test...")
        history = {
            'left_forces': [],
            'right_forces': [],
            'heights': [],
            'pitches': [],
            'total_forces': [],
            'force_ratios': []
        }

        stabilization_count = 0
        max_steps = 300  # 300 steps = 3 seconds at default timestep

        for i in range(max_steps):
            p.stepSimulation()

            # Get force data with error checking
            try:
                left_force, right_force = analyzer.get_contact_forces()
                total_force = left_force[2] + right_force[2]
                force_ratio = min(left_force[2], right_force[2]) / max(left_force[2], right_force[2]) if max(left_force[2], right_force[2]) > 0 else 0
            except Exception as e:
                print(f"\nError getting force data: {str(e)}")
                break

            # Get robot pose with error checking
            try:
                pos, orn = p.getBasePositionAndOrientation(controller.robot_id)
                euler = p.getEulerFromQuaternion(orn)
            except Exception as e:
                print(f"\nError getting pose data: {str(e)}")
                break

            # Store data
            history['left_forces'].append(left_force[2])
            history['right_forces'].append(right_force[2])
            history['heights'].append(pos[2])
            history['pitches'].append(euler[1])
            history['total_forces'].append(total_force)
            history['force_ratios'].append(force_ratio)

            # Print detailed status every 50 steps
            if i % 50 == 0:
                print(f"\nStep {i}:")
                print(f"Forces (N):")
                print(f"  Left foot:  {left_force[2]:.1f}")
                print(f"  Right foot: {right_force[2]:.1f}")
                print(f"  Total:      {total_force:.1f}")
                print(f"  Force ratio: {force_ratio:.2f}")
                print(f"Position:")
                print(f"  Height: {pos[2]:.3f}m (target: {controller.base_height:.3f}m)")
                print(f"  Pitch:  {euler[1]:.3f}rad")
                print(f"  Roll:   {euler[0]:.3f}rad")
                print(f"  X-pos:  {pos[0]:.3f}m")
                print(f"  Y-pos:  {pos[1]:.3f}m")

            # Check stability criteria
            if i >= 200:  # After 2 seconds
                is_stable = True
                if total_force < controller.min_total_force:
                    print(f"\nWarning: Total force too low ({total_force:.1f}N)")
                    is_stable = False
                if left_force[2] < controller.min_stable_force or right_force[2] < controller.min_stable_force:
                    print(f"\nWarning: Uneven force distribution - L: {left_force[2]:.1f}N, R: {right_force[2]:.1f}N")
                    is_stable = False
                if abs(euler[1]) > controller.max_pitch:
                    print(f"\nWarning: Excessive pitch angle ({euler[1]:.3f}rad)")
                    is_stable = False
                if is_stable:
                    stabilization_count += 1
                else:
                    stabilization_count = 0

                if stabilization_count >= 50:  # Stable for 50 consecutive steps
                    print("\nStable pose achieved!")
                    break

        # Final analysis
        print("\nFinal Statistics:")
        print(f"Average height: {np.mean(history['heights']):.3f}m")
        print(f"Height variation: ±{np.std(history['heights']):.3f}m")
        print(f"Average pitch: {np.mean(history['pitches']):.3f}rad")
        print(f"Average total force: {np.mean(history['total_forces'][-50:]):.1f}N")
        print(f"Average force ratio: {np.mean(history['force_ratios'][-50:]):.2f}")
        print(f"Force balance:")
        print(f"  Left foot:  {np.mean(history['left_forces'][-50:]):.1f}N ±{np.std(history['left_forces'][-50:]):.1f}N")
        print(f"  Right foot: {np.mean(history['right_forces'][-50:]):.1f}N ±{np.std(history['right_forces'][-50:]):.1f}N")

    except Exception as e:
        print(f"\nError in test_stable_pose: {str(e)}")
    finally:
        p.disconnect()

if __name__ == "__main__":
    test_stable_pose()
