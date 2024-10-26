import pybullet as p
import time
import numpy as np
import csv
from datetime import datetime
from walking_controller import WalkingController
from force_analysis import ForceAnalyzer

def main():
    # Initialize simulation
    p.connect(p.DIRECT)  # Use DIRECT mode for headless operation
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)

    # Create ground plane with good friction
    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)

    # Initialize force analyzer and walking controller
    force_analyzer = ForceAnalyzer(connection_mode=p.DIRECT)
    controller = WalkingController(force_analyzer)

    # Create CSV file for logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"walking_data_{timestamp}.csv"
    csv_file = open(csv_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Time', 'Left_Force', 'Right_Force', 'Total_Force', 'Force_Ratio',
                        'Base_Height', 'Pitch', 'Roll', 'Yaw', 'X_Pos', 'Y_Pos', 'Z_Pos'])

    # Reset to stable pose and wait for settling
    print("Resetting to stable pose...")
    controller.reset_to_stable_pose()

    # Let the robot settle in stable pose with monitoring
    print("\nSettling phase:")
    for i in range(240):  # 1 second settling time
        p.stepSimulation()
        if i % 60 == 0:  # Monitor every 0.25 seconds
            left_force, right_force = force_analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]
            force_ratio = min(left_force[2], right_force[2]) / (max(left_force[2], right_force[2]) + 1e-6)
            print(f"Settling - Total force: {total_force:.1f}N, Force ratio: {force_ratio:.2f}")

    print("\nStarting walking sequence...")
    start_time = time.time()
    simulation_time = 0

    # Initialize monitoring variables
    min_force = float('inf')
    max_force = 0
    min_ratio = 1.0
    max_pitch = 0

    try:
        # Run simulation for 10 seconds
        while simulation_time < 10:
            # Update simulation time
            simulation_time = time.time() - start_time

            # Start walking motion
            controller.start_walking(simulation_time)

            # Step simulation
            p.stepSimulation()

            # Get force readings and pose for monitoring
            left_force, right_force = force_analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]
            force_ratio = min(left_force[2], right_force[2]) / (max(left_force[2], right_force[2]) + 1e-6)

            # Get base pose
            pos, orn = p.getBasePositionAndOrientation(force_analyzer.robot_id)
            euler = p.getEulerFromQuaternion(orn)

            # Log data to CSV
            csv_writer.writerow([
                simulation_time,
                left_force[2],
                right_force[2],
                total_force,
                force_ratio,
                pos[2],
                euler[1],  # pitch
                euler[0],  # roll
                euler[2],  # yaw
                pos[0],    # x
                pos[1],    # y
                pos[2]     # z
            ])

            # Update monitoring stats
            min_force = min(min_force, total_force)
            max_force = max(max_force, total_force)
            min_ratio = min(min_ratio, force_ratio)
            max_pitch = max(max_pitch, abs(euler[1]))

            # Monitor stability every 0.1 seconds (more frequent updates)
            if int(simulation_time * 100) % 10 == 0:
                print(f"\nTime: {simulation_time:.2f}s")
                print(f"Total force: {total_force:.1f}N")
                print(f"Left foot: {left_force[2]:.1f}N")
                print(f"Right foot: {right_force[2]:.1f}N")
                print(f"Force ratio: {force_ratio:.2f}")
                print(f"Base height: {pos[2]:.3f}m")
                print(f"Pitch: {np.degrees(euler[1]):.1f}°")
                print(f"Roll: {np.degrees(euler[0]):.1f}°")
                print(f"Lateral position: {pos[1]:.3f}m")

    except Exception as e:
        print(f"Error during walking: {str(e)}")
    finally:
        # Print summary statistics
        print("\nWalking Test Summary:")
        print(f"Minimum total force: {min_force:.1f}N")
        print(f"Maximum total force: {max_force:.1f}N")
        print(f"Minimum force ratio: {min_ratio:.2f}")
        print(f"Maximum pitch: {np.degrees(max_pitch):.1f}°")
        csv_file.close()
        p.disconnect()

if __name__ == "__main__":
    main()
