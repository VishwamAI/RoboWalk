import pybullet as p
import time
import numpy as np
from force_analysis import ForceAnalyzer
from walking_controller_fixed import WalkingController
import csv
from datetime import datetime

def main():
    # Initialize PyBullet in DIRECT mode for faster simulation
    p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)

    # Create ground plane with increased friction
    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)
    p.changeDynamics(0, -1, lateralFriction=1.0)

    # Initialize force analyzer and walking controller
    analyzer = ForceAnalyzer()
    controller = WalkingController(analyzer)

    # Prepare CSV logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = f"walking_test_{timestamp}.csv"
    csv_file = open(csv_filename, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['Time', 'Pitch', 'Total_Force', 'Force_Ratio', 'Left_Force', 'Right_Force', 'Phase'])

    try:
        print("\nStarting walking test...")
        start_time = time.time()
        test_duration = 10.0  # 10 seconds test

        # Start walking sequence
        controller.start_walking(test_duration)

        # Log final statistics
        print("\nTest completed. Analyzing results...")
        csv_file.close()

        # Load and analyze results
        with open(csv_filename, 'r') as f:
            lines = f.readlines()[1:]  # Skip header
            if lines:
                data = [line.strip().split(',') for line in lines]
                force_ratios = [float(line[3]) for line in data]
                total_forces = [float(line[2]) for line in data]
                pitches = [float(line[1]) for line in data]

                print("\nTest Statistics:")
                print(f"Average force ratio: {np.mean(force_ratios):.3f}")
                print(f"Min/Max force ratio: {min(force_ratios):.3f}/{max(force_ratios):.3f}")
                print(f"Average total force: {np.mean(total_forces):.1f}N")
                print(f"Min/Max total force: {min(total_forces):.1f}N/{max(total_forces):.1f}N")
                print(f"Average pitch: {np.mean(pitches):.1f}°")
                print(f"Min/Max pitch: {min(pitches):.1f}°/{max(pitches):.1f}°")

    except Exception as e:
        print(f"Test failed: {str(e)}")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()
