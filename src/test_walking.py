import pybullet as p
import time
import numpy as np
from force_analysis import ForceAnalyzer
from walking_controller import WalkingController

def main():
    # Initialize PyBullet in DIRECT mode for headless environment
    physicsClient = p.connect(p.DIRECT)
    print(f"Connected to PyBullet with client ID: {physicsClient}")

    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)  # Disable real-time simulation for better control

    # Initialize force analyzer and walking controller
    analyzer = ForceAnalyzer()
    analyzer.reset_robot()  # This loads the robot and sets initial pose

    # Wait for initial stabilization
    print("\nInitial stabilization phase...")
    stable_count = 0
    max_stable_count = 10  # Number of consecutive stable readings needed

    for step in range(1000):  # About 4 seconds of simulation time
        p.stepSimulation()

        # Get force readings
        left_force, right_force = analyzer.get_contact_forces()
        total_force = left_force[2] + right_force[2]

        # Check stability
        if abs(left_force[2] - right_force[2]) < 500 and total_force > 8000:  # Roughly balanced forces
            stable_count += 1
        else:
            stable_count = 0

        if step % 60 == 0:  # Print every ~0.25 seconds
            print(f"\nStep {step}:")
            print(f"Total vertical force: {total_force:.2f}N")
            print(f"Left force: {left_force[2]:.2f}N")
            print(f"Right force: {right_force[2]:.2f}N")
            print(f"Stability counter: {stable_count}/{max_stable_count}")

        if stable_count >= max_stable_count:
            print("\nInitial stability achieved!")
            break

    # Initialize walking controller
    walker = WalkingController(analyzer)

    print("\nStarting walking sequence...")
    start_time = time.time()
    try:
        duration = 10.0  # 10 seconds of walking

        while time.time() - start_time < duration:
            t = time.time() - start_time
            walker.apply_joint_positions(t)
            p.stepSimulation()

            # Get force feedback
            left_force, right_force = analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]

            # Calculate center of pressure (CoP)
            left_pos = np.array(p.getLinkState(analyzer.robot_id, 2)[0])
            right_pos = np.array(p.getLinkState(analyzer.robot_id, 5)[0])
            cop_x = (left_pos[0] * left_force[2] + right_pos[0] * right_force[2]) / (total_force + 1e-6)

            # Print detailed status every 0.5 seconds
            if int(t * 2) % 1 == 0:
                print(f"\nTime: {t:.1f}s")
                print(f"Total vertical force: {total_force:.2f}N")
                print(f"Left/Right force ratio: {left_force[2]/(total_force + 1e-6):.2f}")
                print(f"Center of Pressure X: {cop_x:.3f}m")

                # Get robot state
                base_pos, base_orn = p.getBasePositionAndOrientation(analyzer.robot_id)
                print(f"Robot position: x={base_pos[0]:.3f}, y={base_pos[1]:.3f}, z={base_pos[2]:.3f}")

            time.sleep(1./240.)

    except KeyboardInterrupt:
        print("\nWalking sequence interrupted by user")
    finally:
        # Print final statistics
        final_time = time.time() - start_time
        base_pos, _ = p.getBasePositionAndOrientation(analyzer.robot_id)
        print("\nFinal Statistics:")
        print(f"Total simulation time: {final_time:.1f}s")
        print(f"Total distance traveled: {base_pos[0]:.3f}m")

        p.disconnect()
        print("\nSimulation ended")

if __name__ == "__main__":
    main()
