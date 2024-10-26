import pybullet as p
import numpy as np
import time
from force_analysis import ForceAnalyzer

def main():
    # Initialize PyBullet in GUI mode
    try:
        if p.isConnected():
            p.disconnect()
    except p.error:
        pass  # Ignore if no existing connection

    # Start in DIRECT mode by default
    client = p.connect(p.DIRECT)
    is_gui = False
    print("Running in DIRECT mode for force analysis")

    if client < 0:
        print("Failed to connect to PyBullet")
        return

    # Basic visualization settings (only in GUI mode)
    if is_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)

    # Create force analyzer with appropriate mode
    analyzer = ForceAnalyzer(connection_mode=p.GUI if is_gui else p.DIRECT)

    # Reset robot and get initial stance
    stance_angle = analyzer.reset_robot()

    # Initialize debug parameters and visualization only in GUI mode
    if is_gui:
        height_slider = p.addUserDebugParameter("Robot Height", 0.1, 0.5, 0.25)
        force_slider = p.addUserDebugParameter("Joint Force", 0, 2000, 1000)
        lean_slider = p.addUserDebugParameter("Forward Lean", -0.1, 0.1, 0.0)

        # Create text overlay for force display
        force_text = p.addUserDebugText("", [0, 0, 0], [0, 0, 0])

        # Add coordinate system visualization
        p.addUserDebugLine([0, 0, 0], [1, 0, 0], [1, 0, 0], 2.0)  # X axis (red)
        p.addUserDebugLine([0, 0, 0], [0, 1, 0], [0, 1, 0], 2.0)  # Y axis (green)
        p.addUserDebugLine([0, 0, 0], [0, 0, 1], [0, 0, 1], 2.0)  # Z axis (blue)
    else:
        # Default values for DIRECT mode
        height = 0.25
        force = 1000
        lean = 0.0
        force_text = None

    print("\n=== Debug Analysis Started ===")
    print(f"Initial stance angle: {stance_angle:.3f} rad")
    print("\nRobot Configuration:")
    print(f"- Initial height: {height:.3f}m")
    print(f"- Joint force: {force:.1f}N")
    print(f"- Forward lean: {lean:.3f}rad")
    print("\nForce Analysis:")
    print("- Contact forces will be logged every step")
    print("- Press Ctrl+C to exit")

    try:
        step_count = 0
        while True:
            # Update robot parameters from sliders if in GUI mode
            if is_gui:
                height = p.readUserDebugParameter(height_slider)
                force = p.readUserDebugParameter(force_slider)
                lean = p.readUserDebugParameter(lean_slider)

            # Get current forces with detailed debug output
            left_force, right_force = analyzer.get_contact_forces()
            total_vertical_force = left_force[2] + right_force[2]

            # Print detailed force analysis every 60 steps (4 times per second)
            if step_count % 60 == 0:
                print("\n=== Force Analysis at Step", step_count, "===")
                print(f"Left Foot Forces:")
                print(f"  Vector: [{left_force[0]:.3f}, {left_force[1]:.3f}, {left_force[2]:.3f}]N")
                print(f"  Magnitude: {np.linalg.norm(left_force):.3f}N")
                print(f"Right Foot Forces:")
                print(f"  Vector: [{right_force[0]:.3f}, {right_force[1]:.3f}, {right_force[2]:.3f}]N")
                print(f"  Magnitude: {np.linalg.norm(right_force):.3f}N")
                print(f"Total Vertical Force: {total_vertical_force:.3f}N")

            # Update force display with more detailed information (GUI mode only)
            if is_gui:
                text_pos = p.getBasePositionAndOrientation(analyzer.robot_id)[0]
                text_pos = [text_pos[0], text_pos[1], text_pos[2] + 0.5]
                p.removeUserDebugItem(force_text)
                force_text = p.addUserDebugText(
                    f"Height: {height:.3f}m\n"
                    f"Force: {force:.1f}N\n"
                    f"Lean: {lean:.3f}rad\n"
                    f"Total Vertical Force: {total_vertical_force:.2f}N\n"
                    f"Left Force: [{left_force[0]:.2f}, {left_force[1]:.2f}, {left_force[2]:.2f}]N\n"
                    f"Right Force: [{right_force[0]:.2f}, {right_force[1]:.2f}, {right_force[2]:.2f}]N",
                    text_pos,
                    [1, 1, 1]  # White text for better visibility
                )

            # Update robot position and orientation
            base_pos = [0, 0, height]
            base_orn = p.getQuaternionFromEuler([lean, 0, 0])
            p.resetBasePositionAndOrientation(analyzer.robot_id, base_pos, base_orn)

            # Update joint forces with stance angle
            p.setJointMotorControl2(
                analyzer.robot_id, 0,
                p.POSITION_CONTROL,
                targetPosition=stance_angle,
                force=force,
                maxVelocity=0.5
            )
            p.setJointMotorControl2(
                analyzer.robot_id, 1,
                p.POSITION_CONTROL,
                targetPosition=-stance_angle,
                force=force,
                maxVelocity=0.5
            )

            # Step simulation and increment counter
            p.stepSimulation()
            step_count += 1
            if step_count % 60 == 0:  # Add small delay between debug outputs
                time.sleep(0.25)
            else:
                time.sleep(1./240.)

    except KeyboardInterrupt:
        print("\nExiting debug visualization...")
        p.disconnect()

if __name__ == "__main__":
    main()
