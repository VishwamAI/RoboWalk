import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
from force_analysis import ForceAnalyzer
from models.cpg_controller import CPGController
import time
from datetime import datetime

def analyze_walking_forces(duration=5.0, render=False):
    print("Initializing walking analysis...")

    # Initialize analyzer
    analyzer = ForceAnalyzer()
    cpg = CPGController(frequency=0.5, coupling_strength=2.0)

    # Setup data collection
    timestep = 1.0/240.0
    total_steps = int(duration / timestep)

    # Data arrays
    times = np.zeros(total_steps)
    left_forces = np.zeros((total_steps, 3))
    right_forces = np.zeros((total_steps, 3))
    com_positions = np.zeros((total_steps, 3))
    cpg_phases = np.zeros((total_steps, 2))  # Store left and right phases

    # Initial setup
    stance_angle = analyzer.reset_robot()
    stance_width_angle = np.arcsin(0.1 / (2 * 0.5))  # From constants

    print("Starting simulation loop...")
    print(f"Initial stance angle: {stance_angle:.3f} rad")
    print(f"Stance width angle: {stance_width_angle:.3f} rad")

    # Stability check parameters
    max_height = 0.6  # Maximum allowed height
    min_height = 0.3  # Minimum allowed height
    max_tilt = 0.5    # Maximum allowed tilt in radians

    # Initial stabilization phase
    print("\nRunning initial stabilization...")
    for _ in range(480):  # 2 seconds of stabilization
        p.setJointMotorControl2(analyzer.robot_id, 0, p.POSITION_CONTROL,
                              stance_angle, force=500, maxVelocity=0.5)
        p.setJointMotorControl2(analyzer.robot_id, 1, p.POSITION_CONTROL,
                              -stance_angle, force=500, maxVelocity=0.5)
        p.stepSimulation()
        if _ % 120 == 0:
            com_pos = analyzer.get_com_position()
            print(f"Stabilization height: {com_pos[2]:.3f}m")

    # Main simulation loop
    for step in range(total_steps):
        if step % 240 == 0:  # Print progress every second
            print(f"Simulation progress: {(step/total_steps)*100:.1f}%")

        # Calculate force ramp-up factor (gradual increase over 2 seconds)
        force_factor = min(1.0, step / (480.0))
        current_force = 180 + (320 * force_factor)  # Ramp up from 180N to 500N

        # Get CPG signals and phases
        left_angle, right_angle = cpg.get_joint_angles(stance_angle, stance_width_angle)
        left_phase, right_phase = cpg.get_phases()

        # Debug output for every 240 steps (1 second)
        if step % 240 == 0:
            print(f"\nStep {step}:")
            print(f"CPG Phases - Left: {left_phase:.2f}, Right: {right_phase:.2f}")
            print(f"Joint Angles - Left: {left_angle:.2f}, Right: {right_angle:.2f}")
            print(f"Current force: {current_force:.1f}N")

        # Apply joint positions with ramped force
        p.setJointMotorControl2(analyzer.robot_id, 0, p.POSITION_CONTROL,
                              left_angle, force=current_force, maxVelocity=1.0)
        p.setJointMotorControl2(analyzer.robot_id, 1, p.POSITION_CONTROL,
                              right_angle, force=current_force, maxVelocity=1.0)

        # Step simulation
        p.stepSimulation()
        # Collect data
        left_force, right_force = analyzer.get_contact_forces()
        com_pos = analyzer.get_com_position()

        # Check simulation stability
        _, orientation = p.getBasePositionAndOrientation(analyzer.robot_id)
        euler = p.getEulerFromQuaternion(orientation)

        if (com_pos[2] > max_height or com_pos[2] < min_height or
            abs(euler[0]) > max_tilt or abs(euler[1]) > max_tilt):
            print("\nSimulation stability check failed!")
            print(f"Height: {com_pos[2]:.3f}m")
            print(f"Tilt (roll, pitch): ({euler[0]:.3f}, {euler[1]:.3f}) rad")
            break

        # Store data
        times[step] = step * timestep
        left_forces[step] = left_force
        right_forces[step] = right_force
        com_positions[step] = com_pos
        cpg_phases[step] = [left_phase, right_phase]

        # Debug force information every second
        if step % 240 == 0:
            print("\nForce Analysis:")
            print(f"Left leg forces (N) - X: {left_force[0]:.2f}, Y: {left_force[1]:.2f}, Z: {left_force[2]:.2f}")
            print(f"Right leg forces (N) - X: {right_force[0]:.2f}, Y: {right_force[1]:.2f}, Z: {right_force[2]:.2f}")
            print(f"COM position (m) - X: {com_pos[0]:.3f}, Y: {com_pos[1]:.3f}, Z: {com_pos[2]:.3f}")

    print("\nGenerating analysis plots...")

    # Create visualization
    plt.figure(figsize=(15, 12))

    # Plot vertical forces
    plt.subplot(4, 1, 1)
    plt.plot(times, left_forces[:, 2], label='Left Leg Z-Force')
    plt.plot(times, right_forces[:, 2], label='Right Leg Z-Force')
    plt.title('Vertical Ground Reaction Forces')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.legend()
    plt.grid(True)

    # Plot horizontal forces
    plt.subplot(4, 1, 2)
    plt.plot(times, left_forces[:, 0], label='Left Leg X-Force')
    plt.plot(times, right_forces[:, 0], label='Right Leg X-Force')
    plt.title('Horizontal Ground Reaction Forces')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.legend()
    plt.grid(True)

    # Plot COM trajectory
    plt.subplot(4, 1, 3)
    plt.plot(times, com_positions[:, 0], label='X Position')
    plt.plot(times, com_positions[:, 2], label='Z Height')
    plt.title('Center of Mass Trajectory')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid(True)

    # Plot CPG phases
    plt.subplot(4, 1, 4)
    plt.plot(times, cpg_phases[:, 0], label='Left Phase')
    plt.plot(times, cpg_phases[:, 1], label='Right Phase')
    plt.title('CPG Phase Evolution')
    plt.xlabel('Time (s)')
    plt.ylabel('Phase')
    plt.legend()
    plt.grid(True)

    # Save plot
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plt.tight_layout()
    plt.savefig(f'walking_analysis_{timestamp}.png')

    # Print summary statistics
    print("\nWalking Analysis Summary:")
    print(f"Average vertical force - Left: {np.mean(left_forces[:, 2]):.2f}N, Right: {np.mean(right_forces[:, 2]):.2f}N")
    print(f"Average horizontal force - Left: {np.mean(left_forces[:, 0]):.2f}N, Right: {np.mean(right_forces[:, 0]):.2f}N")
    print(f"Final X position: {com_positions[-1, 0]:.3f}m")
    print(f"Height variation: {np.std(com_positions[:, 2]):.3f}m")
    print(f"Average forward velocity: {com_positions[-1, 0]/times[-1]:.3f} m/s")
    print(f"Phase coordination: {np.mean(np.abs(cpg_phases[:, 0] - cpg_phases[:, 1])):.3f} rad")

if __name__ == "__main__":
    analyze_walking_forces(duration=5.0)
