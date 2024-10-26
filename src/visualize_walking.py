python
import pybullet as p
import pybullet_data
import numpy as np
import time
from models.urdf_walker import URDFWalker
from models.constants import *
from models.cpg_controller import CPGController
import csv
from datetime import datetime

def run_walking_test(render=False):  # Default to headless mode
    # Initialize PyBullet in DIRECT mode
    client = p.connect(p.DIRECT)  # Always use DIRECT mode
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")

    # Load our robot
    walker = URDFWalker()
    robot_id = walker.load_robot()

    # Walking parameters
    push_force = 180    # Increased for better momentum
    cpg = CPGController(frequency=0.5, coupling_strength=2.0)  # Slower, stronger coupling

    # Initialize robot position
    vertical_angle = np.arccos(HEIGHT_TARGET / LEG_LENGTH)
    stance_angle = np.pi/2 - vertical_angle
    stance_width_angle = np.arcsin(INITIAL_STANCE_WIDTH / (2 * LEG_LENGTH))
    p.resetBasePositionAndOrientation(robot_id, [0, 0, HEIGHT_TARGET],
                                    p.getQuaternionFromEuler([0.03, 0, 0]))  # Increased forward lean

    # Tracking variables
    start_pos = np.array([0, 0, HEIGHT_TARGET])
    last_height = HEIGHT_TARGET
    last_pos = [0, 0, HEIGHT_TARGET]
    stabilization_steps = 480  # Reduced stabilization time
    pre_walking_steps = 120    # Shorter transition phase

    # Setup metrics logging
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    metrics_file = f"walking_metrics_{timestamp}.csv"
    metrics_headers = ['step', 'x_pos', 'height', 'height_change', 'forward_velocity',
                      'phase', 'left_force', 'right_force', 'stability_score']

    with open(metrics_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(metrics_headers)

        # Initialize force variables
        force_l = push_force
        force_r = push_force

        # Main simulation loop
        for step in range(2000):
            if step < stabilization_steps:
                # Initial stabilization phase with forward bias
                left_angle = stance_angle + stance_width_angle - 0.02
                right_angle = -(stance_angle + stance_width_angle - 0.02)
                p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, left_angle, force=push_force)
                p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, right_angle, force=push_force)
            elif step < (stabilization_steps + pre_walking_steps):
                # Pre-walking phase - gradual CPG activation
                progress = (step - stabilization_steps) / pre_walking_steps
                cpg.amplitude = 0.15 * progress  # Gradually increase amplitude
                left_angle, right_angle = cpg.get_joint_angles(stance_angle, stance_width_angle)

                # Apply gradual force increase
                current_force = push_force * (1.0 + 0.2 * progress)
                p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, left_angle, force=current_force)
                p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, right_angle, force=current_force)
            else:
                # CPG-driven walking phase
                left_angle, right_angle = cpg.get_joint_angles(stance_angle, stance_width_angle)

                # Get current state
                pos, orient = p.getBasePositionAndOrientation(robot_id)
                current_height = pos[2]
                height_change = current_height - last_height
                forward_velocity = (pos[0] - last_pos[0]) * 240  # Convert to m/s

                # Calculate stability score (0-1, lower is better)
                lin_vel, ang_vel = p.getBaseVelocity(robot_id)
                stability_score = (abs(ang_vel[1]) + abs(height_change * 10)) / 2

                # Emergency recovery if needed
                if height_change < -0.01:
                    recovery_force = push_force * 2.0
                    p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, left_angle, force=recovery_force)
                    p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, right_angle, force=recovery_force)
                else:
                    # Normal walking forces with phase-dependent modulation
                    phase = cpg.phase_left / (2 * np.pi)
                    force_l = push_force * (1.2 + 0.3 * np.sin(cpg.phase_left))
                    force_r = push_force * (1.2 + 0.3 * np.sin(cpg.phase_right))

                    p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, left_angle, force=force_l)
                    p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, right_angle, force=force_r)

                # Log metrics
                metrics = [step, pos[0], current_height, height_change, forward_velocity,
                          phase, force_l, force_r, stability_score]
                writer.writerow(metrics)

                # Update tracking variables
                last_height = current_height
                last_pos = pos

            p.stepSimulation()

            # Check if target distance reached
            pos, _ = p.getBasePositionAndOrientation(robot_id)
            if pos[0] > 0.5:
                print(f"Success! Distance walked: {pos[0]:.3f}m")
                print(f"Metrics saved to: {metrics_file}")
                break
            elif pos[2] < 0.2:  # Robot has fallen
                print(f"Failed: Robot fell. Distance: {pos[0]:.3f}m")
                print(f"Metrics saved to: {metrics_file}")
                break

        return pos[0]  # Return distance walked
if __name__ == "__main__":
    distance = run_walking_test(render=False)  # Default to headless mode
    print(f"Final distance: {distance:.3f}m")
                break

        return pos[0]  # Return distance walked
if __name__ == "__main__":
    distance = run_walking_test(render=False)  # Default to headless mode
    print(f"Final distance: {distance:.3f}m")
