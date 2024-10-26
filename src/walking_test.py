import pybullet as p
import numpy as np
import time
from models.urdf_walker import URDFWalker
from models.constants import *

def run_walking_test():
    # Initialize PyBullet
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1.0/240.0)

    # Load robot and ground
    walker = URDFWalker()
    ground_id = p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, ground_id)
    robot_id = walker.load_robot()

    # Walking parameters
    step_height = 0.07   # Slightly increased step height
    step_length = 0.12   # Increased step length for better progression
    cycle_time = 0.7     # Adjusted cycle time for momentum
    steps_per_cycle = int(cycle_time * 240)
    vertical_angle = np.arccos(HEIGHT_TARGET / LEG_LENGTH)
    stance_angle = np.pi/2 - vertical_angle
    stance_width_angle = np.arcsin(INITIAL_STANCE_WIDTH / (2 * LEG_LENGTH))
    push_force = 200     # Increased force for better push-off
    base_lean = 0.08    # Base forward lean angle

    # Initialize position with slight forward lean
    p.resetBasePositionAndOrientation(robot_id, [0, 0, HEIGHT_TARGET],
                                    p.getQuaternionFromEuler([base_lean, 0, 0]))
    start_pos = np.array([0, 0, 0])
    last_height = HEIGHT_TARGET
    last_x_pos = 0
    current_velocity = 0
    stabilization_steps = 240  # Allow initial stabilization

    # Main walking loop
    for step in range(1000):
        # Initial stabilization period
        if step < stabilization_steps:
            left_angle = stance_angle + stance_width_angle
            right_angle = -(stance_angle + stance_width_angle)
            p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, left_angle, force=push_force)
            p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, right_angle, force=push_force)
            p.stepSimulation()
            continue

        phase = ((step - stabilization_steps) % steps_per_cycle) / steps_per_cycle

        # Get current state
        pos, _ = p.getBasePositionAndOrientation(robot_id)
        current_height = pos[2]
        height_change = current_height - last_height
        current_velocity = (pos[0] - last_x_pos) * 240  # Convert to m/s
        last_height = current_height
        last_x_pos = pos[0]

        # Dynamic lean adjustment based on velocity
        target_velocity = 0.2  # Target forward velocity in m/s
        velocity_error = target_velocity - current_velocity
        lean_adjustment = np.clip(velocity_error * 0.2, -0.05, 0.05)
        forward_lean = base_lean + lean_adjustment

        # Progressive COM shift with momentum transfer
        com_shift = 0.04 * np.sin(2 * np.pi * phase)
        momentum_factor = np.clip(current_velocity * 0.5, 0, 0.1)

        # Emergency recovery if height drops too quickly
        if height_change < -0.01:
            left_angle = stance_angle + stance_width_angle
            right_angle = -(stance_angle + stance_width_angle)
            p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, left_angle, force=push_force*2.0)
            p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, right_angle, force=push_force*2.0)
            continue

        # Compute leg angles with momentum-based adjustments
        if phase < 0.5:
            # Left leg swing phase with momentum transfer
            swing_phase = phase * 2
            height_l = step_height * np.sin(swing_phase * np.pi)
            forward_l = step_length * (1 - np.cos(swing_phase * np.pi)) + momentum_factor
            left_angle = stance_angle + stance_width_angle + np.arctan2(height_l, forward_l) + forward_lean
            right_angle = -(stance_angle + stance_width_angle) - forward_lean + com_shift

            # Dynamic force control based on phase and momentum
            push_timing = np.sin(swing_phase * np.pi)
            force_l = push_force * (0.9 + momentum_factor)
            force_r = push_force * (1.5 + com_shift + push_timing * 0.3)
        else:
            # Right leg swing phase with momentum transfer
            swing_phase = (phase - 0.5) * 2
            height_r = step_height * np.sin(swing_phase * np.pi)
            forward_r = step_length * (1 - np.cos(swing_phase * np.pi)) + momentum_factor
            left_angle = stance_angle + stance_width_angle + forward_lean - com_shift
            right_angle = -(stance_angle + stance_width_angle + np.arctan2(height_r, forward_r)) - forward_lean

            # Dynamic force control based on phase and momentum
            push_timing = np.sin(swing_phase * np.pi)
            force_l = push_force * (1.5 - com_shift + push_timing * 0.3)
            force_r = push_force * (0.9 + momentum_factor)

        # Apply joint positions with dynamic force control
        p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, left_angle, force=force_l)
        p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, right_angle, force=force_r)
        p.stepSimulation()

        if step % 100 == 0:
            pos, _ = p.getBasePositionAndOrientation(robot_id)
            print(f"Step {step}: Position = {pos}")
            if pos[2] < HEIGHT_TARGET * 0.5:
                print("Robot has fallen!")
                return False

    final_pos, _ = p.getBasePositionAndOrientation(robot_id)
    distance = np.linalg.norm(np.array(final_pos) - start_pos)
    print(f"Total distance walked: {distance:.3f}m")
    return distance > 0.5

if __name__ == "__main__":
    success = run_walking_test()
    print(f"Walking test {'successful' if success else 'failed'}")
