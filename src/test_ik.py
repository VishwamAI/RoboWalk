import pybullet as p
import pybullet_data
import numpy as np
import time
import os

def calculate_ik(target_pos, leg_side="left"):
    """Calculate inverse kinematics for a leg.

    Args:
        target_pos: [x, y, z] target foot position in world frame
        leg_side: "left" or "right" to determine hip offset

    Returns:
        tuple: (hip_angle, knee_angle) in radians
    """
    # Robot dimensions from debug_links.py
    hip_offset = 0.15 if leg_side == "right" else -0.15
    upper_leg = 0.4  # Upper leg length
    lower_leg = 0.4  # Lower leg length

    # Transform target to leg's local frame
    # Origin at hip joint, +x forward, +y outward, +z up
    local_x = target_pos[0]  # Forward/back
    local_y = target_pos[1] - hip_offset  # Lateral distance from hip
    local_z = target_pos[2]  # Height (negative is down)

    print(f"\nTarget in leg frame: [{local_x:.3f}, {local_y:.3f}, {local_z:.3f}]")

    # First handle lateral offset with hip abduction
    # Project into frontal (YZ) plane
    yz_dist = np.sqrt(local_y**2 + local_z**2)
    print(f"YZ plane distance: {yz_dist:.3f}m")

    # Calculate hip abduction to handle lateral offset
    hip_abduction = np.arctan2(local_y, -local_z)  # Negative z because down is negative
    hip_abduction = np.clip(hip_abduction, -0.4, 0.4)
    print(f"Hip abduction: {np.degrees(hip_abduction):.1f}°")

    # Project leg length into sagittal plane after abduction
    # This is the effective working plane for hip flexion and knee angle
    eff_z = np.sqrt(yz_dist**2 - (yz_dist * np.sin(hip_abduction))**2)
    print(f"Effective height in sagittal plane: {eff_z:.3f}m")

    # Now solve sagittal plane (XZ) angles
    sagittal_dist = np.sqrt(local_x**2 + eff_z**2)
    print(f"Sagittal plane distance: {sagittal_dist:.3f}m")

    if sagittal_dist > (upper_leg + lower_leg):
        raise ValueError(f"Target out of reach: {sagittal_dist:.3f}m > {upper_leg + lower_leg:.3f}m")
    elif sagittal_dist < abs(upper_leg - lower_leg):
        raise ValueError(f"Target too close: {sagittal_dist:.3f}m < {abs(upper_leg - lower_leg):.3f}m")

    # Use law of cosines to find knee angle
    cos_knee = (upper_leg**2 + lower_leg**2 - sagittal_dist**2) / (2 * upper_leg * lower_leg)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    knee_angle = -(np.pi - np.arccos(cos_knee))  # Negative for backward bend
    knee_angle = np.clip(knee_angle, -0.4, 0.4)
    print(f"Knee angle: {np.degrees(knee_angle):.1f}°")

    # Calculate hip flexion in sagittal plane
    # First find angle to target in sagittal plane
    target_angle = np.arctan2(local_x, -eff_z)  # Angle from vertical to target
    print(f"Target angle from vertical: {np.degrees(target_angle):.1f}°")

    # Then find angle of upper leg using law of cosines
    cos_hip = (upper_leg**2 + sagittal_dist**2 - lower_leg**2) / (2 * upper_leg * sagittal_dist)
    cos_hip = np.clip(cos_hip, -1.0, 1.0)
    leg_angle = np.arccos(cos_hip)
    hip_flexion = target_angle + leg_angle  # Add because we want forward angle from vertical
    hip_flexion = np.clip(hip_flexion, -0.4, 0.4)
    print(f"Hip flexion: {np.degrees(hip_flexion):.1f}°")

    # Verify solution by calculating achieved position
    # First calculate position in sagittal plane
    sag_x = upper_leg * np.sin(hip_flexion) + lower_leg * np.sin(hip_flexion + knee_angle)
    sag_z = -(upper_leg * np.cos(hip_flexion) + lower_leg * np.cos(hip_flexion + knee_angle))

    # Then transform back to world frame
    achieved_x = sag_x
    achieved_y = hip_offset + yz_dist * np.sin(hip_abduction)  # Transform back to world frame
    achieved_z = sag_z * np.cos(hip_abduction)

    error = np.sqrt((achieved_x - target_pos[0])**2 +
                   (achieved_y - target_pos[1])**2 +
                   (achieved_z - target_pos[2])**2)
    print(f"Achieved in world frame: [{achieved_x:.3f}, {achieved_y:.3f}, {achieved_z:.3f}]")
    print(f"Error: {error*1000:.1f}mm")

    if error > 0.01:  # 1cm tolerance
        raise ValueError(f"Solution error too large: {error:.3f}m")

    return hip_abduction, hip_flexion, knee_angle

def main():
    # Initialize PyBullet
    p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load robot
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "models/simple_biped_stable.urdf")
    robot_id = p.loadURDF(urdf_path)

    # Calculate conservative standing height
    # With 0.2 rad bend at both hip and knee:
    # Upper leg contribution: 0.4m * cos(0.2) ≈ 0.394m
    # Lower leg contribution: 0.4m * cos(0.2) ≈ 0.394m
    # Total height = 0.788m
    leg_angle = 0.1  # Start with just 0.1 rad bend for near-vertical stance
    standing_height = -0.75  # About 75% of full extension (0.8m)
    step_height = standing_height + 0.005  # Lift foot by 5mm during step

    # Test positions
    test_positions = [
        # Near-vertical standing position
        ([0.0, 0.15, standing_height], "right"),
        ([0.0, -0.15, standing_height], "left"),
        # Very slight forward lean (5mm)
        ([0.005, 0.15, standing_height], "right"),
        ([0.005, -0.15, standing_height], "left"),
        # Small sideways adjustment (5mm)
        ([0.0, 0.155, standing_height], "right"),
        ([0.0, -0.155, standing_height], "left"),
        # Minimal step motion (5mm up, 5mm forward)
        ([0.005, 0.15, step_height], "right"),
        ([0.005, -0.15, step_height], "left")
    ]

    print("\nTesting IK calculations:")
    print("-" * 50)
    print(f"Standing height: {standing_height:.3f}m")
    print(f"Step height: {step_height:.3f}m")
    print(f"Using {np.degrees(leg_angle):.1f}° initial bend")
    print("-" * 50)

    for pos, side in test_positions:
        try:
            hip_angle, knee_angle = calculate_ik(pos, side)
            print(f"\n{side.title()} leg target: {pos}")
            print(f"Hip angle: {hip_angle:.3f} rad ({np.degrees(hip_angle):.1f}°)")
            print(f"Knee angle: {knee_angle:.3f} rad ({np.degrees(knee_angle):.1f}°)")

            # Apply joint angles
            if side == "left":
                p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, hip_angle)
                p.setJointMotorControl2(robot_id, 1, p.POSITION_CONTROL, knee_angle)
            else:
                p.setJointMotorControl2(robot_id, 3, p.POSITION_CONTROL, hip_angle)
                p.setJointMotorControl2(robot_id, 4, p.POSITION_CONTROL, knee_angle)

            p.stepSimulation()

        except ValueError as e:
            print(f"\n{side.title()} leg target: {pos}")
            print(f"Error: {str(e)}")

    p.disconnect()

if __name__ == "__main__":
    main()
