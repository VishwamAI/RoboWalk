import numpy as np
import matplotlib.pyplot as plt
import os

def validate_leg_geometry(upper_leg, lower_leg, total_dist, target_angle):
    """Validate that target is reachable and angles are possible"""
    max_reach = upper_leg + lower_leg
    min_reach = abs(upper_leg - lower_leg)

    if total_dist > max_reach:
        raise ValueError(f"Target too far: {total_dist:.3f}m > max reach {max_reach:.3f}m")
    if total_dist < min_reach:
        raise ValueError(f"Target too close: {total_dist:.3f}m < min reach {min_reach:.3f}m")

    # For standing positions, skip angle validation
    if total_dist >= max_reach * 0.99:  # Within 1% of max reach
        return

    # Check if target angle would require impossible joint angles
    max_angle = np.arcsin((max_reach - 0.01) / total_dist)  # Slight reduction to avoid numerical issues
    if abs(target_angle) > max_angle:
        raise ValueError(f"Target angle {np.degrees(target_angle):.1f}° exceeds maximum possible {np.degrees(max_angle):.1f}°")

def plot_leg_geometry(hip_pos, knee_pos, ankle_pos, target_pos, hip_angle, knee_angle, filename):
    """Plot the leg geometry and target position with angle annotations"""
    plt.figure(figsize=(6, 8))
    # Plot leg segments
    plt.plot([hip_pos[0], knee_pos[0]], [hip_pos[1], knee_pos[1]], 'b-', label='Upper Leg')
    plt.plot([knee_pos[0], ankle_pos[0]], [knee_pos[1], ankle_pos[1]], 'g-', label='Lower Leg')
    # Plot points
    plt.plot(hip_pos[0], hip_pos[1], 'ko', label='Hip')
    plt.plot(knee_pos[0], knee_pos[1], 'ko', label='Knee')
    plt.plot(ankle_pos[0], ankle_pos[1], 'ko', label='Ankle')
    plt.plot(target_pos[0], target_pos[1], 'rx', label='Target')

    # Add angle annotations
    plt.annotate(f'Hip: {np.degrees(hip_angle):.1f}°',
                xy=(0, 0), xytext=(-0.2, 0.1))
    plt.annotate(f'Knee: {np.degrees(knee_angle):.1f}°',
                xy=(knee_pos[0], knee_pos[1]), xytext=(knee_pos[0]-0.2, knee_pos[1]+0.1))

    plt.grid(True)
    plt.axis('equal')
    plt.legend()

    # Save plot to file
    os.makedirs('plots', exist_ok=True)
    plt.savefig(f'plots/{filename}.png')
    plt.close()


def calculate_2d_ik(target_x, target_z, upper_leg=0.4, lower_leg=0.4, visualize=True):
    """
    Calculate inverse kinematics in 2D (sagittal plane only)
    target_x: forward distance from hip (positive forward)
    target_z: vertical distance from hip (negative down)
    """
    # Calculate total distance and angle to target
    total_dist = np.sqrt(target_x**2 + target_z**2)
    target_angle = np.arctan2(target_x, -target_z)  # Angle from vertical
    print(f"\nTarget: [{target_x:.3f}, {target_z:.3f}]")
    print(f"Distance to target: {total_dist:.3f}m")
    print(f"Target angle from vertical: {np.degrees(target_angle):.1f}°")

    # Validate geometry is possible
    validate_leg_geometry(upper_leg, lower_leg, total_dist, target_angle)

    # Step 1: Calculate effective leg length needed
    # Account for the fact that knee bend reduces total reach
    max_length = upper_leg + lower_leg
    target_ratio = total_dist / max_length
    # Adjust ratio to account for knee bend effect on Z position
    adjusted_ratio = target_ratio + 0.02  # Compensate for Z-axis offset
    adjusted_ratio = np.clip(adjusted_ratio, 0.7, 1.0)
    print(f"Target ratio: {target_ratio:.3f}")
    print(f"Adjusted ratio: {adjusted_ratio:.3f}")

    # Step 2: Calculate knee angle based on adjusted ratio
    # Map ratio 0.7-1.0 to knee angle -0.4-0.0
    knee_angle = -0.4 * (1.0 - (adjusted_ratio - 0.7) / 0.3)
    knee_angle = np.clip(knee_angle, -0.4, 0.0)
    print(f"Knee angle: {np.degrees(knee_angle):.1f}°")

    # Step 3: Calculate effective leg vector with knee bent
    # First calculate knee position with zero hip angle
    knee_x = upper_leg * np.sin(0)
    knee_z = -upper_leg * np.cos(0)
    # Calculate ankle position relative to knee
    ankle_x = knee_x + lower_leg * np.sin(knee_angle)
    ankle_z = knee_z - lower_leg * np.cos(knee_angle)
    # Calculate leg vector orientation
    leg_angle = np.arctan2(ankle_x, -ankle_z)
    print(f"Leg vector angle: {np.degrees(leg_angle):.1f}°")

    # Step 4: Calculate hip angle to achieve target orientation
    hip_flexion = target_angle - leg_angle
    hip_flexion = np.clip(hip_flexion, -0.4, 0.4)
    print(f"Raw hip angle: {np.degrees(target_angle - leg_angle):.1f}°")
    print(f"Clipped hip angle: {np.degrees(hip_flexion):.1f}°")

    # Calculate final positions using forward kinematics
    # First segment (hip to knee)
    knee_x = upper_leg * np.sin(hip_flexion)
    knee_z = -upper_leg * np.cos(hip_flexion)
    print(f"Knee position: [{knee_x:.3f}, {knee_z:.3f}]")

    # Second segment (knee to ankle)
    ankle_x = knee_x + lower_leg * np.sin(hip_flexion + knee_angle)
    ankle_z = knee_z - lower_leg * np.cos(hip_flexion + knee_angle)
    print(f"Achieved: [{ankle_x:.3f}, {ankle_z:.3f}]")

    # Validate achieved position and segment lengths
    upper_achieved = np.sqrt(knee_x**2 + knee_z**2)
    lower_achieved = np.sqrt((ankle_x-knee_x)**2 + (ankle_z-knee_z)**2)
    print(f"Upper segment length: {upper_achieved:.3f}m (should be {upper_leg:.3f}m)")
    print(f"Lower segment length: {lower_achieved:.3f}m (should be {lower_leg:.3f}m)")

    error = np.sqrt((ankle_x - target_x)**2 + (ankle_z - target_z)**2)
    print(f"Error: {error*1000:.1f}mm")

    # Validate achieved position
    if error > 0.01:  # More than 1cm error
        print("WARNING: Large position error!")
        print(f"Target vs Achieved:")
        print(f"X: {target_x:.3f} vs {ankle_x:.3f} (diff: {(ankle_x-target_x)*1000:.1f}mm)")
        print(f"Z: {target_z:.3f} vs {ankle_z:.3f} (diff: {(ankle_z-target_z)*1000:.1f}mm)")

    if visualize:
        plot_leg_geometry(
            (0, 0),  # hip position
            (knee_x, knee_z),  # knee position
            (ankle_x, ankle_z),  # ankle position
            (target_x, target_z),  # target position
            hip_flexion,  # hip angle
            knee_angle,  # knee angle
            f"leg_pos_{target_x:.2f}_{-target_z:.2f}"  # filename
        )

    return hip_flexion, knee_angle

def main():
    # Test cases with different heights and forward positions
    test_positions = [
        (0.0, -0.784, "Standing position"),  # At minimum reach
        (0.05, -0.785, "Small forward step"),  # Slightly compressed
        (0.0, -0.790, "Slight compression"),  # More compressed
        (0.08, -0.787, "Medium forward step"),  # Forward with compression
        (0.12, -0.785, "Large forward step"),  # Maximum safe forward reach
        (-0.05, -0.785, "Small backward step"),  # Backward with compression
        (0.0, -0.795, "Maximum compression"),  # Near minimum reach
        (0.06, -0.788, "Combined forward and compress")  # Combined movement
    ]

    print("Testing IK calculations...")
    for x, z, desc in test_positions:
        print(f"\nTesting {desc}:")
        try:
            calculate_2d_ik(x, z)
        except ValueError as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()
