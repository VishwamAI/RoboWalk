import numpy as np
import matplotlib.pyplot as plt

def calculate_leg_angles(target_x, target_z, upper_leg=0.4, lower_leg=0.4):
    """
    Calculate leg angles to reach target position
    Returns: (hip_angle, knee_angle) in radians
    """
    total_dist = np.sqrt(target_x**2 + target_z**2)

    # Check if position is reachable
    max_reach = upper_leg + lower_leg
    min_reach = abs(upper_leg - lower_leg)
    if not (min_reach <= total_dist <= max_reach):
        return None, None

    # Calculate knee angle using law of cosines
    cos_knee = (upper_leg**2 + lower_leg**2 - total_dist**2) / (2 * upper_leg * lower_leg)
    cos_knee = np.clip(cos_knee, -1.0, 1.0)
    knee_angle = -(np.pi - np.arccos(cos_knee))  # Negative for backward bend

    # Calculate angle to target from vertical
    target_angle = np.arctan2(target_x, -target_z)

    # Calculate angle between upper leg and target line
    cos_alpha = (upper_leg**2 + total_dist**2 - lower_leg**2) / (2 * upper_leg * total_dist)
    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
    alpha = np.arccos(cos_alpha)

    # Calculate hip angle
    hip_angle = target_angle - alpha if target_x >= 0 else target_angle + alpha

    return hip_angle, knee_angle

def plot_leg_configuration(hip_x, hip_z, target_x, target_z, hip_angle, knee_angle, upper_leg=0.4, lower_leg=0.4):
    """
    Plot leg configuration showing upper and lower leg segments
    """
    # Calculate knee position
    knee_x = hip_x + upper_leg * np.sin(hip_angle)
    knee_z = hip_z - upper_leg * np.cos(hip_angle)

    # Calculate foot position
    foot_x = knee_x + lower_leg * np.sin(hip_angle + knee_angle)
    foot_z = knee_z - lower_leg * np.cos(hip_angle + knee_angle)

    # Plot leg segments
    plt.plot([hip_x, knee_x], [hip_z, knee_z], 'b-', linewidth=2, label='Upper Leg')
    plt.plot([knee_x, foot_x], [foot_z, knee_z], 'g-', linewidth=2, label='Lower Leg')
    plt.plot([hip_x], [hip_z], 'ko', label='Hip')
    plt.plot([knee_x], [knee_z], 'ko', label='Knee')
    plt.plot([foot_x], [foot_z], 'ko', label='Foot')
    plt.plot([target_x], [target_z], 'rx', label='Target')

def plot_leg_positions(upper_leg=0.4, lower_leg=0.4):
    """
    Plot a grid of possible leg positions to visualize the workspace
    """
    # Create a grid of target positions
    x = np.linspace(-0.3, 0.3, 20)
    z = np.linspace(-0.8, -0.4, 20)
    X, Z = np.meshgrid(x, z)

    # Plot setup
    plt.figure(figsize=(12, 8))
    plt.grid(True)
    plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    plt.axvline(x=0, color='k', linestyle='-', alpha=0.3)

    # Calculate reachable positions
    valid_positions = []
    invalid_positions = []

    for i in range(len(x)):
        for j in range(len(z)):
            target_x = X[i,j]
            target_z = Z[i,j]
            hip_angle, knee_angle = calculate_leg_angles(target_x, target_z, upper_leg, lower_leg)

            if hip_angle is not None and -0.4 <= hip_angle <= 0.4 and -0.4 <= knee_angle <= 0.4:
                valid_positions.append((target_x, target_z))
            else:
                invalid_positions.append((target_x, target_z))

    # Plot positions
    if valid_positions:
        valid_x, valid_z = zip(*valid_positions)
        plt.plot(valid_x, valid_z, 'g.', alpha=0.5, label='Valid Positions')

    if invalid_positions:
        invalid_x, invalid_z = zip(*invalid_positions)
        plt.plot(invalid_x, invalid_z, 'r.', alpha=0.2, label='Invalid Positions')

    # Test positions
    test_positions = [
        (0.0, -0.60, "Standing"),
        (0.05, -0.60, "Small Forward"),
        (0.0, -0.55, "Knee Bend"),
        (0.1, -0.58, "Medium Forward"),
        (-0.05, -0.60, "Small Back")
    ]

    # Plot test configurations
    for x, z, name in test_positions:
        hip_angle, knee_angle = calculate_leg_angles(x, z)
        if hip_angle is not None:
            plot_leg_configuration(0, 0, x, z, hip_angle, knee_angle)
            plt.annotate(name, (x, z), xytext=(5, 5), textcoords='offset points')

    plt.axis('equal')
    plt.title('Leg Workspace Analysis')
    plt.xlabel('X Position (m)')
    plt.ylabel('Z Position (m)')
    plt.legend()
    plt.savefig('leg_workspace.png')
    plt.close()

if __name__ == "__main__":
    plot_leg_positions()
    print("Visualization saved as 'leg_workspace.png'")
