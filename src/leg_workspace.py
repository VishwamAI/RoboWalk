import numpy as np
import matplotlib.pyplot as plt

def plot_leg_workspace(upper_leg=0.4, lower_leg=0.4, joint_limit=0.4):
    """
    Plot the workspace of a 2D leg with joint limits.
    
    Args:
        upper_leg: Length of upper leg segment (m)
        lower_leg: Length of lower leg segment (m)
        joint_limit: Maximum absolute angle for hip and knee joints (rad)
    """
    # Create grid of hip and knee angles within limits
    angles = np.linspace(-joint_limit, joint_limit, 50)
    hip_angles, knee_angles = np.meshgrid(angles, angles)
    
    # Calculate end effector positions for each angle combination
    x = np.zeros_like(hip_angles)
    z = np.zeros_like(hip_angles)
    
    for i in range(hip_angles.shape[0]):
        for j in range(hip_angles.shape[1]):
            hip = hip_angles[i,j]
            knee = knee_angles[i,j]
            
            # Calculate knee position
            knee_x = upper_leg * np.sin(hip)
            knee_z = -upper_leg * np.cos(hip)
            
            # Calculate ankle position
            ankle_x = knee_x + lower_leg * np.sin(hip + knee)
            ankle_z = knee_z - lower_leg * np.cos(hip + knee)
            
            x[i,j] = ankle_x
            z[i,j] = ankle_z
    
    # Create plot
    plt.figure(figsize=(10, 10))
    plt.scatter(x, z, c='blue', alpha=0.1, s=1)
    
    # Plot leg segments for a few example configurations
    example_angles = [-0.3, -0.15, 0, 0.15, 0.3]
    for hip in example_angles:
        for knee in example_angles:
            knee_x = upper_leg * np.sin(hip)
            knee_z = -upper_leg * np.cos(hip)
            ankle_x = knee_x + lower_leg * np.sin(hip + knee)
            ankle_z = knee_z - lower_leg * np.cos(hip + knee)
            
            # Plot upper leg
            plt.plot([0, knee_x], [0, knee_z], 'r-', alpha=0.2)
            # Plot lower leg
            plt.plot([knee_x, ankle_x], [knee_z, ankle_z], 'g-', alpha=0.2)
    
    # Add reference lines and labels
    plt.axhline(y=0, color='k', linestyle=':')
    plt.axvline(x=0, color='k', linestyle=':')
    plt.grid(True)
    plt.axis('equal')
    plt.title('Leg Workspace with Joint Limits')
    plt.xlabel('X Position (m)')
    plt.ylabel('Z Position (m)')
    
    # Save plot
    plt.savefig('leg_workspace.png')
    plt.close()

def main():
    # Plot workspace with default parameters
    plot_leg_workspace()
    print("Leg workspace visualization saved as 'leg_workspace.png'")
    
    # Calculate some useful statistics
    upper_leg = 0.4
    lower_leg = 0.4
    joint_limit = 0.4
    
    # Maximum reach
    max_reach = upper_leg + lower_leg
    print(f"\nMaximum reach: {max_reach:.3f}m")
    
    # Minimum reach (with maximum knee bend)
    min_reach = np.sqrt(upper_leg**2 + lower_leg**2 - 
                       2*upper_leg*lower_leg*np.cos(np.pi - joint_limit))
    print(f"Minimum reach: {min_reach:.3f}m")
    
    # Recommended operating range
    print(f"Recommended operating range: {min_reach:.3f}m to {max_reach*0.9:.3f}m")
    print(f"(90% of max reach to avoid singularities)")

if __name__ == "__main__":
    main()
