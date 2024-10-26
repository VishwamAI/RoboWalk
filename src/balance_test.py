import pybullet as p
import time
import numpy as np
from api.hardware_abstraction import RCUInterface, NPUInterface, WalkingRobotAPI
from models.urdf_walker import URDFWalker
from models.constants import *

def run_balance_test():
    # Initialize PyBullet
    client = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    # Load ground plane
    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)

    # Initialize robot API with modified gains
    robot_api = WalkingRobotAPI()
    robot_api.rcu.update_pid_gains(DEFAULT_PID_GAINS)

    # Load robot
    walker = URDFWalker()
    robot_id = walker.load_robot()

    # Calculate initial stable position
    # For target height h and leg length L:
    # cos(stance_angle) = h/L, where stance_angle is measured from vertical
    # We want the angle from horizontal, so we subtract from pi/2
    vertical_angle = np.arccos(HEIGHT_TARGET / LEG_LENGTH)
    stance_angle = np.pi/2 - vertical_angle

    # Add outward angle for stability
    stance_width_angle = np.arcsin(INITIAL_STANCE_WIDTH / (2 * LEG_LENGTH))
    left_angle = stance_angle + stance_width_angle
    right_angle = -(stance_angle + stance_width_angle)
    initial_angles = [left_angle, right_angle]

    # Set initial position at target height
    initial_pos = [0, 0, HEIGHT_TARGET]
    p.resetBasePositionAndOrientation(robot_id, initial_pos, [0, 0, 0, 1])

    # Calculate required force based on leg geometry
    # F = (m*g)/(2*cos(stance_angle)) for each leg
    required_force = (ROBOT_MASS * 9.81) / (2 * np.cos(vertical_angle))
    force_ramp_steps = 100
    force_increment = required_force / force_ramp_steps

    print("\nStarting balance test...")
    print("Initializing stable stance...")
    print(f"Target height: {HEIGHT_TARGET}m")
    print(f"Leg length: {LEG_LENGTH}m")
    print(f"Vertical angle: {np.degrees(vertical_angle):.1f} degrees")
    print(f"Stance angle from horizontal: {np.degrees(stance_angle):.1f} degrees")
    print(f"Final leg angles: L={np.degrees(left_angle):.1f}, R={np.degrees(right_angle):.1f} degrees")
    print(f"Required force per leg: {required_force:.1f}N")

    # Gradual force ramp-up phase
    for step in range(force_ramp_steps):
        current_force = force_increment * (step + 1)
        for i in range(len(initial_angles)):
            p.setJointMotorControl2(
                robot_id,
                i,
                p.POSITION_CONTROL,
                initial_angles[i],
                force=current_force,
                positionGain=0.5,  # Increased gains for better height maintenance
                velocityGain=0.3
            )
        p.stepSimulation()
        time.sleep(0.01)

    # Reset to full gains for the test
    for i in range(len(initial_angles)):
        p.setJointMotorControl2(
            robot_id,
            i,
            p.POSITION_CONTROL,
            initial_angles[i],
            force=required_force * 1.5,  # Add safety margin
            positionGain=2.0,  # Increased gains for height maintenance
            velocityGain=1.0
        )

    print("Initialization complete. Starting stability test...")
    heights = []
    joint_torques = []

    try:
        for step in range(500):  # Test for 500 steps
            # Get current state
            robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
            current_height = robot_pos[2]
            heights.append(current_height)

            # Get joint states
            joint_states = []
            for i in range(p.getNumJoints(robot_id)):
                state = p.getJointState(robot_id, i)
                joint_states.append({
                    'position': state[0],
                    'velocity': state[1],
                    'torque': state[3]
                })
                joint_torques.append(state[3])

            # Print progress every 100 steps
            if step % 100 == 0:
                print(f"\nStep {step}:")
                print(f"Height: {current_height:.3f}m")
                print(f"Joint Positions: {[state['position'] for state in joint_states]}")
                print(f"Joint Torques: {[state['torque'] for state in joint_states]}")

            # Step simulation
            p.stepSimulation()
            time.sleep(0.01)

            # Check for collapse
            if current_height < HEIGHT_TARGET * 0.5:  # Use height target from constants
                print("\nTest failed: Robot collapsed!")
                break

    except Exception as e:
        print(f"\nTest failed with error: {str(e)}")
        return False
    finally:
        p.disconnect()

    # Analyze results
    heights = np.array(heights)
    joint_torques = np.array(joint_torques)

    print("\nBalance Test Results:")
    print(f"Final height: {heights[-1]:.3f}m")
    print(f"Height variation: {np.std(heights):.3f}m")
    print(f"Average joint torque: {np.mean(joint_torques):.3f}Nm")
    print(f"Max joint torque: {np.max(np.abs(joint_torques)):.3f}Nm")

    success = heights[-1] > HEIGHT_TARGET * 0.8 and np.std(heights) < 0.1
    print(f"\nBalance test {'successful' if success else 'failed'}")
    return success

if __name__ == "__main__":
    run_balance_test()
