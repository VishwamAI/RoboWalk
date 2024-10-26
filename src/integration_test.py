"""
Integration test for the walking robot system
Tests the complete walking functionality using our custom chip design
"""

import numpy as np
import pybullet as p
import time
from api.hardware_abstraction import WalkingRobotAPI
from models.urdf_walker import URDFWalker

def run_walking_test():
    # Initialize PyBullet simulation
    client = p.connect(p.DIRECT)  # Headless mode for testing
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    # Load ground plane
    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)

    # Initialize robot API and URDF model
    robot_api = WalkingRobotAPI()
    robot_api.initialize()

    # Load robot URDF
    walker = URDFWalker()
    robot_id = walker.load_robot()

    # Test parameters
    stabilization_steps = 300  # Allow time for initial stabilization
    walking_steps = 1000
    simulation_steps = stabilization_steps + walking_steps
    dt = 0.01

    # Walking parameters
    stabilization_params = {
        'step_height': 0.0,    # No stepping during stabilization
        'step_length': 0.0,    # No forward motion
        'cycle_time': 1.0,     # Slower control for stability
        'damping': 0.5         # Higher damping for stability
    }

    walking_params = {
        'step_height': 0.05,   # Increased height for better clearance
        'step_length': 0.1,    # Longer steps for momentum
        'cycle_time': 0.8,     # Slightly faster for momentum
        'damping': 0.3         # Normal damping for walking
    }

    # Start with stabilization parameters
    robot_api.set_walking_parameters(stabilization_params)

    print("Starting walking test...")
    print("Phase 1: Initial Stabilization")

    # Initialize arrays for logging
    positions = []
    velocities = []
    safety_status = []

    try:
        # Stabilization phase
        for step in range(stabilization_steps):
            # Update pattern (will maintain stance due to zero step parameters)
            robot_api.update_walking_pattern(dt)

            # Get system status
            status = robot_api.get_system_status()

            # Apply joint positions to simulation
            joint_positions = status['joint_positions']
            for i in range(len(joint_positions)):
                p.setJointMotorControl2(
                    robot_id,
                    i,
                    p.POSITION_CONTROL,
                    joint_positions[i],
                    force=500.0  # Higher force for stability
                )

            # Step simulation
            p.stepSimulation()

            # Log data
            robot_pos, robot_ori = p.getBasePositionAndOrientation(robot_id)
            positions.append(robot_pos)
            lin_vel, ang_vel = p.getBaseVelocity(robot_id)
            velocities.append(lin_vel)
            safety_status.append(status['safety']['is_safe'])

            # Check for stability
            if robot_pos[2] < 0.1:
                raise Exception("Robot collapsed during stabilization!")

            # Print progress
            if step % 100 == 0:
                print(f"Stabilization Step {step}/{stabilization_steps}")
                print(f"Position: {robot_pos}")
                print(f"Safety Status: {status['safety']}")
                print("-" * 50)

        print("\nPhase 2: Walking")
        # Switch to walking parameters
        robot_api.set_walking_parameters(walking_params)

        # Walking phase
        for step in range(walking_steps):
            # Update walking pattern
            robot_api.update_walking_pattern(dt)

            # Get system status
            status = robot_api.get_system_status()

            # Apply joint positions to simulation
            joint_positions = status['joint_positions']
            for i in range(len(joint_positions)):
                p.setJointMotorControl2(
                    robot_id,
                    i,
                    p.POSITION_CONTROL,
                    joint_positions[i],
                    force=300.0  # Normal force for walking
                )

            # Step simulation
            p.stepSimulation()

            # Log data
            robot_pos, robot_ori = p.getBasePositionAndOrientation(robot_id)
            positions.append(robot_pos)
            lin_vel, ang_vel = p.getBaseVelocity(robot_id)
            velocities.append(lin_vel)
            safety_status.append(status['safety']['is_safe'])

            # Check for stability
            if abs(robot_pos[0]) > 1.0 or abs(robot_pos[1]) > 1.0 or robot_pos[2] < 0.1:
                raise Exception("Robot lost stability during walking!")

            # Print progress
            if step % 100 == 0:
                print(f"Walking Step {step}/{walking_steps}")
                print(f"Position: {robot_pos}")
                print(f"Safety Status: {status['safety']}")
                print("-" * 50)

            time.sleep(dt)  # Simulate real-time control

    except Exception as e:
        print(f"Test failed: {str(e)}")
        return False
    finally:
        p.disconnect()

    # Analyze results
    positions = np.array(positions)
    velocities = np.array(velocities)

    # Calculate metrics
    total_distance = np.linalg.norm(positions[-1] - positions[0])
    average_velocity = np.mean(np.linalg.norm(velocities, axis=1))
    stability_score = np.mean([abs(pos[2] - positions[0][2]) for pos in positions])
    safety_percentage = np.mean(safety_status) * 100

    print("\nTest Results:")
    print(f"Total distance walked: {total_distance:.3f} meters")
    print(f"Average velocity: {average_velocity:.3f} m/s")
    print(f"Stability score: {stability_score:.3f} (lower is better)")
    print(f"Safety system active: {safety_percentage:.1f}% of the time")

    # Success criteria
    success = (
        total_distance > 0.5 and  # Walked at least 0.5 meters
        stability_score < 0.1 and  # Maintained stability
        safety_percentage > 95.0   # Safety system working
    )
    
    if success:
        print("\nWalking test PASSED!")
    else:
        print("\nWalking test FAILED!")
    
    return success

if __name__ == "__main__":
    success = run_walking_test()
    exit(0 if success else 1)
