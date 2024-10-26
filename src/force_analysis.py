import pybullet as p
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from models.urdf_walker import URDFWalker
from models.constants import *

class ForceAnalyzer:
    def __init__(self, connection_mode=p.DIRECT):
        # Initialize PyBullet
        try:
            if p.isConnected():
                p.disconnect()
        except p.error:
            pass  # Ignore if no existing connection

        self.client = p.connect(connection_mode)
        if self.client < 0:
            raise RuntimeError("Failed to connect to PyBullet")

        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0/240.0)
        p.setRealTimeSimulation(0)

        # Enable basic debug visualization
        if p.getConnectionInfo(self.client)['connectionMethod'] == p.GUI:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)

        # Create ground plane with higher friction
        ground_shape = p.createCollisionShape(
            p.GEOM_PLANE,
            planeNormal=[0, 0, 1]
        )
        ground_visual = p.createVisualShape(
            p.GEOM_PLANE,
            rgbaColor=[0.9, 0.9, 0.9, 1]
        )
        plane_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=ground_shape,
            baseVisualShapeIndex=ground_visual,
            basePosition=[0, 0, 0]
        )
        p.changeDynamics(plane_id, -1,
                        lateralFriction=2.0,  # Increased friction
                        rollingFriction=0.2,  # Increased rolling friction
                        spinningFriction=0.2,  # Increased spinning friction
                        restitution=0.001,    # Reduced restitution
                        contactStiffness=10000,  # Added contact stiffness
                        contactDamping=1000)   # Added contact damping

        # Load robot with improved contact parameters
        self.walker = URDFWalker(client=self.client)  # Pass client to ensure same simulation
        self.robot_id = self.walker.load_robot()

        # Set robot contact properties
        p.changeDynamics(self.robot_id, -1,  # Base link
                        contactStiffness=10000,
                        contactDamping=1000)

        # Initialize data storage
        self.force_history = {'left': [], 'right': []}
        self.com_history = []
        self.time_points = []

    def reset_robot(self):
        # Calculate initial stance for better ground contact
        hip_offset = 0.15  # Distance from torso center to hip joint
        upper_leg = 0.4  # Upper leg length
        lower_leg = 0.4  # Lower leg length
        foot_height = 0.025  # Half of foot height (0.05m)
        stance_angle = 0.2  # ~11.5 degrees outward, within joint limits
        # Total height: torso-to-hip + legs + foot
        leg_height = (upper_leg + lower_leg) * np.cos(stance_angle)
        initial_height = hip_offset + leg_height + foot_height - 0.02  # Lower by 2cm for better contact
        initial_x = -0.05  # Shift back by 5cm to counter forward lean

        # Set ground plane properties for better friction
        p.changeDynamics(0, -1,  # Ground plane
                        lateralFriction=2.0,
                        rollingFriction=0.2,
                        spinningFriction=0.2,
                        restitution=0.001,
                        contactStiffness=10000,
                        contactDamping=1000)

        # Position robot with slight backward lean for stability
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [initial_x, 0, initial_height],  # Using geometry-based height
            p.getQuaternionFromEuler([0, -0.1, 0])  # Reduced backward lean to ~5.7 degrees
        )

        # Set joint properties for better control
        for joint_id in [0, 1]:  # Both hip joints
            p.changeDynamics(self.robot_id, joint_id,
                           jointDamping=2.0,
                           angularDamping=1.0,
                           contactStiffness=10000,
                           contactDamping=1000)

        # Set initial joint positions with higher forces
        p.setJointMotorControl2(self.robot_id, 0, p.POSITION_CONTROL,
                              targetPosition=stance_angle,
                              force=5000,  # Increased force for better stability
                              maxVelocity=0.1)  # Further reduced velocity for better control
        p.setJointMotorControl2(self.robot_id, 1, p.POSITION_CONTROL,
                              targetPosition=-stance_angle,
                              force=5000,  # Increased force for better stability
                              maxVelocity=0.1)  # Further reduced velocity for better control

        # Add debug visualization for contact points only in GUI mode
        if p.getConnectionInfo(self.client)['connectionMethod'] == p.GUI:
            p.configureDebugVisualizer(p.COV_ENABLE_CONTACT_POINTS, 1)
            p.addUserDebugLine([0, 0, 0], [0, 0, initial_height], [1, 0, 0], 2.0)

        # Initial stabilization phase with gravity compensation
        stable_count = 0
        for i in range(240):  # Reduced to match URDFWalker's stabilization time
            # Apply gravity compensation and downward forces
            if i < 120:  # First half of stabilization
                # Gravity compensation
                p.applyExternalForce(self.robot_id, -1,  # Apply to base
                                   forceObj=[0, 0, 9.81 * 10],  # Increased gravity compensation
                                   posObj=[0, 0, 0],
                                   flags=p.WORLD_FRAME)
                # Additional downward force
                p.applyExternalForce(self.robot_id, -1,
                                   forceObj=[0, 0, -500],  # Strong downward force
                                   posObj=[0, 0, 0],
                                   flags=p.WORLD_FRAME)

            p.stepSimulation()

            # Check stability every 10 steps
            if i % 10 == 0:
                if self._check_stability():
                    stable_count += 1
                    if stable_count > 5:  # Require 5 consecutive stable checks
                        print(f"Stabilized after {i} steps")
                        break
                else:
                    stable_count = 0

        return stance_angle

    def get_contact_forces(self):
        left_force = np.zeros(3)
        right_force = np.zeros(3)

        # Get all contact points for the robot
        contact_points = p.getContactPoints(self.robot_id)

        print("\n=== RAW CONTACT DATA ===")
        print(f"Number of contact points: {len(contact_points)}")
        print("Contact point data structure:")
        for i, contact in enumerate(contact_points):
            print(f"\nContact {i+1} raw data:")
            print("Index : Value")
            print("-" * 20)
            for j, value in enumerate(contact):
                print(f"{j:2d}    : {value}")

        # Expected indices in contact point data:
        # 0,1: bodyA, bodyB (0=ground, 1=robot)
        # 2,3,4: linkIndexA, linkIndexB (-1 for base)
        # 5: positionOnA (x,y,z tuple)
        # 6: positionOnB (x,y,z tuple)
        # 7: contactNormal (x,y,z tuple)
        # 8: contact distance
        # 9: normal force magnitude
        # 10: lateral friction force 1
        # 11: lateral friction direction 1
        # 12: lateral friction force 2
        # 13: lateral friction direction 2

        print("\n=== FORCE CALCULATION ===")
        for contact in contact_points:
            try:
                # Validate data structure
                if len(contact) < 14:
                    print(f"Error: Contact point has insufficient data (length={len(contact)})")
                    continue

                # Extract basic contact information
                bodyA = int(contact[1])
                bodyB = int(contact[2])
                linkA = int(contact[3])
                linkB = int(contact[4])

                print(f"\nContact between bodies {bodyA} and {bodyB}")
                print(f"Link indices: A={linkA}, B={linkB}")

                # Determine which body is the robot (id=1)
                if bodyA != 1 and bodyB != 1:
                    print("Error: Neither body is the robot")
                    continue

                # Extract force data using correct indices from box test
                normal_force = float(contact[9])  # Normal force magnitude
                normal_dir_tuple = contact[7]  # Contact normal direction tuple
                normal_dir = np.array([float(normal_dir_tuple[0]),
                                     float(normal_dir_tuple[1]),
                                     float(normal_dir_tuple[2])])

                print(f"Normal force: {normal_force:.2f}N")
                print(f"Normal direction: [{normal_dir[0]:.3f}, {normal_dir[1]:.3f}, {normal_dir[2]:.3f}]")

                # Calculate force vector using verified box test approach
                force_vec = normal_dir * abs(normal_force)
                print(f"Force vector: [{force_vec[0]:.3f}, {force_vec[1]:.3f}, {force_vec[2]:.3f}]N")

                # Determine which foot (using correct foot link indices from debug_links.py)
                robot_link = linkA if bodyA == 1 else linkB
                if robot_link == 2:  # Left foot
                    left_force += force_vec
                    print(f"Added to left foot: {np.linalg.norm(left_force):.2f}N")
                elif robot_link == 5:  # Right foot
                    right_force += force_vec
                    print(f"Added to right foot: {np.linalg.norm(right_force):.2f}N")
                else:
                    print(f"Warning: Contact on unexpected link {robot_link}")

            except Exception as e:
                print(f"Error processing contact: {str(e)}")
                print("Raw contact data:", contact)

        print("\n=== FINAL RESULTS ===")
        print(f"Left foot total force: {np.linalg.norm(left_force):.2f}N")
        print(f"Right foot total force: {np.linalg.norm(right_force):.2f}N")
        print(f"Left force vector: {left_force}")
        print(f"Right force vector: {right_force}")

        return left_force, right_force

    def get_com_position(self):
        pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        return np.array(pos)

    def _check_stability(self):
        # Get current state
        pos, orientation = p.getBasePositionAndOrientation(self.robot_id)
        euler = p.getEulerFromQuaternion(orientation)

        # Check orientation (roll and pitch)
        if abs(euler[0]) > 0.3 or abs(euler[1]) > 0.3:  # More lenient tolerance (±17°)
            print(f"Debug: Unstable orientation - Roll: {euler[0]:.3f}, Pitch: {euler[1]:.3f}")
            return False

        # Check height
        upper_leg = 0.4  # Upper leg length
        lower_leg = 0.4  # Lower leg length
        foot_height = 0.025  # Half of foot height
        stance_angle = 0.2  # ~11.5 degrees outward
        target_height = (upper_leg + lower_leg) * np.cos(stance_angle) + foot_height
        if abs(pos[2] - target_height) > 0.15:  # More lenient tolerance for height
            print(f"Debug: Unstable height - Current: {pos[2]:.3f}m, Target: {target_height:.3f}m")
            return False

        # Check ground contact
        left_force, right_force = self.get_contact_forces()
        total_vertical_force = left_force[2] + right_force[2]
        min_force = 9.81 * 10  # Minimum force based on robot mass (~10kg)
        if total_vertical_force < min_force:  # Dynamic force threshold
            print(f"Debug: Insufficient ground contact - Force: {total_vertical_force:.1f}N")
            return False

        return True

    @property
    def connection_mode(self):
        return p.getConnectionInfo(self.client)['connectionMethod']

if __name__ == "__main__":
    analyzer = ForceAnalyzer(connection_mode=p.DIRECT)
    print("Force Analyzer initialized successfully!")
