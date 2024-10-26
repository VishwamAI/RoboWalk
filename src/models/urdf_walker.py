import pybullet as p
import time
import math
import os

class URDFWalker:
    def __init__(self, client=None):
        # Initialize PyBullet
        self.client = client
        self.robot = None
        self.num_joints = 0
        self.joint_ids = {}

        if self.client is None:
            self.client = p.connect(p.DIRECT)
            p.setGravity(0, 0, -9.81)
            p.setRealTimeSimulation(0)

            # Load ground plane
            p.createCollisionShape(p.GEOM_PLANE)
            p.createMultiBody(0, 0)

    def load_robot(self):
        # Get the path to the URDF file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_path = os.path.join(current_dir, 'simple_biped_stable.urdf')

        # Calculate initial height based on leg geometry and stance angle
        hip_offset = 0.15  # Distance from torso center to hip joint
        upper_leg = 0.4  # Upper leg length
        lower_leg = 0.4  # Lower leg length
        foot_height = 0.025  # Half of foot height (0.05m)
        stance_angle = 0.2  # Reduced to ~11.5 degrees outward
        # Height is measured from ground to torso, accounting for both legs and foot
        leg_height = (upper_leg + lower_leg) * math.cos(stance_angle)
        initial_height = hip_offset + leg_height + foot_height - 0.02  # Lower by 2cm for better contact

        # Calculate initial position to account for center of mass
        # Move slightly backward to counteract forward pitch
        initial_x = -0.05  # Reduced backward shift to 5cm

        # Add slight backward pitch to initial orientation
        backward_pitch = -0.1  # Reduced to ~5.7 degrees backward lean
        initial_orientation = p.getQuaternionFromEuler([0, backward_pitch, 0])

        # Set ground plane properties for better friction
        p.changeDynamics(0, -1,  # Ground plane
                        lateralFriction=2.0,  # Increased friction
                        rollingFriction=0.2,  # Increased rolling friction
                        spinningFriction=0.2,  # Increased spinning friction
                        restitution=0.001,    # Reduced bounce
                        contactStiffness=10000,  # Added contact stiffness
                        contactDamping=1000)   # Added contact damping

        # Load the robot from URDF
        self.robot = p.loadURDF(
            urdf_path,
            basePosition=[initial_x, 0, initial_height],  # Adjusted position
            baseOrientation=initial_orientation,  # Reduced backward pitch
            useFixedBase=False
        )

        # Get joint information
        self.num_joints = p.getNumJoints(self.robot)
        self.joint_ids = {}

        # Store joint IDs
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.robot, i)
            joint_name = joint_info[1].decode('utf-8')
            self.joint_ids[joint_name] = i

        # Set joint properties for better control
        for joint_name in ['left_hip', 'right_hip']:
            if joint_name in self.joint_ids:
                p.changeDynamics(self.robot, self.joint_ids[joint_name],
                               jointDamping=2.0,  # Increased damping
                               angularDamping=1.0,  # Increased damping
                               contactStiffness=10000,  # Added contact stiffness
                               contactDamping=1000)   # Added contact damping

        # Set initial joint positions for stable stance
        if 'left_hip' in self.joint_ids:
            p.resetJointState(self.robot, self.joint_ids['left_hip'], -stance_angle)
            p.setJointMotorControl2(
                self.robot, self.joint_ids['left_hip'],
                p.POSITION_CONTROL, targetPosition=-stance_angle,
                force=5000  # Increased force for better stability
            )

        if 'right_hip' in self.joint_ids:
            p.resetJointState(self.robot, self.joint_ids['right_hip'], stance_angle)
            p.setJointMotorControl2(
                self.robot, self.joint_ids['right_hip'],
                p.POSITION_CONTROL, targetPosition=stance_angle,
                force=5000  # Increased force for better stability
            )

        print(f"Loaded robot with {self.num_joints} joints")
        print("Joint IDs:", self.joint_ids)

        # Extended stabilization phase with gravity compensation and monitoring
        stable_count = 0
        for i in range(240):  # Increased to 240 steps (1 second at 240Hz)
            # Apply stronger downward force during initial phase
            if i < 120:
                # Gravity compensation plus additional downward force
                p.applyExternalForce(self.robot, -1,  # Apply to base
                                   forceObj=[0, 0, -9.81 * 10],  # Strong downward force
                                   posObj=[0, 0, 0],
                                   flags=p.WORLD_FRAME)

            # Apply strong joint control during stabilization
            p.setJointMotorControl2(
                self.robot, self.joint_ids['left_hip'],
                p.POSITION_CONTROL, targetPosition=-stance_angle,
                force=5000  # Higher force during stabilization
            )
            p.setJointMotorControl2(
                self.robot, self.joint_ids['right_hip'],
                p.POSITION_CONTROL, targetPosition=stance_angle,
                force=5000  # Higher force during stabilization
            )

            p.stepSimulation()

            # Monitor stability every 30 steps
            if i % 30 == 0:
                pos, _ = p.getBasePositionAndOrientation(self.robot)
                if abs(pos[2] - initial_height) < 0.02:  # Height within 2cm
                    stable_count += 1
                    if stable_count >= 3:  # Three consecutive stable checks
                        print(f"Stabilized at step {i}")
                        break
                else:
                    stable_count = 0

        return self.robot

    def simulate_step(self, t):
        if self.robot is None:
            raise RuntimeError("Robot not loaded. Call load_robot() first.")

        # Generate walking motion using sine waves
        left_angle = math.sin(t * 2) * 0.2  # Reduced amplitude for stability
        right_angle = -left_angle  # Opposite phase

        # Apply joint positions with higher control forces
        if 'left_hip' in self.joint_ids:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=self.joint_ids['left_hip'],
                controlMode=p.POSITION_CONTROL,
                targetPosition=left_angle,
                force=1000,  # Increased force for better control
                maxVelocity=2.0
            )

        if 'right_hip' in self.joint_ids:
            p.setJointMotorControl2(
                bodyUniqueId=self.robot,
                jointIndex=self.joint_ids['right_hip'],
                controlMode=p.POSITION_CONTROL,
                targetPosition=right_angle,
                force=1000,  # Increased force for better control
                maxVelocity=2.0
            )

        # Step simulation multiple times for better physics accuracy
        for _ in range(4):
            p.stepSimulation()

    def get_robot_position(self):
        if self.robot is None:
            raise RuntimeError("Robot not loaded. Call load_robot() first.")
        return p.getBasePositionAndOrientation(self.robot)[0]

    def cleanup(self):
        if self.client is not None and self.client != 0:  # 0 is the shared GUI client
            p.disconnect(self.client)

def main():
    print("Initializing URDFWalker simulation...")
    walker = URDFWalker(client=p.connect(p.GUI))  # Explicitly pass client
    walker.load_robot()  # Explicitly load the robot after initialization
    print("Running walking simulation...")

    try:
        start_pos = walker.get_robot_position()
        for i in range(500):  # 5 seconds at 100Hz
            walker.simulate_step(i * 0.01)
            if i % 50 == 0:
                current_pos = walker.get_robot_position()
                distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(current_pos, start_pos)))
                print(f"Step {i}: Position = {current_pos}, Distance traveled = {distance:.3f}m")
            time.sleep(0.01)
    except Exception as e:
        print(f"Simulation error: {e}")
    finally:
        walker.cleanup()
        print("Simulation completed")

if __name__ == "__main__":
    main()
