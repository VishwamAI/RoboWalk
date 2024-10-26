import pybullet as p
import time
import math
import numpy as np

class SimpleWalker:
    def __init__(self):
        # Initialize PyBullet in DIRECT mode (CPU-only)
        self.client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        
        # Create ground plane
        self.plane_id = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, self.plane_id)
        
        # Create robot body
        self.create_robot()
    
    def create_robot(self):
        # Create main body
        body_height = 0.5
        self.body = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.1, body_height/2])
        
        # Create legs
        self.leg_length = 0.4
        self.leg_radius = 0.02
        self.leg_shape = p.createCollisionShape(p.GEOM_CYLINDER, 
                                              radius=self.leg_radius,
                                              height=self.leg_length)
        
        # Create robot with body
        self.robot_id = p.createMultiBody(
            baseMass=1,
            baseCollisionShapeIndex=self.body,
            basePosition=[0, 0, body_height + self.leg_length],
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Add legs as joints
        self.left_leg = p.createMultiBody(
            baseMass=0.5,
            baseCollisionShapeIndex=self.leg_shape,
            basePosition=[-0.1, 0, body_height],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        self.right_leg = p.createMultiBody(
            baseMass=0.5,
            baseCollisionShapeIndex=self.leg_shape,
            basePosition=[0.1, 0, body_height],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        # Create constraints to attach legs to body
        self.left_joint = p.createConstraint(
            self.robot_id, -1, self.left_leg, -1,
            p.JOINT_REVOLUTE, [0, 1, 0],
            [-0.1, 0, -body_height/2], [0, 0, self.leg_length/2],
            childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        self.right_joint = p.createConstraint(
            self.robot_id, -1, self.right_leg, -1,
            p.JOINT_REVOLUTE, [0, 1, 0],
            [0.1, 0, -body_height/2], [0, 0, self.leg_length/2],
            childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
    
    def simulate_step(self, t):
        # Simple walking motion using sine waves
        left_angle = math.sin(t * 2) * 0.5
        right_angle = math.sin(t * 2 + math.pi) * 0.5
        
        p.setJointMotorControl2(self.left_leg, 0, p.POSITION_CONTROL, left_angle)
        p.setJointMotorControl2(self.right_leg, 0, p.POSITION_CONTROL, right_angle)
        
        p.stepSimulation()
    
    def cleanup(self):
        p.disconnect(self.client)

# Test the walker
print("Initializing SimpleWalker simulation...")
walker = SimpleWalker()
print("Running simulation for 5 seconds...")

for i in range(500):  # 500 steps = ~5 seconds
    walker.simulate_step(i * 0.01)
    if i % 50 == 0:  # Print every 50 steps
        pos, _ = p.getBasePositionAndOrientation(walker.robot_id)
        print(f"Step {i}: Robot position = {pos}")
    time.sleep(0.01)

walker.cleanup()
print("Simulation completed successfully!")
