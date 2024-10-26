import pybullet as p
import time
import math
import numpy as np

class BipedalRobot:
    def __init__(self):
        # Initialize PyBullet in DIRECT mode
        self.client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        
        # Create ground
        self.ground_id = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, self.ground_id)
        
        # Robot dimensions
        self.torso_height = 0.4
        self.torso_width = 0.2
        self.leg_length = 0.3
        self.foot_length = 0.1
        
        # Create robot parts
        self.create_robot()
        
    def create_robot(self):
        # Create torso
        self.torso = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.torso_width/2, self.torso_width/2, self.torso_height/2]
        )
        
        # Initial position: slightly elevated to allow for leg attachment
        self.robot_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=self.torso,
            basePosition=[0, 0, self.torso_height + self.leg_length],
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Create legs
        self.create_leg("left", [-self.torso_width/4, 0, 0])
        self.create_leg("right", [self.torso_width/4, 0, 0])
        
    def create_leg(self, side, offset):
        # Upper leg
        upper_leg = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.05, 0.05, self.leg_length/2]
        )
        
        # Lower leg
        lower_leg = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[0.05, 0.05, self.leg_length/2]
        )
        
        # Foot
        foot = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[self.foot_length/2, 0.05, 0.02]
        )
        
        # Create joints and links
        hip_joint = p.createMultiBody(
            baseMass=0.5,
            baseCollisionShapeIndex=upper_leg,
            basePosition=[offset[0], offset[1], self.torso_height],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        knee_joint = p.createMultiBody(
            baseMass=0.5,
            baseCollisionShapeIndex=lower_leg,
            basePosition=[offset[0], offset[1], self.torso_height - self.leg_length],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        foot_joint = p.createMultiBody(
            baseMass=0.2,
            baseCollisionShapeIndex=foot,
            basePosition=[offset[0], offset[1], self.torso_height - 2*self.leg_length],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        # Create constraints between parts
        p.createConstraint(
            self.robot_id, -1, hip_joint, -1,
            p.JOINT_REVOLUTE, [0, 1, 0],
            [offset[0], offset[1], -self.torso_height/2],
            [0, 0, self.leg_length/2]
        )
        
        p.createConstraint(
            hip_joint, -1, knee_joint, -1,
            p.JOINT_REVOLUTE, [0, 1, 0],
            [0, 0, -self.leg_length/2],
            [0, 0, self.leg_length/2]
        )
        
        p.createConstraint(
            knee_joint, -1, foot_joint, -1,
            p.JOINT_REVOLUTE, [0, 1, 0],
            [0, 0, -self.leg_length/2],
            [0, 0, 0]
        )
        
        # Store joint IDs for later control
        if side == "left":
            self.left_leg = {"hip": hip_joint, "knee": knee_joint, "foot": foot_joint}
        else:
            self.right_leg = {"hip": hip_joint, "knee": knee_joint, "foot": foot_joint}
    
    def simulate_step(self, t):
        # Basic walking gait using sine waves
        # Hip joints move in opposite phases
        left_hip_angle = math.sin(t * 2) * 0.4
        right_hip_angle = math.sin(t * 2 + math.pi) * 0.4
        
        # Knee joints flex more during leg lift
        left_knee_angle = abs(math.sin(t * 2)) * 0.6
        right_knee_angle = abs(math.sin(t * 2 + math.pi)) * 0.6
        
        # Apply joint motors
        p.setJointMotorControl2(self.left_leg["hip"], 0, p.POSITION_CONTROL, left_hip_angle)
        p.setJointMotorControl2(self.right_leg["hip"], 0, p.POSITION_CONTROL, right_hip_angle)
        p.setJointMotorControl2(self.left_leg["knee"], 0, p.POSITION_CONTROL, left_knee_angle)
        p.setJointMotorControl2(self.right_leg["knee"], 0, p.POSITION_CONTROL, right_knee_angle)
        
        p.stepSimulation()
    
    def cleanup(self):
        p.disconnect(self.client)

if __name__ == "__main__":
    print("Initializing Bipedal Robot simulation...")
    robot = BipedalRobot()
    print("Running walking simulation...")
    
    try:
        for i in range(500):  # 5 seconds at 100Hz
            robot.simulate_step(i * 0.01)
            if i % 50 == 0:  # Print every 0.5 seconds
                pos, _ = p.getBasePositionAndOrientation(robot.robot_id)
                print(f"Step {i}: Robot position = {pos}")
            time.sleep(0.01)
    except Exception as e:
        print(f"Simulation error: {e}")
    finally:
        robot.cleanup()
        print("Simulation completed")
