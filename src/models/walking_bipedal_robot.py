import pybullet as p
import time
import math
import numpy as np

class WalkingBipedalRobot:
    def __init__(self):
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
        
        self.create_robot()
    
    def create_robot(self):
        # Create torso
        self.torso = p.createCollisionShape(p.GEOM_BOX, 
            halfExtents=[self.torso_width/2, self.torso_width/2, self.torso_height/2])
        
        # Create leg shape
        self.leg_shape = p.createCollisionShape(p.GEOM_BOX, 
            halfExtents=[0.05, 0.05, self.leg_length/2])
        
        # Create main body
        self.robot_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=self.torso,
            basePosition=[0, 0, self.leg_length + self.torso_height/2],
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Create legs with joints
        self.left_leg = self.create_leg(-self.torso_width/4)
        self.right_leg = self.create_leg(self.torso_width/4)
        
        # Enable motor control
        p.setJointMotorControl2(self.left_leg, 0, p.POSITION_CONTROL, force=500)
        p.setJointMotorControl2(self.right_leg, 0, p.POSITION_CONTROL, force=500)
    
    def create_leg(self, offset_x):
        # Create leg
        leg = p.createMultiBody(
            baseMass=0.5,
            baseCollisionShapeIndex=self.leg_shape,
            basePosition=[offset_x, 0, self.leg_length/2]
        )
        
        # Create revolute joint for hip
        joint = p.createConstraint(
            self.robot_id, -1, leg, -1,
            p.JOINT_REVOLUTE,
            jointAxis=[0, 1, 0],  # Rotate around Y axis
            parentFramePosition=[offset_x, 0, -self.torso_height/2],
            childFramePosition=[0, 0, self.leg_length/2],
            parentFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        return leg
    
    def simulate_step(self, t):
        # Generate walking motion using sine waves
        left_angle = math.sin(t * 2) * 0.3  # Reduced amplitude for stability
        right_angle = math.sin(t * 2 + math.pi) * 0.3  # Opposite phase
        
        # Apply joint motors
        p.setJointMotorControl2(self.left_leg, 0, p.POSITION_CONTROL, 
                              targetPosition=left_angle, force=500)
        p.setJointMotorControl2(self.right_leg, 0, p.POSITION_CONTROL, 
                              targetPosition=right_angle, force=500)
        
        p.stepSimulation()
    
    def get_position(self):
        return p.getBasePositionAndOrientation(self.robot_id)[0]
    
    def cleanup(self):
        p.disconnect(self.client)

if __name__ == "__main__":
    print("Initializing Walking Bipedal Robot simulation...")
    robot = WalkingBipedalRobot()
    print("Running walking simulation...")
    
    try:
        start_pos = robot.get_position()
        for i in range(500):  # 5 seconds at 100Hz
            robot.simulate_step(i * 0.01)
            if i % 50 == 0:
                current_pos = robot.get_position()
                distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(current_pos, start_pos)))
                print(f"Step {i}: Position = {current_pos}, Distance traveled = {distance:.3f}m")
            time.sleep(0.01)
    except Exception as e:
        print(f"Simulation error: {e}")
    finally:
        robot.cleanup()
        print("Simulation completed")
