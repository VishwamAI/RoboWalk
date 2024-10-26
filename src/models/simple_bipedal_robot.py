import pybullet as p
import time
import math
import numpy as np

class SimpleBipedalRobot:
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
        
        # Create legs
        self.leg_shape = p.createCollisionShape(p.GEOM_CYLINDER, 
            radius=0.05, height=self.leg_length)
        
        # Create main body
        self.robot_id = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=self.torso,
            basePosition=[0, 0, self.leg_length + self.torso_height/2]
        )
        
        # Create legs and joints
        self.create_leg("left", -self.torso_width/4)
        self.create_leg("right", self.torso_width/4)
    
    def create_leg(self, side, offset_x):
        leg = p.createMultiBody(
            baseMass=0.5,
            baseCollisionShapeIndex=self.leg_shape,
            basePosition=[offset_x, 0, self.leg_length/2]
        )
        
        joint = p.createConstraint(
            self.robot_id, -1, leg, -1,
            p.JOINT_FIXED, [0, 0, 0],
            [offset_x, 0, -self.torso_height/2],
            [0, 0, self.leg_length/2]
        )
        
        if side == "left":
            self.left_leg = leg
        else:
            self.right_leg = leg
    
    def simulate_step(self):
        p.stepSimulation()
    
    def cleanup(self):
        p.disconnect(self.client)

if __name__ == "__main__":
    robot = SimpleBipedalRobot()
    try:
        for i in range(200):
            robot.simulate_step()
            if i % 50 == 0:
                pos, _ = p.getBasePositionAndOrientation(robot.robot_id)
                print(f"Step {i}: Robot position = {pos}")
            time.sleep(0.01)
    finally:
        robot.cleanup()
