import pybullet as p
import time
import math
import numpy as np

class RevisedWalkingRobot:
    def __init__(self):
        # Connect to PyBullet in DIRECT mode
        self.client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        
        # Create ground plane
        self.plane_id = p.createCollisionShape(p.GEOM_PLANE)
        p.createMultiBody(0, self.plane_id)
        
        # Robot dimensions
        self.torso_height = 0.4
        self.torso_width = 0.2
        self.leg_length = 0.3
        
        self.create_robot()
        
    def create_robot(self):
        # Create torso (base)
        self.torso = p.createCollisionShape(p.GEOM_BOX, 
            halfExtents=[self.torso_width/2, self.torso_width/2, self.torso_height/2])
            
        # Create leg links
        self.upper_leg = p.createCollisionShape(p.GEOM_BOX,
            halfExtents=[0.05, 0.05, self.leg_length/4])
        self.lower_leg = p.createCollisionShape(p.GEOM_BOX,
            halfExtents=[0.05, 0.05, self.leg_length/4])
            
        # Create the base (torso)
        base_mass = 1.0
        self.robot_id = p.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=self.torso,
            basePosition=[0, 0, self.leg_length + self.torso_height/2],
            baseOrientation=[0, 0, 0, 1]
        )
        
        # Create legs
        self.create_leg("left", -self.torso_width/4)
        self.create_leg("right", self.torso_width/4)
        
    def create_leg(self, side, offset_x):
        # Mass for leg segments
        leg_mass = 0.5
        
        # Create upper leg
        upper_leg = p.createMultiBody(
            baseMass=leg_mass,
            baseCollisionShapeIndex=self.upper_leg,
            basePosition=[offset_x, 0, self.leg_length],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        # Create hip joint (connecting torso to upper leg)
        hip_joint = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=-1,
            childBodyUniqueId=upper_leg,
            childLinkIndex=-1,
            jointType=p.JOINT_REVOLUTE,
            jointAxis=[0, 1, 0],
            parentFramePosition=[offset_x, 0, -self.torso_height/2],
            childFramePosition=[0, 0, self.leg_length/4]
        )
        
        # Create lower leg
        lower_leg = p.createMultiBody(
            baseMass=leg_mass,
            baseCollisionShapeIndex=self.lower_leg,
            basePosition=[offset_x, 0, self.leg_length/2],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0])
        )
        
        # Create knee joint (connecting upper leg to lower leg)
        knee_joint = p.createConstraint(
            parentBodyUniqueId=upper_leg,
            parentLinkIndex=-1,
            childBodyUniqueId=lower_leg,
            childLinkIndex=-1,
            jointType=p.JOINT_REVOLUTE,
            jointAxis=[0, 1, 0],
            parentFramePosition=[0, 0, -self.leg_length/4],
            childFramePosition=[0, 0, self.leg_length/4]
        )
        
        # Store joint IDs
        if side == "left":
            self.left_hip = hip_joint
            self.left_knee = knee_joint
            self.left_upper_leg = upper_leg
            self.left_lower_leg = lower_leg
        else:
            self.right_hip = hip_joint
            self.right_knee = knee_joint
            self.right_upper_leg = upper_leg
            self.right_lower_leg = lower_leg
            
    def simulate_step(self, t):
        # Generate walking motion using sine waves
        hip_angle = math.sin(t * 2) * 0.2  # Reduced amplitude for stability
        knee_angle = abs(math.sin(t * 2)) * 0.3  # Always positive bend
        
        # Left leg
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=p.getConstraintUniqueId(self.left_hip),
            controlMode=p.POSITION_CONTROL,
            targetPosition=hip_angle,
            force=500
        )
        p.setJointMotorControl2(
            bodyUniqueId=self.left_upper_leg,
            jointIndex=p.getConstraintUniqueId(self.left_knee),
            controlMode=p.POSITION_CONTROL,
            targetPosition=knee_angle,
            force=500
        )
        
        # Right leg (opposite phase)
        p.setJointMotorControl2(
            bodyUniqueId=self.robot_id,
            jointIndex=p.getConstraintUniqueId(self.right_hip),
            controlMode=p.POSITION_CONTROL,
            targetPosition=-hip_angle,
            force=500
        )
        p.setJointMotorControl2(
            bodyUniqueId=self.right_upper_leg,
            jointIndex=p.getConstraintUniqueId(self.right_knee),
            controlMode=p.POSITION_CONTROL,
            targetPosition=knee_angle,
            force=500
        )
        
        p.stepSimulation()
        
    def get_position(self):
        return p.getBasePositionAndOrientation(self.robot_id)[0]
    
    def cleanup(self):
        p.disconnect(self.client)

if __name__ == "__main__":
    print("Initializing Revised Walking Robot simulation...")
    robot = RevisedWalkingRobot()
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
