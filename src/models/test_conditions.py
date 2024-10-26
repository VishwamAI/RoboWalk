import pybullet as p
import time
import math
import os
import numpy as np

class RobotTester:
    def __init__(self):
        # Initialize PyBullet
        self.client = p.connect(p.DIRECT)
        p.setGravity(0, 0, -9.81)
        p.setRealTimeSimulation(0)
        
        # Get the path to the URDF file
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.urdf_path = os.path.join(current_dir, 'simple_biped.urdf')
        
    def setup_environment(self, friction=0.7, slope=0.0):
        # Reset simulation
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        
        # Create ground plane
        ground_shape = p.createCollisionShape(p.GEOM_PLANE)
        
        # Set ground orientation for slope
        if slope != 0.0:
            ground_orn = p.getQuaternionFromEuler([0, slope, 0])
        else:
            ground_orn = [0, 0, 0, 1]
            
        ground = p.createMultiBody(0, ground_shape, baseOrientation=ground_orn)
        
        # Set ground friction
        p.changeDynamics(ground, -1, lateralFriction=friction)
        
        # Load robot
        self.robot = p.loadURDF(
            self.urdf_path,
            basePosition=[0, 0, 1],
            useFixedBase=False
        )
        
        # Get and store joint information
        self.joint_ids = {}
        for i in range(p.getNumJoints(self.robot)):
            joint_info = p.getJointInfo(self.robot, i)
            joint_name = joint_info[1].decode('utf-8')
            self.joint_ids[joint_name] = i
    
    def run_test(self, duration=500, frequency=2.0, amplitude=0.2):
        positions = []
        
        for i in range(duration):
            t = i * 0.01
            # Generate walking motion
            left_angle = math.sin(t * frequency) * amplitude
            right_angle = -left_angle
            
            # Apply joint positions
            if 'left_hip' in self.joint_ids:
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot,
                    jointIndex=self.joint_ids['left_hip'],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=left_angle,
                    force=100
                )
            
            if 'right_hip' in self.joint_ids:
                p.setJointMotorControl2(
                    bodyUniqueId=self.robot,
                    jointIndex=self.joint_ids['right_hip'],
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=right_angle,
                    force=100
                )
            
            p.stepSimulation()
            
            if i % 50 == 0:
                pos = p.getBasePositionAndOrientation(self.robot)[0]
                positions.append(pos)
                
            time.sleep(0.01)
            
        return positions
    
    def cleanup(self):
        p.disconnect(self.client)

def main():
    print("Starting Robot Testing Suite...")
    tester = RobotTester()
    
    # Test conditions
    test_conditions = [
        {"name": "Normal Ground", "friction": 0.7, "slope": 0.0},
        {"name": "Low Friction", "friction": 0.2, "slope": 0.0},
        {"name": "Uphill", "friction": 0.7, "slope": 0.1},
        {"name": "High Frequency", "friction": 0.7, "slope": 0.0, "frequency": 3.0},
    ]
    
    try:
        for condition in test_conditions:
            print(f"\nTesting condition: {condition['name']}")
            tester.setup_environment(
                friction=condition['friction'],
                slope=condition['slope']
            )
            
            frequency = condition.get('frequency', 2.0)
            positions = tester.run_test(frequency=frequency)
            
            # Calculate total distance traveled
            start_pos = positions[0]
            end_pos = positions[-1]
            distance = math.sqrt(sum((a - b) ** 2 for a, b in zip(end_pos, start_pos)))
            
            print(f"Results for {condition['name']}:")
            print(f"- Distance traveled: {distance:.3f}m")
            print(f"- Final height: {end_pos[2]:.3f}m")
            print(f"- Forward/Backward movement: {end_pos[1]:.3f}m")
            
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        tester.cleanup()
        print("\nTesting completed")

if __name__ == "__main__":
    main()
