import pybullet as p
import time
import numpy as np

def run_basic_test():
    print("Initializing PyBullet...")
    client = p.connect(p.DIRECT)
    print(f"Connected with ID: {client}")
    
    # Set up simulation parameters
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    
    # Create ground plane
    ground = p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, ground)
    
    # Create a simple cube
    cube_size = 0.2
    cube_mass = 1.0
    cube_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[cube_size]*3)
    cube = p.createMultiBody(
        baseMass=cube_mass,
        baseCollisionShapeIndex=cube_shape,
        basePosition=[0, 0, 1]
    )
    
    print("\nRunning basic physics simulation...")
    for i in range(100):
        p.stepSimulation()
        if i % 20 == 0:  # Print every 20 steps
            pos, ori = p.getBasePositionAndOrientation(cube)
            print(f"Step {i}: Cube position = {pos}")
        time.sleep(0.01)
    
    p.disconnect()
    print("\nBasic test completed successfully!")

if __name__ == "__main__":
    try:
        run_basic_test()
    except Exception as e:
        print(f"Error during test: {e}")
