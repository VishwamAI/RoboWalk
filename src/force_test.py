import pybullet as p
import pybullet_data
import time
import numpy as np

# Initialize PyBullet
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set up simulation
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

# Load ground plane
planeId = p.loadURDF("plane.urdf")

# Create a simple box
boxStartPos = [0, 0, 1]
boxStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
boxId = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
boxMass = 1.0
boxUid = p.createMultiBody(boxMass, boxId, -1, boxStartPos, boxStartOrientation)

# Set box dynamics
p.changeDynamics(boxUid, -1,
                lateralFriction=0.5,
                spinningFriction=0.1,
                rollingFriction=0.1,
                restitution=0.1,
                contactStiffness=30000,
                contactDamping=1000)

print("\nStarting simulation...")
print("Waiting for box to settle...")

def analyze_contact_forces(contacts):
    total_normal_force = 0  # Scalar total normal force
    total_force_vector = np.zeros(3)  # 3D force vector

    for i, contact in enumerate(contacts):
        # Print raw contact data for inspection
        print(f"\nContact point {i} raw data:")
        for j, value in enumerate(contact):
            print(f"  Index {j}: {value}")

        # Extract normal force (scalar)
        normal_force = float(contact[9])  # Normal force magnitude
        total_normal_force += abs(normal_force)

        # Extract normal direction from tuple
        normal_dir_tuple = contact[7]  # This is a tuple (nx, ny, nz)
        normal_dir = np.array([float(normal_dir_tuple[0]),
                             float(normal_dir_tuple[1]),
                             float(normal_dir_tuple[2])])

        # Calculate force vector for this contact
        force_vector = normal_dir * abs(normal_force)
        total_force_vector += force_vector

        # Print detailed force information for this contact
        print(f"\nContact {i} force details:")
        print(f"  Normal force magnitude: {abs(normal_force):.3f}N")
        print(f"  Normal direction: [{normal_dir[0]:.3f}, {normal_dir[1]:.3f}, {normal_dir[2]:.3f}]")
        print(f"  Force vector: [{force_vector[0]:.3f}, {force_vector[1]:.3f}, {force_vector[2]:.3f}]N")

    return total_normal_force, total_force_vector

# Run simulation
last_height = boxStartPos[2]
settling_threshold = 1e-4  # Height change threshold for settling
settled_steps = 0

for i in range(500):
    p.stepSimulation()

    # Get box state
    pos, orn = p.getBasePositionAndOrientation(boxUid)
    lin_vel, ang_vel = p.getBaseVelocity(boxUid)

    # Check settling
    height_change = abs(pos[2] - last_height)
    last_height = pos[2]

    if height_change < settling_threshold:
        settled_steps += 1
    else:
        settled_steps = 0

    if i % 20 == 0:
        print(f"\nStep {i}:")
        print(f"Position: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        print(f"Linear velocity: [{lin_vel[0]:.3f}, {lin_vel[1]:.3f}, {lin_vel[2]:.3f}]")
        print(f"Angular velocity: [{ang_vel[0]:.3f}, {ang_vel[1]:.3f}, {ang_vel[2]:.3f}]")

        # Get contact points
        contacts = p.getContactPoints(boxUid, planeId)
        if contacts:
            print(f"Number of contact points: {len(contacts)}")
            total_normal_force, force_vector = analyze_contact_forces(contacts)

            print("\nForce Analysis:")
            print(f"Total normal force: {total_normal_force:.3f}N")
            print(f"Total force vector: [{force_vector[0]:.3f}, {force_vector[1]:.3f}, {force_vector[2]:.3f}]N")
            print(f"Force vector magnitude: {np.linalg.norm(force_vector):.3f}N")
            print(f"Expected force (mass * g): {boxMass * 9.81:.3f}N")

            for j, contact in enumerate(contacts):
                pos_on_box = contact[5]
                pos_on_ground = contact[6]
                print(f"\nContact point {j+1} details:")
                print(f"Position on box: [{pos_on_box[0]:.3f}, {pos_on_box[1]:.3f}, {pos_on_box[2]:.3f}]")
                print(f"Position on ground: [{pos_on_ground[0]:.3f}, {pos_on_ground[1]:.3f}, {pos_on_ground[2]:.3f}]")
                print(f"Contact distance: {contact[8]:.6f}")
        else:
            print("No contacts detected")

    if settled_steps >= 50:
        print("\nBox has settled!")
        break

    time.sleep(0.01)

p.disconnect()
