import pybullet as p
import pybullet_data
import time
import os

# Initialize PyBullet
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Get absolute path to URDF
current_dir = os.path.dirname(os.path.abspath(__file__))
urdf_path = os.path.join(current_dir, "models/simple_biped_stable.urdf")

# Load plane and robot
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF(urdf_path)

# Get number of joints and links
numJoints = p.getNumJoints(robotId)
print(f"\nRobot has {numJoints} joints")

# Print information about each link
print("\nLink Information:")
print("-" * 80)
print(f"Base link (index -1): {p.getBodyInfo(robotId)[0].decode('utf-8')}")
print(f"Base link position: {p.getBasePositionAndOrientation(robotId)[0]}")
print(f"Base link orientation: {p.getBasePositionAndOrientation(robotId)[1]}")
print("-" * 80)

for i in range(numJoints):
    jointInfo = p.getJointInfo(robotId, i)
    linkName = jointInfo[12].decode('utf-8')
    parentIndex = jointInfo[16]
    jointName = jointInfo[1].decode('utf-8')
    jointType = jointInfo[2]
    linkIndex = i

    print(f"Link index {linkIndex}: {linkName}")
    print(f"  Joint name: {jointName}")
    print(f"  Joint type: {jointType}")
    print(f"  Parent link index: {parentIndex}")

    # Get link state
    linkState = p.getLinkState(robotId, linkIndex)
    print(f"  World position: {linkState[0]}")
    print(f"  World orientation: {linkState[1]}")
    print(f"  Local inertial position: {linkState[2]}")
    print(f"  Local inertial orientation: {linkState[3]}")
    print("-" * 80)

# Print joint limits and other properties
print("\nJoint Properties:")
print("-" * 80)
for i in range(numJoints):
    jointInfo = p.getJointInfo(robotId, i)
    print(f"Joint {i} ({jointInfo[1].decode('utf-8')}):")
    print(f"  Lower limit: {jointInfo[8]}")
    print(f"  Upper limit: {jointInfo[9]}")
    print(f"  Max force: {jointInfo[10]}")
    print(f"  Max velocity: {jointInfo[11]}")
    print("-" * 80)

p.disconnect()
