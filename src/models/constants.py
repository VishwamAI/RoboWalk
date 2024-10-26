# Physical constants and parameters for the robot
ROBOT_MASS = 10.0  # kg
LEG_LENGTH = 0.5   # meters
GRAVITY = 9.81     # m/s^2

# Calculated force requirements
MIN_FORCE_PER_LEG = ROBOT_MASS * GRAVITY  # Minimum force to support weight
SAFETY_FACTOR = 2.0
RECOMMENDED_FORCE = MIN_FORCE_PER_LEG * SAFETY_FACTOR

# Control parameters
MAX_JOINT_ANGLE = 0.4  # radians
INITIAL_STANCE_WIDTH = 0.2  # meters
HEIGHT_TARGET = 0.45  # meters - adjusted to be 90% of leg length for stability

# PID Gains
DEFAULT_PID_GAINS = {
    'height': {'kp': 200.0, 'ki': 20.0, 'kd': 40.0},
    'lateral': {'kp': 10.0, 'ki': 1.0, 'kd': 2.0},
    'joints': {'kp': 100.0, 'ki': 10.0, 'kd': 20.0}
}
