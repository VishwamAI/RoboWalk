import numpy as np
import pybullet as p
import time
from force_analysis import ForceAnalyzer
from scipy.spatial.transform import Rotation

class WalkingController:
    def __init__(self, force_analyzer):
        self.analyzer = force_analyzer
        self.robot_id = force_analyzer.robot_id

        # Walking parameters
        self.step_height = 0.008  # Maximum foot lift height
        self.step_length = 0.02   # Forward step size
        self.cycle_time = 3.0     # Time for one complete step cycle
        self.phase_offset = np.pi  # Phase difference between legs
        self.startup_time = 2.0   # Time to gradually increase motion
        self.base_height = 0.77   # Target standing height
        self.stabilization_time = 3.0  # Time to stabilize before walking

        # Stability parameters
        self.min_stable_force = 80.0   # Minimum force per foot
        self.max_pitch = 0.15     # Maximum allowable pitch (radians)
        self.min_total_force = 180.0  # Minimum total vertical force
        self.force_ratio_threshold = 0.5  # Minimum force ratio between feet
        self.lateral_limit = 0.03  # Maximum lateral drift

        # Joint indices (from URDF)
        self.left_hip = 0
        self.right_hip = 1
        self.left_knee = 2
        self.right_knee = 3
        self.left_ankle = 4
        self.right_ankle = 5

        # Control parameters
        self.max_force = 4000     # Maximum joint force
        self.max_velocity = 0.3   # Maximum joint velocity
        self.settling_steps = 200  # Number of simulation steps for settling
        self.stance_width = 0.12  # Lateral distance between feet

        # PID control parameters
        self.kp = 0.0005         # Reduced proportional gain
        self.ki = 0.0001         # Reduced integral gain
        self.kd = 0.0002         # Reduced derivative gain

        # Debug logging parameters
        self.log_interval = 0.1   # Log every 0.1 seconds
        self.phase_log = []       # Store phase transition data
        self.force_log = []       # Store force feedback data
        self.stability_log = []   # Store stability metrics

        # Initialize stable pose
        self.reset_to_stable_pose()


    def reset_to_stable_pose(self):
        """Reset robot to a stable standing pose."""
        # Reset base position and orientation
        initial_height = self.base_height
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [0, 0, initial_height],
            p.getQuaternionFromEuler([0, 0, 0])
        )

        # Calculate initial joint angles for stable stance
        self.left_hip_angle = 0.06  # Increased outward angle for left leg
        self.right_hip_angle = -0.015  # More inward angle for right leg
        self.left_knee_angle = -0.18  # More bent left knee to shift weight
        self.right_knee_angle = -0.04  # Less bent right knee
        self.left_ankle_angle = -0.025  # More inward angle for left foot
        self.right_ankle_angle = 0.01  # Increased outward angle for right foot

        # Set initial pose with asymmetric configuration
        joints = [self.left_hip, self.left_knee, self.left_ankle,
                 self.right_hip, self.right_knee, self.right_ankle]
        angles = [self.left_hip_angle, self.left_knee_angle, self.left_ankle_angle,
                 self.right_hip_angle, self.right_knee_angle, self.right_ankle_angle]

        # Joint limits for safety
        self.joint_limits = {
            'hip': (-0.35, 0.35),
            'knee': (-0.35, 0.0),
            'ankle': (-0.25, 0.25)
        }

        # Apply initial pose with reduced forces for gentler settling
        for joint, angle in zip(joints, angles):
            p.resetJointState(self.robot_id, joint, angle)
            p.setJointMotorControl2(
                self.robot_id, joint,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=self.max_force * 0.3,  # Further reduced force for gentler contact
                maxVelocity=self.max_velocity * 0.03  # Even slower movement
            )

        # Iterative balance adjustment with force and COM feedback
        max_attempts = 100  # Reduced settling attempts
        max_time = 5.0  # Maximum time limit in seconds
        target_ratio = 0.55  # Target force ratio between feet
        best_ratio = 0
        best_angles = angles.copy()
        com_target = [-0.02, 0, self.base_height]  # Shifted COM target towards left foot
        start_time = time.time()

        for attempt in range(max_attempts):
            if time.time() - start_time > max_time:
                print("Warning: Stabilization time limit reached")
                break

            p.stepSimulation()
            left_force, right_force = self.analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]

            # Get current COM position
            com_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
            com_error = [t - c for t, c in zip(com_target, com_pos)]

            if total_force > self.min_total_force:
                force_ratio = min(left_force[2], right_force[2]) / total_force

                # Save best configuration
                if force_ratio > best_ratio:
                    best_ratio = force_ratio
                    best_angles = [p.getJointState(self.robot_id, joint)[0]
                                 for joint in joints]

                if force_ratio > target_ratio and abs(com_error[0]) < 0.01:
                    break

                # Combined force and COM-based adjustment
                force_diff = right_force[2] - left_force[2]  # Reversed to favor left foot
                lateral_adjustment = 0.005 * (force_diff / total_force)  # Increased adjustment
                com_adjustment = 0.008 * com_error[0]  # Increased COM adjustment

                # Calculate new angles with joint limits
                new_left_hip = self.left_hip_angle - lateral_adjustment - com_adjustment
                new_right_hip = self.right_hip_angle + lateral_adjustment + com_adjustment
                new_left_ankle = self.left_ankle_angle - lateral_adjustment + com_adjustment
                new_right_ankle = self.right_ankle_angle + lateral_adjustment - com_adjustment

                # Apply limits
                new_left_hip = max(self.joint_limits['hip'][0],
                                 min(self.joint_limits['hip'][1], new_left_hip))
                new_right_hip = max(self.joint_limits['hip'][0],
                                  min(self.joint_limits['hip'][1], new_right_hip))
                new_left_ankle = max(self.joint_limits['ankle'][0],
                                   min(self.joint_limits['ankle'][1], new_left_ankle))
                new_right_ankle = max(self.joint_limits['ankle'][0],
                                    min(self.joint_limits['ankle'][1], new_right_ankle))

                # Apply adjustments with force feedback
                force_scale = min(1.0, total_force / self.min_total_force)
                applied_force = self.max_force * 0.3 * force_scale  # Reduced force

                # Update joint positions
                p.setJointMotorControl2(self.robot_id, self.left_hip,
                    p.POSITION_CONTROL,
                    targetPosition=new_left_hip,
                    force=applied_force)
                p.setJointMotorControl2(self.robot_id, self.right_hip,
                    p.POSITION_CONTROL,
                    targetPosition=new_right_hip,
                    force=applied_force)
                p.setJointMotorControl2(self.robot_id, self.left_ankle,
                    p.POSITION_CONTROL,
                    targetPosition=new_left_ankle,
                    force=applied_force)
                p.setJointMotorControl2(self.robot_id, self.right_ankle,
                    p.POSITION_CONTROL,
                    targetPosition=new_right_ankle,
                    force=applied_force)

        # Restore best configuration if current isn't optimal
        if best_ratio > force_ratio:
            for joint, angle in zip(joints, best_angles):
                p.setJointMotorControl2(self.robot_id, joint,
                    p.POSITION_CONTROL,
                    targetPosition=angle,
                    force=self.max_force * 0.3)

        # Verify final ground contact and force distribution
        left_force, right_force = self.analyzer.get_contact_forces()
        total_force = left_force[2] + right_force[2]
        if total_force < self.min_total_force:
            print(f"Warning: Insufficient total force - {total_force:.1f}N")
        force_ratio = min(left_force[2], right_force[2]) / (total_force + 1e-6)
        if force_ratio < self.force_ratio_threshold:
            print(f"Warning: Uneven force distribution - L: {left_force[2]:.1f}N, R: {right_force[2]:.1f}N")

    def generate_walking_pattern(self, t):
        """Generate walking pattern based on time."""
        # Scale motion during startup with smoother acceleration
        scale = min(1.0, (t / self.startup_time)**3)  # Cubic scaling for even smoother start

        # Base walking pattern parameters with reduced initial frequency
        base_freq = 2 * np.pi / self.cycle_time
        step_freq = base_freq * (0.3 + 0.7 * scale)  # More gradual frequency increase
        phase = t * step_freq

        # Initialize foot positions
        left_lift = 0.0
        left_forward = 0.0
        right_lift = 0.0
        right_forward = 0.0

        # Get current forces for weight transfer
        left_force, right_force = self.analyzer.get_contact_forces()
        total_force = left_force[2] + right_force[2]

        if total_force > self.min_total_force:
            # Calculate weight distribution with enhanced smoothing
            left_ratio = left_force[2] / total_force
            right_ratio = right_force[2] / total_force
            force_balance = min(left_ratio, right_ratio) / max(left_ratio, right_ratio)

            # Track force ratio history for stability
            target_ratio = 0.85
            force_error = abs(force_balance - target_ratio)
            transition_scale = max(0.0, min(1.0, 1.0 - force_error * 2.0))

            # Left leg movement with force-aware transitions
            left_phase = phase
            if 0 < left_phase % (2*np.pi) < np.pi:
                # Extended pre-transfer phase (50% of cycle for stability)
                if left_phase % (2*np.pi) < 0.5*np.pi:
                    lift_scale = (1 - np.cos(left_phase % (2*np.pi) / (0.5*np.pi) * np.pi)) / 6
                    left_lift = 0.08 * self.step_height * lift_scale * transition_scale
                    left_forward = -0.08 * self.step_length * lift_scale * transition_scale
                # Main transfer phase (30% of cycle)
                elif left_phase % (2*np.pi) < 0.8*np.pi:
                    phase_norm = ((left_phase % (2*np.pi)) - 0.5*np.pi) / (0.3*np.pi)
                    height_scale = np.sin(phase_norm * np.pi) * transition_scale
                    left_lift = self.step_height * height_scale * 0.4
                    left_forward = self.step_length * (phase_norm - 0.5) * 0.4 * transition_scale
                # Post-transfer phase (20% of cycle)
                else:
                    lower_scale = (1 + np.cos((left_phase % (2*np.pi) - 0.8*np.pi) / (0.2*np.pi) * np.pi)) / 6
                    left_lift = 0.08 * self.step_height * lower_scale * transition_scale
                    left_forward = 0.08 * self.step_length * (1 - lower_scale) * transition_scale

            # Right leg movement with force-aware transitions
            right_phase = phase + np.pi
            if 0 < right_phase % (2*np.pi) < np.pi:
                # Extended pre-transfer phase
                if right_phase % (2*np.pi) < 0.5*np.pi:
                    lift_scale = (1 - np.cos(right_phase % (2*np.pi) / (0.5*np.pi) * np.pi)) / 6
                    right_lift = 0.08 * self.step_height * lift_scale * transition_scale
                    right_forward = -0.08 * self.step_length * lift_scale * transition_scale
                # Main transfer phase
                elif right_phase % (2*np.pi) < 0.8*np.pi:
                    phase_norm = ((right_phase % (2*np.pi)) - 0.5*np.pi) / (0.3*np.pi)
                    height_scale = np.sin(phase_norm * np.pi) * transition_scale
                    right_lift = self.step_height * height_scale * 0.4
                    right_forward = self.step_length * (phase_norm - 0.5) * 0.4 * transition_scale
                # Post-transfer phase
                else:
                    lower_scale = (1 + np.cos((right_phase % (2*np.pi) - 0.8*np.pi) / (0.2*np.pi) * np.pi)) / 6
                    right_lift = 0.08 * self.step_height * lower_scale * transition_scale
                    right_forward = 0.08 * self.step_length * (1 - lower_scale) * transition_scale

            # Force-based transition scaling with enhanced smoothing
            force_scale = min(1.0, force_balance * 1.5)  # Reduced force scaling
            smooth_scale = (1 - np.cos(force_scale * np.pi)) / 6  # Further reduced scaling

            # Apply force-based scaling
            left_lift *= smooth_scale
            right_lift *= smooth_scale

            # Apply startup scaling with gentler transitions
            startup_smooth = (1 - np.cos(scale * np.pi)) / 6  # Further reduced startup scaling
            left_lift *= startup_smooth
            left_forward *= startup_smooth
            right_lift *= startup_smooth
            right_forward *= startup_smooth

        return left_lift, left_forward, right_lift, right_forward

    def inverse_kinematics(self, foot_height, foot_forward, is_left):
        """Convert foot position to joint angles."""
        # Robot geometry
        l1 = 0.4  # Upper leg length
        l2 = 0.4  # Lower leg length

        # Target foot position relative to hip
        x = foot_forward
        z = -self.base_height + foot_height

        try:
            # Calculate total distance to target
            total_dist = np.sqrt(x*x + z*z)
            target_angle = np.arctan2(x, -z)  # Angle from vertical to target

            # Validate target is within workspace
            max_reach = l1 + l2
            min_reach = 0.75  # Slightly less than minimum reach for better range
            if total_dist > max_reach * 0.97 or total_dist < min_reach:
                print(f"Warning: Target distance {total_dist:.3f}m near limits [{min_reach:.3f}, {max_reach*0.97:.3f}]")
                total_dist = np.clip(total_dist, min_reach, max_reach * 0.97)

            # Use law of cosines to find knee angle
            cos_knee = (total_dist**2 - l1**2 - l2**2) / (-2 * l1 * l2)
            cos_knee = np.clip(cos_knee, -1.0, 1.0)
            knee_angle = -(np.pi - np.arccos(cos_knee))
            knee_angle = np.clip(knee_angle, -0.35, 0.0)  # Limit knee extension

            # Find hip angle using target angle and leg geometry
            inner_angle = np.arccos((l1**2 + total_dist**2 - l2**2) / (2 * l1 * total_dist))
            hip_angle = target_angle - inner_angle
            hip_angle = np.clip(hip_angle, -0.35, 0.35)  # Limit hip motion

            # Apply stance angle offset for left/right leg
            stance = self.stance_width if is_left else -self.stance_width
            hip_angle += stance

            # Calculate ankle angle to keep foot parallel to ground
            ankle_angle = -hip_angle - knee_angle

            return hip_angle, knee_angle, ankle_angle

        except Exception as e:
            print(f"IK failed: {str(e)}")
            return 0.0, -0.1, 0.1  # Slightly bent safe position

    def apply_joint_positions(self, t, phase="walking"):
        """Apply walking pattern to robot joints."""
        # Get current ground forces
        left_force, right_force = self.analyzer.get_contact_forces()
        total_force = left_force[2] + right_force[2]

        # Emergency stop check
        if total_force < self.min_total_force:
            print("Warning: Insufficient ground contact force")
            return False

        if phase == "stabilize":
            # Use current joint angles for stabilization
            joints = [self.left_hip, self.left_knee, self.left_ankle,
                     self.right_hip, self.right_knee, self.right_ankle]
            angles = [self.left_hip_angle, self.left_knee_angle, self.left_ankle_angle,
                     self.right_hip_angle, self.right_knee_angle, self.right_ankle_angle]
        else:
            # Generate foot positions with force feedback
            left_lift, left_forward, right_lift, right_forward = self.generate_walking_pattern(t)

            # Calculate force-based velocity scaling
            force_ratio = min(left_force[2], right_force[2]) / (total_force + 1e-6)
            velocity_scale = min(1.0, 2.0 * force_ratio)

            # Convert to joint angles
            left_angles = self.inverse_kinematics(left_lift, left_forward, True)
            right_angles = self.inverse_kinematics(right_lift, right_forward, False)

            # Prepare joint arrays
            joints = [self.left_hip, self.left_knee, self.left_ankle,
                     self.right_hip, self.right_knee, self.right_ankle]
            angles = list(left_angles) + list(right_angles)

        # Apply joint positions with force-based velocity control
        for joint, angle in zip(joints, angles):
            p.setJointMotorControl2(
                self.robot_id,
                joint,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=self.max_force * 0.8,  # Reduced force for smoother motion
                maxVelocity=self.max_velocity * (0.5 if phase == "stabilize" else velocity_scale)
            )

        return True

    def start_walking(self, duration=10.0):
        """Start the walking motion for specified duration."""
        print("\nInitializing stable pose...")
        self.reset_to_stable_pose()

        # Initialize phase tracking
        current_phase = "weight_balancing"  # phases: weight_balancing, transition, walking
        phase_start_time = time.time()
        start_time = time.time()
        last_print_time = start_time
        stable_steps = 0
        recovery_attempts = 0
        max_recovery_attempts = 3
        phase_durations = {
            "weight_balancing": 5.0,  # Extended balancing phase
            "transition": 3.0,       # Smooth transition phase
            "walking": duration
        }

        # Initialize feedback control parameters with reduced gains
        target_force_ratio = 0.95     # Higher target ratio for better balance
        force_error_sum = 0.0        # Integral term for force control
        last_force_error = 0.0       # For derivative term
        kp, ki, kd = 0.0003, 0.0001, 0.0001  # Further reduced PID gains

        # Initialize logging
        phase_history = []
        force_history = []
        stability_metrics = []

        while time.time() - start_time < duration:
            p.stepSimulation()
            time.sleep(1./240.)

            # Get robot state
            pos, ori = p.getBasePositionAndOrientation(self.robot_id)
            euler = p.getEulerFromQuaternion(ori)
            pitch = abs(euler[1])

            # Get force feedback
            left_force, right_force = self.analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]
            force_ratio = min(left_force[2], right_force[2]) / (max(left_force[2], right_force[2]) + 1e-6)

            # Log current state
            force_history.append((left_force[2], right_force[2], total_force))
            stability_metrics.append((pitch, force_ratio))

            # Force-based feedback control with micro-adjustments
            force_error = target_force_ratio - force_ratio
            force_error_sum = max(-0.5, min(0.5, force_error_sum + force_error))  # Tighter integral limits
            force_error_derivative = force_error - last_force_error
            force_adjustment = (kp * force_error +
                             ki * force_error_sum +
                             kd * force_error_derivative)
            force_adjustment = max(-0.005, min(0.005, force_adjustment))  # Smaller adjustment limit
            last_force_error = force_error

            # Phase-specific stability checks
            if current_phase == "weight_balancing":
                is_stable = force_ratio > 0.85 and pitch < np.radians(1.0)
            else:
                is_stable = force_ratio > 0.7 and pitch < np.radians(2.0)

            # Update stability tracking
            if is_stable:
                stable_steps += 1
            else:
                stable_steps = 0

            # Phase transition logic
            phase_duration = time.time() - phase_start_time
            if current_phase == "weight_balancing":
                if stable_steps >= 100 and phase_duration >= phase_durations["weight_balancing"]:
                    current_phase = "transition"
                    phase_start_time = time.time()
                    print("\nTransitioning to transition phase")
                    print(f"Balance achieved - Force ratio: {force_ratio:.3f}, Pitch: {np.degrees(pitch):.1f}°")
            elif current_phase == "transition":
                if phase_duration >= phase_durations["transition"]:
                    current_phase = "walking"
                    phase_start_time = time.time()
                    print("\nStarting walking phase")

            # Apply phase-specific control
            if current_phase == "weight_balancing":
                self._adjust_com_position(force_adjustment)
            elif current_phase == "transition":
                progress = phase_duration / phase_durations["transition"]
                shift = 0.002 * np.sin(np.pi * progress)  # Micro-oscillation
                self._adjust_com_position(shift)
            elif current_phase == "walking":
                if not self.apply_joint_positions(time.time() - phase_start_time, "walking"):
                    print("Walking motion failed - emergency stop")
                    print(f"Final metrics - Force ratio: {force_ratio:.3f}, Pitch: {np.degrees(pitch):.1f}°")
                    break

            # Print detailed status updates
            current_time = time.time()
            if current_time - last_print_time >= 0.5:
                print(f"\n=== {current_phase.capitalize()} Status ===")
                print(f"Time: {current_time - start_time:.1f}s")
                print(f"Pitch: {np.degrees(pitch):.1f}°")
                print(f"Total force: {total_force:.1f}N")
                print(f"Force ratio: {force_ratio:.3f}")
                print(f"Force adjustment: {force_adjustment:.4f}")
                print(f"Stable steps: {stable_steps}")
                print(f"Error sum: {force_error_sum:.4f}")
                last_print_time = current_time

            # Check emergency stop conditions
            if (force_ratio < 0.15 or pitch > np.radians(5.0) or
                total_force < 150.0 or total_force > 400.0):
                print("Emergency stop triggered")
                print(f"Final metrics - Force ratio: {force_ratio:.3f}, Pitch: {np.degrees(pitch):.1f}°")
                break

    def _check_phase_stability(self, phase, pitch, total_force, force_ratio, pos):
        """Check stability based on current phase."""
        if phase == "stabilizing":
            return (pitch < 0.15 and          # ~8.6 degrees pitch
                    total_force > 250 and     # Minimum force
                    force_ratio > 0.65 and    # Weight distribution
                    abs(pos[1]) < 0.05)       # Lateral constraint
        elif phase == "weight_shift":
            return (pitch < 0.2 and           # ~11.5 degrees pitch
                    total_force > 200 and     # Reduced force requirement
                    force_ratio > 0.45)       # Less strict distribution
        else:  # walking phase
            return (pitch < 0.25 and          # ~14.3 degrees pitch
                    total_force > 180 and     # Minimum walking force
                    force_ratio > 0.35)       # Dynamic distribution

    def _handle_phase_transition(self, current_phase, stable_steps, force_ratio,
                               phase_start_time, phase_durations, recovery_attempts,
                               max_recovery_attempts):
        """Handle phase transitions based on stability and timing."""
        phase_elapsed = time.time() - phase_start_time

        if current_phase == "stabilizing":
            if stable_steps > 120:  # 0.5 seconds of stability
                return "weight_shift"
            elif phase_elapsed > phase_durations["stabilizing"]:
                if recovery_attempts < max_recovery_attempts:
                    print("Stabilization timeout - attempting recovery")
                    self.reset_to_stable_pose()
                    return "stabilizing"
                else:
                    print("Max recovery attempts reached")
                    return "emergency_stop"

        elif current_phase == "weight_shift":
            if stable_steps > 60 and force_ratio > 0.45:  # 0.25s stable weight shift
                return "walking"
            elif phase_elapsed > phase_durations["weight_shift"]:
                return "stabilizing"

        return current_phase

    def _check_emergency_stop(self, phase, pitch, total_force, force_ratio):
        """Check if emergency stop conditions are met."""
        if phase == "stabilizing":
            return (pitch > 0.3 or            # ~17.2 degrees
                    total_force < 150 or      # Minimum force
                    force_ratio < 0.3)        # Severe imbalance
        elif phase == "weight_shift":
            return (pitch > 0.35 or           # ~20.1 degrees
                    total_force < 120 or      # Critical force
                    force_ratio < 0.25)       # Critical imbalance
        else:  # walking phase
            return (pitch > 0.4 or            # ~22.9 degrees
                    total_force < 100 or      # Emergency force threshold
                    force_ratio < 0.2)        # Emergency imbalance

    def _adjust_com_position(self, shift_amount):
        """Adjust the robot's center of mass position."""
        max_iterations = 30  # Increased maximum iterations
        best_ratio = 0.0    # Track best force ratio achieved
        best_positions = None  # Store joint positions for best ratio
        target_ratio = 0.85  # Higher target force ratio
        convergence_threshold = 0.01  # Tighter convergence requirement
        position_history = []  # Track position changes

        # Get initial positions
        initial_left_hip = p.getJointState(self.robot_id, self.left_hip)[0]
        initial_right_hip = p.getJointState(self.robot_id, self.right_hip)[0]
        initial_left_ankle = p.getJointState(self.robot_id, self.left_ankle)[0]
        initial_right_ankle = p.getJointState(self.robot_id, self.right_ankle)[0]

        for iteration in range(max_iterations):
            # Get current joint states
            left_hip_pos = p.getJointState(self.robot_id, self.left_hip)[0]
            right_hip_pos = p.getJointState(self.robot_id, self.right_hip)[0]
            left_ankle_pos = p.getJointState(self.robot_id, self.left_ankle)[0]
            right_ankle_pos = p.getJointState(self.robot_id, self.right_ankle)[0]

            # Calculate total position change
            total_change = abs(left_hip_pos - initial_left_hip) + abs(right_hip_pos - initial_right_hip) + \
                         abs(left_ankle_pos - initial_left_ankle) + abs(right_ankle_pos - initial_right_ankle)

            # Get current force ratio
            left_force, right_force = self.analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]
            force_ratio = min(left_force[2], right_force[2]) / (total_force + 1e-6)
            ratio_error = abs(target_ratio - force_ratio)

            # Store position history
            position_history.append((force_ratio, (left_hip_pos, right_hip_pos, left_ankle_pos, right_ankle_pos)))

            # Check if this is the best ratio so far
            if force_ratio > best_ratio:
                best_ratio = force_ratio
                best_positions = (left_hip_pos, right_hip_pos, left_ankle_pos, right_ankle_pos)

            # Check for convergence or excessive movement
            if ratio_error < convergence_threshold or total_change > 0.2:
                break

            # Adaptive adjustment based on force ratio and iteration count
            base_adjustment = 0.001  # Smaller base adjustment
            ratio_factor = max(0.1, min(1.0, ratio_error))  # Scale based on error
            iteration_factor = max(0.2, 1.0 - iteration / max_iterations)  # Decrease with iterations
            adjusted_shift = base_adjustment * ratio_factor * iteration_factor

            # Calculate new positions with stricter joint limits
            new_left_hip = max(-0.25, min(0.25, left_hip_pos + adjusted_shift))
            new_right_hip = max(-0.25, min(0.25, right_hip_pos - adjusted_shift * 0.8))  # Asymmetric adjustment
            new_left_ankle = max(-0.15, min(0.15, left_ankle_pos + adjusted_shift * 0.5))
            new_right_ankle = max(-0.15, min(0.15, right_ankle_pos - adjusted_shift * 0.4))

            # Apply positions with smooth force scaling
            force_scale = max(0.3, min(0.8, 1.0 - ratio_error))  # Smoother force scaling
            applied_force = self.max_force * force_scale

            # Apply new positions with reduced force
            p.setJointMotorControl2(self.robot_id, self.left_hip,
                p.POSITION_CONTROL, targetPosition=new_left_hip, force=applied_force)
            p.setJointMotorControl2(self.robot_id, self.right_hip,
                p.POSITION_CONTROL, targetPosition=new_right_hip, force=applied_force)
            p.setJointMotorControl2(self.robot_id, self.left_ankle,
                p.POSITION_CONTROL, targetPosition=new_left_ankle, force=applied_force)
            p.setJointMotorControl2(self.robot_id, self.right_ankle,
                p.POSITION_CONTROL, targetPosition=new_right_ankle, force=applied_force)
