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
        self.step_height = 0.015  # Maximum foot lift height
        self.step_length = 0.04   # Forward step size
        self.cycle_time = 2.5     # Time for one complete step cycle
        self.phase_offset = np.pi  # Phase difference between legs
        self.startup_time = 1.0   # Time to gradually increase motion
        self.base_height = 0.77   # Target standing height
        self.stabilization_time = 2.0  # Time to stabilize before walking

        # Stability parameters
        self.min_stable_force = 100.0  # Minimum force per foot for stability
        self.max_pitch = 0.2      # Maximum allowable pitch (radians)
        self.min_total_force = 200.0  # Minimum total vertical force
        self.force_ratio_threshold = 0.2  # Minimum force ratio between feet

        # Joint indices (from URDF)
        self.left_hip = 0
        self.right_hip = 1
        self.left_knee = 2
        self.right_knee = 3
        self.left_ankle = 4
        self.right_ankle = 5

        # Control parameters
        self.max_force = 5000     # Maximum joint force
        self.max_velocity = 0.5   # Maximum joint velocity
        self.settling_steps = 100  # Number of simulation steps for settling

        # Initialize stable pose
        self.reset_to_stable_pose()

    def reset_to_stable_pose(self):
        """Reset robot to a stable standing pose."""
        # Reset base position and orientation
        initial_height = 0.77  # Target standing height
        p.resetBasePositionAndOrientation(
            self.robot_id,
            [0, 0, initial_height],
            p.getQuaternionFromEuler([0, 0, 0])
        )

        # Calculate initial joint angles for stable stance
        hip_angle = 0.0  # Start with neutral hip angle
        knee_angle = -0.2  # Slight knee bend
        ankle_angle = 0.0  # Start with neutral ankle

        # Set initial pose with symmetric configuration
        joints = [self.left_hip, self.left_knee, self.left_ankle,
                 self.right_hip, self.right_knee, self.right_ankle]
        angles = [hip_angle, knee_angle, ankle_angle,
                 -hip_angle, knee_angle, -ankle_angle]

        # Apply initial pose with reduced forces for gentler settling
        for joint, angle in zip(joints, angles):
            p.resetJointState(self.robot_id, joint, angle)
            p.setJointMotorControl2(
                self.robot_id, joint,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=self.max_force * 0.7,  # Reduced force for gentler contact
                maxVelocity=self.max_velocity * 0.1  # Very slow movement
            )

        # Iterative balance adjustment with force feedback
        max_attempts = 150  # Increased settling attempts
        target_ratio = 0.48  # Higher target force ratio
        best_ratio = 0
        best_angles = angles.copy()

        for attempt in range(max_attempts):
            p.stepSimulation()
            left_force, right_force = self.analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]

            if total_force > self.min_total_force:
                force_ratio = min(left_force[2], right_force[2]) / total_force

                # Save best configuration
                if force_ratio > best_ratio:
                    best_ratio = force_ratio
                    best_angles = [p.getJointState(self.robot_id, joint)[0] for joint in joints]

                if force_ratio > target_ratio:
                    break

                # Fine-grained dynamic adjustment
                force_diff = left_force[2] - right_force[2]
                adjustment = 0.001 * (force_diff / total_force)  # Proportional adjustment

                # Adjust both hips and ankles
                p.setJointMotorControl2(self.robot_id, self.left_hip, p.POSITION_CONTROL,
                                      targetPosition=hip_angle - adjustment,
                                      force=self.max_force * 0.7)
                p.setJointMotorControl2(self.robot_id, self.right_hip, p.POSITION_CONTROL,
                                      targetPosition=-hip_angle + adjustment,
                                      force=self.max_force * 0.7)
                p.setJointMotorControl2(self.robot_id, self.left_ankle, p.POSITION_CONTROL,
                                      targetPosition=ankle_angle - adjustment,
                                      force=self.max_force * 0.7)
                p.setJointMotorControl2(self.robot_id, self.right_ankle, p.POSITION_CONTROL,
                                      targetPosition=-ankle_angle + adjustment,
                                      force=self.max_force * 0.7)

        # Restore best configuration if current isn't optimal
        if best_ratio > force_ratio:
            for joint, angle in zip(joints, best_angles):
                p.setJointMotorControl2(self.robot_id, joint, p.POSITION_CONTROL,
                                      targetPosition=angle,
                                      force=self.max_force * 0.7)

        # Verify final ground contact and force distribution
        left_force, right_force = self.analyzer.get_contact_forces()
        total_force = left_force[2] + right_force[2]
        if total_force < self.min_total_force:
            print(f"Warning: Insufficient total force - {total_force:.1f}N")
        force_ratio = min(left_force[2], right_force[2]) / (total_force + 1e-6)
        if force_ratio < 0.4:
            print(f"Warning: Uneven force distribution - L: {left_force[2]:.1f}N, R: {right_force[2]:.1f}N")

    def generate_walking_pattern(self, t):
        """Generate walking pattern based on time."""
        # Calculate startup scale (0 to 1 over startup_time)
        startup_scale = min(1.0, t / self.startup_time)
        startup_scale = startup_scale * startup_scale  # Quadratic scaling for smoother start

        # Basic cycle timing
        omega = 2.0 * np.pi / self.cycle_time
        phase = omega * t

        # Multi-phase weight transfer timing
        pre_transfer_phase = phase - np.pi/3  # Earlier pre-transfer
        main_transfer_phase = phase - np.pi/6  # Main weight shift
        post_transfer_phase = phase  # Stabilization phase

        # Calculate weight distribution with three-phase transition
        pre_weight = 0.5 + 0.2 * np.cos(pre_transfer_phase)  # Gentle initial shift
        main_weight = 0.5 + 0.3 * np.cos(main_transfer_phase)  # Main transfer
        post_weight = 0.5 + 0.1 * np.cos(post_transfer_phase)  # Final adjustment

        # Combine weight phases with smooth interpolation
        transition_pre = np.clip(np.sin(pre_transfer_phase), 0, 1)
        transition_main = np.clip(np.sin(main_transfer_phase), 0, 1)
        transition_post = np.clip(np.sin(post_transfer_phase), 0, 1)

        left_weight = (pre_weight * (1-transition_pre) +
                      main_weight * transition_pre * (1-transition_main) +
                      post_weight * transition_main * transition_post)
        right_weight = 1.0 - left_weight  # Ensure total weight is 1.0

        # Gradual foot lifting with force-based thresholds
        lift_threshold = 0.3  # More conservative lift threshold
        lift_scale = np.clip((lift_threshold - left_weight) / lift_threshold, 0, 1)
        left_lift = self.step_height * 0.5 * np.sin(phase) * startup_scale * lift_scale

        right_lift_scale = np.clip((lift_threshold - right_weight) / lift_threshold, 0, 1)
        right_lift = self.step_height * 0.5 * np.sin(phase + np.pi) * startup_scale * right_lift_scale

        # Progressive forward motion with stability checks
        step_scale = 0.3  # Further reduced step length
        forward_threshold = 0.25  # Threshold for forward motion

        left_forward_scale = np.clip((forward_threshold - left_weight) / forward_threshold, 0, 1)
        left_forward = (self.step_length * step_scale *
                       np.sin(phase) * startup_scale * left_forward_scale)

        right_forward_scale = np.clip((forward_threshold - right_weight) / forward_threshold, 0, 1)
        right_forward = (self.step_length * step_scale *
                        np.sin(phase + np.pi) * startup_scale * right_forward_scale)

        return left_lift, left_forward, right_lift, right_forward

    def inverse_kinematics(self, foot_height, foot_forward, is_left):
        """Convert foot position to joint angles."""
        # Robot geometry
        l1 = 0.4  # Upper leg length
        l2 = 0.4  # Lower leg length

        # Target foot position relative to hip
        x = foot_forward
        z = -0.77 + foot_height  # Slightly higher than minimum reach (0.784m) for safety

        try:
            # Calculate total distance to target
            total_dist = np.sqrt(x*x + z*z)
            target_angle = np.arctan2(x, -z)  # Angle from vertical to target

            # Validate target is within workspace
            max_reach = l1 + l2
            min_reach = 0.75  # Slightly less than minimum reach for better range
            if total_dist > max_reach * 0.97 or total_dist < min_reach:
                print(f"Warning: Target distance {total_dist:.3f}m near limits [{min_reach:.3f}, {max_reach*0.97:.3f}]")
                # Allow slightly out of range but clip to limits
                total_dist = np.clip(total_dist, min_reach, max_reach * 0.97)

            # Calculate leg extension ratio and adjust for Z-axis accuracy
            target_ratio = total_dist / max_reach
            adjusted_ratio = target_ratio + 0.015  # Reduced Z-axis compensation
            adjusted_ratio = np.clip(adjusted_ratio, 0.7, 0.97)

            # Calculate knee angle based on adjusted ratio
            knee_joint = -0.35 * (1.0 - (adjusted_ratio - 0.7) / 0.27)
            knee_joint = np.clip(knee_joint, -0.35, 0.0)

            # Calculate leg vector orientation with bent knee
            knee_x = l1 * np.sin(0)  # Zero hip angle initially
            knee_z = -l1 * np.cos(0)
            ankle_x = knee_x + l2 * np.sin(knee_joint)
            ankle_z = knee_z - l2 * np.cos(knee_joint)
            leg_angle = np.arctan2(ankle_x, -ankle_z)

            # Calculate hip angle to achieve target orientation
            hip_joint = target_angle - leg_angle
            hip_joint = np.clip(hip_joint, -0.35, 0.35)  # Reduced hip limits

            # Apply stance angle offset for left/right leg
            stance = 0.15 if is_left else -0.15  # Reduced stance width
            hip_joint += stance

            # Keep foot parallel to ground
            ankle_joint = -hip_joint - knee_joint

            return hip_joint, knee_joint, ankle_joint

        except Exception as e:
            print(f"IK failed: {str(e)}")
            return 0.0, -0.1, 0.1  # Slightly bent safe position

    def apply_joint_positions(self, t):
        """Apply walking pattern to robot joints."""
        # Get current ground forces
        left_force, right_force = self.analyzer.get_contact_forces()
        total_force = left_force[2] + right_force[2]

        # Only proceed if we have good ground contact
        if total_force < 100:  # Increased minimum force requirement
            print("Warning: Insufficient ground contact force")
            return

        # Generate foot positions with force feedback
        left_lift, left_forward, right_lift, right_forward = self.generate_walking_pattern(t)

        # Scale motions based on force distribution
        left_ratio = left_force[2] / (total_force + 1e-6)
        right_ratio = right_force[2] / (total_force + 1e-6)

        # Implement smoother weight transfer
        weight_transfer_threshold = 0.6  # Reduced from 0.7
        transfer_scale = 0.7  # Smoother reduction factor

        # Reduce motion on heavily loaded leg with smoother scaling
        if left_ratio > weight_transfer_threshold:
            scale = 1.0 - (left_ratio - weight_transfer_threshold) / (1 - weight_transfer_threshold)
            scale = max(transfer_scale, scale)
            left_lift *= scale
            left_forward *= scale
        if right_ratio > weight_transfer_threshold:
            scale = 1.0 - (right_ratio - weight_transfer_threshold) / (1 - weight_transfer_threshold)
            scale = max(transfer_scale, scale)
            right_lift *= scale
            right_forward *= scale

        # Reduce step length for better stability
        step_scale = 0.6  # Reduced step length
        left_forward *= step_scale
        right_forward *= step_scale

        # Convert to joint angles
        left_angles = self.inverse_kinematics(left_lift, left_forward, True)
        right_angles = self.inverse_kinematics(right_lift, right_forward, False)

        # Apply to joints with force-based velocity control
        joints = [self.left_hip, self.left_knee, self.left_ankle,
                 self.right_hip, self.right_knee, self.right_ankle]
        angles = list(left_angles) + list(right_angles)

        for joint, angle in zip(joints, angles):
            # Adjust velocity based on force ratio for smoother transitions
            velocity_scale = min(1.0, 2.0 * min(left_ratio, right_ratio))
            p.setJointMotorControl2(
                self.robot_id,
                joint,
                p.POSITION_CONTROL,
                targetPosition=angle,
                force=self.max_force * 0.8,  # Reduced force for smoother motion
                maxVelocity=self.max_velocity * velocity_scale
            )

python
    def start_walking(self, duration=10.0):
        """Start the walking motion for specified duration."""
        print("\nInitializing stable pose...")
        self.reset_to_stable_pose()

        # Wait for initial stabilization
        print("\nWaiting for stability...")
        start_time = time.time()
        last_print_time = 0
        stable_steps = 0
        max_pitch = 0
        min_force = float('inf')
        stabilization_start = time.time()

        # Phase tracking with explicit timing
        current_phase = "stabilizing"  # phases: stabilizing, weight_shift, walking
        phase_start_time = time.time()
        weight_shift_complete = False
        recovery_attempts = 0
        max_recovery_attempts = 3
        phase_durations = {
            "stabilizing": 3.0,
            "weight_shift": 2.0
        }

        while time.time() - stabilization_start < self.stabilization_time:
            p.stepSimulation()
            time.sleep(1./240.)

            # Get robot state
            pos, ori = p.getBasePositionAndOrientation(self.robot_id)
            euler = p.getEulerFromQuaternion(ori)
            pitch = abs(euler[1])  # Y-axis rotation

            # Get force feedback
            left_force, right_force = self.analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]
            force_ratio = min(left_force[2], right_force[2]) / (max(left_force[2], right_force[2]) + 1e-6)

            # Update stability metrics
            max_pitch = max(max_pitch, pitch)
            min_force = min(min_force, total_force)

            # Phase-specific stability conditions
            if current_phase == "stabilizing":
                is_stable = (
                    pitch < 0.15 and          # ~8.6 degrees pitch
                    total_force > 250 and     # Minimum force
                    force_ratio > 0.65 and    # Weight distribution
                    abs(pos[1]) < 0.05        # Lateral constraint
                )
            else:  # weight_shift phase
                is_stable = (
                    pitch < 0.2 and           # ~11.5 degrees pitch
                    total_force > 200 and     # Reduced force requirement
                    force_ratio > 0.45        # Less strict distribution
                )

            if is_stable:
                stable_steps += 1
                if stable_steps % 60 == 0:  # Debug output every 0.25s of stability
                    print(f"Continuous stable steps: {stable_steps}")
            else:
                if stable_steps > 0:  # Only print reset if we had some stability
                    print(f"Stability lost at {stable_steps} steps")
                stable_steps = 0

            # Print detailed status every 0.5 seconds
            current_time = time.time()
            phase_elapsed = current_time - phase_start_time
            if current_time - last_print_time >= 0.5:
                print(f"\n=== {current_phase.capitalize()} Status at {phase_elapsed:.1f}s ===")
                print(f"Pitch: {np.degrees(pitch):.1f}°")
                print(f"Total force: {total_force:.1f}N")
                print(f"Force ratio: {force_ratio:.2f}")
                print(f"Stable steps: {stable_steps}")
                print(f"Left force: {left_force[2]:.1f}N")
                print(f"Right force: {right_force[2]:.1f}N")
                last_print_time = current_time

            # Phase transition logic
            if current_phase == "stabilizing" and stable_steps >= 240:
                print("\n=== PHASE TRANSITION ===")
                print("Stability achieved! Starting weight shift phase...")
                print(f"Final stabilization metrics:")
                print(f"- Stable steps: {stable_steps}")
                print(f"- Force ratio: {force_ratio:.2f}")
                print(f"- Total force: {total_force:.1f}N")
                current_phase = "weight_shift"
                phase_start_time = time.time()
                stable_steps = 0  # Reset for new phase
                break

        # Begin walking sequence with enhanced monitoring and phased transitions
        start_time = time.time()
        while time.time() - start_time < duration:
            t = time.time() - start_time
            current_time = time.time()

            # Get robot state
            pos, ori = p.getBasePositionAndOrientation(self.robot_id)
            euler = p.getEulerFromQuaternion(ori)
            pitch = abs(euler[1])  # Y-axis rotation

            # Get force feedback
            left_force, right_force = self.analyzer.get_contact_forces()
            total_force = left_force[2] + right_force[2]
            force_ratio = min(left_force[2], right_force[2]) / (max(left_force[2], right_force[2]) + 1e-6)

            # Phase-specific stability checks and transitions
            if current_phase == "weight_shift":
                phase_elapsed = time.time() - phase_start_time
                if force_ratio > 0.45 and stable_steps >= 120:  # 0.5s of stability
                    if not weight_shift_complete:
                        print("\n=== PHASE TRANSITION ===")
                        print("Weight shift successful, initiating walking...")
                        print(f"- Time in weight shift: {phase_elapsed:.1f}s")
                        print(f"- Final force ratio: {force_ratio:.2f}")
                        current_phase = "walking"
                        phase_start_time = time.time()
                        weight_shift_complete = True
                elif phase_elapsed > phase_durations["weight_shift"]:
                    if recovery_attempts < max_recovery_attempts:
                        print("\n=== RECOVERY ATTEMPT ===")
                        print(f"Weight shift timeout after {phase_elapsed:.1f}s")
                        print(f"Force ratio: {force_ratio:.2f}")
                        self.reset_to_stable_pose()
                        recovery_attempts += 1
                        phase_start_time = time.time()
                        stable_steps = 0
                    else:
                        print("\n=== TERMINATION ===")
                        print("Max recovery attempts reached")
                        print(f"Final force ratio: {force_ratio:.2f}")
                        break

            # Enhanced emergency stop conditions with phase-specific thresholds
            emergency_stop = False
            if current_phase == "weight_shift":
                emergency_stop = (pitch > 0.2 or total_force < 200 or force_ratio < 0.3)
            else:  # walking phase
                emergency_stop = (
                    pitch > 0.3 or            # Reduced to ~17 degrees pitch
                    total_force < 150 or      # Increased minimum force
                    force_ratio < 0.2 or      # Force balance check
                    abs(pos[1]) > 0.1         # Reduced lateral drift limit
                )

            if emergency_stop:
                print("\nEmergency stop triggered:")
                print(f"Phase: {current_phase}")
                print(f"Pitch: {np.degrees(pitch):.1f}°")
                print(f"Total force: {total_force:.1f}N")
                print(f"Force ratio: {force_ratio:.2f}")
                print(f"Lateral position: {pos[1]:.3f}m")

                if recovery_attempts < max_recovery_attempts:
                    print("Attempting recovery...")
                    self.reset_to_stable_pose()
                    current_phase = "weight_shift"
                    weight_shift_complete = False
                    recovery_attempts += 1
                    transition_start_time = time.time()
                    continue
                else:
                    break

            # Apply walking pattern based on current phase
            if current_phase == "walking":
                self.apply_joint_positions(t)
            elif current_phase == "weight_shift":
                self.apply_joint_positions(t * 0.3)  # Slower motion during weight shift

            p.stepSimulation()
            time.sleep(1./240.)  # Match PyBullet's default timestep

            # Print detailed status every 0.5 seconds
            if current_time - last_print_time >= 0.5:
                print(f"\n=== {current_phase.capitalize()} Status at {t:.1f}s ===")
                print(f"Position: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
                print(f"Pitch: {np.degrees(pitch):.1f}°")
                print(f"Total force: {total_force:.1f}N")
                print(f"Force ratio: {force_ratio:.2f}")
                print(f"Max pitch: {np.degrees(max_pitch):.1f}°")
                print(f"Min force: {min_force:.1f}N")
                last_print_time = current_time
