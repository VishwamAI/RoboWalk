import numpy as np

class CPGController:
    def __init__(self, frequency=0.8, coupling_strength=2.5):  # Slower frequency, stronger coupling
        self.frequency = frequency  # Natural frequency of oscillation
        self.coupling_strength = coupling_strength
        self.phase_left = 0.0
        self.phase_right = np.pi  # Start in anti-phase
        self.amplitude = 0.3  # Increased amplitude for stronger movements
        self.offset = 0.1  # Added forward bias
        self.dt = 1.0/240.0  # PyBullet timestep

    def update(self):
        # Update phases using coupled oscillator equations
        omega = 2 * np.pi * self.frequency
        self.phase_left += (omega + self.coupling_strength * np.sin(self.phase_right - self.phase_left - np.pi)) * self.dt
        self.phase_right += (omega + self.coupling_strength * np.sin(self.phase_left - self.phase_right - np.pi)) * self.dt

        # Normalize phases to [0, 2π]
        self.phase_left = self.phase_left % (2 * np.pi)
        self.phase_right = self.phase_right % (2 * np.pi)

        # Generate rhythmic signals
        left_signal = self.amplitude * np.sin(self.phase_left) + self.offset
        right_signal = self.amplitude * np.sin(self.phase_right) + self.offset

        return left_signal, right_signal

    def get_joint_angles(self, stance_angle, stance_width_angle):
        left_signal, right_signal = self.update()

        # Convert CPG signals to joint angles with forward bias
        left_angle = stance_angle + stance_width_angle + left_signal - 0.05  # Forward bias
        right_angle = -(stance_angle + stance_width_angle) + right_signal - 0.05  # Forward bias

        return left_angle, right_angle

    def get_phases(self):
        """Return the current phase values of both oscillators."""
        return self.phase_left, self.phase_right
