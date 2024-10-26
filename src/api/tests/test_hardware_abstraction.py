"""
Unit tests for the Walking Robot Hardware Abstraction Layer
"""

import unittest
import numpy as np
from ..hardware_abstraction import (
    MemoryManager,
    NPUInterface,
    RCUInterface,
    HISInterface,
    WalkingRobotAPI
)

class TestMemoryManager(unittest.TestCase):
    def setUp(self):
        self.memory_manager = MemoryManager()
    
    def test_prefetch(self):
        """Test memory prefetching functionality"""
        test_addresses = [0x1000, 0x2000, 0x3000]
        self.memory_manager.prefetch(test_addresses)
        self.assertEqual(self.memory_manager.prefetch_queue, test_addresses)
    
    def test_attention_cache(self):
        """Test attention weight caching"""
        test_weights = np.random.rand(8, 8)
        layer_id = 1
        self.memory_manager.cache_attention_weights(layer_id, test_weights)
        cached_weights = self.memory_manager.get_cached_attention(layer_id)
        np.testing.assert_array_equal(cached_weights, test_weights)

class TestNPUInterface(unittest.TestCase):
    def setUp(self):
        self.memory_manager = MemoryManager()
        self.npu = NPUInterface(self.memory_manager)
    
    def test_matrix_multiply(self):
        """Test matrix multiplication using NPU"""
        a = np.array([[1, 2], [3, 4]])
        b = np.array([[5, 6], [7, 8]])
        result = self.npu.matrix_multiply(a, b)
        expected = np.array([[19, 22], [43, 50]])
        np.testing.assert_array_equal(result, expected)
    
    def test_gait_pattern(self):
        """Test walking gait pattern generation"""
        params = {'step_height': 0.1, 'step_length': 0.2}
        phase = 0.0
        joint_angles = self.npu.compute_gait_pattern(phase, params)
        self.assertEqual(len(joint_angles), 8)
        self.assertTrue(np.all(np.isfinite(joint_angles)))

class TestRCUInterface(unittest.TestCase):
    def setUp(self):
        self.rcu = RCUInterface()
    
    def test_joint_positions(self):
        """Test setting and getting joint positions"""
        test_positions = np.array([0.1] * 8)
        self.rcu.set_joint_positions(test_positions)
        current_positions, _ = self.rcu.get_joint_states()
        np.testing.assert_array_equal(current_positions, test_positions)
    
    def test_pid_gains(self):
        """Test PID gain updates"""
        test_kp, test_ki, test_kd = 2.0, 0.2, 0.3
        self.rcu.update_pid_gains(test_kp, test_ki, test_kd)
        self.assertEqual(self.rcu.pid_gains['kp'], test_kp)
        self.assertEqual(self.rcu.pid_gains['ki'], test_ki)
        self.assertEqual(self.rcu.pid_gains['kd'], test_kd)

class TestHISInterface(unittest.TestCase):
    def setUp(self):
        self.his = HISInterface()
    
    def test_safety_status(self):
        """Test safety status monitoring"""
        status = self.his.check_safety_status()
        self.assertFalse(status['emergency_stop'])
        self.assertEqual(status['fault_status'], 0x00)
        self.assertTrue(status['is_safe'])
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        self.his.trigger_emergency_stop()
        status = self.his.check_safety_status()
        self.assertTrue(status['emergency_stop'])
        self.assertFalse(status['is_safe'])

class TestWalkingRobotAPI(unittest.TestCase):
    def setUp(self):
        self.api = WalkingRobotAPI()
    
    def test_initialization(self):
        """Test robot initialization"""
        self.api.initialize()
        self.assertEqual(self.api.walking_phase, 0.0)
        self.assertFalse(self.api.his.emergency_stop)
        np.testing.assert_array_equal(self.api.rcu.joint_states, np.zeros(8))
    
    def test_walking_pattern(self):
        """Test walking pattern updates"""
        self.api.initialize()
        dt = 0.1
        initial_phase = self.api.walking_phase
        self.api.update_walking_pattern(dt)
        self.assertGreater(self.api.walking_phase, initial_phase)
    
    def test_system_status(self):
        """Test system status reporting"""
        status = self.api.get_system_status()
        self.assertIn('safety', status)
        self.assertIn('joint_positions', status)
        self.assertIn('joint_velocities', status)
        self.assertIn('walking_phase', status)
        self.assertIn('gait_params', status)

if __name__ == '__main__':
    unittest.main()
