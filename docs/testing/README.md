# Testing Procedures

## Test Categories

### 1. Unit Tests
Located in `src/test_*.py` files.

#### Walking Controller Tests
```python
# test_walking_controller.py
def test_force_distribution():
    """
    Verifies force distribution between feet:
    - Target ratio: 0.85
    - Minimum total force: 180N
    - Maximum convergence time: 2.0s
    """

def test_com_adjustment():
    """
    Tests Center of Mass adjustment:
    - Position limits
    - Convergence speed
    - Stability maintenance
    """
```

### 2. Integration Tests

#### Walking Sequence Test
```bash
python src/implement_walking.py
```
Validates:
- Stable pose achievement
- Force distribution
- Walking pattern execution
- Emergency stop functionality

### 3. Performance Tests

#### Force Distribution Test
```bash
python src/force_test.py
```
Measures:
- Force ratio accuracy
- Convergence time
- Stability metrics

#### Walking Pattern Test
```bash
python src/test_walking.py
```
Evaluates:
- Step accuracy
- Cycle timing
- Energy efficiency

### 4. Validation Procedures

#### Initial Setup Validation
1. Check Python environment:
   ```bash
   python --version  # Should be 3.12
   pip list  # Verify dependencies
   ```

2. Verify PyBullet installation:
   ```bash
   python src/version_check.py
   ```

#### Robot Model Validation
1. Load URDF model:
   ```bash
   python src/debug_contact.py
   ```
2. Verify collision geometries
3. Check joint limits

#### Force Distribution Validation
1. Run force analysis:
   ```bash
   python src/force_analysis.py
   ```
2. Verify:
   - Left foot force: ~130.80N
   - Right foot force: ~163.47N
   - Force ratio: ~0.800

#### Walking Controller Validation
1. Test stable pose:
   ```bash
   python src/test_stable_pose.py
   ```
2. Verify walking pattern:
   ```bash
   python src/test_walking_controller.py
   ```

### 5. Acceptance Criteria

#### Force Distribution
- [x] Force ratio within 0.75-0.95
- [x] Total force > 180N
- [x] Convergence time < 2.0s

#### Stability
- [x] Pitch deviation < 5°
- [x] No unwanted ground contact
- [x] Smooth transitions

#### Walking Performance
- [x] Stable step execution
- [x] Maintained force distribution
- [x] Emergency stop functionality

### 6. Test Environment Setup

#### Required Tools
- Python 3.12
- PyBullet
- NumPy 2.x

#### Configuration
```python
# test_config.py
CONFIG = {
    'use_gui': True,  # Set False for headless testing
    'time_step': 1/240,
    'max_force': 500,
    'target_ratio': 0.85
}
```

### 7. Troubleshooting Tests

#### Debug Tools
1. Contact point visualization:
   ```bash
   python src/debug_contact.py
   ```

2. Force analysis:
   ```bash
   python src/force_analysis.py
   ```

3. IK validation:
   ```bash
   python src/test_ik.py
   ```

#### Common Issues
1. Zero ground forces:
   - Check collision geometries
   - Verify initial pose
   - Inspect ground plane

2. Force distribution:
   - Monitor COM position
   - Check joint limits
   - Verify force calculations

### 8. Performance Metrics

#### Baseline Metrics
- Force ratio: 0.800
- Total force: 294.3N
- Pitch: 0.0°

#### Acceptance Thresholds
- Force ratio: ±0.1 from target
- Total force: >180N
- Pitch deviation: <5°

### 9. Test Reports

#### Required Information
1. Test configuration
2. Environment details
3. Performance metrics
4. Error logs

#### Report Format
```
Test Report
-----------
Date: YYYY-MM-DD
Configuration: {config}
Results:
- Force ratio: X.XXX
- Total force: XXX.XN
- Pitch: X.X°
Status: PASS/FAIL
```

### 10. Continuous Integration

#### Automated Tests
1. Unit tests
2. Integration tests
3. Performance validation

#### Test Schedule
- Pre-commit: Unit tests
- Daily: Integration tests
- Weekly: Performance tests
