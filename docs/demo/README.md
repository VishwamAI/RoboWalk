# Demo Setup Guide

## Quick Start Demo

### 1. Basic Walking Demo
```bash
# Run the basic walking demonstration
python src/test_walking_controller.py
```

This demo shows:
- Initial stabilization
- Force distribution
- Walking pattern execution

### 2. Visualization Options

#### GUI Mode
```python
# In walking_controller_fixed.py
use_gui = True  # Enable visual simulation
```

#### Debug Visualization
```python
# Enable force vectors
p.addUserDebugLine(...)

# Enable COM trajectory
p.addUserDebugPoints(...)
```

## Demo Scenarios

### 1. Stable Standing Demo
```bash
python src/test_stable_pose.py
```
Demonstrates:
- Force distribution
- COM adjustment
- Stability maintenance

### 2. Walking Pattern Demo
```bash
python src/implement_walking.py
```
Shows:
- Walking pattern generation
- Smooth transitions
- Force-aware control

### 3. Force Analysis Demo
```bash
python src/force_test.py
```
Visualizes:
- Ground contact forces
- Force distribution
- Center of pressure

## Parameter Adjustment

### 1. Walking Parameters
```python
# In walking_controller_fixed.py
startup_time = 2.0    # Longer for smoother start
cycle_time = 1.2     # Slower for stability
step_height = 0.04   # Lower for safety
step_length = 0.08   # Shorter steps
```

### 2. Force Parameters
```python
target_ratio = 0.85    # Force distribution target
min_total_force = 180  # Minimum safe force
```

### 3. Stability Parameters
```python
max_pitch = 5.0        # Maximum pitch deviation
force_timeout = 10.0   # Force calculation timeout
```

## Demo Configurations

### 1. Basic Configuration
```python
CONFIG = {
    'use_gui': True,
    'debug_lines': True,
    'force_display': True
}
```

### 2. Performance Configuration
```python
CONFIG = {
    'use_gui': False,
    'time_step': 1/240,
    'max_force': 500
}
```

## Running Demonstrations

### 1. Preparation
```bash
# Ensure environment is ready
python src/version_check.py

# Verify URDF loading
python src/debug_contact.py
```

### 2. Basic Walking Demo
```bash
# Run with default parameters
python src/test_walking_controller.py

# Expected output:
# - Force ratio: ~0.800
# - Total force: ~294.3N
# - Pitch: ~0.0°
```

### 3. Advanced Demonstrations

#### Force Distribution Demo
```bash
python src/force_analysis.py --display_forces
```

#### Walking Pattern Demo
```bash
python src/implement_walking.py --debug_view
```

## Troubleshooting Demos

### 1. Common Issues

#### Zero Forces
- Check ground plane configuration
- Verify collision geometries
- Adjust initial pose

#### Unstable Walking
- Reduce step length
- Increase cycle time
- Adjust force parameters

### 2. Debug Tools

#### Force Visualization
```bash
python src/debug_contact.py --show_forces
```

#### State Monitor
```bash
python src/test_walking_controller.py --verbose
```

## Demo Results

### 1. Expected Metrics
- Force ratio: 0.75-0.95
- Total force: >180N
- Pitch deviation: <5°

### 2. Performance Indicators
- Smooth transitions
- Stable walking
- Even force distribution

## Safety Guidelines

### 1. Operating Limits
- Maximum step length: 0.1m
- Maximum step height: 0.05m
- Maximum velocity: 0.5m/s

### 2. Emergency Procedures
- Emergency stop: Ctrl+C
- Reset pose: 'R' key
- Force recalibration: 'F' key
