# Deployment Guide

## Environment Setup

### System Requirements
- Ubuntu 22.04 or later
- Python 3.12
- GPU support (optional, for improved performance)

### Installation Steps

1. **Set up Python Environment**
   ```bash
   # Install pyenv for Python version management
   curl https://pyenv.run | bash
   
   # Add to ~/.bashrc
   echo 'export PATH="$HOME/.pyenv/bin:$PATH"' >> ~/.bashrc
   echo 'eval "$(pyenv init -)"' >> ~/.bashrc
   echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc
   
   # Install Python 3.12
   pyenv install 3.12.0
   pyenv global 3.12.0
   ```

2. **Clone Repository**
   ```bash
   git clone <repository-url>
   cd robotics_workspace
   ```

3. **Install Dependencies**
   ```bash
   pip install numpy>=2.0.0
   pip install pybullet
   ```

### Configuration

1. **PyBullet Configuration**
   - GUI Mode (default): Set `use_gui=True` in configuration
   - Headless Mode: Set `use_gui=False` for environments without display

2. **Robot Model Configuration**
   - URDF file location: `src/models/simple_biped_stable.urdf`
   - Default parameters in `walking_controller_fixed.py`:
     ```python
     startup_time = 2.0
     cycle_time = 1.2
     step_height = 0.04
     step_length = 0.08
     ```

### Performance Tuning

1. **Force Distribution**
   - Target force ratio: 0.85
   - Minimum total force: 180N
   - Adjust `force_scale` and `transition_scale` for smoother transitions

2. **Stability Parameters**
   - Maximum pitch deviation: ±5°
   - Force balance timeout: 10 seconds
   - Emergency stop conditions in `walking_controller_fixed.py`

### Troubleshooting

1. **Common Issues**

   a. Zero Ground Contact Forces
   - Check URDF collision geometries
   - Verify initial pose configuration
   - Ensure proper ground plane setup

   b. Force Distribution Issues
   - Adjust COM position parameters
   - Check force feedback gains
   - Verify joint angle limits

   c. Walking Stability Issues
   - Reduce step length/height
   - Increase stabilization time
   - Adjust PID gains

2. **Debugging Tools**
   - Use `debug_contact.py` for force visualization
   - Enable PyBullet debug lines with `p.addUserDebugLine()`
   - Monitor force ratios with `force_test.py`

### Health Monitoring

1. **Key Metrics**
   - Force distribution ratio
   - Total ground contact force
   - Pitch angle
   - Joint positions and velocities

2. **Logging**
   - Default log location: `logs/`
   - CSV format for force data
   - Performance metrics tracking

### Backup and Recovery

1. **Configuration Backup**
   - Save custom parameters
   - Backup URDF modifications
   - Store PID gain settings

2. **Emergency Recovery**
   - Emergency stop procedure
   - Reset to stable pose
   - Force recalibration

## Support

For technical support or bug reports, please:
1. Check the troubleshooting guide
2. Review debug logs
3. Submit detailed issue reports with:
   - System configuration
   - Error messages
   - Reproduction steps
