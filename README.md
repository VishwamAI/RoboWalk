# Bipedal Robot Walking Implementation

## Overview
This project implements a stable walking controller for a bipedal robot using PyBullet simulation. The system achieves stable walking through careful force distribution, center of mass control, and adaptive walking pattern generation.

## Key Features
- Stable force distribution (achieved 0.800 force ratio)
- Perfect pitch alignment (0.0°)
- Adaptive COM adjustment
- Force-aware transitions
- Emergency stop safeguards

## System Requirements
### Prerequisites
- Ubuntu 22.04 or later
- Python 3.12+
- pip package manager
- X server for GUI visualization (optional)

### Dependencies
- PyBullet 3.2.5+: Physics simulation engine
- NumPy 2.0.0+: Numerical computations
- Matplotlib 3.7.1+: Visualization and plotting
- SciPy 1.11.3+: Scientific computing utilities

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/VishwamAI/RoboWalk.git
   cd RoboWalk
   ```

2. Create and activate a virtual environment (recommended):
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage
### Basic Walking Demo
Run the walking demonstration:
```bash
python src/test_walking_controller.py
```

### Force Analysis
Test force distribution and stability:
```bash
python src/force_test.py
```

### Inverse Kinematics Testing
Verify leg kinematics:
```bash
python src/test_ik.py
```

### Configuration Parameters
Key parameters in `walking_controller_fixed.py`:
- `FORCE_RATIO_TARGET = 0.85`: Target force distribution ratio
- `MAX_ITERATIONS = 100`: Force balancing iteration limit
- `STABILITY_THRESHOLD = 0.1`: Maximum allowed pitch deviation
- `STEP_HEIGHT = 0.05`: Walking step height in meters
- `STEP_LENGTH = 0.1`: Walking step length in meters

## Project Structure
```
robotics_workspace/
├── src/
│   ├── models/                    # Robot URDF models
│   │   └── simple_biped_stable.urdf
│   ├── walking_controller_fixed.py # Main walking controller
│   ├── test_walking_controller.py # Testing script
│   └── debug_contact.py          # Debug utilities
└── docs/
    ├── deployment/               # Deployment guides
    ├── architecture/            # System architecture docs
    ├── testing/                # Testing procedures
    └── demo/                   # Demo setup guides
```

## Documentation
- [Deployment Guide](docs/deployment/README.md): Setup and deployment instructions
- [Architecture Documentation](docs/architecture/README.md): System design and components
- [Testing Procedures](docs/testing/README.md): Test cases and validation
- [Demo Setup](docs/demo/README.md): Demo configurations and examples

## Performance Metrics
- Left foot force: 130.80N
- Right foot force: 163.47N
- Force ratio: 0.800 (target: 0.85)
- Total force: 294.3N
- Pitch: 0.0°

## Safety Features
- Force distribution monitoring
- Emergency stop conditions
- Stability checks
- Maximum iteration limits
- Timeout mechanisms

## Troubleshooting
### Common Issues
1. Zero Ground Contact Forces
   - Check robot initial pose
   - Verify collision geometries
   - Ensure proper URDF loading

2. Unstable Walking
   - Adjust force ratio target
   - Increase stabilization time
   - Reduce step height/length

3. PyBullet GUI Issues
   - Ensure X server is running
   - Check PyBullet version compatibility
   - Try DIRECT mode for headless operation

### Debug Tools
- `debug_contact.py`: Visualize contact points
- `force_analysis.py`: Analyze force distribution
- `leg_workspace.py`: Verify leg kinematics

## Development
### Testing
Run the test suite:
```bash
python -m pytest src/tests/
```

### Code Style
Follow PEP 8 guidelines and use type hints.

## Contributing
Please read our contributing guidelines before submitting pull requests.

## License
MIT License
