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
- Python 3.12
- PyBullet
- NumPy 2.x
- Ubuntu (tested on Ubuntu 22.04)

## Quick Start
1. Clone the repository
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Run the walking demonstration:
   ```bash
   python src/test_walking_controller.py
   ```

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
- [Deployment Guide](docs/deployment/README.md)
- [Architecture Documentation](docs/architecture/README.md)
- [Testing Procedures](docs/testing/README.md)
- [Demo Setup](docs/demo/README.md)

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

## Contributing
Please read our contributing guidelines before submitting pull requests.

## License
MIT License
