# Simple Autonomous Car SDK

[![CI](https://github.com/yourusername/simple-autonomous-car/actions/workflows/ci.yml/badge.svg)](https://github.com/yourusername/simple-autonomous-car/actions/workflows/ci.yml)
[![PyPI version](https://badge.fury.io/py/simple-autonomous-car.svg)](https://badge.fury.io/py/simple-autonomous-car)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A comprehensive Python SDK for building autonomous vehicle systems with modular sensors, controllers, planners, and alert systems. Designed for researchers, engineers, and developers working on autonomous vehicle systems.

## Features

- 🚗 **Modular Car System**: Add multiple sensors, controllers, and planners
- 🎯 **Multiple Coordinate Frames**: Global, Ego, Sensor, and Frenet frame support
- 👁️ **Modular Sensors**: LiDAR and extensible sensor system (add Camera, Radar, etc.)
- 🎮 **Control Systems**: Built-in Pure Pursuit and PID controllers, extensible architecture
- 🗺️ **Planning Systems**: Track planner and extensible planning framework
- 🚨 **Alert Systems**: Track bounds alert system with extensible architecture
- 🔄 **Frame Conversions**: Comprehensive utilities for coordinate transformations
- 📊 **Visualization**: Rich visualization tools for debugging and analysis
- 🧪 **Well Tested**: Comprehensive test suite with CI/CD

## Installation

### From PyPI

```bash
pip install simple-autonomous-car
```

### From Source

```bash
git clone https://github.com/yourusername/simple-autonomous-car.git
cd simple-autonomous-car
pip install -e .
```

### With uv (Recommended)

```bash
uv pip install simple-autonomous-car
```

## Quick Start

```python
from simple_autonomous_car import (
    Track,
    Car,
    CarState,
    LiDARSensor,
    PurePursuitController,
    TrackPlanner,
    FrenetMap,
    TrackBoundsAlert,
)

# Create track and car
track = Track.create_simple_track()
car = Car(initial_state=CarState(x=0.0, y=0.0, heading=0.0, velocity=8.0))

# Add sensors to car
ground_truth_map = GroundTruthMap(track)
perceived_map = PerceivedMap(ground_truth_map)
lidar = LiDARSensor(ground_truth_map, perceived_map, max_range=40.0)
car.add_sensor(lidar)

# Create planner and controller
planner = TrackPlanner(track)
controller = PurePursuitController(target_velocity=10.0)

# Control loop
for step in range(100):
    plan = planner.plan(car.state)
    perception_data = car.sense_all()
    control = controller.compute_control(car.state, perception_data, plan)
    car.update(dt=0.1, **control)
```

## Architecture

### Modular Design

```
Car
├── Sensors (LiDAR, Camera, Radar, etc.)
├── Controller (Pure Pursuit, PID, MPC, etc.)
└── Planner (Track Planner, A*, RRT, etc.)
```

### Data Flow

```
Planner → Plan → Controller → Control Commands → Car
                ↑
         Sensors → Perception Data
```

## Documentation

Comprehensive documentation is available in the [`docs/`](docs/) directory:

- **[Getting Started](docs/getting-started/installation.md)** - Installation and setup
- **[Quick Start](docs/getting-started/quickstart.md)** - Get up and running quickly
- **[Building Controllers](notebooks/building_controller.ipynb)** - Interactive controller tutorial
- **[Building Alert Systems](docs/guides/building-alert-systems.md)** - Guide for building alert systems
- **[Track Bounds Alert](docs/guides/track-bounds-alert.md)** - Detailed alert system documentation
- **[Frame Conversions](docs/guides/frame-conversions.md)** - Understanding coordinate systems
- **[API Reference](docs/api/overview.md)** - Complete API documentation

See the [Documentation Index](docs/index.md) for a complete overview.

## Key Concepts

### Modular Components

- **Car**: Vehicle with dynamics, can have multiple sensors
- **Sensors**: Modular sensor system (LiDAR, Camera, Radar, etc.)
- **Controllers**: Control algorithms (Pure Pursuit, PID, MPC, etc.)
- **Planners**: Path planning algorithms (Track Planner, A*, RRT, etc.)

### Coordinate Frames

- **Global Frame**: World coordinates (track reference)
- **Ego Frame**: Car-centered (x=forward, y=left)
- **Sensor Frame**: Sensor-centered coordinates
- **Frenet Frame**: Path-aligned (s=distance along path, d=lateral offset)

## Tutorials and Examples

### Jupyter Notebooks

Interactive tutorials are available in the `notebooks/` directory:

- **[Building Controllers](notebooks/building_controller.ipynb)** - Learn how to build controllers
- **[Costmap Tutorial](notebooks/costmap_tutorial.ipynb)** - Learn about costmaps and obstacle representation
- **[Planner Tutorial](notebooks/planner_tutorial.ipynb)** - Learn how to build and use planners
- **[Track Bounds Alert](notebooks/track_bounds_alert.ipynb)** - Learn how to build alert systems

### Code Examples

- **[Unified Simulation Runner](src/simulations/simulation.py)** - Config-based simulation runner with enhanced visualization
  - Run with: `python -m simulations.simulation simple_track` or `python -m simulations.simulation race_track`
  - Supports custom configs via `--config` flag

## Notebooks

The SDK includes comprehensive Jupyter notebooks organized by category:

### Tutorials (`notebooks/tutorials/`)
- [Costmap Tutorial](notebooks/tutorials/costmap_tutorial.ipynb) - Learn costmaps
- [Planner Tutorial](notebooks/tutorials/planner_tutorial.ipynb) - Learn path planning
- [Track Bounds Alert Tutorial](notebooks/tutorials/track_bounds_alert.ipynb) - Learn alert systems

### Building Custom Components (`notebooks/building/`)
- [Building Simulations](notebooks/building/building_simulation.ipynb) - Build complete simulations
- [Building Controllers](notebooks/building/building_controller.ipynb) - Build custom controllers
- [Building Custom Sensors](notebooks/building/building_custom_sensors.ipynb) - Build custom sensors
- [Building Custom Planners](notebooks/building/building_custom_planners.ipynb) - Build custom planners

### Learning Notebooks (`notebooks/learning/`) - Fill-in-the-Blank
**Incomplete notebooks** where you fill in the code - perfect for hands-on learning:
- [Learning: Build a Costmap](notebooks/learning/learning_build_costmap.ipynb) - Implement your own costmap
- [Learning: Build a Planner](notebooks/learning/learning_build_planner.ipynb) - Implement your own planner
- [Learning: Build a Controller](notebooks/learning/learning_build_controller.ipynb) - Implement your own controller
- [Learning: Build a Sensor](notebooks/learning/learning_build_sensor.ipynb) - Implement your own sensor
- [Learning: Advanced Planning](notebooks/learning/advanced_planning_algorithms.ipynb) - Implement A* and RRT

## Examples

### Adding Multiple Sensors

```python
# Add front LiDAR
front_lidar = LiDARSensor(..., name="front_lidar", pose_ego=np.array([1.0, 0.0, 0.0]))
car.add_sensor(front_lidar)

# Add rear LiDAR
rear_lidar = LiDARSensor(..., name="rear_lidar", pose_ego=np.array([-1.0, 0.0, np.pi]))
car.add_sensor(rear_lidar)

# Get data from all sensors
perception_data = car.sense_all()
```

### Using Controller with Planner

```python
# Create planner and controller
planner = TrackPlanner(track)
controller = PurePursuitController(target_velocity=10.0)

# Control loop
plan = planner.plan(car.state)
control = controller.compute_control(car.state, perception_data, plan)
car.update(dt=0.1, **control)
```

### Building Custom Components

```python
# Custom sensor
class MySensor(BaseSensor):
    def sense(self, car_state, environment_data):
        # Your sensor logic
        return PerceptionPoints(points, frame="ego")

# Custom controller
class MyController(BaseController):
    def compute_control(self, car_state, perception_data, plan):
        # Your control logic
        return {"acceleration": 0.0, "steering_rate": 0.0}

# Custom planner
class MyPlanner(BasePlanner):
    def plan(self, car_state, perception_data, goal):
        # Your planning logic
        return waypoints
```

## Development

```bash
# Clone repository
git clone https://github.com/yourusername/simple-autonomous-car.git
cd simple-autonomous-car

# Install with uv
uv sync --dev

# Run tests
uv run pytest

# Run linting
uv run black src tests
uv run ruff check src tests
uv run mypy src

# Install pre-commit hooks
uv run pre-commit install
```

## Project Structure

```
simple_autonomous_car/
├── src/simple_autonomous_car/
│   ├── car/             # Car model with sensor support
│   ├── sensors/         # Modular sensor system
│   ├── control/         # Control algorithms
│   ├── planning/        # Path planning algorithms
│   ├── alerts/          # Alert systems
│   ├── maps/            # Map representations
│   ├── perception/      # Perception data structures
│   ├── frames/          # Frame conversion utilities
│   ├── track/           # Track generation
│   └── visualization/   # Visualization tools
├── docs/                # Documentation
├── notebooks/           # Jupyter notebooks
├── tests/               # Test suite
└── src/simulations/     # Example simulations
```

## Contributing

Contributions are welcome! Please see [Contributing Guidelines](docs/contributing.md) for details.

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Citation

If you use this SDK in your research, please cite:

```bibtex
@software{simple_autonomous_car,
  title = {Simple Autonomous Car SDK},
  author = {Your Name},
  year = {2024},
  url = {https://github.com/yourusername/simple-autonomous-car}
}
```

## Acknowledgments

Inspired by the [AutonomousVehicleControlBeginnersGuide](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide) repository.
