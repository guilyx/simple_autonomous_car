# Simulations

This directory contains simulation runners and configurations.

## Structure

- `simulation.py` - Unified simulation runner that loads configs
- `enhanced_visualization.py` - Visualization orchestrator using component.visualize() methods
- `configs/` - Configuration files for different simulation scenarios
  - `simple_track.py` - Simple rectangular track configuration
  - `race_track.py` - Figure-8 race track configuration

## Usage

### Run with built-in configs

```bash
# Simple track
python -m simulations.simulation simple_track

# Race track
python -m simulations.simulation race_track
```

### Run with custom config

```bash
python -m simulations.simulation path/to/my_config.py
```

### List available configs

```bash
python -m simulations.simulation --list
```

## Creating Custom Configs

Create a Python file with a `CONFIG` dictionary:

```python
import numpy as np

CONFIG = {
    "track": {
        "type": "simple",  # or "oval" or "figure8"
        "length": 100.0,
        "width": 50.0,
        "track_width": 6.0,
        "num_points": 300
    },
    "car": {
        "initial_velocity": 10.0,
        "max_velocity": 25.0,
        "wheelbase": 2.5,
        "max_steering_angle": np.pi / 4,
    },
    # ... other config sections
}
```

## Component-Based Visualization

All visualization is now component-based. Each component has its own `visualize()` method:

- `track.visualize(ax, frame, **kwargs)`
- `planner.visualize(ax, car_state, plan, frame, **kwargs)`
- `controller.visualize(ax, car_state, plan, frame, **kwargs)`
- `costmap.visualize(ax, car_state, frame, **kwargs)`

The `enhanced_visualization` module orchestrates these calls to create the World View and Ego View.
