# Grid Map Navigation Guide

This guide explains how to use the grid map environment with obstacles and goal-based navigation.

## Overview

The grid map environment provides an alternative to race tracks, allowing you to:
- Create environments with obstacles
- Navigate to specific goal positions
- Use A* path planning to find optimal paths

## Components

### GridMap

The `GridMap` class represents a grid-based environment with obstacles.

```python
from simple_autonomous_car import GridMap

# Create a random grid map
grid_map = GridMap.create_random_map(
    width=50.0,
    height=50.0,
    resolution=0.5,
    num_obstacles=15,
    obstacle_size=2.0,
    seed=42,  # For reproducibility
)

# Or create with specific obstacles
obstacles = np.array([
    [10.0, 10.0],
    [-5.0, 15.0],
    [20.0, -10.0],
])
grid_map = GridMap(
    width=50.0,
    height=50.0,
    resolution=0.5,
    obstacles=obstacles,
    obstacle_size=2.0,
)
```

### GoalPlanner

The `GoalPlanner` uses A* algorithm to find paths from start to goal, avoiding obstacles.

```python
from simple_autonomous_car import GoalPlanner

planner = GoalPlanner(
    grid_map=grid_map,
    resolution=0.5,
)

# Plan path to goal
goal = np.array([20.0, 20.0])
plan = planner.plan(car.state, goal=goal)
```

## Running the Simulation

### Using the Built-in Config

```bash
python -m simulations.simulation grid_map
```

### Custom Configuration

Create a config file similar to `src/simulations/configs/grid_map.py`:

```python
CONFIG = {
    "map": {
        "type": "grid",
        "width": 50.0,
        "height": 50.0,
        "resolution": 0.5,
        "num_obstacles": 15,
        "obstacle_size": 2.0,
        "seed": 42,
    },
    "car": {
        "initial_position": [-20.0, -20.0],
        "initial_heading": 0.0,
        "initial_velocity": 5.0,
        # ... other car params
    },
    "goal": {
        "position": [20.0, 20.0],
    },
    # ... rest of config
}
```

Then run:
```bash
python -m simulations.simulation path/to/your_config.py
```

## Example Usage

See `examples/grid_map_example.py` for a complete example.

## Key Differences from Track Environment

1. **Environment**: Uses `GridMap` instead of `Track`
2. **Planner**: Uses `GoalPlanner` instead of `TrackPlanner`
3. **Goal**: Requires a goal position (not just following a track)
4. **No Alerts**: Track bounds alerts don't apply to grid maps

## API Compatibility

The grid map components are designed to work with the same APIs:
- `GridMap.visualize()` - Same interface as `Track.visualize()`
- `GoalPlanner.plan()` - Extends `BasePlanner.plan()` with `goal` parameter
- Controllers work the same way (PurePursuitController, etc.)
- Costmaps work the same way

This means you can swap between track and grid map environments without changing your controller or other components!
