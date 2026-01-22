# Costmap Guide

The costmap system provides obstacle representation and inflation for safe path planning and control.

## Overview

Costmaps convert raw perception data into a grid-based representation where each cell has a cost value:
- **0.0** = Free space (safe to traverse)
- **1.0** = Occupied (obstacle)
- **0.0-1.0** = Inflated (cost increases near obstacles)

## Key Features

- **Grid-based representation**: Efficient 2D grid for cost queries
- **Obstacle inflation**: Automatically inflates obstacles for safety margins
- **Frame support**: Works in global or ego (car-centered) frame
- **Integration**: Works with planners and controllers

## Basic Usage

### Creating a Costmap

```python
from simple_autonomous_car import GridCostmap

# Create costmap
costmap = GridCostmap(
    width=50.0,           # Width in meters
    height=50.0,          # Height in meters
    resolution=0.5,       # 0.5 meters per cell
    inflation_radius=2.0, # Inflate obstacles by 2 meters
    frame="ego",          # Ego frame (moves with car) or "global"
)
```

### Updating from Perception Data

```python
# Get perception from sensors
perception_data = car.sense_all()

# Update costmap
costmap.update(perception_data, car.state)
```

### Querying Costs

```python
# Get cost at a specific position
position = np.array([10.0, 5.0])
cost = costmap.get_cost(position, frame="global", car_state=car.state)

# Get cost in a region
region = costmap.get_cost_region(
    center=np.array([10.0, 5.0]),
    size=5.0,  # 5m x 5m region
    frame="global",
    car_state=car.state
)
```

## Using with Planners

Planners can use costmaps to avoid obstacles:

```python
from simple_autonomous_car import TrackPlanner, GridCostmap

planner = TrackPlanner(track)
costmap = GridCostmap(width=50.0, height=50.0, resolution=0.5)

# Update costmap
perception_data = car.sense_all()
costmap.update(perception_data, car.state)

# Generate plan (planner can use costmap)
plan = planner.plan(
    car.state,
    perception_data=perception_data,
    costmap=costmap  # Planner can avoid high-cost areas
)
```

## Using with Controllers

Controllers can use costmaps to adjust behavior:

```python
from simple_autonomous_car import PurePursuitController, GridCostmap

controller = PurePursuitController(target_velocity=10.0)
costmap = GridCostmap(width=50.0, height=50.0, resolution=0.5)

# Update costmap
perception_data = car.sense_all()
costmap.update(perception_data, car.state)

# Compute control (controller adjusts velocity based on cost ahead)
control = controller.compute_control(
    car.state,
    perception_data=perception_data,
    costmap=costmap,  # Controller reduces velocity in high-cost areas
    plan=plan,
)
```

## Complete Example

```python
from simple_autonomous_car import (
    Track, Car, CarState,
    GroundTruthMap, PerceivedMap, LiDARSensor,
    TrackPlanner, PurePursuitController,
    GridCostmap, plot_costmap_with_path
)

# Setup
track = Track.create_simple_track()
car = Car(initial_state=CarState(x=0, y=0, heading=0))

gt_map = GroundTruthMap(track)
perceived_map = PerceivedMap(gt_map)
lidar = LiDARSensor(gt_map, perceived_map, max_range=40.0)
car.add_sensor(lidar)

# Create costmap
costmap = GridCostmap(
    width=60.0,
    height=60.0,
    resolution=0.5,
    inflation_radius=2.0,
    frame="ego",
)

# Create planner and controller
planner = TrackPlanner(track)
controller = PurePursuitController(target_velocity=8.0)

# Simulation loop
for step in range(100):
    # 1. Get perception
    perception_data = car.sense_all()
    
    # 2. Update costmap
    costmap.update(perception_data, car.state)
    
    # 3. Generate plan (with costmap)
    plan = planner.plan(car.state, perception_data=perception_data, costmap=costmap)
    
    # 4. Compute control (with costmap)
    control = controller.compute_control(
        car.state,
        perception_data=perception_data,
        costmap=costmap,
        plan=plan,
    )
    
    # 5. Update car
    car.update(dt=0.1, **control)
```

## Visualization

Use the component's `visualize()` method:

```python
import matplotlib.pyplot as plt

# Plot costmap in global frame
fig, ax = plt.subplots(figsize=(12, 10))
costmap.visualize(ax=ax, car_state=car.state, frame="global", alpha=0.4)
plt.show()

# Plot costmap in ego frame
fig, ax = plt.subplots(figsize=(12, 10))
costmap.visualize(ax=ax, car_state=car.state, frame="ego", alpha=0.6)
plt.show()

# Plot costmap with plan
fig, ax = plt.subplots(figsize=(12, 10))
costmap.visualize(ax=ax, car_state=car.state, frame="global", alpha=0.3)
planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="global", color="green")
plt.show()
```

## Parameters

### Resolution
- **Lower** (e.g., 0.1m): Higher detail, more memory
- **Higher** (e.g., 1.0m): Lower detail, less memory
- **Recommended**: 0.5m for most applications

### Inflation Radius
- **Small** (e.g., 1.0m): Tighter paths, less safety margin
- **Large** (e.g., 3.0m): Wider paths, more safety margin
- **Recommended**: 2.0m for typical vehicles

### Frame
- **"ego"**: Costmap moves with car (good for local planning)
- **"global"**: Fixed in world frame (good for global planning)

## Integration Points

### Data Flow

```
Sensors → Perception Data → Costmap → Planners/Controllers
                ↓
         Raw sensor data (also available)
```

### Available Data Sources

Planners and controllers can use:
1. **Raw perception data**: Direct sensor measurements
2. **Costmap data**: Processed obstacle representation
3. **Map data**: Ground truth map information

All three can be used together for comprehensive decision-making.

## Advanced Usage

### Custom Inflation

```python
from simple_autonomous_car.costmap.inflation import inflate_obstacles

# Custom inflation
inflated = inflate_obstacles(
    costmap.costmap,
    inflation_radius=3.0,
    resolution=0.5,
    method="linear"  # or "binary"
)
```

### Distance Transform

```python
from simple_autonomous_car.costmap.inflation import compute_distance_transform

# Get distance to nearest obstacle
distances = compute_distance_transform(costmap.costmap, resolution=0.5)
```

## Best Practices

1. **Update regularly**: Update costmap every time perception data changes
2. **Choose appropriate resolution**: Balance detail vs. performance
3. **Set inflation radius**: Based on vehicle size and safety requirements
4. **Use ego frame for local planning**: More efficient for reactive behaviors
5. **Combine with raw perception**: Use both for robust decision-making
