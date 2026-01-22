# Visualization Guide

Component-based visualization for all components of the Simple Autonomous Car SDK.

## Overview

The SDK uses **component-based visualization** where each component has its own `visualize()` method:
- **Track**: `track.visualize(ax, frame, **kwargs)`
- **Planner**: `planner.visualize(ax, car_state, plan, frame, **kwargs)`
- **Controller**: `controller.visualize(ax, car_state, plan, frame, **kwargs)`
- **Costmap**: `costmap.visualize(ax, car_state, frame, **kwargs)`

Utility functions are provided for non-component data:
- **Perception**: `plot_perception(perception_points, car_state, ax, frame, **kwargs)`
- **Car**: `plot_car(car, ax, frame, **kwargs)`

## Quick Start

```python
import matplotlib.pyplot as plt
from simple_autonomous_car import Track, Car, CarState, TrackPlanner, PurePursuitController
from simple_autonomous_car.visualization import plot_car, plot_perception

# Create components
track = Track.create_simple_track()
car = Car(initial_state=CarState(x=0, y=0, heading=0))
planner = TrackPlanner(track)
controller = PurePursuitController()
plan = planner.plan(car.state)

# Visualize using component methods
fig, ax = plt.subplots(figsize=(12, 10))
track.visualize(ax=ax, frame="global", show_centerline=True, show_bounds=True)
planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="global", color="green")
controller.visualize(ax=ax, car_state=car.state, plan=plan, frame="global")
plot_car(car, ax=ax, frame="global", show_heading=True)
plt.show()
```

## Component Visualizations

### Track Visualization

Visualize track boundaries and centerline.

```python
# Basic usage
track.visualize(ax=ax, frame="global")

# With options
track.visualize(
    ax=ax,
    frame="global",
    show_centerline=True,
    show_bounds=True,
    centerline_color="blue",
    bounds_color="black"
)
```

### Planner Visualization

Visualize planned path from a planner.

```python
plan = planner.plan(car.state)

# Basic usage
planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="global")

# With options
planner.visualize(
    ax=ax,
    car_state=car.state,
    plan=plan,
    frame="global",
    color="green",
    linewidth=2.5,
    show_waypoints=True
)
```

### Controller Visualization

Visualize controller state (lookahead point, steering arc).

```python
# Basic usage
controller.visualize(ax=ax, car_state=car.state, plan=plan, frame="global")

# With options
controller.visualize(
    ax=ax,
    car_state=car.state,
    plan=plan,
    frame="global",
    lookahead_color="magenta",
    arc_color="darkgreen",
    wheelbase=2.5
)
```

### Costmap Visualization

Visualize costmap overlay.

```python
# Basic usage
costmap.visualize(ax=ax, car_state=car.state, frame="global")

# With options
costmap.visualize(
    ax=ax,
    car_state=car.state,
    frame="global",
    alpha=0.4,
    cmap="RdYlGn_r"
)
```

### Perception Visualization

Visualize sensor data (LiDAR, etc.) - utility function.

```python
from simple_autonomous_car.visualization import plot_perception

perception = lidar.sense(car.state)

# In global frame
plot_perception(perception, car.state, ax=ax, frame="global", color="red")

# In ego frame
plot_perception(perception, car.state, ax=ax, frame="ego", color="red")
```

### Car Visualization

Visualize car position and orientation - utility function.

```python
from simple_autonomous_car.visualization import plot_car

# Basic usage
plot_car(car, ax=ax, frame="global")

# With heading arrow
plot_car(car, ax=ax, frame="global", show_heading=True, color="blue")
```

## Frame Transformations

All component `visualize()` methods support frame transformations:

```python
# Global frame (default)
track.visualize(ax=ax, frame="global")
planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="global")
controller.visualize(ax=ax, car_state=car.state, plan=plan, frame="global")
costmap.visualize(ax=ax, car_state=car.state, frame="global")

# Ego frame (car-centered)
track.visualize(ax=ax, car_state=car.state, frame="ego", horizon=50.0)
planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="ego")
controller.visualize(ax=ax, car_state=car.state, plan=plan, frame="ego")
costmap.visualize(ax=ax, car_state=car.state, frame="ego")
```

## Complete Example

```python
import matplotlib.pyplot as plt
from simple_autonomous_car import (
    Track, Car, CarState, TrackPlanner, PurePursuitController, GridCostmap
)
from simple_autonomous_car.visualization import plot_car, plot_perception

# Setup
track = Track.create_simple_track()
start_point, start_heading = track.get_point_at_distance(0.0)
car = Car(initial_state=CarState(x=start_point[0], y=start_point[1], heading=start_heading))
planner = TrackPlanner(track)
controller = PurePursuitController()
costmap = GridCostmap(width=50.0, height=50.0, resolution=0.5)

# Generate plan
plan = planner.plan(car.state)

# Visualize everything
fig, ax = plt.subplots(figsize=(12, 10))

# Plot in order (background to foreground)
track.visualize(ax=ax, frame="global", show_centerline=True, show_bounds=True)
costmap.visualize(ax=ax, car_state=car.state, frame="global", alpha=0.3)
planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="global", color="green")
controller.visualize(ax=ax, car_state=car.state, plan=plan, frame="global")
plot_car(car, ax=ax, frame="global", show_heading=True)

ax.set_title("Complete Visualization")
ax.legend()
plt.show()
```

## Controller Visualization Details

The controller visualization shows:
- **Lookahead point**: Magenta circle showing the target point
- **Lookahead line**: Dashed magenta line from car to lookahead point
- **Steering arc**: Dark green solid arc showing the turning path
- **Direction arrow**: Arrow at end of arc showing travel direction

The arc is automatically calculated from the controller's desired steering angle and is shown when:
- Steering angle > 1e-6 radians
- Turning radius > 0.1m (reasonable bounds)

# Basic usage
ax = plot_controller_state(controller, car.state, plan)
plt.show()

# With all features
ax = plot_controller_state(
    controller,
    car.state,
    plan,
    show_curvature=True,
    show_steering=True,
    show_lookahead=True
)
plt.show()
```

## Controller-Specific Visualizations

### Pure Pursuit Controller

Detailed visualization for Pure Pursuit controller including:
- Lookahead point and line
- Steering circle (turning radius)
- Path curvature
- Control commands

```python
from simple_autonomous_car import plot_pure_pursuit_state

# Detailed Pure Pursuit visualization
ax = plot_pure_pursuit_state(
    controller,
    car.state,
    plan,
    show_curvature=True,
    show_steering_circle=True,
    show_lookahead=True
)
plt.show()
```

### Control History

Plot control command history over time.

```python
from simple_autonomous_car import plot_control_history

# Collect control history
history = []
for step in range(100):
    control = controller.compute_control(car.state, plan=plan)
    history.append({
        'steering_rate': control['steering_rate'],
        'acceleration': control['acceleration'],
        'velocity': car.state.velocity,
        'time': step * 0.1
    })

# Plot history
ax = plot_control_history(
    history,
    show_steering=True,
    show_acceleration=True,
    show_velocity=True
)
plt.show()
```

## Combined Visualizations

### Plot All

Plot everything together in one view.

```python
from simple_autonomous_car import plot_all

# Plot all components
ax = plot_all(
    track=track,
    car=car,
    plan=plan,
    perception_points=perception,
    frenet_map=frenet_map,
    controller=controller
)
plt.show()
```

## Examples

### Example 1: Visualize Planner Output

```python
from simple_autonomous_car import Track, TrackPlanner, Car, CarState, plot_track, plot_plan

track = Track.create_simple_track()
planner = TrackPlanner(track)
car = Car(initial_state=CarState(x=0, y=0, heading=0))

plan = planner.plan(car.state)

# Visualize
ax = plot_track(track)
plot_plan(plan, ax=ax, color="green")
plt.show()
```

### Example 2: Visualize Sensor Data

```python
from simple_autonomous_car import (
    Track, Car, CarState, LiDARSensor, GroundTruthMap, PerceivedMap,
    plot_track, plot_perception, plot_car
)

track = Track.create_simple_track()
car = Car(initial_state=CarState(x=0, y=0, heading=0))

gt_map = GroundTruthMap(track)
perceived_map = PerceivedMap(gt_map)
lidar = LiDARSensor(gt_map, perceived_map, max_range=40.0)
car.add_sensor(lidar)

perception = car.sense_all()["lidar"]

# Visualize
ax = plot_track(track)
plot_perception(perception, car.state, ax=ax, frame="global")
plot_car(car, ax=ax)
plt.show()
```

### Example 3: Visualize Controller with Curvature

```python
from simple_autonomous_car import (
    Track, Car, CarState, PurePursuitController, TrackPlanner,
    plot_track, plot_controller_state, plot_pure_pursuit_state
)

track = Track.create_simple_track()
car = Car(initial_state=CarState(x=0, y=0, heading=0))
planner = TrackPlanner(track)
controller = PurePursuitController(lookahead_distance=10.0)

plan = planner.plan(car.state)

# Visualize controller state
ax = plot_track(track)
plot_pure_pursuit_state(
    controller,
    car.state,
    plan,
    ax=ax,
    show_curvature=True,
    show_steering_circle=True,
    show_lookahead=True
)
plt.show()
```

## Tips

1. **Reuse axes**: Pass `ax` parameter to plot multiple things on same axes
2. **Frame selection**: Use `frame="global"` for world view, `frame="ego"` for car view
3. **Curvature colors**: Red = high curvature, green = low curvature
4. **Steering circle**: Shows the turning radius based on current steering angle
5. **Lookahead visualization**: Shows where the controller is looking ahead

## Available Functions

- `plot_track()` - Plot track boundaries and centerline
- `plot_plan()` - Plot planned path
- `plot_perception()` - Plot sensor data
- `plot_car()` - Plot car position and heading
- `plot_controller_state()` - Plot controller state
- `plot_frenet_map()` - Plot Frenet map bounds
- `plot_all()` - Plot everything together
- `plot_pure_pursuit_state()` - Detailed Pure Pursuit visualization
- `plot_control_history()` - Plot control command history

All functions return `plt.Axes` objects, so you can chain them or customize further.
