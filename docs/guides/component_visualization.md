# Component-Based Visualization

## Overview

All components (Controllers, Planners, Costmaps, Tracks) now have their own `visualize()` methods that handle their own visualization logic. This makes visualization:

- **Reusable**: Any component can be visualized independently
- **Maintainable**: Visualization logic lives with the component
- **Extensible**: Easy to add new components with custom visualizations
- **No hardcoded values**: Components use their own data

## Component Visualization Methods

### Track.visualize()

```python
track.visualize(
    ax=ax,
    car_state=car.state,  # Optional, for ego frame
    frame="global",  # "global" or "ego"
    show_centerline=True,
    show_bounds=True,
    horizon=60.0,  # For ego frame filtering
)
```

### Planner.visualize()

```python
planner.visualize(
    ax=ax,
    car_state=car.state,
    plan=plan,  # Optional, will compute if not provided
    frame="global",  # "global" or "ego"
    color="green",
    linewidth=2.5,
    linestyle="--",
    show_waypoints=True,
)
```

### Controller.visualize()

```python
controller.visualize(
    ax=ax,
    car_state=car.state,
    plan=plan,
    frame="global",  # "global" or "ego"
    wheelbase=2.5,
    lookahead_color="magenta",
    arc_color="darkgreen",
    arc_linewidth=3.5,
)
```

### Costmap.visualize()

```python
costmap.visualize(
    ax=ax,
    car_state=car.state,
    frame="global",  # "global" or "ego"
    alpha=0.5,
    cmap="RdYlGn_r",
    show_car=False,
)
```

## Usage Examples

### In a Simulation Loop

```python
for step in range(num_steps):
    # ... compute plan, control, etc ...
    
    # Update visualization - components handle their own plotting
    track.visualize(ax=ax_world, frame="global")
    costmap.visualize(ax=ax_world, car_state=car.state, frame="global")
    planner.visualize(ax=ax_world, car_state=car.state, plan=plan, frame="global")
    controller.visualize(ax=ax_world, car_state=car.state, plan=plan, frame="global")
    
    plt.pause(0.01)
```

### In a Notebook

```python
fig, ax = plt.subplots(figsize=(12, 10))

# Plot all components using their visualize methods
track.visualize(ax=ax, frame="global", show_centerline=True, show_bounds=True)
costmap.visualize(ax=ax, car_state=car.state, frame="global", alpha=0.3)
planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="global", color="green")
controller.visualize(ax=ax, car_state=car.state, plan=plan, frame="global")

# Plot car (still uses utility function)
from simple_autonomous_car.visualization import plot_car
plot_car(car, ax=ax, frame="global", show_heading=True)

plt.show()
```

### Using the Orchestrator

For simulations, you can use the enhanced visualization orchestrator:

```python
from simulations import enhanced_visualization

fig, axes, cache = enhanced_visualization.create_enhanced_visualization()

# In simulation loop:
enhanced_visualization.update_all_views(
    axes=axes,
    track=track,
    car=car,
    plan=plan,
    perception_points=perception_points,
    costmap=costmap,
    controller=controller,
    planner=planner,
    view_bounds=view_bounds,
    horizon=horizon,
)
```

## Migration from Old Methods

**Note**: The old `plot_track()`, `plot_plan()`, and `plot_costmap()` functions have been **removed**. Use component methods instead.

### Old Way (No Longer Available)

```python
# ❌ These functions no longer exist
from simple_autonomous_car.visualization import plot_track, plot_plan, plot_costmap

plot_track(track, ax=ax)
plot_plan(plan, ax=ax, color="green")
plot_costmap(costmap, car.state, ax=ax)
```

### New Way (Use This)

```python
# ✅ Use component.visualize() methods
track.visualize(ax=ax, frame="global")
planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="global", color="green")
costmap.visualize(ax=ax, car_state=car.state, frame="global")
```

### Utility Functions (Still Available)

```python
# ✅ These utility functions still exist for non-component data
from simple_autonomous_car.visualization import plot_car, plot_perception

plot_car(car, ax=ax, frame="global", show_heading=True)
plot_perception(perception_points, car.state, ax=ax, frame="global")
```

## Benefits

1. **Component owns its visualization**: Each component knows how to visualize itself
2. **Easy to customize**: Pass kwargs to customize colors, styles, etc.
3. **Frame-aware**: Components handle frame transformations automatically
4. **Reusable**: Same method works in notebooks, simulations, examples
5. **No hardcoded values**: Components use their own data and parameters

## Controller Visualization Details

The controller visualization shows:
- **Lookahead point**: Magenta circle showing the target point
- **Lookahead line**: Dashed magenta line from car to lookahead point
- **Steering arc**: Dark green solid arc showing the turning path
- **Direction arrow**: Arrow at end of arc showing travel direction

The arc is automatically calculated from the controller's desired steering angle and is only shown when:
- Steering angle > 1e-6 radians
- Turning radius is between 0.1m and 1000m (reasonable bounds)
