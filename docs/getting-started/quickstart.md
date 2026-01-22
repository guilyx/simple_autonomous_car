# Quick Start Guide

Get up and running with Simple Autonomous Car SDK in minutes!

## Basic Example

Here's a minimal example to get you started:

```python
from simple_autonomous_car import (
    Track,
    Car,
    CarState,
    GroundTruthMap,
    PerceivedMap,
    Sensor,
    FrenetMap,
    TrackBoundsAlert,
    AlertVisualizer,
)

# 1. Create a track
track = Track.create_simple_track(length=80.0, width=40.0, track_width=5.0)

# 2. Create a car at the start
start_point, start_heading = track.get_point_at_distance(0.0)
car = Car(
    initial_state=CarState(
        x=start_point[0],
        y=start_point[1],
        heading=start_heading,
        velocity=8.0
    )
)

# 3. Create maps
ground_truth_map = GroundTruthMap(track)
perceived_map = PerceivedMap(ground_truth_map)

# 4. Create sensor
sensor = Sensor(ground_truth_map, perceived_map, max_range=40.0)

# 5. Create Frenet map and alert system
frenet_map = FrenetMap(track)
alert_system = TrackBoundsAlert(
    frenet_map,
    warning_threshold=1.0,
    critical_threshold=2.0,
)

# 6. Run a simple simulation
for step in range(100):
    # Update car
    car.update(dt=0.1, acceleration=0.0, steering_rate=0.05)
    
    # Get perception
    perception_points = sensor.get_perception_point_cloud_car_frame(car.state)
    
    # Check alerts
    alert_result = alert_system.check(perception_points, car.state)
    
    if alert_result["has_critical"]:
        print(f"CRITICAL: Max deviation = {alert_result['max_deviation']:.2f}m")
    elif alert_result["has_warning"]:
        print(f"WARNING: Max deviation = {alert_result['max_deviation']:.2f}m")
```

## Running Examples

### Simple Simulation

The simplest simulation for building warning systems:

```bash
# Run with built-in simple track config
python -m simulations.simulation simple_track

# Or run with race track config
python -m simulations.simulation race_track
```

Features:
- Simple car motion
- Clear data access points
- Explicit warning system hook
- Built-in visualization

### Interactive Notebook

Learn by doing with the Jupyter notebook:

```bash
# Start Jupyter
jupyter notebook notebooks/track_bounds_alert.ipynb
```

The notebook teaches you how to build the alert system step-by-step with visualizations at every stage.

## Key Concepts

### Coordinate Frames

The SDK uses multiple coordinate frames:

- **Global Frame**: World coordinates (X, Y)
- **Ego Frame**: Car-centered (x=forward, y=left)
- **Frenet Frame**: Path-aligned (s=distance, d=lateral offset)

### Data Flow

```
Sensor → PerceptionPoints (ego frame) → Frenet conversion → Alert check
```

### Visualization

Use `AlertVisualizer` for automatic visualization:

```python
viz = AlertVisualizer(track, frenet_map)
viz.create_figure()
viz.plot_alert_result(alert_result, perception_points, car)
viz.show()
```

## Next Steps

- [Building Alert Systems](../guides/building-alert-systems.md) - Learn to build custom alert systems
- [Track Bounds Alert](../guides/track-bounds-alert.md) - Deep dive into the track bounds alert
- [Frame Conversions](../guides/frame-conversions.md) - Understand coordinate transformations
- [API Reference](../api/overview.md) - Explore the full API
