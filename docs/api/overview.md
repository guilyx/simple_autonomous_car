# API Overview

Complete API reference for the Simple Autonomous Car SDK.

## Core Modules

### Track Module

**Location**: `simple_autonomous_car.track`

- `Track` - Track generation and management
  - `create_simple_track()` - Create a simple track
  - `create_oval_track()` - Create an oval track
  - `get_point_at_distance()` - Get point at distance along track
  - `get_bounds_at_point()` - Get boundaries at a point

### Car Module

**Location**: `simple_autonomous_car.car`

- `Car` - Vehicle model with dynamics
  - `update()` - Update car state
  - `get_corners()` - Get car corner points

- `CarState` - Vehicle state
  - `position()` - Get position as array
  - `transform_to_car_frame()` - Transform point to car frame
  - `transform_to_world_frame()` - Transform point to world frame

### Maps Module

**Location**: `simple_autonomous_car.maps`

- `GroundTruthMap` - Perfect reference map
  - `get_visible_segments()` - Get visible track segments

- `PerceivedMap` - Noisy perception map
  - `update_perceived_state()` - Update perceived car state
  - `get_perceived_segments()` - Get perceived segments in car frame
  - `get_perceived_segments_world_frame()` - Get in world frame

- `Sensor` - Sensor simulation
  - `get_map_lines_car_frame()` - Get ground truth map lines
  - `get_perception_point_cloud_car_frame()` - Get perception point cloud

- `FrenetMap` - Map in Frenet coordinates
  - `get_bounds_at_s()` - Get bounds at distance s
  - `get_bounds_in_range()` - Get bounds in s range

### Perception Module

**Location**: `simple_autonomous_car.perception`

- `PerceptionPoints` - Perception data structure
  - `to_ego_frame()` - Convert to ego frame
  - `to_global_frame()` - Convert to global frame
  - `filter_by_distance()` - Filter by distance

### Frames Module

**Location**: `simple_autonomous_car.frames`

- `FrenetFrame` - Frenet coordinate system
  - `global_to_frenet()` - Convert global to Frenet
  - `frenet_to_global()` - Convert Frenet to global
  - `ego_to_frenet()` - Convert ego to Frenet
  - `frenet_to_ego()` - Convert Frenet to ego

- Conversion Functions:
  - `global_to_frenet()` - Global → Frenet
  - `frenet_to_global()` - Frenet → Global
  - `ego_to_frenet()` - Ego → Frenet
  - `frenet_to_ego()` - Frenet → Ego
  - `sensor_to_ego()` - Sensor → Ego
  - `ego_to_sensor()` - Ego → Sensor

### Alerts Module

**Location**: `simple_autonomous_car.alerts`

- `TrackBoundsAlert` - Track bounds alert system
  - `check()` - Check for alerts
  - `get_recent_alerts()` - Get alert history
  - `reset_history()` - Clear history

### Visualization Module

**Location**: `simple_autonomous_car.visualization`

- `AlertVisualizer` - Alert visualization
  - `create_figure()` - Create visualization figure
  - `plot_alert_result()` - Plot alert result
  - `plot_alert_history()` - Plot alert history
  - `show()` - Display figure

**Note**: `TrackVisualizer` has been removed. Use component-based visualization methods instead.

### Detection Module

**Location**: `simple_autonomous_car.detection`

- `LocalizationErrorDetector` - Error detection
  - `compute_errors()` - Compute error metrics
  - `detect_errors()` - Detect if errors exceed threshold

## Quick Reference

### Import Everything

```python
from simple_autonomous_car import (
    # Track
    Track,
    # Car
    Car,
    CarState,
    # Maps
    GroundTruthMap,
    PerceivedMap,
    Sensor,
    FrenetMap,
    # Perception
    PerceptionPoints,
    # Frames
    FrenetFrame,
    global_to_frenet,
    frenet_to_global,
    ego_to_frenet,
    frenet_to_ego,
    sensor_to_ego,
    ego_to_sensor,
    # Alerts
    TrackBoundsAlert,
    # Visualization
    AlertVisualizer,
    # Detection
    LocalizationErrorDetector,
)
```

## Common Patterns

### Pattern 1: Basic Setup

```python
track = Track.create_simple_track()
frenet_map = FrenetMap(track)
ground_truth_map = GroundTruthMap(track)
perceived_map = PerceivedMap(ground_truth_map)
sensor = Sensor(ground_truth_map, perceived_map)
```

### Pattern 2: Getting Perception

```python
perception_points = sensor.get_perception_point_cloud_car_frame(car.state)
```

### Pattern 3: Frame Conversion

```python
# Ego to Frenet
s, d = ego_to_frenet(point, car_state, frenet_frame)

# Frenet to Global
point = frenet_to_global(s, d, frenet_frame)
```

### Pattern 4: Alert Check

```python
alert_system = TrackBoundsAlert(frenet_map)
result = alert_system.check(perception_points, car.state)
```

## See Also

- [Module Reference](modules.md) - Detailed module documentation
- [Guides](../guides/) - Usage guides and tutorials
