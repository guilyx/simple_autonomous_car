# Building Alert Systems

This comprehensive guide shows you how to build alert and warning systems on top of the Simple Autonomous Car SDK.

## Overview

The SDK provides a foundation for building various alert systems:
- **Track Bounds Alerts** - Detect boundary violations
- **Localization Alerts** - Monitor localization errors
- **Perception Quality Alerts** - Validate sensor data
- **Safety Alerts** - Prevent dangerous situations
- **Custom Alerts** - Build your own detection logic

## Architecture Overview

### Core Components

1. **Data Sources**
   - `PerceptionPoints` - Sensor data in ego frame
   - `FrenetMap` - Ground truth map in Frenet coordinates
   - `CarState` - Current vehicle state

2. **Frame Conversions**
   - Convert between Global, Ego, Sensor, and Frenet frames
   - Use `FrenetFrame` for path-aligned coordinates

3. **Alert Logic**
   - Compare perception vs ground truth
   - Calculate deviations
   - Apply thresholds
   - Generate alerts

4. **Visualization**
   - Use `AlertVisualizer` for automatic plotting
   - No custom visualization code needed!

## Building a Track Bounds Alert System

The track bounds alert is the primary alert system. See the [Track Bounds Alert Guide](track-bounds-alert.md) for detailed documentation, or follow the [Jupyter Notebook](../../notebooks/track_bounds_alert.ipynb) for a step-by-step tutorial.

### Quick Example

```python
from simple_autonomous_car import (
    Track,
    FrenetMap,
    Sensor,
    TrackBoundsAlert,
)

# Setup
track = Track.create_simple_track()
frenet_map = FrenetMap(track)
sensor = Sensor(ground_truth_map, perceived_map)

# Create alert system
alert_system = TrackBoundsAlert(
    frenet_map,
    warning_threshold=1.0,    # 1m deviation = warning
    critical_threshold=2.0,    # 2m deviation = critical
    lookahead_distance=20.0,  # Check 20m ahead
)

# In simulation loop
perception_points = sensor.get_perception_point_cloud_car_frame(car.state)
result = alert_system.check(perception_points, car.state)

if result["has_critical"]:
    # Take emergency action
    pass
elif result["has_warning"]:
    # Take cautionary action
    pass
```

## Building Custom Alert Systems

### Step 1: Define Your Alert Logic

```python
class MyCustomAlert:
    def __init__(self, threshold=1.0):
        self.threshold = threshold
        self.alert_history = []
    
    def check(self, perception_points, car_state, frenet_map):
        # Your custom detection logic here
        # Return dict with alert information
        return {
            "has_alert": False,
            "severity": "normal",
            "details": {},
        }
```

### Step 2: Use Frame Conversions

```python
from simple_autonomous_car import ego_to_frenet, frenet_to_global

# Convert perception to Frenet for comparison
for point in perception_points.points:
    s, d = ego_to_frenet(point, car_state, frenet_map.frenet_frame)
    # Compare with ground truth at s
    d_inner, d_outer = frenet_map.get_bounds_at_s(s)
    # Your logic here
```

### Step 3: Integrate with Visualization

```python
from simple_autonomous_car import AlertVisualizer

viz = AlertVisualizer(track, frenet_map)
viz.create_figure()
viz.plot_alert_result(alert_result, perception_points, car)
viz.show()
```

## Alert System Ideas

### 1. Speed Limit Alert

Warn when approaching curves too fast:

```python
def check_speed_limit(car_state, frenet_map, max_speed=15.0):
    s, d = frenet_map.frenet_frame.global_to_frenet(car_state.position())
    
    # Get curvature at current position
    # Calculate safe speed for curvature
    # Compare with current speed
    
    if car_state.velocity > safe_speed:
        return {"has_alert": True, "severity": "warning"}
    return {"has_alert": False}
```

### 2. Obstacle Alert

Detect objects on track:

```python
def check_obstacles(perception_points, car_state, frenet_map):
    # Filter points that don't match track boundaries
    # These might be obstacles
    
    obstacles = []
    for point in perception_points.points:
        s, d = ego_to_frenet(point, car_state, frenet_map.frenet_frame)
        d_inner, d_outer = frenet_map.get_bounds_at_s(s)
        
        # Points inside track but not on boundaries might be obstacles
        if d_inner < d < d_outer:
            # Check if point is far from boundaries
            if abs(d) > threshold:
                obstacles.append((s, d))
    
    return {"has_alert": len(obstacles) > 0, "obstacles": obstacles}
```

### 3. Localization Drift Alert

Detect gradual localization errors:

```python
class LocalizationDriftAlert:
    def __init__(self, max_drift_rate=0.1):
        self.max_drift_rate = max_drift_rate
        self.error_history = []
    
    def check(self, current_error, dt):
        if len(self.error_history) > 0:
            drift_rate = (current_error - self.error_history[-1]) / dt
            if drift_rate > self.max_drift_rate:
                return {"has_alert": True, "drift_rate": drift_rate}
        
        self.error_history.append(current_error)
        return {"has_alert": False}
```

### 4. Sensor Degradation Alert

Detect sensor failures:

```python
def check_sensor_health(perception_points, expected_density=100):
    # Check point density
    if len(perception_points) < expected_density * 0.5:
        return {"has_alert": True, "issue": "low_density"}
    
    # Check point distribution
    # Check for systematic errors
    # etc.
    
    return {"has_alert": False}
```

## Best Practices

### 1. Use Appropriate Thresholds

Set thresholds based on:
- Track width
- Sensor accuracy
- Safety requirements
- System capabilities

### 2. Filter Data Early

```python
# Filter by distance first
filtered_points = perception_points.filter_by_distance(max_distance=30.0)

# Then process
result = alert_system.check(filtered_points, car_state)
```

### 3. Use Frenet Frame for Comparison

Frenet frame makes comparison easy:

```python
# Convert to Frenet
s, d = frenet_map.frenet_frame.global_to_frenet(point)

# Get ground truth at same s
d_inner, d_outer = frenet_map.get_bounds_at_s(s)

# Compare d values directly
deviation = abs(d - d_inner) if d < d_inner else abs(d - d_outer)
```

### 4. Maintain Alert History

```python
class AlertSystem:
    def __init__(self):
        self.alert_history = []
    
    def check(self, ...):
        result = self._compute_alerts(...)
        self.alert_history.append(result)
        return result
    
    def get_recent_alerts(self, num=10):
        return self.alert_history[-num:]
```

### 5. Use Built-in Visualization

Don't write custom plotting code:

```python
viz = AlertVisualizer(track, frenet_map)
viz.plot_alert_result(result, perception_points, car)
viz.plot_alert_history(alert_history)
```

## Integration Examples

### With Control System

```python
def control_with_alerts(car_state, perception_points, alert_system):
    # Check alerts
    alert_result = alert_system.check(perception_points, car_state)
    
    # Adjust control based on alerts
    if alert_result["has_critical"]:
        target_velocity = 5.0  # Slow down significantly
        steering_gain = 0.5    # Reduce steering aggressiveness
    elif alert_result["has_warning"]:
        target_velocity = 10.0  # Moderate speed reduction
        steering_gain = 0.8
    else:
        target_velocity = 15.0  # Normal operation
        steering_gain = 1.0
    
    return compute_control(car_state, target_velocity, steering_gain)
```

### With Logging

```python
import logging

logger = logging.getLogger("alerts")

def check_and_log(alert_system, perception_points, car_state):
    result = alert_system.check(perception_points, car_state)
    
    if result["has_critical"]:
        logger.critical(
            f"Critical alert: {result['max_deviation']:.2f}m at "
            f"({car_state.x:.2f}, {car_state.y:.2f})"
        )
    elif result["has_warning"]:
        logger.warning(f"Warning: {result['max_deviation']:.2f}m")
    
    return result
```

## Testing Your Alert System

### Unit Tests

```python
def test_alert_system():
    # Create test scenario
    track = Track.create_simple_track()
    frenet_map = FrenetMap(track)
    alert_system = TrackBoundsAlert(frenet_map, warning_threshold=1.0)
    
    # Create test perception with known deviation
    test_points = PerceptionPoints(
        np.array([[10.0, 2.0], [11.0, 2.5]]),  # Points with deviation
        frame="ego"
    )
    
    # Test
    result = alert_system.check(test_points, car_state)
    assert result["has_warning"] == True
```

### Simulation Testing

```python
# Test with different noise levels
for noise in [0.1, 0.2, 0.3, 0.5]:
    perceived_map.measurement_noise_std = noise
    perception_points = sensor.get_perception_point_cloud_car_frame(car.state)
    result = alert_system.check(perception_points, car.state)
    print(f"Noise {noise}: {result['max_deviation']:.2f}m")
```

## Next Steps

- [Track Bounds Alert Guide](track-bounds-alert.md) - Detailed documentation
- [Jupyter Notebook](../../notebooks/track_bounds_alert.ipynb) - Interactive tutorial
- [API Reference](../api/overview.md) - Full API documentation
- [Frame Conversions](frame-conversions.md) - Coordinate system guide
