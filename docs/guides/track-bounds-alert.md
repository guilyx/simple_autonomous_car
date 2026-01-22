# Track Bounds Alert System

The Track Bounds Alert System detects when perceived track boundaries deviate significantly from the ground truth map boundaries. This is critical for autonomous vehicle safety and localization validation.

## Overview

### Purpose

The alert system answers the question: **"Are we where we think we are, and are we about to leave the track?"**

It does this by:
1. Converting perception data to Frenet coordinates
2. Comparing perceived boundaries to ground truth map
3. Calculating lateral deviations
4. Generating alerts when deviations exceed thresholds

### Use Cases

- **Localization Validation**: Verify that localization is accurate
- **Safety Systems**: Prevent leaving the track
- **Perception Quality Monitoring**: Detect sensor degradation
- **Autonomous Driving**: Ensure safe operation

## Architecture

### Components

1. **TrackBoundsAlert**: Main alert class
2. **FrenetMap**: Map representation in Frenet coordinates
3. **PerceptionPoints**: Perceived boundaries as points in ego frame
4. **FrenetFrame**: Coordinate conversion utilities

### Data Flow

```
Sensor → PerceptionPoints (ego) → Frenet conversion → Deviation calculation → Alert generation
```

## Installation and Setup

```python
from simple_autonomous_car import (
    Track,
    FrenetMap,
    Sensor,
    TrackBoundsAlert,
)

# Create track
track = Track.create_simple_track()

# Create Frenet map
frenet_map = FrenetMap(track)

# Create alert system
alert_system = TrackBoundsAlert(
    frenet_map,
    warning_threshold=1.0,    # meters
    critical_threshold=2.0,   # meters
    lookahead_distance=20.0,  # meters
)
```

## Usage

### Basic Usage

```python
# In your simulation loop
perception_points = sensor.get_perception_point_cloud_car_frame(car.state)
alert_result = alert_system.check(perception_points, car.state)

if alert_result["has_critical"]:
    print(f"CRITICAL: Max deviation = {alert_result['max_deviation']:.2f}m")
elif alert_result["has_warning"]:
    print(f"WARNING: Max deviation = {alert_result['max_deviation']:.2f}m")
```

### Advanced Usage

#### Accessing Detailed Results

```python
result = alert_system.check(perception_points, car.state)

# Get all deviations
deviations = result["deviations"]  # Array of deviation values

# Get specific alert points
for alert_point in result["alert_points"]:
    s = alert_point["s"]           # Distance along path
    d = alert_point["d"]           # Lateral position
    deviation = alert_point["deviation"]  # Deviation amount
    print(f"Alert at s={s:.2f}, d={d:.2f}, deviation={deviation:.2f}m")
```

#### Custom Thresholds

```python
alert_system = TrackBoundsAlert(
    frenet_map,
    warning_threshold=0.5,   # Stricter warning
    critical_threshold=1.5,  # Stricter critical
    lookahead_distance=30.0, # Look further ahead
)
```

#### Alert History

```python
# Get recent alerts
recent_alerts = alert_system.get_recent_alerts(num_recent=10)

for alert in recent_alerts:
    step = alert["step"]
    result = alert["result"]
    print(f"Step {step}: Max deviation = {result['max_deviation']:.2f}m")

# Clear history
alert_system.reset_history()
```

## Algorithm Details

### Deviation Calculation

For each perceived point:

1. **Convert to Frenet**: `(s, d_perceived) = global_to_frenet(point)`
2. **Get Ground Truth**: `(d_inner, d_outer) = frenet_map.get_bounds_at_s(s)`
3. **Calculate Deviation**:
   - If `d_perceived < d_inner`: deviation = `|d_perceived - d_inner|` (too far left)
   - If `d_perceived > d_outer`: deviation = `|d_perceived - d_outer|` (too far right)
   - Otherwise: deviation = 0 (within bounds)

### Alert Levels

- **Normal**: All deviations < `warning_threshold`
- **Warning**: At least one deviation > `warning_threshold` but < `critical_threshold`
- **Critical**: At least one deviation > `critical_threshold`

### Lookahead Distance

Only points within `lookahead_distance` ahead of the vehicle are checked. This:
- Reduces computational load
- Focuses on immediate safety concerns
- Can be adjusted based on vehicle speed

## Visualization

### Using AlertVisualizer

```python
from simple_autonomous_car import AlertVisualizer

viz = AlertVisualizer(track, frenet_map)
viz.create_figure()
viz.plot_alert_result(alert_result, perception_points, car)
viz.show()
```

### Plotting Alert History

```python
viz.plot_alert_history(alert_history)
```

## Integration Examples

### With Control System

```python
def control_with_alerts(car_state, perception_points, alert_system):
    alert_result = alert_system.check(perception_points, car_state)
    
    if alert_result["has_critical"]:
        target_velocity = 5.0  # Emergency: reduce speed significantly
    elif alert_result["has_warning"]:
        target_velocity = 10.0  # Caution: reduce speed moderately
    else:
        target_velocity = 15.0  # Normal: maintain speed
    
    return compute_steering(car_state, target_velocity)
```

### With Logging

```python
import logging

logger = logging.getLogger("alerts")

def check_and_log(alert_system, perception_points, car_state):
    result = alert_system.check(perception_points, car_state)
    
    if result["has_critical"]:
        logger.critical(
            f"Critical track bounds violation: "
            f"max_deviation={result['max_deviation']:.2f}m, "
            f"position=({car_state.x:.2f}, {car_state.y:.2f})"
        )
    elif result["has_warning"]:
        logger.warning(f"Track bounds warning: {result['max_deviation']:.2f}m")
    
    return result
```

## Performance Considerations

### Optimization Tips

1. **Filter Points Early**: Use `PerceptionPoints.filter_by_distance()` before checking
2. **Adjust Lookahead**: Reduce `lookahead_distance` if performance is an issue
3. **Sample Points**: If too many points, sample them before checking
4. **Cache Results**: Cache Frenet frame conversions when possible

### Example Optimization

```python
# Filter points by distance first
filtered_points = perception_points.filter_by_distance(max_distance=30.0)

# Then check alerts
result = alert_system.check(filtered_points, car.state)
```

## Troubleshooting

### No Alerts Triggered

- Check that `perception_points` contains data
- Verify thresholds are appropriate
- Ensure `lookahead_distance` is sufficient
- Check that Frenet frame conversion is working

### Too Many Alerts

- Increase thresholds
- Reduce `lookahead_distance`
- Check perception noise levels
- Verify map accuracy

### Performance Issues

- Reduce number of perception points
- Decrease `lookahead_distance`
- Use point sampling
- Optimize Frenet frame computations

## Best Practices

1. **Set Appropriate Thresholds**: Based on track width and sensor accuracy
2. **Monitor Alert History**: Track patterns over time
3. **Combine with Other Alerts**: Use multiple alert systems together
4. **Test Thoroughly**: Test with various noise levels and scenarios
5. **Document Thresholds**: Document why thresholds were chosen

## Interactive Learning

For a hands-on tutorial, see the [Jupyter Notebook](../../notebooks/track_bounds_alert.ipynb) which teaches you how to build the alert system step-by-step with visualizations at every stage.

## See Also

- [Building Alert Systems](building-alert-systems.md) - General guide for building alerts
- [Frame Conversions](frame-conversions.md) - Understanding coordinate systems
- [API Reference](../api/overview.md) - Complete API documentation
