# Frame Conversions

Understanding and using coordinate frame conversions in the Simple Autonomous Car SDK.

## Overview

The SDK supports multiple coordinate frames to handle different aspects of autonomous vehicle systems:

- **Global Frame**: World coordinates (track reference)
- **Ego Frame**: Car-centered (x=forward, y=left)
- **Sensor Frame**: Sensor-centered coordinates
- **Frenet Frame**: Path-aligned (s=distance along path, d=lateral offset)

## Frame Definitions

### Global Frame

- **Origin**: Track origin (usually start/finish line)
- **X-axis**: East (or track reference direction)
- **Y-axis**: North (or perpendicular to track)
- **Use Case**: Absolute positioning, map storage

### Ego Frame

- **Origin**: Car position
- **X-axis**: Forward direction (car's heading)
- **Y-axis**: Left direction (perpendicular to heading)
- **Use Case**: Sensor data, local perception

### Sensor Frame

- **Origin**: Sensor position (usually at car origin)
- **X-axis**: Sensor forward direction
- **Y-axis**: Sensor left direction
- **Use Case**: Raw sensor measurements

### Frenet Frame

- **s**: Distance along the path (longitudinal)
- **d**: Lateral offset from path (positive = left, negative = right)
- **Use Case**: Path-aligned operations, easy comparison

## Conversion Functions

### Global ↔ Frenet

```python
from simple_autonomous_car import global_to_frenet, frenet_to_global, FrenetFrame

# Create Frenet frame
frenet_frame = FrenetFrame(track)

# Global to Frenet
point_global = np.array([10.0, 5.0])
s, d = global_to_frenet(point_global, frenet_frame)

# Frenet to Global
point_global = frenet_to_global(s, d, frenet_frame)
```

### Ego ↔ Frenet

```python
from simple_autonomous_car import ego_to_frenet, frenet_to_ego

# Ego to Frenet
point_ego = np.array([5.0, 2.0])  # 5m forward, 2m left
s, d = ego_to_frenet(point_ego, car_state, frenet_frame)

# Frenet to Ego
point_ego = frenet_to_ego(s, d, car_state, frenet_frame)
```

### Sensor ↔ Ego

```python
from simple_autonomous_car import sensor_to_ego, ego_to_sensor

# Sensor to Ego
point_sensor = np.array([3.0, 1.0])
sensor_pose_ego = np.array([0.0, 0.0, 0.0])  # [x, y, heading]
point_ego = sensor_to_ego(point_sensor, sensor_pose_ego)

# Ego to Sensor
point_sensor = ego_to_sensor(point_ego, sensor_pose_ego)
```

## Using CarState for Conversions

`CarState` provides convenient methods:

```python
# Transform point from world to car frame
point_world = np.array([10.0, 5.0])
point_ego = car_state.transform_to_car_frame(point_world)

# Transform point from car to world frame
point_world = car_state.transform_to_world_frame(point_ego)
```

## Common Patterns

### Pattern 1: Sensor Data to Frenet

```python
# Sensor → Ego → Global → Frenet
point_sensor = np.array([5.0, 2.0])
point_ego = sensor_to_ego(point_sensor, sensor_pose_ego)
point_global = car_state.transform_to_world_frame(point_ego)
s, d = global_to_frenet(point_global, frenet_frame)
```

### Pattern 2: Comparing Perception to Map

```python
# Convert perception to Frenet for easy comparison
for point in perception_points.points:
    point_global = car_state.transform_to_world_frame(point)
    s, d = frenet_frame.global_to_frenet(point_global)
    
    # Get ground truth at same s
    d_inner, d_outer = frenet_map.get_bounds_at_s(s)
    
    # Compare d values directly
    if d < d_inner or d > d_outer:
        # Outside bounds!
        pass
```

### Pattern 3: Path Planning in Frenet

```python
# Plan path in Frenet space (easier!)
s_start = 0.0
s_end = 100.0
d_target = 0.0  # Stay on centerline

# Generate path points
s_values = np.linspace(s_start, s_end, 50)
path_points = []

for s in s_values:
    point_global = frenet_to_global(s, d_target, frenet_frame)
    path_points.append(point_global)
```

## Best Practices

### 1. Always Use Same Frame for Comparison

```python
# ❌ Bad: Comparing in different frames
gt_point_global = ground_truth_map.get_point(...)
perceived_point_ego = perception_points[0]
error = np.linalg.norm(gt_point_global - perceived_point_ego)  # Wrong!

# ✅ Good: Convert to same frame first
gt_point_ego = car_state.transform_to_car_frame(gt_point_global)
error = np.linalg.norm(gt_point_ego - perceived_point_ego)  # Correct!
```

### 2. Use Frenet Frame for Path-Aligned Operations

```python
# ✅ Good: Use Frenet for path-aligned comparisons
s, d = frenet_frame.global_to_frenet(point)
d_inner, d_outer = frenet_map.get_bounds_at_s(s)
deviation = abs(d - d_inner) if d < d_inner else abs(d - d_outer)
```

### 3. Cache Frenet Frame Conversions

```python
# Cache Frenet frame for repeated use
frenet_frame = FrenetFrame(track)

# Reuse for multiple conversions
for point in many_points:
    s, d = frenet_frame.global_to_frenet(point)  # Efficient
```

## Error Handling

Frame conversions can fail if:
- Point is too far from track (for Frenet conversion)
- Invalid sensor pose
- Numerical issues

Always handle errors gracefully:

```python
try:
    s, d = frenet_frame.global_to_frenet(point)
except Exception as e:
    # Handle error (skip point, use default, etc.)
    continue
```

## Performance Tips

1. **Batch Conversions**: Convert multiple points at once when possible
2. **Cache Frenet Frame**: Don't recreate `FrenetFrame` for each conversion
3. **Filter First**: Filter points by distance before converting
4. **Use Vectorized Operations**: NumPy operations are faster

## Examples

See the [Jupyter Notebook](../../notebooks/track_bounds_alert.ipynb) for interactive examples of frame conversions with visualizations.

## See Also

- [Building Alert Systems](building-alert-systems.md) - Uses frame conversions
- [Track Bounds Alert](track-bounds-alert.md) - Practical application
- [API Reference](../api/overview.md) - Complete API documentation
