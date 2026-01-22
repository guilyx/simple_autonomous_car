# Module Reference

Detailed documentation for each module in the Simple Autonomous Car SDK.

## Track Module

### `Track`

Represents a racing track with centerline and boundaries.

#### Methods

- `create_simple_track(length, width, track_width, num_points)` - Create simple track
- `create_oval_track(length, width, track_width, num_points)` - Create oval track
- `get_point_at_distance(distance)` - Get point at distance along track
- `get_bounds_at_point(point)` - Get boundaries at a point

#### Attributes

- `centerline` - Track centerline points (N, 2)
- `inner_bound` - Inner boundary points (N, 2)
- `outer_bound` - Outer boundary points (N, 2)
- `track_width` - Track width (meters)

## Car Module

### `Car`

Racing car model with bicycle dynamics.

#### Methods

- `update(dt, acceleration, steering_rate)` - Update car state
- `get_corners(length, width)` - Get car corner points

#### Attributes

- `state` - `CarState` object
- `wheelbase` - Distance between axles (meters)
- `max_velocity` - Maximum velocity (m/s)
- `max_steering_angle` - Maximum steering angle (radians)

### `CarState`

Vehicle state information.

#### Attributes

- `x` - X position in world frame
- `y` - Y position in world frame
- `heading` - Heading angle (radians, 0 = east)
- `velocity` - Forward velocity (m/s)
- `steering_angle` - Steering angle (radians)

#### Methods

- `position()` - Get position as [x, y] array
- `transform_to_car_frame(point)` - Transform point to car frame
- `transform_to_world_frame(point)` - Transform point to world frame
- `rotation_matrix()` - Get rotation matrix

## Maps Module

### `GroundTruthMap`

Perfect reference map of the track.

#### Methods

- `get_visible_segments(car_position, car_heading, horizon, fov)` - Get visible segments
- `get_all_bounds()` - Get all boundaries

### `PerceivedMap`

Noisy/imperfect perception map.

#### Parameters

- `ground_truth` - Ground truth map
- `position_noise_std` - Position estimation error (meters)
- `orientation_noise_std` - Orientation estimation error (radians)
- `measurement_noise_std` - Measurement noise (meters)

#### Methods

- `update_perceived_state(car_state)` - Update perceived car state
- `get_perceived_segments(horizon, fov)` - Get segments in car frame
- `get_perceived_segments_world_frame(horizon, fov)` - Get in world frame

### `Sensor`

Sensor that generates point clouds.

#### Parameters

- `ground_truth_map` - Ground truth map
- `perceived_map` - Perceived map
- `max_range` - Maximum sensor range (meters)
- `angular_resolution` - Angular resolution (radians)
- `point_noise_std` - Point measurement noise (meters)

#### Methods

- `get_map_lines_car_frame(car_state)` - Get map lines in car frame
- `get_perception_point_cloud_car_frame(car_state)` - Get perception point cloud

### `FrenetMap`

Map representation in Frenet coordinates.

#### Methods

- `get_bounds_at_s(s)` - Get bounds at distance s
- `get_bounds_in_range(s_start, s_end)` - Get bounds in range

#### Attributes

- `frenet_frame` - `FrenetFrame` instance
- `inner_bound_frenet` - Inner bounds in Frenet (N, 2)
- `outer_bound_frenet` - Outer bounds in Frenet (N, 2)

## Perception Module

### `PerceptionPoints`

Represents perception data as vector of points.

#### Parameters

- `points` - Array of shape (N, 2) with [x, y] coordinates
- `frame` - Frame of points ("ego", "sensor", "global")

#### Methods

- `to_ego_frame(car_state)` - Convert to ego frame
- `to_global_frame(car_state)` - Convert to global frame
- `filter_by_distance(max_distance)` - Filter by distance

#### Attributes

- `points` - Point array (N, 2)
- `frame` - Current frame

## Frames Module

### `FrenetFrame`

Frenet coordinate system along a path.

#### Methods

- `global_to_frenet(point)` - Convert global to Frenet (s, d)
- `frenet_to_global(s, d)` - Convert Frenet to global point
- `get_tangent_normal(s)` - Get tangent and normal at s
- `get_closest_point(point)` - Find closest point on path

#### Attributes

- `total_length` - Total path length (meters)
- `cumulative_distances` - Cumulative distances array

### Conversion Functions

- `global_to_frenet(point, frenet_frame)` - Global → Frenet
- `frenet_to_global(s, d, frenet_frame)` - Frenet → Global
- `ego_to_frenet(point, car_state, frenet_frame)` - Ego → Frenet
- `frenet_to_ego(s, d, car_state, frenet_frame)` - Frenet → Ego
- `sensor_to_ego(point, sensor_pose_ego)` - Sensor → Ego
- `ego_to_sensor(point, sensor_pose_ego)` - Ego → Sensor

## Alerts Module

### `TrackBoundsAlert`

Track bounds alert system.

#### Parameters

- `frenet_map` - Frenet map instance
- `warning_threshold` - Warning threshold (meters)
- `critical_threshold` - Critical threshold (meters)
- `lookahead_distance` - Lookahead distance (meters)

#### Methods

- `check(perception_points, car_state)` - Check for alerts
- `get_recent_alerts(num_recent)` - Get alert history
- `reset_history()` - Clear history

#### Returns (from `check()`)

```python
{
    "has_warning": bool,
    "has_critical": bool,
    "max_deviation": float,
    "mean_deviation": float,
    "deviations": np.ndarray,
    "alert_points": List[Dict],
}
```

## Visualization Module

### `AlertVisualizer`

Visualization utilities for alert systems.

#### Methods

- `create_figure()` - Create figure with axes
- `plot_track(ax)` - Plot track boundaries
- `plot_perception(perception_points, car, highlight_alerts, alert_points)` - Plot perception
- `plot_car(car, ax)` - Plot car position
- `plot_alert_result(alert_result, perception_points, car)` - Complete alert visualization
- `plot_alert_history(alert_history)` - Plot alert history
- `show()` - Display figure

**Note**: `TrackVisualizer` has been removed. Use component-based visualization:
- `track.visualize(ax, frame, **kwargs)`
- `planner.visualize(ax, car_state, plan, frame, **kwargs)`
- `controller.visualize(ax, car_state, plan, frame, **kwargs)`
- `costmap.visualize(ax, car_state, frame, **kwargs)`

Or use `enhanced_visualization` module for simulation visualizations.

## Detection Module

### `LocalizationErrorDetector`

Error detection between perceived and ground truth.

#### Parameters

- `ground_truth_map` - Ground truth map
- `perceived_map` - Perceived map
- `error_threshold` - Error threshold (meters)

#### Methods

- `compute_errors(car_state, horizon, fov)` - Compute error metrics
- `detect_errors(car_state, horizon, fov)` - Detect if errors exceed threshold

#### Returns (from `compute_errors()`)

```python
{
    "centerline_errors": np.ndarray,
    "inner_bound_errors": np.ndarray,
    "outer_bound_errors": np.ndarray,
    "max_error": float,
    "mean_error": float,
}
```

## See Also

- [API Overview](overview.md) - High-level API overview
- [Guides](../guides/) - Usage guides
