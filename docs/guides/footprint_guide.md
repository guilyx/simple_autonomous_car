# Footprint Guide

The footprint module provides accurate vehicle shape representation for collision avoidance and costmap inflation.

## Overview

A **footprint** represents the 2D shape of your vehicle. Instead of using an arbitrary inflation radius in costmaps, footprints calculate the exact minimum safe distance based on your vehicle's actual dimensions.

### Why Use Footprints?

**Traditional approach (arbitrary inflation):**
```python
costmap = GridCostmap(inflation_radius=2.5)  # Why 2.5? Is it enough? Too much?
```

**Footprint-based approach (accurate):**
```python
footprint = RectangularFootprint(length=4.5, width=1.8)
costmap = GridCostmap(footprint=footprint, footprint_padding=0.3)  # Accurate!
```

**Benefits:**
- ✅ **Accurate**: Inflation matches actual vehicle size
- ✅ **Efficient**: No over-inflation (wasteful) or under-inflation (unsafe)
- ✅ **Flexible**: Easy to change vehicle size
- ✅ **Safe**: Ensures entire vehicle footprint avoids obstacles

## Basic Usage

### Creating a Footprint

```python
from simple_autonomous_car import RectangularFootprint, CircularFootprint

# Rectangular footprint (typical for cars)
car_footprint = RectangularFootprint(
    length=4.5,  # meters (front to back)
    width=1.8,   # meters (left to right)
)

# Circular footprint (simpler, less accurate)
circular_footprint = CircularFootprint(radius=2.5)
```

### Using with Costmaps

```python
from simple_autonomous_car import GridCostmap

# Create costmap with footprint
costmap = GridCostmap(
    width=50.0,
    height=50.0,
    resolution=0.5,
    footprint=car_footprint,      # Pass footprint here
    footprint_padding=0.3,         # 30cm safety margin
    frame="ego"
)

# Inflation radius is automatically calculated!
print(f"Inflation radius: {costmap.inflation_radius:.2f}m")
# Output: Inflation radius: 2.78m (based on footprint diagonal + padding)
```

### Footprint Properties

```python
# Get bounding radius (distance from center to farthest point)
bounding_radius = footprint.get_bounding_radius()

# Get inflation radius (bounding radius + padding)
inflation_radius = footprint.get_inflation_radius(padding=0.5)

# Get footprint vertices at a position/heading
vertices = footprint.get_vertices(
    position=np.array([10.0, 5.0]),
    heading=np.pi/4
)

# Check if a point is inside footprint
is_inside = footprint.contains_point(
    point=np.array([10.5, 5.2]),
    position=np.array([10.0, 5.0]),
    heading=0.0
)

# Check collision with obstacles
has_collision = footprint.check_collision(
    obstacles=obstacle_array,
    position=np.array([10.0, 5.0]),
    heading=0.0
)
```

## Integration with Costmaps

### Automatic Inflation Calculation

When you provide a footprint to `GridCostmap`, the inflation radius is automatically calculated:

```python
footprint = RectangularFootprint(length=4.5, width=1.8)
# Bounding radius = sqrt((4.5/2)² + (1.8/2)²) ≈ 2.48m

costmap = GridCostmap(
    footprint=footprint,
    footprint_padding=0.3
)
# Inflation radius = 2.48 + 0.3 = 2.78m (automatically set)
```

### Manual Override

You can still provide `inflation_radius` manually if needed:

```python
costmap = GridCostmap(
    footprint=footprint,
    inflation_radius=3.0  # Overrides footprint calculation
)
```

## Creating Custom Footprints

Extend `BaseFootprint` to create custom shapes:

```python
from simple_autonomous_car import BaseFootprint

class LShapedFootprint(BaseFootprint):
    """L-shaped vehicle (e.g., forklift)."""
    
    def __init__(self, front_length, rear_length, width):
        super().__init__()
        self.front_length = front_length
        self.rear_length = rear_length
        self.width = width
        # Calculate bounding radius...
    
    def get_vertices(self, position, heading):
        # Define L-shape vertices, rotate, translate
        pass
    
    def get_bounding_radius(self):
        # Return max distance from center to corner
        pass
    
    def contains_point(self, point, position, heading):
        # Check if point is inside L-shape
        pass
```

## Using in Simulation

```python
from simple_autonomous_car import (
    GridMap, GridCostmap, Car, GoalPlanner, PurePursuitController,
    RectangularFootprint
)

# Create footprint
footprint = RectangularFootprint(length=4.5, width=1.8)

# Create costmap with footprint
costmap = GridCostmap(
    width=40.0,
    height=40.0,
    resolution=0.5,
    footprint=footprint,
    footprint_padding=0.3,
    frame="ego"
)

# Use in simulation
grid_map = GridMap.create_random_map(...)
all_obstacles = np.vstack([grid_map.obstacles, grid_map.get_boundary_obstacles()])

# Update costmap (inflation uses footprint automatically)
costmap.update(static_obstacles=all_obstacles, car_state=car.state)

# Plan with footprint-aware costmap
planner = GoalPlanner(grid_map=grid_map)
plan = planner.plan(car.state, costmap=costmap, goal=goal)
```

## Path Collision Checking

Footprints can be used to validate paths:

```python
def is_path_safe(path, footprint, obstacles):
    """Check if entire path is collision-free."""
    for waypoint in path:
        if footprint.check_collision(obstacles, waypoint, heading):
            return False
    return True
```

## Best Practices

1. **Choose appropriate footprint type**:
   - `RectangularFootprint`: For cars, trucks (most common)
   - `CircularFootprint`: For simple/quick prototyping

2. **Set reasonable padding**:
   - Too small: Risk of collision
   - Too large: Overly conservative, inefficient paths
   - Recommended: 0.2-0.5 meters

3. **Match footprint to actual vehicle**:
   - Measure your vehicle's length and width
   - Account for mirrors, bumpers, etc.

4. **Use footprint in all costmaps**:
   - Consistent safety margins
   - Accurate planning

## API Reference

### BaseFootprint

- `get_vertices(position, heading)` → `np.ndarray`: Get footprint vertices
- `get_bounding_radius()` → `float`: Get bounding radius
- `get_inflation_radius(padding)` → `float`: Get inflation radius
- `contains_point(point, position, heading)` → `bool`: Check if point inside
- `check_collision(obstacles, position, heading)` → `bool`: Check collision

### RectangularFootprint

- `length`: Vehicle length (meters)
- `width`: Vehicle width (meters)
- `center_offset`: Offset from vehicle position (optional)

### CircularFootprint

- `radius`: Vehicle radius (meters)

## See Also

- [Costmap Guide](costmap.md) - General costmap usage
- [Footprint Integration Notebook](../notebooks/learning/costmaps/footprint_integration.ipynb) - Hands-on tutorial
