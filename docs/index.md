# Simple Autonomous Car SDK Documentation

Welcome to the Simple Autonomous Car SDK documentation! This comprehensive guide will help you build autonomous vehicle systems with modular sensors, controllers, planners, and alert systems.

## Documentation Structure

### Getting Started
- [Installation](getting-started/installation.md) - How to install and set up the SDK
- [Quick Start Guide](getting-started/quickstart.md) - Get up and running quickly

### Tutorials
Step-by-step guides for learning SDK components:
- [Costmap Tutorial](notebooks/tutorials/costmap_tutorial.ipynb) - Learn how to use costmaps for obstacle representation
- [Planner Tutorial](notebooks/tutorials/planner_tutorial.ipynb) - Learn how to build and use planners
- [Track Bounds Alert Tutorial](notebooks/tutorials/track_bounds_alert.ipynb) - Learn how to build track bounds alert systems
- [Sensor Fusion Tutorial](notebooks/tutorials/sensor_fusion_tutorial.ipynb) - Learn how to combine data from multiple sensors
- [Performance Optimization](notebooks/tutorials/performance_optimization.ipynb) - Learn how to optimize simulation performance

### Building Custom Components
Complete guides for extending the SDK:
- [Building Simulations](notebooks/building/building_simulation.ipynb) - Learn how to build complete simulations
- [Building Controllers](notebooks/building/building_controller.ipynb) - Learn how to build custom control algorithms
- [Building Custom Sensors](notebooks/building/building_custom_sensors.ipynb) - Learn how to build custom sensors (Camera, Radar, etc.)
- [Building Custom Planners](notebooks/building/building_custom_planners.ipynb) - Learn how to build custom planning algorithms

### Learning Notebooks (Fill-in-the-Blank)
**Incomplete notebooks** where you fill in the code - perfect for hands-on learning:
- [Learning: Build a Costmap](notebooks/learning/learning_build_costmap.ipynb) - Implement your own costmap
- [Learning: Build a Planner](notebooks/learning/learning_build_planner.ipynb) - Implement your own planner
- [Learning: Build a Controller](notebooks/learning/learning_build_controller.ipynb) - Implement your own controller
- [Learning: Build a Sensor](notebooks/learning/learning_build_sensor.ipynb) - Implement your own sensor
- [Learning: Advanced Planning Algorithms](notebooks/learning/advanced_planning_algorithms.ipynb) - Implement A* and RRT

### Reference Guides
- [Visualization Guide](guides/visualization.md) - Easy-to-use visualization functions for all components
- [Costmap Guide](guides/costmap.md) - Costmap system for obstacle representation and inflation
- [Building Alert Systems](guides/building-alert-systems.md) - Comprehensive guide for building alert and warning systems
- [Track Bounds Alert System](guides/track-bounds-alert.md) - Detailed documentation for the track bounds alert system
- [Frame Conversions](guides/frame-conversions.md) - Understanding and using coordinate frame conversions

### API Reference
- [API Overview](api/overview.md) - Overview of the SDK API
- [Module Reference](api/modules.md) - Detailed module documentation

### Additional Resources
- [Contributing](../CONTRIBUTING.md) - How to contribute to the project
- [Notebooks](../notebooks/) - All Jupyter notebooks organized by category
- [Examples](../notebooks/examples/) - Example notebooks for specific use cases
- [Changelog](../CHANGELOG.md) - Version history

## Quick Navigation

**New to the SDK?** Start with [Installation](getting-started/installation.md) and [Quick Start](getting-started/quickstart.md).

**Building an alert system?** See [Building Alert Systems](guides/building-alert-systems.md) and [Track Bounds Alert](guides/track-bounds-alert.md).

**Need API details?** Check the [API Reference](api/overview.md).

## Key Concepts

### Coordinate Frames
- **Global Frame**: World coordinates (track reference)
- **Ego Frame**: Car-centered (x=forward, y=left)
- **Sensor Frame**: Sensor-centered coordinates
- **Frenet Frame**: Path-aligned (s=distance along path, d=lateral offset)

### Core Components
- **Track**: Racing track with boundaries
- **Car**: Vehicle model with dynamics and sensor support
- **Sensors**: Modular sensor system (LiDAR, Camera, Radar, etc.)
- **Controllers**: Control algorithms (Pure Pursuit, PID, etc.)
- **Planners**: Path planning algorithms (Track Planner, A*, etc.)
- **Costmaps**: Grid-based obstacle representation with inflation
- **FrenetMap**: Map in Frenet coordinates
- **PerceptionPoints**: Vector of perception data
- **AlertVisualizer**: Built-in visualization tools
