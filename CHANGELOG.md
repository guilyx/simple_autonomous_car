# Changelog

All notable changes to this project will be documented in this file.

## [0.1.0] - 2026-XX-XX

### Added
- **Modular Sensor System**: Complete sensor architecture
  - `BaseSensor` abstract class for all sensors
  - `LiDARSensor` implementation
  - Car can have multiple sensors via `car.add_sensor()`
  - Sensors can be enabled/disabled individually

- **Control System**: Complete control architecture
  - `BaseController` abstract class
  - `PurePursuitController` for path following
  - `PIDController` for PID control
  - Controllers work with plans from planners

- **Planning System**: Path planning framework
  - `BasePlanner` abstract class
  - `TrackPlanner` for track following
  - Planners generate waypoints for controllers

- **Enhanced Car Class**:
  - Support for multiple sensors
  - `car.add_sensor()`, `car.remove_sensor()`, `car.get_sensor()`
  - `car.sense_all()` to get data from all sensors

- **Controller Notebook**: Interactive tutorial for building controllers

### Changed
- **Project Renamed**: From `car-loc-viz` to `simple-autonomous-car`
- **Package Renamed**: From `car_loc_viz` to `simple_autonomous_car`
- **Architecture**: Modular OOP design with pluggable components
- **Sensor System**: Replaced single `Sensor` class with modular `BaseSensor` system

### Removed
- Old `Sensor` class (replaced by modular sensor system)

## [0.0.2] - 2026-XX-XX

### Added
- **Frenet Frame Support**: Complete Frenet coordinate system implementation
  - `FrenetFrame` class for path-aligned coordinates
  - Conversion utilities: `global_to_frenet`, `frenet_to_global`, `ego_to_frenet`, `frenet_to_ego`, `sensor_to_ego`, `ego_to_sensor`
  - `FrenetMap` for map representation in Frenet coordinates

- **PerceptionPoints**: New data structure for perception data
  - Vector of points in local (ego) frame
  - Frame conversion utilities
  - Distance filtering

- **Alert System**: Track Bounds Alert System
  - `TrackBoundsAlert` class for detecting boundary deviations
  - Comprehensive documentation
  - Jupyter notebook with examples

- **Documentation**:
  - Building on Top guide
  - Track Bounds Alert System documentation
  - Jupyter notebook tutorials

- **CI/CD**:
  - GitHub Actions workflows for lint, test, and build
  - PyPI publishing workflow
  - Multi-platform testing (Linux, Windows, macOS)
  - Multiple Python version support (3.10, 3.11, 3.12)

- **SDK Structure**:
  - Clean module organization
  - Comprehensive `__init__.py` exports
  - Type hints throughout
  - Error handling

### Changed
- Refactored code structure for better modularity
- Updated sensor to return `PerceptionPoints` instead of raw arrays
- Improved project metadata for PyPI publishing
- Enhanced documentation structure

### Fixed
- Frame conversion utilities
- Error handling in alert system
- Import structure

## [0.0.1] - 2026-XX-XX

### Added
- Initial release
- Basic track generation
- Car model and dynamics
- Ground truth and perceived maps
- Basic visualization
- Error detection
