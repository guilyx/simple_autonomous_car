"""Simple Autonomous Car SDK.

A comprehensive SDK for building autonomous vehicle systems with modular sensors,
controllers, planners, and alert systems.
"""

__version__ = "0.3.0"

# Core modules
# Constants (available as simple_autonomous_car.constants)
import simple_autonomous_car.constants
from simple_autonomous_car.alerts import TrackBoundsAlert
from simple_autonomous_car.car import Car, CarState
from simple_autonomous_car.control import BaseController, PIDController, PurePursuitController
from simple_autonomous_car.costmap import BaseCostmap, GridCostmap, inflate_obstacles
from simple_autonomous_car.detection import LocalizationErrorDetector
from simple_autonomous_car.filters import BaseFilter, KalmanFilter, ParticleFilter
from simple_autonomous_car.frames import (
    FrenetFrame,
    ego_to_frenet,
    ego_to_sensor,
    frenet_to_ego,
    frenet_to_global,
    global_to_frenet,
    sensor_to_ego,
)
from simple_autonomous_car.maps import FrenetMap, GridMap, GroundTruthMap, PerceivedMap
from simple_autonomous_car.perception import PerceptionPoints
from simple_autonomous_car.planning import BasePlanner, GoalPlanner, TrackPlanner
from simple_autonomous_car.sensors import BaseSensor, LiDARSensor
from simple_autonomous_car.track import Track
from simple_autonomous_car.visualization import (
    AlertVisualizer,
    plot_car,
    plot_control_history,
    plot_perception,
    plot_pure_pursuit_state,
)

__all__ = [
    # Version
    "__version__",
    # Track
    "Track",
    # Car
    "Car",
    "CarState",
    # Maps
    "GroundTruthMap",
    "PerceivedMap",
    "FrenetMap",
    "GridMap",
    # Perception
    "PerceptionPoints",
    # Frames
    "FrenetFrame",
    "global_to_frenet",
    "frenet_to_global",
    "ego_to_frenet",
    "frenet_to_ego",
    "sensor_to_ego",
    "ego_to_sensor",
    # Sensors
    "BaseSensor",
    "LiDARSensor",
    # Control
    "BaseController",
    "PurePursuitController",
    "PIDController",
    # Planning
    "BasePlanner",
    "TrackPlanner",
    "GoalPlanner",
    # Costmap
    "BaseCostmap",
    "GridCostmap",
    "inflate_obstacles",
    # Detection
    "LocalizationErrorDetector",
    # Alerts
    "TrackBoundsAlert",
    # Filters
    "BaseFilter",
    "KalmanFilter",
    "ParticleFilter",
    # Footprints
    "BaseFootprint",
    "RectangularFootprint",
    "CircularFootprint",
    # Visualization
    "AlertVisualizer",
    "plot_perception",
    "plot_car",
    "plot_pure_pursuit_state",
    "plot_control_history",
]
