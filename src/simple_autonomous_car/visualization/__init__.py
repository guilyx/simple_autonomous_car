"""Visualization and animation utilities.

All visualization is now component-based. Use component.visualize() methods:
- track.visualize()
- planner.visualize()
- controller.visualize()
- costmap.visualize()

Utility functions are provided for non-component data (perception, car).
"""

from simple_autonomous_car.visualization.alert_viz import AlertVisualizer
from simple_autonomous_car.visualization.utils import plot_perception, plot_car
from simple_autonomous_car.control.controller_viz import (
    plot_pure_pursuit_state,
    plot_control_history,
)

__all__ = [
    "AlertVisualizer",
    "plot_perception",
    "plot_car",
    "plot_pure_pursuit_state",
    "plot_control_history",
]
