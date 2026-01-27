"""Base controller class for autonomous vehicle control."""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

if TYPE_CHECKING:
    from simple_autonomous_car.car.car import CarState
    from simple_autonomous_car.costmap.base_costmap import BaseCostmap

from simple_autonomous_car.perception.perception import PerceptionPoints


class BaseController(ABC):
    """
    Base class for all controllers.

    Controllers compute control commands (acceleration, steering_rate)
    based on the current car state, perception data, and optionally a plan.

    Attributes
    ----------
    name : str
        Controller name/identifier.
    enabled : bool
        Whether the controller is enabled.
    """

    def __init__(self, name: str = "controller", enabled: bool = True):
        """
        Initialize base controller.

        Parameters
        ----------
        name : str, default="controller"
            Controller name/identifier.
        enabled : bool, default=True
            Whether the controller is enabled.
        """
        self.name = name
        self.enabled = enabled

    @abstractmethod
    def compute_control(
        self,
        car_state: "CarState",
        perception_data: dict[str, PerceptionPoints] | None = None,
        costmap: Optional["BaseCostmap"] = None,
        plan: np.ndarray | None = None,
        goal: np.ndarray | None = None,
        goal_tolerance: float | None = None,
        dt: float = 0.1,
    ) -> dict[str, float]:
        """
        Compute control commands.

        Parameters
        ----------
        car_state : CarState
            Current car state.
        perception_data : dict, optional
            Dictionary of perception data from sensors.
            Keys are sensor names, values are PerceptionPoints.
        costmap : BaseCostmap, optional
            Costmap for obstacle avoidance.
        plan : np.ndarray, optional
            Planned path as array of shape (N, 2) with [x, y] waypoints.
        goal : np.ndarray, optional
            Goal position [x, y] for goal-based velocity adaptation.
        dt : float, default=0.1
            Time step in seconds.

        Returns
        -------
        Dict[str, float]
            Control commands with keys:
            - "acceleration": Acceleration in m/s²
            - "steering_rate": Steering rate in rad/s
        """
        pass

    def is_enabled(self) -> bool:
        """Check if controller is enabled."""
        return self.enabled

    def enable(self) -> None:
        """Enable the controller."""
        self.enabled = True

    def disable(self) -> None:
        """Disable the controller."""
        self.enabled = False

    def get_visualization_data(
        self, car_state: "CarState", plan: np.ndarray | None = None, **kwargs: Any
    ) -> dict[str, Any]:
        """
        Get visualization data for this controller.

        This method should be overridden by subclasses to provide
        controller-specific visualization data (e.g., lookahead points,
        steering arcs, control commands).

        Parameters
        ----------
        car_state : CarState
            Current car state.
        plan : np.ndarray, optional
            Planned path.
        **kwargs
            Additional arguments.

        Returns
        -------
        Dict
            Dictionary containing visualization data. Default implementation
            returns empty dict. Subclasses should override to provide specific data.
        """
        return {}

    def visualize(
        self,
        ax: Any,
        car_state: "CarState",
        plan: np.ndarray | None = None,
        frame: str = "global",
        **kwargs: Any,
    ) -> None:
        """
        Visualize controller state on the given axes.

        This method should be overridden by subclasses to plot
        controller-specific visualizations (e.g., lookahead points,
        steering arcs, control commands).

        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Axes to plot on.
        car_state : CarState
            Current car state.
        plan : np.ndarray, optional
            Planned path.
        frame : str, default="global"
            Frame to plot in: "global" or "ego".
        **kwargs
            Additional visualization arguments (colors, linewidths, etc.).
        """
        # Default implementation does nothing
        # Subclasses should override to provide visualization
        pass
