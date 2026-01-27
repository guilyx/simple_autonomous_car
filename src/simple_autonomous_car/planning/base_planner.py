"""Base planner class for path planning."""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any, Optional

import numpy as np

if TYPE_CHECKING:
    from simple_autonomous_car.car.car import CarState


if TYPE_CHECKING:
    from simple_autonomous_car.costmap.base_costmap import BaseCostmap


class BasePlanner(ABC):
    """
    Base class for all path planners.

    Planners generate paths (sequences of waypoints) that the car should follow.
    Controllers can then use these plans to compute control commands.

    Attributes
    ----------
    name : str
        Planner name/identifier.
    enabled : bool
        Whether the planner is enabled.
    """

    def __init__(self, name: str = "planner", enabled: bool = True):
        """
        Initialize base planner.

        Parameters
        ----------
        name : str, default="planner"
            Planner name/identifier.
        enabled : bool, default=True
            Whether the planner is enabled.
        """
        self.name = name
        self.enabled = enabled

    @abstractmethod
    def plan(
        self,
        car_state: "CarState",
        perception_data: dict | None = None,
        costmap: Optional["BaseCostmap"] = None,
        goal: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Generate a plan (path) for the car to follow.

        Parameters
        ----------
        car_state : CarState
            Current car state.
        perception_data : dict, optional
            Dictionary of perception data from sensors.
        costmap : BaseCostmap, optional
            Costmap for obstacle avoidance.
        goal : np.ndarray, optional
            Goal position as [x, y] array.

        Returns
        -------
        np.ndarray
            Planned path as array of shape (N, 2) with [x, y] waypoints.
            Empty array if planning fails or planner is disabled.
        """
        pass

    def is_enabled(self) -> bool:
        """Check if planner is enabled."""
        return self.enabled

    def enable(self) -> None:
        """Enable the planner."""
        self.enabled = True

    def disable(self) -> None:
        """Disable the planner."""
        self.enabled = False

    def get_visualization_data(
        self, car_state: "CarState", plan: np.ndarray | None = None, **kwargs: Any
    ) -> dict[str, Any]:
        """
        Get visualization data for this planner.

        This method should be overridden by subclasses to provide
        planner-specific visualization data (e.g., waypoints, path segments,
        planning metadata).

        Parameters
        ----------
        car_state : CarState
            Current car state.
        plan : np.ndarray, optional
            Planned path (if already computed).
        **kwargs
            Additional arguments.

        Returns
        -------
        Dict
            Dictionary containing visualization data. Default implementation
            returns the plan if provided. Subclasses should override to provide
            additional data.
        """
        if plan is not None:
            return {"plan": plan}
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
        Visualize planned path on the given axes.

        This method should be overridden by subclasses to plot
        planner-specific visualizations (e.g., waypoints, path segments).

        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Axes to plot on.
        car_state : CarState
            Current car state.
        plan : np.ndarray, optional
            Planned path (if already computed, otherwise will be computed).
        frame : str, default="global"
            Frame to plot in: "global" or "ego".
        **kwargs
            Additional visualization arguments (colors, linewidths, etc.).
        """
        # Default implementation: plot plan if available
        if plan is not None and len(plan) > 0:
            # Transform plan to desired frame
            if frame == "ego":
                plan_plot = np.array([car_state.transform_to_car_frame(point) for point in plan])
            else:
                plan_plot = plan

            # Extract visualization parameters
            color = kwargs.pop("color", "green")
            linewidth = kwargs.pop("linewidth", 2.5)
            linestyle = kwargs.pop("linestyle", "--")
            show_waypoints = kwargs.pop("show_waypoints", frame == "global")
            alpha = kwargs.pop("alpha", 0.8)

            # Plot plan
            if show_waypoints:
                ax.plot(
                    plan_plot[:, 0],
                    plan_plot[:, 1],
                    "o-",
                    color=color,
                    linewidth=linewidth,
                    linestyle=linestyle,
                    markersize=4,
                    label="Plan",
                    alpha=alpha,
                    **kwargs,
                )
            else:
                ax.plot(
                    plan_plot[:, 0],
                    plan_plot[:, 1],
                    "-",
                    color=color,
                    linewidth=linewidth,
                    linestyle=linestyle,
                    label="Plan",
                    alpha=alpha,
                    **kwargs,
                )
