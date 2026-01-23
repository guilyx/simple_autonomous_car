"""Base costmap class for obstacle representation."""

from abc import ABC, abstractmethod
import numpy as np
from typing import Optional, Tuple, Dict

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.perception.perception import PerceptionPoints


class BaseCostmap(ABC):
    """
    Base class for all costmaps.

    Costmaps represent the cost of traversing different areas in the environment,
    typically derived from perception data (obstacles, track boundaries, etc.).

    Attributes
    ----------
    resolution : float
        Resolution of the costmap (meters per cell).
    inflation_radius : float
        Inflation radius for obstacles (meters).
    enabled : bool
        Whether the costmap is enabled.
    """

    def __init__(
        self,
        resolution: float = 0.5,
        inflation_radius: float = 1.0,
        enabled: bool = True,
    ):
        """
        Initialize base costmap.

        Parameters
        ----------
        resolution : float, default=0.5
            Resolution of the costmap in meters per cell.
        inflation_radius : float, default=1.0
            Inflation radius for obstacles in meters.
        enabled : bool, default=True
            Whether the costmap is enabled.
        """
        self.resolution = resolution
        self.inflation_radius = inflation_radius
        self.enabled = enabled

    @abstractmethod
    def update(
        self,
        perception_data: Optional[dict] = None,
        car_state: Optional[CarState] = None,
        static_obstacles: Optional[np.ndarray] = None,
        frame: str = "global",
    ) -> None:
        """
        Update costmap from perception data or static obstacles.

        Parameters
        ----------
        perception_data : dict, optional
            Dictionary of perception data from sensors.
            Keys are sensor names, values are PerceptionPoints.
        car_state : CarState, optional
            Current car state (required for ego frame or perception_data).
        static_obstacles : np.ndarray, optional
            Static obstacles as array of shape (N, 2) with [x, y] positions in global frame.
            Used for static maps when perception_data is not available.
        frame : str, default="global"
            Frame to use for costmap ("global" or "ego").
        """
        pass

    @abstractmethod
    def get_cost(self, position: np.ndarray, frame: str = "global") -> float:
        """
        Get cost at a specific position.

        Parameters
        ----------
        position : np.ndarray
            Position as [x, y] array.
        frame : str, default="global"
            Frame of the position ("global" or "ego").

        Returns
        -------
        float
            Cost value (0.0 = free, 1.0 = occupied, 0.0-1.0 = inflated).
        """
        pass

    @abstractmethod
    def get_cost_region(
        self,
        center: np.ndarray,
        size: float,
        frame: str = "global",
    ) -> np.ndarray:
        """
        Get cost values in a region.

        Parameters
        ----------
        center : np.ndarray
            Center position as [x, y].
        size : float
            Size of the region in meters.
        frame : str, default="global"
            Frame of the center position.

        Returns
        -------
        np.ndarray
            2D array of cost values.
        """
        pass

    def is_enabled(self) -> bool:
        """Check if costmap is enabled."""
        return self.enabled

    def enable(self) -> None:
        """Enable the costmap."""
        self.enabled = True

    def disable(self) -> None:
        """Disable the costmap."""
        self.enabled = False

    def get_visualization_data(
        self,
        car_state: Optional[CarState] = None,
        **kwargs
    ) -> Dict:
        """
        Get visualization data for this costmap.
        
        This method should be overridden by subclasses to provide
        costmap-specific visualization data (e.g., costmap array, bounds, origin).
        
        Parameters
        ----------
        car_state : CarState, optional
            Current car state (for frame transformations).
        **kwargs
            Additional arguments.
        
        Returns
        -------
        Dict
            Dictionary containing visualization data. Default implementation
            returns basic info. Subclasses should override to provide costmap data.
        """
        return {
            "enabled": self.enabled,
            "resolution": self.resolution,
            "inflation_radius": self.inflation_radius,
        }
    
    def visualize(
        self,
        ax,
        car_state: Optional[CarState] = None,
        frame: str = "global",
        **kwargs
    ) -> None:
        """
        Visualize costmap on the given axes.
        
        This method should be overridden by subclasses to plot
        costmap-specific visualizations.
        
        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Axes to plot on.
        car_state : CarState, optional
            Current car state (for frame transformations).
        frame : str, default="global"
            Frame to plot in: "global" or "ego".
        **kwargs
            Additional visualization arguments (colors, alpha, etc.).
        """
        # Default implementation does nothing
        # Subclasses should override to provide visualization
        pass
