"""Base footprint class for vehicle shape representation."""

from abc import ABC, abstractmethod

import numpy as np


class BaseFootprint(ABC):
    """
    Base class for vehicle footprints.

    A footprint represents the shape of the vehicle in 2D space.
    It's used to ensure the entire vehicle (not just a point) avoids obstacles.

    Attributes
    ----------
    name : str
        Footprint name/identifier.
    """

    def __init__(self, name: str = "footprint"):
        """
        Initialize base footprint.

        Parameters
        ----------
        name : str, default="footprint"
            Footprint name/identifier.
        """
        self.name = name

    @abstractmethod
    def get_vertices(self, position: np.ndarray, heading: float) -> np.ndarray:
        """
        Get footprint vertices in world frame.

        Parameters
        ----------
        position : np.ndarray
            Vehicle position [x, y] in world frame.
        heading : float
            Vehicle heading angle in radians.

        Returns
        -------
        np.ndarray
            Array of shape (N, 2) with vertex positions [[x1, y1], [x2, y2], ...].
        """
        pass

    @abstractmethod
    def get_bounding_radius(self) -> float:
        """
        Get bounding radius (distance from center to farthest point).

        This is used for efficient collision checking and costmap inflation.

        Returns
        -------
        float
            Bounding radius in meters.
        """
        pass

    @abstractmethod
    def contains_point(self, point: np.ndarray, position: np.ndarray, heading: float) -> bool:
        """
        Check if a point is inside the footprint.

        Parameters
        ----------
        point : np.ndarray
            Point [x, y] in world frame.
        position : np.ndarray
            Vehicle position [x, y] in world frame.
        heading : float
            Vehicle heading angle in radians.

        Returns
        -------
        bool
            True if point is inside footprint, False otherwise.
        """
        pass

    def get_inflation_radius(self, padding: float = 0.0) -> float:
        """
        Get inflation radius for costmap (bounding radius + padding).

        Parameters
        ----------
        padding : float, default=0.0
            Additional safety padding in meters.

        Returns
        -------
        float
            Inflation radius in meters.
        """
        return self.get_bounding_radius() + padding

    def check_collision(self, obstacles: np.ndarray, position: np.ndarray, heading: float) -> bool:
        """
        Check if footprint collides with any obstacles.

        Parameters
        ----------
        obstacles : np.ndarray
            Array of obstacle positions [[x1, y1], [x2, y2], ...].
        position : np.ndarray
            Vehicle position [x, y] in world frame.
        heading : float
            Vehicle heading angle in radians.

        Returns
        -------
        bool
            True if collision detected, False otherwise.
        """
        if len(obstacles) == 0:
            return False

        # Simple collision check: if any obstacle is inside footprint
        for obstacle in obstacles:
            if self.contains_point(obstacle, position, heading):
                return True

        return False
