"""Circular footprint for vehicles."""

import numpy as np

from simple_autonomous_car.footprint.base_footprint import BaseFootprint


class CircularFootprint(BaseFootprint):
    """
    Circular footprint for vehicles.
    
    Represents a vehicle as a circle (simpler but less accurate for cars).
    
    Parameters
    ----------
    radius : float
        Vehicle radius in meters.
    name : str, default="circular_footprint"
        Footprint name.
    """
    
    def __init__(self, radius: float, name: str = "circular_footprint"):
        super().__init__(name=name)
        self.radius = radius
    
    def get_vertices(self, position: np.ndarray, heading: float) -> np.ndarray:
        """
        Get circle vertices (approximated as polygon) in world frame.
        
        Parameters
        ----------
        position : np.ndarray
            Vehicle position [x, y] in world frame.
        heading : float
            Vehicle heading angle (not used for circle, but kept for interface compatibility).
            
        Returns
        -------
        np.ndarray
            Array of shape (N, 2) with circle vertices (approximated as 16-sided polygon).
        """
        # Approximate circle as 16-sided polygon
        num_vertices = 16
        angles = np.linspace(0, 2 * np.pi, num_vertices, endpoint=False)
        
        vertices = position + self.radius * np.column_stack([np.cos(angles), np.sin(angles)])
        
        return vertices
    
    def get_bounding_radius(self) -> float:
        """Get bounding radius (same as circle radius)."""
        return self.radius
    
    def contains_point(self, point: np.ndarray, position: np.ndarray, heading: float) -> bool:
        """
        Check if point is inside circle.
        
        Parameters
        ----------
        point : np.ndarray
            Point [x, y] in world frame.
        position : np.ndarray
            Vehicle position [x, y] in world frame.
        heading : float
            Vehicle heading angle (not used for circle).
            
        Returns
        -------
        bool
            True if point is inside circle.
        """
        distance = np.linalg.norm(point - position)
        return distance <= self.radius
