"""Rectangular footprint for vehicles."""

import numpy as np
from typing import Optional

from simple_autonomous_car.footprint.base_footprint import BaseFootprint


class RectangularFootprint(BaseFootprint):
    """
    Rectangular footprint for vehicles.
    
    Represents a vehicle as a rectangle (typical for cars).
    
    Parameters
    ----------
    length : float
        Vehicle length in meters (front to back).
    width : float
        Vehicle width in meters (left to right).
    center_offset : np.ndarray, optional
        Offset of footprint center from vehicle position [x, y] in vehicle frame.
        Default: [0, 0] (center at vehicle position).
    name : str, default="rectangular_footprint"
        Footprint name.
    """
    
    def __init__(
        self,
        length: float,
        width: float,
        center_offset: Optional[np.ndarray] = None,
        name: str = "rectangular_footprint",
    ):
        super().__init__(name=name)
        self.length = length
        self.width = width
        self.center_offset = center_offset if center_offset is not None else np.array([0.0, 0.0])
        
        # Calculate bounding radius (half diagonal)
        self._bounding_radius = np.sqrt((length / 2) ** 2 + (width / 2) ** 2)
    
    def get_vertices(self, position: np.ndarray, heading: float) -> np.ndarray:
        """
        Get rectangle vertices in world frame.
        
        Parameters
        ----------
        position : np.ndarray
            Vehicle position [x, y] in world frame.
        heading : float
            Vehicle heading angle in radians.
            
        Returns
        -------
        np.ndarray
            Array of shape (4, 2) with rectangle corners.
        """
        # Define rectangle corners in vehicle frame (centered at origin)
        half_length = self.length / 2
        half_width = self.width / 2
        
        corners_vehicle = np.array([
            [half_length, half_width],      # Front-right
            [-half_length, half_width],     # Rear-right
            [-half_length, -half_width],    # Rear-left
            [half_length, -half_width],     # Front-left
        ])
        
        # Apply center offset
        corners_vehicle += self.center_offset
        
        # Rotate by heading
        cos_h = np.cos(heading)
        sin_h = np.sin(heading)
        rotation_matrix = np.array([
            [cos_h, -sin_h],
            [sin_h, cos_h]
        ])
        corners_rotated = (rotation_matrix @ corners_vehicle.T).T
        
        # Translate to world position
        vertices = corners_rotated + position
        
        return vertices
    
    def get_bounding_radius(self) -> float:
        """Get bounding radius (half diagonal of rectangle)."""
        return self._bounding_radius
    
    def contains_point(self, point: np.ndarray, position: np.ndarray, heading: float) -> bool:
        """
        Check if point is inside rectangle.
        
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
            True if point is inside rectangle.
        """
        # Transform point to vehicle frame
        point_relative = point - position
        
        # Rotate by -heading to align with vehicle frame
        cos_h = np.cos(-heading)
        sin_h = np.sin(-heading)
        rotation_matrix = np.array([
            [cos_h, -sin_h],
            [sin_h, cos_h]
        ])
        point_vehicle = rotation_matrix @ point_relative
        
        # Apply center offset
        point_vehicle -= self.center_offset
        
        # Check if point is within rectangle bounds
        half_length = self.length / 2
        half_width = self.width / 2
        
        return (abs(point_vehicle[0]) <= half_length and 
                abs(point_vehicle[1]) <= half_width)
