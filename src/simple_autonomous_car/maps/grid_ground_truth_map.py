"""Ground truth map for grid-based environments."""

import numpy as np
from typing import Tuple

from simple_autonomous_car.maps.grid_map import GridMap


class GridGroundTruthMap:
    """
    Ground truth map for grid-based environments with obstacles.
    
    This provides a compatible interface with GroundTruthMap for sensors
    to work with grid map environments.
    
    Parameters
    ----------
    grid_map : GridMap
        Grid map with obstacles.
    """
    
    def __init__(self, grid_map: GridMap):
        """Initialize ground truth map from grid map."""
        self.grid_map = grid_map
        self.track = None  # No track for grid maps
    
    def get_visible_segments(
        self, car_position: np.ndarray, car_heading: float, horizon: float, fov: float = 2 * np.pi
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get visible obstacles within horizon and field of view.
        
        Parameters
        ----------
        car_position : np.ndarray
            Car position [x, y] in world frame.
        car_heading : float
            Car heading angle in radians.
        horizon : float
            Maximum distance to consider.
        fov : float, default=2*pi
            Field of view angle in radians.
            
        Returns
        -------
        Tuple[np.ndarray, np.ndarray, np.ndarray]
            (visible_obstacles, empty, empty) - compatible with track-based interface.
        """
        # Get obstacles within range
        obstacles = self.grid_map.obstacles
        if len(obstacles) == 0:
            return np.array([]).reshape(0, 2), np.array([]).reshape(0, 2), np.array([]).reshape(0, 2)
        
        # Calculate distances from car
        vectors = obstacles - car_position
        distances = np.linalg.norm(vectors, axis=1)
        
        # Filter by distance (consider obstacle size)
        within_horizon = distances <= (horizon + self.grid_map.obstacle_size)
        
        # Filter by FOV (if not 360 degrees)
        if fov < 2 * np.pi:
            angles = np.arctan2(vectors[:, 1], vectors[:, 0]) - car_heading
            # Normalize angles to [-pi, pi]
            angles = np.arctan2(np.sin(angles), np.cos(angles))
            within_fov = np.abs(angles) <= fov / 2.0
            visible_mask = within_horizon & within_fov
        else:
            visible_mask = within_horizon
        
        visible_obstacles = obstacles[visible_mask]
        
        # Return in compatible format (centerline, inner, outer)
        # For grid maps, we return obstacles as "centerline", empty arrays for bounds
        return visible_obstacles, np.array([]).reshape(0, 2), np.array([]).reshape(0, 2)
    
    def get_all_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get map boundaries (for compatibility)."""
        bounds, _ = self.grid_map.get_bounds()
        return bounds, bounds.copy()
