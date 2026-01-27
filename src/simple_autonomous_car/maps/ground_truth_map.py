"""Ground truth map representation."""

import numpy as np

from simple_autonomous_car.track.track import Track


class GroundTruthMap:
    """Ground truth map of the track in world coordinates."""

    def __init__(self, track: Track):
        """
        Initialize ground truth map.

        Args:
            track: Track object containing centerline and bounds
        """
        self.track = track
        self.centerline = track.centerline.copy()
        self.inner_bound = track.inner_bound.copy()
        self.outer_bound = track.outer_bound.copy()

    def get_visible_segments(
        self, car_position: np.ndarray, car_heading: float, horizon: float, fov: float = 2 * np.pi
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get visible segments of the track within horizon and field of view.

        Args:
            car_position: Car position [x, y] in world frame
            car_heading: Car heading angle in radians
            horizon: Maximum distance to consider
            fov: Field of view angle in radians (default: 2*pi = 360 degrees)

        Returns:
            Tuple of (visible_centerline, visible_inner_bound, visible_outer_bound)
        """
        # Calculate distances and angles from car
        vectors = self.centerline - car_position
        distances = np.linalg.norm(vectors, axis=1)

        # Filter by distance
        within_horizon = distances <= horizon

        # Filter by FOV (if not 360 degrees)
        if fov < 2 * np.pi:
            angles = np.arctan2(vectors[:, 1], vectors[:, 0]) - car_heading
            # Normalize angles to [-pi, pi]
            angles = np.arctan2(np.sin(angles), np.cos(angles))
            within_fov = np.abs(angles) <= fov / 2.0
            visible_mask = within_horizon & within_fov
        else:
            # 360 degree view - only filter by distance
            visible_mask = within_horizon

        visible_centerline = self.centerline[visible_mask]
        visible_inner = self.inner_bound[visible_mask]
        visible_outer = self.outer_bound[visible_mask]

        return visible_centerline, visible_inner, visible_outer

    def get_all_bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Get all inner and outer bounds."""
        return self.inner_bound, self.outer_bound
