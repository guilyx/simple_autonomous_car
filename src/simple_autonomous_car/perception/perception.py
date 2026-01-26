"""Perception data structures - points in local (ego) frame."""

from typing import TYPE_CHECKING, Optional

import numpy as np

if TYPE_CHECKING:
    from simple_autonomous_car.car.car import CarState


class PerceptionPoints:
    """Represents perception data as a vector of points in local (ego) frame."""

    def __init__(self, points: np.ndarray, frame: str = "ego"):
        """
        Initialize perception points.

        Args:
            points: Array of shape (N, 2) with [x, y] coordinates
            frame: Frame of the points ("ego", "sensor", "global")
        """
        self.points = np.asarray(points, dtype=np.float64)
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError("points must be shape (N, 2)")

        if frame not in ["ego", "sensor", "global"]:
            raise ValueError(f"frame must be one of ['ego', 'sensor', 'global'], got {frame}")

        self.frame = frame

    def to_ego_frame(self, car_state: Optional["CarState"] = None) -> "PerceptionPoints":
        """
        Convert points to ego frame.

        Args:
            car_state: Car state (required if converting from global frame)

        Returns:
            New PerceptionPoints instance in ego frame
        """
        if self.frame == "ego":
            return PerceptionPoints(self.points.copy(), frame="ego")

        if self.frame == "sensor":
            # Assume sensor at ego origin for now
            # Could be extended with sensor_pose_ego parameter
            return PerceptionPoints(self.points.copy(), frame="ego")

        if self.frame == "global":
            if car_state is None:
                raise ValueError("car_state required for global to ego conversion")
            ego_points = np.array(
                [car_state.transform_to_car_frame(point) for point in self.points]
            )
            return PerceptionPoints(ego_points, frame="ego")

        raise ValueError(f"Unknown frame: {self.frame}")

    def to_global_frame(self, car_state: "CarState") -> "PerceptionPoints":
        """
        Convert points to global frame.

        Args:
            car_state: Car state

        Returns:
            New PerceptionPoints instance in global frame
        """
        if self.frame == "global":
            return PerceptionPoints(self.points.copy(), frame="global")

        if self.frame == "ego":
            # Import here to avoid circular import

            global_points = np.array(
                [car_state.transform_to_world_frame(point) for point in self.points]
            )
            return PerceptionPoints(global_points, frame="global")

        if self.frame == "sensor":
            # Convert sensor -> ego -> global
            ego_points = self.to_ego_frame()
            return ego_points.to_global_frame(car_state)

        raise ValueError(f"Unknown frame: {self.frame}")

    def filter_by_distance(self, max_distance: float) -> "PerceptionPoints":
        """
        Filter points by distance from origin.

        Args:
            max_distance: Maximum distance to keep

        Returns:
            New PerceptionPoints instance with filtered points
        """
        distances = np.linalg.norm(self.points, axis=1)
        mask = distances <= max_distance
        return PerceptionPoints(self.points[mask], frame=self.frame)

    def __len__(self) -> int:
        """Return number of points."""
        return len(self.points)

    def __getitem__(self, index: int | slice) -> np.ndarray:
        """Get point(s) by index."""
        return self.points[index]
