"""Frenet frame coordinate system utilities.

Frenet frame: (s, d) where:
- s: distance along the path (longitudinal)
- d: lateral offset from the path (positive = left, negative = right)
"""

from typing import Any

import numpy as np

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.track.track import Track


class FrenetFrame:
    """Frenet frame representation along a path."""

    def __init__(self, track: Track):
        """
        Initialize Frenet frame with a reference path (track centerline).

        Args:
            track: Track object providing the reference path
        """
        self.track = track
        self._compute_cumulative_distances()

    def _compute_cumulative_distances(self) -> None:
        """Precompute cumulative distances along the track."""
        self.cumulative_distances = np.zeros(len(self.track.centerline))
        for i in range(1, len(self.track.centerline)):
            dist = np.linalg.norm(self.track.centerline[i] - self.track.centerline[i - 1])
            self.cumulative_distances[i] = self.cumulative_distances[i - 1] + dist
        self.total_length = self.cumulative_distances[-1]

    def get_closest_point(self, point: np.ndarray) -> tuple[int, float]:
        """
        Find closest point on centerline to given point.

        Args:
            point: Point in global frame [x, y]

        Returns:
            Tuple of (index, distance_along_path)
        """
        distances = np.linalg.norm(self.track.centerline - point, axis=1)
        closest_idx_int = np.argmin(distances)
        closest_idx: int = int(closest_idx_int)
        s = self.cumulative_distances[closest_idx]
        return closest_idx, float(s)

    def get_tangent_normal(self, s: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Get tangent and normal vectors at distance s along path.

        Args:
            s: Distance along path

        Returns:
            Tuple of (tangent_vector, normal_vector)
        """
        s = s % self.total_length  # Wrap around

        # Find segment
        idx = np.searchsorted(self.cumulative_distances, s)
        if idx == 0:  # type: ignore[assignment]
            idx = 1  # type: ignore[assignment]
        if idx >= len(self.track.centerline):  # type: ignore[assignment]
            idx = len(self.track.centerline) - 1  # type: ignore[assignment]

        # Get direction
        direction = self.track.centerline[idx] - self.track.centerline[idx - 1]
        direction_norm = np.linalg.norm(direction)
        if direction_norm > 1e-6:
            tangent = direction / direction_norm
        else:
            tangent = np.array([1.0, 0.0])

        # Normal (perpendicular, pointing left)
        normal = np.array([-tangent[1], tangent[0]])

        return tangent, normal

    def global_to_frenet(self, point: np.ndarray) -> tuple[float, float]:
        """
        Convert point from global frame to Frenet frame.

        Args:
            point: Point in global frame [x, y]

        Returns:
            Tuple of (s, d) where s is distance along path, d is lateral offset
        """
        closest_idx, s = self.get_closest_point(point)

        # Get point on centerline
        centerline_point = self.track.centerline[closest_idx]

        # Get tangent and normal at this point
        tangent, normal = self.get_tangent_normal(s)

        # Compute lateral offset
        vector_to_point = point - centerline_point
        d = np.dot(vector_to_point, normal)

        return s, d

    def frenet_to_global(self, s: float, d: float) -> np.ndarray:
        """
        Convert point from Frenet frame to global frame.

        Args:
            s: Distance along path
            d: Lateral offset (positive = left)

        Returns:
            Point in global frame [x, y]
        """
        s = s % self.total_length  # Wrap around

        # Find segment
        idx_int = np.searchsorted(self.cumulative_distances, s)
        idx: int = int(idx_int.item() if hasattr(idx_int, "item") else idx_int)
        if idx == 0:
            idx = 1
        if idx >= len(self.track.centerline):
            idx = len(self.track.centerline) - 1

        # Interpolate along centerline
        segment_dist = s - self.cumulative_distances[idx - 1]
        segment_length = self.cumulative_distances[idx] - self.cumulative_distances[idx - 1]
        if segment_length > 1e-6:
            t = segment_dist / segment_length
        else:
            t = 0.0

        centerline_point = self.track.centerline[idx - 1] + t * (
            self.track.centerline[idx] - self.track.centerline[idx - 1]
        )

        # Get tangent and normal
        tangent, normal = self.get_tangent_normal(s)

        # Offset by d in normal direction
        global_point = centerline_point + d * normal

        return np.asarray(global_point, dtype=np.float64)


# Convenience functions
def global_to_frenet(point: np.ndarray, frenet_frame: FrenetFrame) -> tuple[float, float]:
    """Convert point from global to Frenet frame."""
    return frenet_frame.global_to_frenet(point)


def frenet_to_global(s: float, d: float, frenet_frame: FrenetFrame) -> np.ndarray:
    """Convert point from Frenet to global frame."""
    return frenet_frame.frenet_to_global(s, d)


def ego_to_frenet(
    point_ego: np.ndarray, car_state: CarState, frenet_frame: FrenetFrame
) -> tuple[float, float]:
    """
    Convert point from ego frame to Frenet frame.

    Args:
        point_ego: Point in ego frame [x, y] (x=forward, y=left)
        car_state: Current car state
        frenet_frame: Frenet frame instance

    Returns:
        Tuple of (s, d) in Frenet frame
    """
    # Convert ego to global
    point_global = car_state.transform_to_world_frame(point_ego)
    # Convert global to Frenet
    return frenet_frame.global_to_frenet(point_global)


# Add method to FrenetFrame class for convenience
def _ego_to_frenet_method(
    self: Any, point_ego: np.ndarray, car_state: CarState
) -> tuple[float, float]:
    """Convert point from ego frame to Frenet frame (instance method)."""
    return ego_to_frenet(point_ego, car_state, self)


FrenetFrame.ego_to_frenet = _ego_to_frenet_method  # type: ignore[attr-defined]


def frenet_to_ego(s: float, d: float, car_state: CarState, frenet_frame: FrenetFrame) -> np.ndarray:
    """
    Convert point from Frenet frame to ego frame.

    Args:
        s: Distance along path
        d: Lateral offset
        car_state: Current car state
        frenet_frame: Frenet frame instance

    Returns:
        Point in ego frame [x, y]
    """
    # Convert Frenet to global
    point_global = frenet_frame.frenet_to_global(s, d)
    # Convert global to ego
    return car_state.transform_to_car_frame(point_global)


def sensor_to_ego(
    point_sensor: np.ndarray, sensor_pose_ego: np.ndarray | None = None
) -> np.ndarray:
    """
    Convert point from sensor frame to ego frame.

    Args:
        point_sensor: Point in sensor frame [x, y]
        sensor_pose_ego: Sensor pose in ego frame [x, y, heading].
                        If None, assumes sensor is at ego origin.

    Returns:
        Point in ego frame [x, y]
    """
    if sensor_pose_ego is None:
        # Sensor at ego origin, no transformation needed
        return point_sensor

    sensor_x, sensor_y, sensor_heading = sensor_pose_ego
    cos_h = np.cos(sensor_heading)
    sin_h = np.sin(sensor_heading)

    # Rotation matrix from sensor to ego
    rotation = np.array([[cos_h, -sin_h], [sin_h, cos_h]])

    # Transform
    point_ego = rotation @ point_sensor + np.array([sensor_x, sensor_y])
    return np.asarray(point_ego, dtype=np.float64)


def ego_to_sensor(point_ego: np.ndarray, sensor_pose_ego: np.ndarray | None = None) -> np.ndarray:
    """
    Convert point from ego frame to sensor frame.

    Args:
        point_ego: Point in ego frame [x, y]
        sensor_pose_ego: Sensor pose in ego frame [x, y, heading].
                        If None, assumes sensor is at ego origin.

    Returns:
        Point in sensor frame [x, y]
    """
    if sensor_pose_ego is None:
        # Sensor at ego origin, no transformation needed
        return point_ego

    sensor_x, sensor_y, sensor_heading = sensor_pose_ego
    cos_h = np.cos(sensor_heading)
    sin_h = np.sin(sensor_heading)

    # Rotation matrix from ego to sensor (inverse)
    rotation = np.array([[cos_h, sin_h], [-sin_h, cos_h]])

    # Transform
    point_sensor = rotation @ (point_ego - np.array([sensor_x, sensor_y]))
    return np.asarray(point_sensor, dtype=np.float64)
