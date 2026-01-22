"""LiDAR sensor implementation."""

import numpy as np
from typing import Dict, Optional

from simple_autonomous_car.maps.ground_truth_map import GroundTruthMap
from simple_autonomous_car.maps.perceived_map import PerceivedMap
from simple_autonomous_car.perception.perception import PerceptionPoints
from simple_autonomous_car.sensors.base_sensor import BaseSensor
from simple_autonomous_car.frames.frenet import sensor_to_ego

# Import CarState here to avoid circular import
from simple_autonomous_car.car.car import CarState


class LiDARSensor(BaseSensor):
    """
    LiDAR sensor that detects track boundaries.

    This sensor simulates a 360° LiDAR that detects track boundaries
    and returns a point cloud. It combines ground truth map data with
    perception errors and sensor noise.

    Parameters
    ----------
    ground_truth_map : GroundTruthMap
        Ground truth map for reference.
    perceived_map : PerceivedMap
        Perceived map that adds localization errors.
    angular_resolution : float, default=0.1
        Angular resolution in radians (spacing between rays).
    point_noise_std : float, default=0.1
        Standard deviation of point measurement noise in meters.
    name : str, default="lidar"
        Sensor name/identifier.
    pose_ego : np.ndarray, optional
        Sensor pose in ego frame [x, y, heading].
    max_range : float, default=50.0
        Maximum sensor range in meters.
    """

    def __init__(
        self,
        ground_truth_map: GroundTruthMap,
        perceived_map: PerceivedMap,
        angular_resolution: float = 0.1,
        point_noise_std: float = 0.1,
        name: str = "lidar",
        pose_ego: Optional[np.ndarray] = None,
        max_range: float = 50.0,
    ):
        super().__init__(name=name, pose_ego=pose_ego, max_range=max_range)
        self.ground_truth_map = ground_truth_map
        self.perceived_map = perceived_map
        self.angular_resolution = angular_resolution
        self.point_noise_std = point_noise_std

    def sense(
        self, car_state: CarState, environment_data: Optional[Dict] = None
    ) -> PerceptionPoints:
        """
        Sense track boundaries and return point cloud.

        Parameters
        ----------
        car_state : CarState
            Current car state.
        environment_data : dict, optional
            Additional environment data (not used for LiDAR).

        Returns
        -------
        PerceptionPoints
            Point cloud in ego frame. Returns empty if sensor is disabled.
        """
        if not self.enabled:
            return PerceptionPoints(np.array([]).reshape(0, 2), frame="ego")

        # Update perceived state
        self.perceived_map.update_perceived_state(car_state)

        # Get perceived segments (360 degree view)
        perceived_centerline, perceived_inner, perceived_outer = (
            self.perceived_map.get_perceived_segments(self.max_range, fov=2 * np.pi)
        )

        # Combine inner and outer bounds into perception points
        if len(perceived_inner) > 0 and len(perceived_outer) > 0:
            perception_points_car = np.vstack([perceived_inner, perceived_outer])
        elif len(perceived_inner) > 0:
            perception_points_car = perceived_inner
        elif len(perceived_outer) > 0:
            perception_points_car = perceived_outer
        else:
            perception_points_car = np.array([]).reshape(0, 2)

        # Add sensor noise
        if len(perception_points_car) > 0:
            point_noise = np.random.normal(
                0, self.point_noise_std, perception_points_car.shape
            )
            perception_points_car = perception_points_car + point_noise

        # Transform from sensor frame to ego frame if sensor is not at origin
        if not np.allclose(self.pose_ego, [0.0, 0.0, 0.0]):
            perception_points_ego = np.array(
                [
                    sensor_to_ego(point, self.pose_ego)
                    for point in perception_points_car
                ]
            )
        else:
            perception_points_ego = perception_points_car

        return PerceptionPoints(perception_points_ego, frame="ego")

    def get_map_lines_car_frame(
        self, car_state: CarState
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Get ground truth map lines in car frame.

        Parameters
        ----------
        car_state : CarState
            Current car state.

        Returns
        -------
        tuple[np.ndarray, np.ndarray]
            Tuple of (map_lines_car_frame, map_lines_world_frame).
        """
        # Get all visible segments (360 degree view)
        gt_centerline, gt_inner, gt_outer = self.ground_truth_map.get_visible_segments(
            car_state.position(), car_state.heading, self.max_range, fov=2 * np.pi
        )

        # Combine inner and outer bounds
        if len(gt_inner) > 0 and len(gt_outer) > 0:
            map_lines_world = np.vstack([gt_inner, gt_outer])
        elif len(gt_inner) > 0:
            map_lines_world = gt_inner
        elif len(gt_outer) > 0:
            map_lines_world = gt_outer
        else:
            map_lines_world = np.array([]).reshape(0, 2)

        # Transform to car frame
        if len(map_lines_world) > 0:
            map_lines_car = np.array(
                [car_state.transform_to_car_frame(point) for point in map_lines_world]
            )
        else:
            map_lines_car = np.array([]).reshape(0, 2)

        return map_lines_car, map_lines_world
