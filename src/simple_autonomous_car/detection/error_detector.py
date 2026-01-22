"""Localization error detection between perceived and ground truth maps."""

import numpy as np
from typing import Tuple, Dict

from simple_autonomous_car.maps.ground_truth_map import GroundTruthMap
from simple_autonomous_car.maps.perceived_map import PerceivedMap
from simple_autonomous_car.car.car import CarState


class LocalizationErrorDetector:
    """Detects errors between perceived and ground truth localization."""

    def __init__(
        self,
        ground_truth_map: GroundTruthMap,
        perceived_map: PerceivedMap,
        error_threshold: float = 1.0,
    ):
        """
        Initialize error detector.

        Args:
            ground_truth_map: Ground truth map
            perceived_map: Perceived map
            error_threshold: Distance threshold for error detection (meters)
        """
        self.ground_truth_map = ground_truth_map
        self.perceived_map = perceived_map
        self.error_threshold = error_threshold

    def compute_errors(
        self, car_state: CarState, horizon: float, fov: float = 2 * np.pi
    ) -> Dict[str, np.ndarray]:
        """
        Compute errors between perceived and ground truth segments.

        Args:
            car_state: Current car state
            horizon: Maximum distance to consider
            fov: Field of view angle in radians

        Returns:
            Dictionary with error metrics:
            - centerline_errors: Distance errors for centerline points
            - inner_bound_errors: Distance errors for inner boundary points
            - outer_bound_errors: Distance errors for outer boundary points
            - max_error: Maximum error across all segments
            - mean_error: Mean error across all segments
        """
        # Update perceived state
        self.perceived_map.update_perceived_state(car_state)

        # Get ground truth segments in car frame
        gt_centerline, gt_inner, gt_outer = self.ground_truth_map.get_visible_segments(
            car_state.position(), car_state.heading, horizon, fov
        )

        if len(gt_centerline) == 0:
            return {
                "centerline_errors": np.array([]),
                "inner_bound_errors": np.array([]),
                "outer_bound_errors": np.array([]),
                "max_error": 0.0,
                "mean_error": 0.0,
            }

        # Transform GT to car frame
        gt_centerline_car = np.array(
            [car_state.transform_to_car_frame(point) for point in gt_centerline]
        )
        gt_inner_car = np.array([car_state.transform_to_car_frame(point) for point in gt_inner])
        gt_outer_car = np.array([car_state.transform_to_car_frame(point) for point in gt_outer])

        # Get perceived segments in car frame
        perceived_centerline_car, perceived_inner_car, perceived_outer_car = (
            self.perceived_map.get_perceived_segments(horizon, fov)
        )

        # Compute errors (using nearest neighbor matching)
        centerline_errors = self._compute_point_errors(
            gt_centerline_car, perceived_centerline_car
        )
        inner_bound_errors = self._compute_point_errors(gt_inner_car, perceived_inner_car)
        outer_bound_errors = self._compute_point_errors(gt_outer_car, perceived_outer_car)

        # Aggregate errors
        all_errors = np.concatenate([centerline_errors, inner_bound_errors, outer_bound_errors])
        max_error = np.max(all_errors) if len(all_errors) > 0 else 0.0
        mean_error = np.mean(all_errors) if len(all_errors) > 0 else 0.0

        return {
            "centerline_errors": centerline_errors,
            "inner_bound_errors": inner_bound_errors,
            "outer_bound_errors": outer_bound_errors,
            "max_error": max_error,
            "mean_error": mean_error,
        }

    def _compute_point_errors(
        self, ground_truth_points: np.ndarray, perceived_points: np.ndarray
    ) -> np.ndarray:
        """
        Compute distance errors between ground truth and perceived points.

        Args:
            ground_truth_points: Ground truth points (N, 2)
            perceived_points: Perceived points (M, 2)

        Returns:
            Array of errors for each ground truth point
        """
        if len(perceived_points) == 0:
            return np.zeros(len(ground_truth_points))

        errors = []
        for gt_point in ground_truth_points:
            # Find nearest perceived point
            distances = np.linalg.norm(perceived_points - gt_point, axis=1)
            min_distance = np.min(distances)
            errors.append(min_distance)

        return np.array(errors)

    def detect_errors(
        self, car_state: CarState, horizon: float, fov: float = 2 * np.pi
    ) -> Dict[str, bool]:
        """
        Detect if errors exceed threshold.

        Args:
            car_state: Current car state
            horizon: Maximum distance to consider
            fov: Field of view angle in radians

        Returns:
            Dictionary with detection flags:
            - has_error: True if any error exceeds threshold
            - centerline_has_error: True if centerline errors exceed threshold
            - inner_bound_has_error: True if inner bound errors exceed threshold
            - outer_bound_has_error: True if outer bound errors exceed threshold
        """
        errors = self.compute_errors(car_state, horizon, fov)

        centerline_has_error = (
            np.any(errors["centerline_errors"] > self.error_threshold)
            if len(errors["centerline_errors"]) > 0
            else False
        )
        inner_bound_has_error = (
            np.any(errors["inner_bound_errors"] > self.error_threshold)
            if len(errors["inner_bound_errors"]) > 0
            else False
        )
        outer_bound_has_error = (
            np.any(errors["outer_bound_errors"] > self.error_threshold)
            if len(errors["outer_bound_errors"]) > 0
            else False
        )

        has_error = centerline_has_error or inner_bound_has_error or outer_bound_has_error

        return {
            "has_error": has_error,
            "centerline_has_error": centerline_has_error,
            "inner_bound_has_error": inner_bound_has_error,
            "outer_bound_has_error": outer_bound_has_error,
        }
