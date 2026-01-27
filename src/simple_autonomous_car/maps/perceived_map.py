"""Perceived map with noise and reference frame transformations."""

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from simple_autonomous_car.car.car import CarState

from simple_autonomous_car.maps.ground_truth_map import GroundTruthMap


class PerceivedMap:
    """Perceived map with potential errors and noise."""

    def __init__(
        self,
        ground_truth: GroundTruthMap,
        position_noise_std: float = 0.1,
        orientation_noise_std: float = 0.05,
        measurement_noise_std: float = 0.2,
    ):
        """
        Initialize perceived map.

        Args:
            ground_truth: Ground truth map
            position_noise_std: Standard deviation of position estimation error
            orientation_noise_std: Standard deviation of orientation estimation error
            measurement_noise_std: Standard deviation of measurement noise
        """
        self.ground_truth = ground_truth
        self.position_noise_std = position_noise_std
        self.orientation_noise_std = orientation_noise_std
        self.measurement_noise_std = measurement_noise_std

        # Perceived car state (with localization errors)
        self.perceived_car_state: CarState | None = None

    def update_perceived_state(self, true_car_state: "CarState") -> None:
        """
        Update perceived car state with localization errors.

        Args:
            true_car_state: True car state
        """
        # Add position noise
        position_noise = np.random.normal(0, self.position_noise_std, 2)
        perceived_x = true_car_state.x + position_noise[0]
        perceived_y = true_car_state.y + position_noise[1]

        # Add orientation noise
        orientation_noise = np.random.normal(0, self.orientation_noise_std)
        perceived_heading = true_car_state.heading + orientation_noise

        # Import here to avoid circular import
        from simple_autonomous_car.car.car import CarState

        self.perceived_car_state = CarState(
            x=perceived_x,
            y=perceived_y,
            heading=perceived_heading,
            velocity=true_car_state.velocity,
            steering_angle=true_car_state.steering_angle,
        )

    def get_perceived_segments(
        self, horizon: float, fov: float = 2 * np.pi
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get perceived track segments in perceived car frame.

        Args:
            horizon: Maximum distance to consider
            fov: Field of view angle in radians

        Returns:
            Tuple of (perceived_centerline_car_frame, perceived_inner_car_frame, perceived_outer_car_frame)
        """
        if self.perceived_car_state is None:
            raise ValueError("Perceived car state not set. Call update_perceived_state first.")

        # Get visible segments from ground truth in world frame
        visible_centerline, visible_inner, visible_outer = self.ground_truth.get_visible_segments(
            self.perceived_car_state.position(),
            self.perceived_car_state.heading,
            horizon,
            fov,
        )

        if len(visible_centerline) == 0:
            return (
                np.array([]).reshape(0, 2),
                np.array([]).reshape(0, 2),
                np.array([]).reshape(0, 2),
            )

        # Add measurement noise
        centerline_noise = np.random.normal(0, self.measurement_noise_std, visible_centerline.shape)
        inner_noise = np.random.normal(0, self.measurement_noise_std, visible_inner.shape)
        outer_noise = np.random.normal(0, self.measurement_noise_std, visible_outer.shape)

        perceived_centerline_world = visible_centerline + centerline_noise
        perceived_inner_world = visible_inner + inner_noise
        perceived_outer_world = visible_outer + outer_noise

        # Transform to perceived car frame
        perceived_centerline_car = np.array(
            [
                self.perceived_car_state.transform_to_car_frame(point)
                for point in perceived_centerline_world
            ]
        )
        perceived_inner_car = np.array(
            [
                self.perceived_car_state.transform_to_car_frame(point)
                for point in perceived_inner_world
            ]
        )
        perceived_outer_car = np.array(
            [
                self.perceived_car_state.transform_to_car_frame(point)
                for point in perceived_outer_world
            ]
        )

        return perceived_centerline_car, perceived_inner_car, perceived_outer_car

    def get_perceived_segments_world_frame(
        self, horizon: float, fov: float = 2 * np.pi
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get perceived track segments in world frame.

        Args:
            horizon: Maximum distance to consider
            fov: Field of view angle in radians

        Returns:
            Tuple of (perceived_centerline_world, perceived_inner_world, perceived_outer_world)
        """
        if self.perceived_car_state is None:
            raise ValueError("Perceived car state not set. Call update_perceived_state first.")

        # Get visible segments from ground truth
        visible_centerline, visible_inner, visible_outer = self.ground_truth.get_visible_segments(
            self.perceived_car_state.position(),
            self.perceived_car_state.heading,
            horizon,
            fov,
        )

        if len(visible_centerline) == 0:
            return (
                np.array([]).reshape(0, 2),
                np.array([]).reshape(0, 2),
                np.array([]).reshape(0, 2),
            )

        # Add measurement noise
        centerline_noise = np.random.normal(0, self.measurement_noise_std, visible_centerline.shape)
        inner_noise = np.random.normal(0, self.measurement_noise_std, visible_inner.shape)
        outer_noise = np.random.normal(0, self.measurement_noise_std, visible_outer.shape)

        perceived_centerline_world = visible_centerline + centerline_noise
        perceived_inner_world = visible_inner + inner_noise
        perceived_outer_world = visible_outer + outer_noise

        return perceived_centerline_world, perceived_inner_world, perceived_outer_world
