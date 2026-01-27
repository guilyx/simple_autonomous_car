"""Car model with state and reference frame handling."""

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np

if TYPE_CHECKING:
    from simple_autonomous_car.perception.perception import PerceptionPoints
    from simple_autonomous_car.sensors.base_sensor import BaseSensor


@dataclass
class CarState:
    """State of the car."""

    x: float  # x position in world frame
    y: float  # y position in world frame
    heading: float  # heading angle in radians (0 = east, counterclockwise)
    velocity: float = 0.0  # forward velocity
    steering_angle: float = 0.0  # steering angle in radians

    def position(self) -> np.ndarray:
        """Get position as numpy array."""
        return np.array([self.x, self.y])

    def rotation_matrix(self) -> np.ndarray:
        """Get rotation matrix from world to car frame."""
        cos_h = np.cos(self.heading)
        sin_h = np.sin(self.heading)
        return np.array([[cos_h, sin_h], [-sin_h, cos_h]])

    def transform_to_car_frame(self, point: np.ndarray) -> np.ndarray:
        """Transform point from world frame to car frame."""
        rotation = self.rotation_matrix()
        translated = point - self.position()
        result = rotation @ translated
        return np.asarray(result, dtype=np.float64)

    def transform_to_world_frame(self, point: np.ndarray) -> np.ndarray:
        """Transform point from car frame to world frame."""
        rotation = self.rotation_matrix().T  # Inverse rotation
        rotated = rotation @ point
        result = rotated + self.position()
        return np.asarray(result, dtype=np.float64)


class Car:
    """
    Autonomous car model with sensor support.

    This class represents an autonomous vehicle that can:
    - Move using bicycle model dynamics
    - Have multiple sensors attached
    - Be controlled by controllers
    - Follow plans from planners

    Attributes
    ----------
    state : CarState
        Current state of the car.
    wheelbase : float
        Distance between front and rear axles (meters).
    max_velocity : float
        Maximum velocity (m/s).
    max_steering_angle : float
        Maximum steering angle (radians).
    sensors : List[BaseSensor]
        List of sensors attached to the car.
    """

    def __init__(
        self,
        initial_state: CarState | None = None,
        wheelbase: float = 2.5,
        max_velocity: float = 30.0,
        max_steering_angle: float = np.pi / 6,
    ):
        """
        Initialize car.

        Parameters
        ----------
        initial_state : CarState, optional
            Initial state of the car. If None, creates car at origin.
        wheelbase : float, default=2.5
            Distance between front and rear axles in meters.
        max_velocity : float, default=30.0
            Maximum velocity in m/s.
        max_steering_angle : float, default=π/6
            Maximum steering angle in radians (30 degrees).
        """
        self.state = (
            initial_state if initial_state is not None else CarState(x=0.0, y=0.0, heading=0.0)
        )
        self.wheelbase = wheelbase
        self.max_velocity = max_velocity
        self.max_steering_angle = max_steering_angle
        self.sensors: list[BaseSensor] = []

    def add_sensor(self, sensor: "BaseSensor") -> None:
        """
        Add a sensor to the car.

        Parameters
        ----------
        sensor : BaseSensor
            Sensor instance to add. Can be LiDAR, Camera, Radar, etc.
        """
        self.sensors.append(sensor)

    def remove_sensor(self, sensor_name: str) -> bool:
        """
        Remove a sensor by name.

        Parameters
        ----------
        sensor_name : str
            Name of the sensor to remove.

        Returns
        -------
        bool
            True if sensor was found and removed, False otherwise.
        """
        for i, sensor in enumerate(self.sensors):
            if sensor.name == sensor_name:
                self.sensors.pop(i)
                return True
        return False

    def get_sensor(self, sensor_name: str) -> Optional["BaseSensor"]:
        """
        Get a sensor by name.

        Parameters
        ----------
        sensor_name : str
            Name of the sensor.

        Returns
        -------
        BaseSensor, optional
            Sensor instance if found, None otherwise.
        """
        for sensor in self.sensors:
            if sensor.name == sensor_name:
                return sensor
        return None

    def sense_all(self, environment_data: dict | None = None) -> dict[str, "PerceptionPoints"]:
        """
        Get perception data from all enabled sensors.

        Parameters
        ----------
        environment_data : dict, optional
            Environment data to pass to sensors.

        Returns
        -------
        Dict[str, PerceptionPoints]
            Dictionary mapping sensor names to their perception data.
        """
        # Import here to avoid circular import

        perception_data = {}
        for sensor in self.sensors:
            if sensor.is_enabled():
                env_data = environment_data if environment_data is not None else {}
                perception_data[sensor.name] = sensor.sense(self.state, env_data)
        return perception_data

    def update(self, dt: float, acceleration: float = 0.0, steering_rate: float = 0.0) -> None:
        """
        Update car state using bicycle model.

        Args:
            dt: Time step
            acceleration: Acceleration (can be negative for braking)
            steering_rate: Rate of change of steering angle
        """
        # Update steering angle
        self.state.steering_angle += steering_rate * dt
        self.state.steering_angle = np.clip(
            self.state.steering_angle, -self.max_steering_angle, self.max_steering_angle
        )

        # Update velocity
        self.state.velocity += acceleration * dt
        self.state.velocity = np.clip(self.state.velocity, 0.0, self.max_velocity)

        # Bicycle model dynamics
        if abs(self.state.steering_angle) > 1e-6:
            turning_radius = self.wheelbase / np.tan(self.state.steering_angle)
            angular_velocity = self.state.velocity / turning_radius
        else:
            angular_velocity = 0.0

        # Update position and heading
        self.state.heading += angular_velocity * dt
        self.state.x += self.state.velocity * np.cos(self.state.heading) * dt
        self.state.y += self.state.velocity * np.sin(self.state.heading) * dt

    def get_corners(self, length: float = 4.0, width: float = 1.8) -> np.ndarray:
        """
        Get car corner points in world frame.

        Args:
            length: Car length
            width: Car width

        Returns:
            Array of shape (4, 2) with corner coordinates
        """
        half_length = length / 2.0
        half_width = width / 2.0

        # Corners in car frame (rear-left, rear-right, front-right, front-left)
        corners_car = np.array(
            [
                [-half_length, -half_width],
                [-half_length, half_width],
                [half_length, half_width],
                [half_length, -half_width],
            ]
        )

        # Transform to world frame
        corners_world = np.array(
            [self.state.transform_to_world_frame(corner) for corner in corners_car]
        )

        return corners_world
