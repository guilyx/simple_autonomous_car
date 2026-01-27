"""Base sensor class for modular sensor system."""

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from simple_autonomous_car.car.car import CarState

from simple_autonomous_car.perception.perception import PerceptionPoints


class BaseSensor(ABC):
    """
    Base class for all sensors.

    This abstract class defines the interface that all sensors must implement.
    Sensors can be added to a car and will provide perception data in ego frame.

    Attributes
    ----------
    name : str
        Name/identifier of the sensor.
    pose_ego : np.ndarray
        Sensor pose in ego frame [x, y, heading]. Default is [0, 0, 0] (at car origin).
    max_range : float
        Maximum sensor range in meters.
    enabled : bool
        Whether the sensor is enabled.
    """

    def __init__(
        self,
        name: str = "sensor",
        pose_ego: np.ndarray | None = None,
        max_range: float = 50.0,
        enabled: bool = True,
    ):
        """
        Initialize base sensor.

        Parameters
        ----------
        name : str, default="sensor"
            Name/identifier for this sensor instance.
        pose_ego : np.ndarray, optional
            Sensor pose in ego frame [x, y, heading] in meters and radians.
            If None, sensor is at car origin with zero heading.
        max_range : float, default=50.0
            Maximum sensor range in meters.
        enabled : bool, default=True
            Whether the sensor is enabled. Disabled sensors return empty data.
        """
        self.name = name
        self.pose_ego = pose_ego if pose_ego is not None else np.array([0.0, 0.0, 0.0])
        self.max_range = max_range
        self.enabled = enabled

    @abstractmethod
    def sense(self, car_state: "CarState", environment_data: dict) -> PerceptionPoints:
        """
        Sense the environment and return perception data.

        This is the main method that all sensors must implement.
        It takes the car state and environment data, and returns
        perception points in ego frame.

        Parameters
        ----------
        car_state : CarState
            Current state of the car.
        environment_data : dict
            Dictionary containing environment data (e.g., ground truth map,
            obstacles, etc.). Structure depends on sensor type.

        Returns
        -------
        PerceptionPoints
            Perception data in ego frame. Returns empty points if sensor is disabled.
        """
        pass

    def is_enabled(self) -> bool:
        """Check if sensor is enabled."""
        return self.enabled

    def enable(self) -> None:
        """Enable the sensor."""
        self.enabled = True

    def disable(self) -> None:
        """Disable the sensor."""
        self.enabled = False

    def set_pose(self, pose_ego: np.ndarray) -> None:
        """
        Set sensor pose in ego frame.

        Parameters
        ----------
        pose_ego : np.ndarray
            Sensor pose [x, y, heading] in ego frame.
        """
        self.pose_ego = np.asarray(pose_ego, dtype=np.float64)
