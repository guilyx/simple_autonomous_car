"""PID controller for path following."""

import numpy as np
from typing import Dict, Optional

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.perception.perception import PerceptionPoints
from simple_autonomous_car.control.base_controller import BaseController
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simple_autonomous_car.costmap.base_costmap import BaseCostmap


class PIDController(BaseController):
    """
    PID (Proportional-Integral-Derivative) controller.

    A simple PID controller for following a path or maintaining a target state.

    Parameters
    ----------
    kp : float, default=1.0
        Proportional gain.
    ki : float, default=0.0
        Integral gain.
    kd : float, default=0.1
        Derivative gain.
    target_velocity : float, default=10.0
        Target velocity in m/s.
    name : str, default="pid"
        Controller name.
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.1,
        target_velocity: float = 10.0,
        name: str = "pid",
    ):
        super().__init__(name=name)
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target_velocity = target_velocity
        self.integral_error = 0.0
        self.last_error = 0.0

    def compute_control(
        self,
        car_state: CarState,
        perception_data: Optional[Dict[str, PerceptionPoints]] = None,
        costmap: Optional["BaseCostmap"] = None,
        plan: Optional[np.ndarray] = None,
        dt: float = 0.1,
    ) -> Dict[str, float]:
        """
        Compute control using PID algorithm.

        Parameters
        ----------
        car_state : CarState
            Current car state.
        perception_data : dict, optional
            Not used by basic PID.
        plan : np.ndarray, optional
            Planned path. If provided, computes error to nearest point.
        dt : float, default=0.1
            Time step in seconds.

        Returns
        -------
        Dict[str, float]
            Control commands with "acceleration" and "steering_rate".
        """
        if not self.enabled:
            return {"acceleration": 0.0, "steering_rate": 0.0}

        # Simple heading control if plan provided
        steering_rate = 0.0
        if plan is not None and len(plan) > 0:
            car_pos = car_state.position()
            distances = np.linalg.norm(plan - car_pos, axis=1)
            closest_idx = np.argmin(distances)
            target_point = plan[closest_idx]

            target_vector = target_point - car_pos
            target_angle = np.arctan2(target_vector[1], target_vector[0])
            angle_error = target_angle - car_state.heading
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

            # PID control
            self.integral_error += angle_error * dt
            derivative_error = (angle_error - self.last_error) / dt
            self.last_error = angle_error

            steering_rate = (
                self.kp * angle_error
                + self.ki * self.integral_error
                + self.kd * derivative_error
            )
            steering_rate = np.clip(steering_rate, -1.0, 1.0)

        # Velocity control
        velocity_error = self.target_velocity - car_state.velocity
        acceleration = 0.5 * velocity_error
        acceleration = np.clip(acceleration, -2.0, 2.0)

        return {"acceleration": acceleration, "steering_rate": steering_rate}

    def reset(self) -> None:
        """Reset PID controller state."""
        self.integral_error = 0.0
        self.last_error = 0.0
