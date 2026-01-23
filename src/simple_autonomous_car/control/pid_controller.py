"""PID controller for path following."""

import numpy as np
from typing import Dict, Optional

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.perception.perception import PerceptionPoints
from simple_autonomous_car.control.base_controller import BaseController
from simple_autonomous_car.constants import DEFAULT_GOAL_TOLERANCE
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
        goal: Optional[np.ndarray] = None,
        goal_tolerance: Optional[float] = None,
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

        # Velocity control (adapt based on goal distance)
        target_velocity = self.target_velocity
        
        # Reduce velocity if approaching goal (smooth deceleration)
        if goal is not None:
            distance_to_goal = np.linalg.norm(car_state.position() - goal)
            tolerance = goal_tolerance if goal_tolerance is not None else DEFAULT_GOAL_TOLERANCE
            
            # If within tolerance, stop completely
            if distance_to_goal <= tolerance:
                target_velocity = 0.0
            else:
                # Start slowing down when within 10 meters of goal
                # Use a smaller slow_down_distance to ensure smooth deceleration even when close
                slow_down_distance = max(10.0, tolerance * 5.0)  # At least 5x tolerance, or 10m
                if distance_to_goal < slow_down_distance:
                    # Smooth velocity reduction: linear from full speed to zero
                    # At distance=tolerance, velocity=0; at distance=slow_down_distance, velocity=target_velocity
                    # Map distance from [tolerance, slow_down_distance] to velocity [0, target_velocity]
                    if slow_down_distance > tolerance:
                        velocity_factor = (distance_to_goal - tolerance) / (slow_down_distance - tolerance)
                        velocity_factor = max(0.0, min(1.0, velocity_factor))  # Clamp to [0, 1]
                        target_velocity = self.target_velocity * velocity_factor
                    else:
                        target_velocity = 0.0
                # Ensure we always slow down when very close (even if above tolerance)
                if distance_to_goal < tolerance * 2.0:
                    # Extra safety: cap velocity when very close
                    max_velocity_near_goal = self.target_velocity * (distance_to_goal / (tolerance * 2.0))
                    target_velocity = min(target_velocity, max_velocity_near_goal)
        
        velocity_error = target_velocity - car_state.velocity
        acceleration = 0.5 * velocity_error
        acceleration = np.clip(acceleration, -2.0, 2.0)

        return {"acceleration": acceleration, "steering_rate": steering_rate}

    def reset(self) -> None:
        """Reset PID controller state."""
        self.integral_error = 0.0
        self.last_error = 0.0
