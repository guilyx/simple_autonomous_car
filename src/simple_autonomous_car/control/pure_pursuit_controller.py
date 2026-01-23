"""Pure Pursuit path following controller."""

import numpy as np
from typing import Dict, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Arc

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.perception.perception import PerceptionPoints
from simple_autonomous_car.control.base_controller import BaseController
from simple_autonomous_car.constants import (
    DEFAULT_WHEELBASE,
    DEFAULT_LOOKAHEAD_DISTANCE,
    DEFAULT_LOOKAHEAD_GAIN,
    DEFAULT_MAX_STEERING_RATE,
    DEFAULT_TARGET_VELOCITY,
    DEFAULT_VELOCITY_GAIN,
    DEFAULT_DT,
    DEFAULT_GOAL_TOLERANCE,
    DEFAULT_ARC_LOOKAHEAD_DISTANCE,
    DEFAULT_MIN_TURNING_RADIUS,
    DEFAULT_MAX_TURNING_RADIUS,
    DEFAULT_ARC_LINEWIDTH,
    DEFAULT_ARC_ALPHA,
    DEFAULT_LOOKAHEAD_LINEWIDTH,
    MIN_LOOKAHEAD_DISTANCE,
    MIN_STEERING_ANGLE_THRESHOLD,
    SMALL_STEERING_ANGLE_THRESHOLD,
    ADAPTIVE_GAIN_DENOMINATOR,
    COST_THRESHOLD,
)
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simple_autonomous_car.costmap.base_costmap import BaseCostmap


class PurePursuitController(BaseController):
    """
    Pure Pursuit path following controller.

    This controller follows a path by looking ahead a certain distance
    and steering towards that point. It's simple, robust, and widely used.

    Parameters
    ----------
    lookahead_distance : float, default=8.0
        Distance ahead to look for target point (meters).
        Larger = smoother but slower response.
    lookahead_gain : float, default=2.0
        Gain for adaptive lookahead (multiplies velocity).
    max_steering_rate : float, default=1.0
        Maximum steering rate in rad/s.
    target_velocity : float, default=10.0
        Target velocity in m/s.
    velocity_gain : float, default=0.5
        Gain for velocity control.
    name : str, default="pure_pursuit"
        Controller name.
    """

    def __init__(
        self,
        lookahead_distance: float = DEFAULT_LOOKAHEAD_DISTANCE,
        lookahead_gain: float = DEFAULT_LOOKAHEAD_GAIN,
        max_steering_rate: float = DEFAULT_MAX_STEERING_RATE,
        target_velocity: float = DEFAULT_TARGET_VELOCITY,
        velocity_gain: float = DEFAULT_VELOCITY_GAIN,
        name: str = "pure_pursuit",
    ):
        super().__init__(name=name)
        self.lookahead_distance = lookahead_distance
        self.lookahead_gain = lookahead_gain
        self.max_steering_rate = max_steering_rate
        self.target_velocity = target_velocity
        self.velocity_gain = velocity_gain

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
        Compute control using Pure Pursuit algorithm.

        Parameters
        ----------
        car_state : CarState
            Current car state.
        perception_data : dict, optional
            Not used by Pure Pursuit (uses plan instead).
        costmap : BaseCostmap, optional
            Costmap for obstacle avoidance (can be used to adjust velocity).
        plan : np.ndarray, optional
            Planned path as array of shape (N, 2) with [x, y] waypoints.
            If None, returns zero control.
        goal : np.ndarray, optional
            Goal position [x, y] for goal-based velocity adaptation.
        goal_tolerance : float, optional
            Distance to goal to consider reached (default: DEFAULT_GOAL_TOLERANCE).
            When within tolerance, target velocity is set to 0.
        dt : float, default=0.1
            Time step in seconds.

        Returns
        -------
        Dict[str, float]
            Control commands with "acceleration" and "steering_rate".
        """
        if not self.enabled or plan is None or len(plan) == 0:
            return {"acceleration": 0.0, "steering_rate": 0.0}

        # Adaptive lookahead based on velocity
        adaptive_lookahead = self.lookahead_distance + self.lookahead_gain * car_state.velocity

        # Find closest point on path first
        car_pos = car_state.position()
        distances_to_path = np.linalg.norm(plan - car_pos, axis=1)
        closest_idx = np.argmin(distances_to_path)

        # Find target point at lookahead distance ahead of closest point
        # Calculate cumulative distances along path from closest point
        path_distances = np.zeros(len(plan))
        for i in range(closest_idx, len(plan) - 1):
            path_distances[i + 1] = path_distances[i] + np.linalg.norm(plan[i + 1] - plan[i])
        for i in range(closest_idx - 1, -1, -1):
            path_distances[i] = path_distances[i + 1] - np.linalg.norm(plan[i + 1] - plan[i])

        # Find point at lookahead distance ahead
        target_distances = path_distances - path_distances[closest_idx]
        ahead_mask = target_distances >= 0
        ahead_distances = target_distances[ahead_mask]

        if len(ahead_distances) == 0:
            # If no points ahead, use last point
            target_idx = len(plan) - 1
        else:
            # Find point closest to lookahead distance
            lookahead_idx = np.argmin(np.abs(ahead_distances - adaptive_lookahead))
            ahead_indices = np.where(ahead_mask)[0]
            target_idx = ahead_indices[lookahead_idx]
            if target_idx >= len(plan):
                target_idx = len(plan) - 1

        target_point = plan[target_idx]

        # Calculate steering using Pure Pursuit
        target_vector = target_point - car_pos
        lookahead_dist = np.linalg.norm(target_vector)

        if lookahead_dist < MIN_LOOKAHEAD_DISTANCE:
            # Very close to target, reduce steering
            steering_rate = -2.0 * car_state.steering_angle
            steering_rate = np.clip(steering_rate, -self.max_steering_rate, self.max_steering_rate)
        else:
            # Pure Pursuit: calculate desired steering angle
            # Using bicycle model: tan(steering) = wheelbase * curvature
            # Curvature = 2 * sin(alpha) / L, where alpha is angle to target, L is lookahead
            target_angle = np.arctan2(target_vector[1], target_vector[0])
            alpha = target_angle - car_state.heading
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Normalize to [-pi, pi]

            # Pure Pursuit: simpler and more stable approach
            # Directly use the angle to target, scaled by lookahead distance
            # This provides smooth, stable path following
            desired_steering = alpha * (1.0 / (1.0 + lookahead_dist / ADAPTIVE_GAIN_DENOMINATOR))  # Adaptive gain

            # Limit steering angle
            max_steering = np.pi / 4  # 45 degrees
            desired_steering = np.clip(desired_steering, -max_steering, max_steering)

            # Compute steering rate (smooth control)
            steering_error = desired_steering - car_state.steering_angle
            steering_rate = 2.0 * steering_error  # Reduced gain for smoother control
            steering_rate = np.clip(steering_rate, -self.max_steering_rate, self.max_steering_rate)

        # Velocity control (adjust based on costmap and goal distance)
        target_velocity = self.target_velocity

        # Reduce velocity if approaching goal (smooth deceleration)
        if goal is not None:
            distance_to_goal = np.linalg.norm(car_state.position() - goal)
            tolerance = goal_tolerance if goal_tolerance is not None else DEFAULT_GOAL_TOLERANCE
            
            # If within tolerance, stop completely
            if distance_to_goal < tolerance:
                target_velocity = 0.0
            else:
                # Start slowing down when within 10 meters of goal
                slow_down_distance = 10.0
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

        # Reduce velocity if high cost ahead
        if costmap is not None and costmap.is_enabled():
            # Check cost ahead of car
            lookahead_pos = car_state.position() + 5.0 * np.array([
                np.cos(car_state.heading),
                np.sin(car_state.heading)
            ])
            cost_ahead = costmap.get_cost(lookahead_pos, frame="global", car_state=car_state)
            # Reduce velocity if cost > threshold
            if cost_ahead > COST_THRESHOLD:
                target_velocity = target_velocity * (1.0 - cost_ahead * 0.5)

        velocity_error = target_velocity - car_state.velocity
        acceleration = self.velocity_gain * velocity_error
        acceleration = np.clip(acceleration, -2.0, 2.0)

        return {"acceleration": acceleration, "steering_rate": steering_rate}

    def get_visualization_data(
        self,
        car_state: CarState,
        plan: Optional[np.ndarray] = None,
        **kwargs
    ) -> Dict:
        """
        Get visualization data for Pure Pursuit controller.

        Returns lookahead point, steering arc parameters, and control commands.

        Parameters
        ----------
        car_state : CarState
            Current car state.
        plan : np.ndarray, optional
            Planned path.
        **kwargs
            Additional arguments.

        Returns
        -------
        Dict
            Dictionary with keys:
            - "lookahead_point": np.ndarray, target point [x, y]
            - "lookahead_distance": float, current lookahead distance
            - "steering_angle": float, current steering angle
            - "turning_radius": float, turning radius (if steering > 0)
            - "control": Dict, control commands
        """
        if not self.enabled or plan is None or len(plan) == 0:
            return {
                "lookahead_point": None,
                "lookahead_distance": self.lookahead_distance,
                "steering_angle": car_state.steering_angle,
                "turning_radius": None,
                "control": {"acceleration": 0.0, "steering_rate": 0.0},
            }

        # Compute control to get current state
        # Remove wheelbase from kwargs before passing to compute_control
        control_kwargs = {k: v for k, v in kwargs.items() if k != "wheelbase"}
        control = self.compute_control(car_state, plan=plan, **control_kwargs)

        # Find lookahead point (same logic as compute_control)
        adaptive_lookahead = self.lookahead_distance + self.lookahead_gain * car_state.velocity
        car_pos = car_state.position()
        distances_to_path = np.linalg.norm(plan - car_pos, axis=1)
        closest_idx = np.argmin(distances_to_path)

        path_distances = np.zeros(len(plan))
        for i in range(closest_idx, len(plan) - 1):
            path_distances[i + 1] = path_distances[i] + np.linalg.norm(plan[i + 1] - plan[i])
        for i in range(closest_idx - 1, -1, -1):
            path_distances[i] = path_distances[i + 1] - np.linalg.norm(plan[i + 1] - plan[i])

        target_distances = path_distances - path_distances[closest_idx]
        ahead_mask = target_distances >= 0
        ahead_distances = target_distances[ahead_mask]

        if len(ahead_distances) == 0:
            target_idx = len(plan) - 1
        else:
            lookahead_idx = np.argmin(np.abs(ahead_distances - adaptive_lookahead))
            ahead_indices = np.where(ahead_mask)[0]
            target_idx = ahead_indices[lookahead_idx]
            if target_idx >= len(plan):
                target_idx = len(plan) - 1

        lookahead_point = plan[target_idx]

        # Calculate turning radius using desired steering angle (from control computation)
        # Always compute desired_steering for visualization to show intended steering, not current state
        # This ensures arc only shows when actually turning, not when steering angle is stale
        wheelbase = kwargs.get("wheelbase", DEFAULT_WHEELBASE)
        turning_radius = None
        steering_angle_for_viz = 0.0  # Default to 0 (straight) for visualization

        # Always compute desired steering for visualization (same logic as compute_control)
        target_vector = lookahead_point - car_pos
        lookahead_dist = np.linalg.norm(target_vector)

        if lookahead_dist > MIN_LOOKAHEAD_DISTANCE:
            target_angle = np.arctan2(target_vector[1], target_vector[0])
            alpha = target_angle - car_state.heading
            alpha = np.arctan2(np.sin(alpha), np.cos(alpha))  # Normalize to [-pi, pi]

            desired_steering = alpha * (1.0 / (1.0 + lookahead_dist / ADAPTIVE_GAIN_DENOMINATOR))
            max_steering = np.pi / 4  # 45 degrees
            desired_steering = np.clip(desired_steering, -max_steering, max_steering)

            # Use desired steering for visualization (shows intended steering, not stale state)
            steering_angle_for_viz = desired_steering
        # else: lookahead_dist too small, use 0.0 (straight) - don't use stale car_state.steering_angle

        # Calculate turning radius from steering angle (always calculate, even for straight)
        if abs(steering_angle_for_viz) > SMALL_STEERING_ANGLE_THRESHOLD:
            # Normal steering - calculate turning radius
            turning_radius = wheelbase / np.tan(steering_angle_for_viz)
        else:
            # Very small or zero steering - use large turning radius (essentially straight)
            turning_radius = None  # Will be handled in visualization as straight line

        return {
            "lookahead_point": lookahead_point,
            "lookahead_distance": adaptive_lookahead,
            "steering_angle": steering_angle_for_viz,  # Use desired steering for visualization
            "turning_radius": turning_radius,
            "control": control,
        }

    def visualize(
        self,
        ax: plt.Axes,
        car_state: CarState,
        plan: Optional[np.ndarray] = None,
        frame: str = "global",
        **kwargs
    ) -> None:
        """
        Visualize Pure Pursuit controller state (lookahead point, steering arc).

        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Axes to plot on.
        car_state : CarState
            Current car state.
        plan : np.ndarray, optional
            Planned path.
        frame : str, default="global"
            Frame to plot in: "global" or "ego".
        **kwargs
            Additional visualization arguments:
            - lookahead_color: str, color for lookahead point/line
            - arc_color: str, color for steering arc
            - arc_linewidth: float, linewidth for steering arc
            - arc_alpha: float, alpha for steering arc
            - wheelbase: float, car wheelbase for turning radius calculation
        """
        if not self.enabled or plan is None or len(plan) == 0:
            return

        # Get visualization data
        wheelbase = kwargs.pop("wheelbase", DEFAULT_WHEELBASE)
        viz_data = self.get_visualization_data(
            car_state,
            plan=plan,
            wheelbase=wheelbase,
        )

        # Extract visualization parameters from kwargs
        lookahead_color = kwargs.pop("lookahead_color", "magenta")
        arc_color = kwargs.pop("arc_color", "violet")  # Changed to violet/purple
        arc_linewidth = kwargs.pop("arc_linewidth", DEFAULT_ARC_LINEWIDTH)
        arc_alpha = kwargs.pop("arc_alpha", DEFAULT_ARC_ALPHA)
        arc_lookahead_distance = kwargs.pop("arc_lookahead_distance", DEFAULT_ARC_LOOKAHEAD_DISTANCE)
        max_turning_radius = kwargs.pop("max_turning_radius", DEFAULT_MAX_TURNING_RADIUS)
        min_turning_radius = kwargs.pop("min_turning_radius", DEFAULT_MIN_TURNING_RADIUS)

        # Get car position and heading for frame transformation
        car_pos = car_state.position()
        car_heading = car_state.heading

        if frame == "ego":
            car_pos_plot = np.array([0.0, 0.0])
            car_heading_plot = 0.0
        else:
            car_pos_plot = car_pos
            car_heading_plot = car_heading

        # Plot lookahead point
        lookahead_point = viz_data.get("lookahead_point")
        if lookahead_point is not None:
            if frame == "ego":
                lookahead_plot = car_state.transform_to_car_frame(lookahead_point)
            else:
                lookahead_plot = lookahead_point

            # Plot lookahead point (line removed per user request)
            ax.plot(
                lookahead_plot[0],
                lookahead_plot[1],
                "o",
                color=lookahead_color,
                markersize=10,
                markeredgecolor="darkmagenta",
                markeredgewidth=2,
                label="Lookahead",
                zorder=6,
            )

        # Plot steering arc
        turning_radius = viz_data.get("turning_radius")
        steering_angle = viz_data.get("steering_angle", car_state.steering_angle)

        # Always show steering path (removed all thresholds per user request)
        # Calculate turning radius if not provided
        if turning_radius is None:
            # Handle very small angles to avoid division by zero
            if abs(steering_angle) < SMALL_STEERING_ANGLE_THRESHOLD:
                # Very small steering - draw straight line forward instead of arc
                # Draw straight line segment in direction of car heading
                if frame == "ego":
                    end_x = arc_lookahead_distance
                    end_y = 0.0
                else:
                    end_x = car_pos_plot[0] + arc_lookahead_distance * np.cos(car_heading_plot)
                    end_y = car_pos_plot[1] + arc_lookahead_distance * np.sin(car_heading_plot)
                
                ax.plot(
                    [car_pos_plot[0], end_x],
                    [car_pos_plot[1], end_y],
                    color=arc_color,
                    linestyle="-",
                    linewidth=arc_linewidth,
                    alpha=arc_alpha,
                    zorder=4,
                    label="Steering Path",
                )
                return  # Done with straight line case
            else:
                turning_radius = wheelbase / np.tan(steering_angle)

        # Draw arc for non-straight steering
        if turning_radius is not None and not np.isnan(turning_radius) and not np.isinf(turning_radius):
            turning_radius_abs = abs(turning_radius)
            # Always show arc (removed min_turning_radius check)
            if turning_radius_abs > 0:
                    # Calculate arc parameters
                    arc_length_rad = arc_lookahead_distance / turning_radius_abs

                    if frame == "ego":
                        center_x = 0.0
                        if steering_angle > 0:  # Left turn
                            center_y = turning_radius_abs
                            theta_start_deg = -90.0
                            theta_end_deg = -90.0 + np.degrees(arc_length_rad)
                        else:  # Right turn
                            center_y = -turning_radius_abs
                            theta_start_deg = 90.0
                            theta_end_deg = 90.0 - np.degrees(arc_length_rad)
                    else:
                        # Global frame
                        perp_angle = car_heading_plot + np.pi / 2
                        if turning_radius < 0:
                            perp_angle += np.pi

                        center_x = car_pos_plot[0] + turning_radius_abs * np.cos(perp_angle)
                        center_y = car_pos_plot[1] + turning_radius_abs * np.sin(perp_angle)

                        dx_to_car = car_pos_plot[0] - center_x
                        dy_to_car = car_pos_plot[1] - center_y
                        theta_start_rad = np.arctan2(dy_to_car, dx_to_car)
                        theta_start_deg = np.degrees(theta_start_rad)

                        if steering_angle > 0:
                            theta_end_deg = theta_start_deg + np.degrees(arc_length_rad)
                        else:
                            theta_end_deg = theta_start_deg - np.degrees(arc_length_rad)

                    # Draw steering arc
                    arc = Arc(
                        (center_x, center_y),
                        2 * turning_radius_abs,
                        2 * turning_radius_abs,
                        angle=0,
                        theta1=theta_start_deg,
                        theta2=theta_end_deg,
                        color=arc_color,
                        linestyle="-",
                        linewidth=arc_linewidth,
                        alpha=arc_alpha,
                        zorder=4,
                        label="Steering Path",
                    )
                    ax.add_patch(arc)

                    # Add arrow at end of arc
                    theta_end_rad = np.radians(theta_end_deg)
                    arc_end_x = center_x + turning_radius_abs * np.cos(theta_end_rad)
                    arc_end_y = center_y + turning_radius_abs * np.sin(theta_end_rad)

                    if steering_angle > 0:
                        direction_angle = theta_end_rad + np.pi / 2
                    else:
                        direction_angle = theta_end_rad - np.pi / 2

                    arrow_length = 2.0
                    dx = arrow_length * np.cos(direction_angle)
                    dy = arrow_length * np.sin(direction_angle)
                    ax.arrow(
                        arc_end_x,
                        arc_end_y,
                        dx,
                        dy,
                        head_width=1.5,
                        head_length=1.0,
                        fc=arc_color,
                        ec="darkmagenta",
                        alpha=arc_alpha,
                        zorder=5,
                    )
