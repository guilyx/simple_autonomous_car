"""Controller-specific visualization utilities."""

import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Dict, List, Tuple

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.control.pure_pursuit_controller import PurePursuitController


def plot_pure_pursuit_state(
    controller: PurePursuitController,
    car_state: CarState,
    plan: np.ndarray,
    ax: Optional[plt.Axes] = None,
    show_curvature: bool = True,
    show_steering_circle: bool = True,
    show_lookahead: bool = True,
) -> plt.Axes:
    """
    Plot Pure Pursuit controller state with detailed visualization.

    Shows:
    - Planned path
    - Car position and heading
    - Lookahead point and line
    - Steering circle (turning radius)
    - Path curvature
    - Control commands

    Parameters
    ----------
    controller : PurePursuitController
        Pure Pursuit controller instance.
    car_state : CarState
        Current car state.
    plan : np.ndarray
        Planned path as array of shape (N, 2).
    ax : plt.Axes, optional
        Axes to plot on. If None, creates new figure.
    show_curvature : bool, default=True
        Whether to show path curvature.
    show_steering_circle : bool, default=True
        Whether to show steering circle (turning radius).
    show_lookahead : bool, default=True
        Whether to show lookahead point and line.

    Returns
    -------
    plt.Axes
        Axes object with controller state plotted.

    Examples
    --------
    >>> from simple_autonomous_car.control.controller_viz import plot_pure_pursuit_state
    >>> ax = plot_pure_pursuit_state(controller, car.state, plan)
    >>> plt.show()
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 10))

    car_pos = car_state.position()

    # Compute control
    control = controller.compute_control(car_state, plan=plan)

    # Plot plan
    if len(plan) > 0:
        ax.plot(plan[:, 0], plan[:, 1], "g-o", linewidth=2, markersize=4, alpha=0.6, label="Plan")

    # Plot car
    ax.plot(car_pos[0], car_pos[1], "bo", markersize=12, label="Car")
    # Car heading arrow
    heading_length = 3.0
    dx = heading_length * np.cos(car_state.heading)
    dy = heading_length * np.sin(car_state.heading)
    ax.arrow(
        car_pos[0],
        car_pos[1],
        dx,
        dy,
        head_width=1.0,
        head_length=0.8,
        fc="blue",
        ec="blue",
        label="Heading",
    )

    # Find lookahead point (simplified version of controller logic)
    if show_lookahead and len(plan) > 0:
        distances_to_path = np.linalg.norm(plan - car_pos, axis=1)
        closest_idx = np.argmin(distances_to_path)

        # Calculate adaptive lookahead
        adaptive_lookahead = controller.lookahead_distance + controller.lookahead_gain * car_state.velocity

        # Find point at lookahead distance ahead
        path_distances = np.zeros(len(plan))
        for i in range(closest_idx, len(plan) - 1):
            path_distances[i + 1] = path_distances[i] + np.linalg.norm(plan[i + 1] - plan[i])
        for i in range(closest_idx - 1, -1, -1):
            path_distances[i] = path_distances[i + 1] - np.linalg.norm(plan[i + 1] - plan[i])

        target_distances = path_distances - path_distances[closest_idx]
        ahead_mask = target_distances >= 0
        ahead_indices = np.where(ahead_mask)[0]

        if len(ahead_indices) > 0:
            ahead_distances = target_distances[ahead_indices]
            lookahead_idx = np.argmin(np.abs(ahead_distances - adaptive_lookahead))
            target_idx = ahead_indices[lookahead_idx]
            if target_idx < len(plan):
                target_point = plan[target_idx]

                # Plot lookahead point
                ax.plot(
                    target_point[0],
                    target_point[1],
                    "ro",
                    markersize=10,
                    label="Lookahead Point",
                )

                # Plot lookahead line
                ax.plot(
                    [car_pos[0], target_point[0]],
                    [car_pos[1], target_point[1]],
                    "r--",
                    linewidth=2,
                    alpha=0.7,
                    label=f"Lookahead ({adaptive_lookahead:.1f}m)",
                )

    # Show steering circle (turning radius)
    if show_steering_circle and abs(car_state.steering_angle) > 1e-6:
        wheelbase = 2.5  # Standard wheelbase
        if abs(car_state.steering_angle) > 1e-6:
            turning_radius = wheelbase / np.tan(car_state.steering_angle)
        else:
            turning_radius = 1e6  # Straight line

        # Calculate circle center (perpendicular to car heading)
        perp_angle = car_state.heading + np.pi / 2
        if turning_radius < 0:
            perp_angle += np.pi
            turning_radius = abs(turning_radius)

        center_x = car_pos[0] + turning_radius * np.cos(perp_angle)
        center_y = car_pos[1] + turning_radius * np.sin(perp_angle)

        # Draw turning circle
        circle = plt.Circle(
            (center_x, center_y),
            abs(turning_radius),
            fill=False,
            color="orange",
            linestyle="--",
            linewidth=2,
            alpha=0.5,
            label=f"Turning Radius ({abs(turning_radius):.1f}m)",
        )
        ax.add_patch(circle)

    # Show curvature along path
    if show_curvature and len(plan) > 2:
        curvatures = []
        for i in range(len(plan) - 2):
            p1, p2, p3 = plan[i], plan[i + 1], plan[i + 2]
            v1 = p2 - p1
            v2 = p3 - p2
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            norm1 = np.linalg.norm(v1)
            norm2 = np.linalg.norm(v2)
            if norm1 > 1e-6 and norm2 > 1e-6:
                curvature = abs(cross) / (norm1 * norm2 * norm1)
            else:
                curvature = 0.0
            curvatures.append(curvature)

        if len(curvatures) > 0:
            max_curvature = max(curvatures) if curvatures else 0.0
            # Plot curvature as color-coded path segments
            for i in range(len(plan) - 2):
                if i < len(curvatures):
                    color_intensity = min(curvatures[i] / max(max_curvature, 1e-6), 1.0)
                    ax.plot(
                        [plan[i][0], plan[i + 1][0]],
                        [plan[i][1], plan[i + 1][1]],
                        "-",
                        color=plt.cm.RdYlGn(1.0 - color_intensity),
                        linewidth=4,
                        alpha=0.6,
                    )

    # Add text with control commands
    info_text = (
        f"Steering: {np.degrees(car_state.steering_angle):.1f}°\n"
        f"Steering Rate: {control['steering_rate']:.2f} rad/s\n"
        f"Acceleration: {control['acceleration']:.2f} m/s²\n"
        f"Velocity: {car_state.velocity:.2f} m/s"
    )
    ax.text(
        0.02,
        0.98,
        info_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
    )

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend(loc="upper right")

    return ax


def plot_control_history(
    control_history: List[Dict],
    ax: Optional[plt.Axes] = None,
    show_steering: bool = True,
    show_acceleration: bool = True,
    show_velocity: bool = True,
) -> plt.Axes:
    """
    Plot control command history over time.

    Parameters
    ----------
    control_history : List[Dict]
        List of control dictionaries with keys: 'steering_rate', 'acceleration', 'velocity', 'time'.
    ax : plt.Axes, optional
        Axes to plot on. If None, creates new figure.
    show_steering : bool, default=True
        Whether to show steering rate.
    show_acceleration : bool, default=True
        Whether to show acceleration.
    show_velocity : bool, default=True
        Whether to show velocity.

    Returns
    -------
    plt.Axes
        Axes object with control history plotted.

    Examples
    --------
    >>> history = [{'steering_rate': 0.1, 'acceleration': 0.5, 'velocity': 10.0, 'time': i*0.1} 
    ...            for i in range(100)]
    >>> ax = plot_control_history(history)
    >>> plt.show()
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 6))

    if len(control_history) == 0:
        return ax

    times = [h.get("time", i) for i, h in enumerate(control_history)]

    if show_steering:
        steering_rates = [h.get("steering_rate", 0.0) for h in control_history]
        ax.plot(times, steering_rates, "b-", linewidth=2, label="Steering Rate (rad/s)")

    if show_acceleration:
        accelerations = [h.get("acceleration", 0.0) for h in control_history]
        ax.plot(times, accelerations, "r-", linewidth=2, label="Acceleration (m/s²)")

    if show_velocity:
        velocities = [h.get("velocity", 0.0) for h in control_history]
        ax.plot(times, velocities, "g-", linewidth=2, label="Velocity (m/s)")

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.grid(True, alpha=0.3)
    ax.legend()

    return ax
