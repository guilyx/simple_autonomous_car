"""Utility visualization functions for non-component data (perception, car)."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from typing import Optional

from simple_autonomous_car.car.car import Car, CarState
from simple_autonomous_car.perception.perception import PerceptionPoints


def plot_perception(
    perception_points: PerceptionPoints,
    car_state: CarState,
    ax: Optional[plt.Axes] = None,
    frame: str = "global",
    color: str = "red",
    label: str = "Perception",
    **kwargs
) -> plt.Axes:
    """
    Plot perception points from sensors.
    
    Parameters
    ----------
    perception_points : PerceptionPoints
        Perception data points.
    car_state : CarState
        Car state for coordinate transformation.
    ax : plt.Axes, optional
        Axes to plot on. If None, creates new figure.
    frame : str, default="global"
        Frame to plot in: "global" or "ego".
    color : str, default="red"
        Color for perception points.
    label : str, default="Perception"
        Label for legend.
    **kwargs
        Additional arguments passed to scatter.
    
    Returns
    -------
    plt.Axes
        Axes object with perception plotted.
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 8))

    if len(perception_points.points) == 0:
        return ax

    # Convert to desired frame
    if frame == "global":
        if perception_points.frame != "global":
            points = perception_points.to_global_frame(car_state).points
        else:
            points = perception_points.points
    else:  # ego frame
        if perception_points.frame != "ego":
            points = perception_points.to_ego_frame(car_state).points
        else:
            points = perception_points.points

    # Extract alpha from kwargs if provided, otherwise use default
    scatter_alpha = kwargs.pop("alpha", 0.6)
    
    ax.scatter(
        points[:, 0],
        points[:, 1],
        c=color,
        s=25,
        alpha=scatter_alpha,
        label=label,
        marker=".",
        edgecolors="darkred" if color in ["crimson", "red"] else None,
        linewidths=0.5 if color in ["crimson", "red"] else 0,
        **kwargs
    )

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    if frame == "global":
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
    else:
        ax.set_xlabel("Forward (m)")
        ax.set_ylabel("Left (m)")

    return ax


def plot_car(
    car: Car,
    ax: Optional[plt.Axes] = None,
    frame: str = "global",
    color: str = "blue",
    show_heading: bool = True,
    **kwargs
) -> plt.Axes:
    """
    Plot car position and orientation.
    
    Parameters
    ----------
    car : Car
        Car to plot.
    ax : plt.Axes, optional
        Axes to plot on. If None, creates new figure.
    frame : str, default="global"
        Frame to plot in: "global" or "ego".
    color : str, default="blue"
        Color for car.
    show_heading : bool, default=True
        Whether to show heading arrow.
    **kwargs
        Additional arguments passed to Polygon.
    
    Returns
    -------
    plt.Axes
        Axes object with car plotted.
    """
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 8))

    if frame == "global":
        car_corners = car.get_corners()
        car_pos = car.state.position()
    else:  # ego frame
        car_corners = np.array([[-2, -0.9], [-2, 0.9], [2, 0.9], [2, -0.9]])
        car_pos = np.array([0.0, 0.0])

    car_poly = Polygon(
        car_corners, closed=True, color=color, alpha=0.7, label="Car", **kwargs
    )
    ax.add_patch(car_poly)

    if show_heading and frame == "global":
        # Draw heading arrow
        arrow_length = 3.0
        dx = arrow_length * np.cos(car.state.heading)
        dy = arrow_length * np.sin(car.state.heading)
        ax.arrow(
            car_pos[0],
            car_pos[1],
            dx,
            dy,
            head_width=1.0,
            head_length=0.8,
            fc=color,
            ec=color,
        )
    elif show_heading and frame == "ego":
        # Draw forward arrow in ego frame
        ax.arrow(
            0, 0,
            3.0, 0,
            head_width=1.2,
            head_length=1.0,
            fc=color,
            ec=color,
        )

    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    return ax
