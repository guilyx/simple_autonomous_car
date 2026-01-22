"""Visualization utilities for alert systems."""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from typing import Dict, Optional

from simple_autonomous_car.track.track import Track
from simple_autonomous_car.car.car import Car
from simple_autonomous_car.perception.perception import PerceptionPoints
from simple_autonomous_car.maps.frenet_map import FrenetMap


class AlertVisualizer:
    """Visualization utilities for alert systems."""

    def __init__(
        self,
        track: Track,
        frenet_map: Optional[FrenetMap] = None,
        figsize: tuple = (16, 8),
    ):
        """
        Initialize alert visualizer.

        Args:
            track: Track object
            frenet_map: Optional Frenet map (will be created if not provided)
            figsize: Figure size
        """
        self.track = track
        self.frenet_map = frenet_map if frenet_map is not None else FrenetMap(track)
        self.figsize = figsize
        self.fig = None
        self.axes = None

    def create_figure(self) -> None:
        """Create figure with world and car frame views."""
        self.fig, (self.ax_world, self.ax_car) = plt.subplots(1, 2, figsize=self.figsize)
        self._setup_axes()

    def _setup_axes(self) -> None:
        """Set up axis labels and properties."""
        # World frame
        self.ax_world.set_title("World Frame - Track Bounds Alert", fontsize=14, fontweight="bold")
        self.ax_world.set_xlabel("X (m)")
        self.ax_world.set_ylabel("Y (m)")
        self.ax_world.set_aspect("equal")
        self.ax_world.grid(True, alpha=0.3)

        # Car frame
        self.ax_car.set_title("Car Frame - Perception Points", fontsize=14, fontweight="bold")
        self.ax_car.set_xlabel("Forward (m)")
        self.ax_car.set_ylabel("Left (m)")
        self.ax_car.set_aspect("equal")
        self.ax_car.grid(True, alpha=0.3)

    def plot_track(self, ax=None) -> None:
        """
        Plot full track boundaries.

        Args:
            ax: Matplotlib axis (uses self.ax_world if None)
        """
        if ax is None:
            ax = self.ax_world

        ax.plot(
            self.track.inner_bound[:, 0],
            self.track.inner_bound[:, 1],
            "k-",
            linewidth=2.5,
            label="Map (Inner)",
            alpha=0.8,
        )
        ax.plot(
            self.track.outer_bound[:, 0],
            self.track.outer_bound[:, 1],
            "k-",
            linewidth=2.5,
            label="Map (Outer)",
            alpha=0.8,
        )

    def plot_perception(
        self,
        perception_points: PerceptionPoints,
        car,
        highlight_alerts: bool = False,
        alert_points: Optional[list] = None,
    ) -> None:
        """
        Plot perception points in both world and car frames.

        Args:
            perception_points: Perception points
            car: Car object
            highlight_alerts: Whether to highlight alert points
            alert_points: List of alert points (if highlight_alerts is True)
        """
        # Ensure points are in ego frame
        if perception_points.frame != "ego":
            perception_points = perception_points.to_ego_frame(car.state)

        # Car frame view
        if len(perception_points) > 0:
            self.ax_car.scatter(
                perception_points.points[:, 0],
                perception_points.points[:, 1],
                c="red",
                s=15,
                alpha=0.6,
                label="Perception",
                marker=".",
            )

        # World frame view
        if len(perception_points) > 0:
            perception_global = perception_points.to_global_frame(car.state)
            self.ax_world.scatter(
                perception_global.points[:, 0],
                perception_global.points[:, 1],
                c="red",
                s=15,
                alpha=0.6,
                label="Perception",
                marker=".",
            )

            # Highlight alert points if requested
            if highlight_alerts and alert_points:
                alert_points_global = []
                for alert_point in alert_points:
                    s, d = alert_point["s"], alert_point["d"]
                    point_global = self.frenet_map.frenet_frame.frenet_to_global(s, d)
                    alert_points_global.append(point_global)

                if alert_points_global:
                    alert_points_global = np.array(alert_points_global)
                    self.ax_world.scatter(
                        alert_points_global[:, 0],
                        alert_points_global[:, 1],
                        c="orange",
                        s=150,
                        marker="x",
                        linewidths=3,
                        label="Alert Points",
                        zorder=10,
                    )

    def plot_car(self, car, ax=None) -> None:
        """
        Plot car position.

        Args:
            car: Car object
            ax: Matplotlib axis (uses self.ax_world if None)
        """
        if ax is None:
            ax = self.ax_world

        car_corners = car.get_corners()
        car_poly = Polygon(car_corners, closed=True, color="blue", alpha=0.7)
        ax.add_patch(car_poly)

        # Also plot in car frame (at origin)
        car_corners_car = np.array([[-2, -0.9], [-2, 0.9], [2, 0.9], [2, -0.9]])
        car_poly_car = Polygon(car_corners_car, closed=True, color="blue", alpha=0.7)
        self.ax_car.add_patch(car_poly_car)

    def plot_alert_result(
        self,
        alert_result: Dict,
        perception_points: PerceptionPoints,
        car,
    ) -> None:
        """
        Plot complete alert visualization.

        Args:
            alert_result: Result from alert system check
            perception_points: Perception points
            car: Car object
        """
        self.ax_world.clear()
        self.ax_car.clear()
        self._setup_axes()

        # Plot track
        self.plot_track()

        # Plot perception with alerts
        self.plot_perception(
            perception_points,
            car,
            highlight_alerts=True,
            alert_points=alert_result.get("alert_points", []),
        )

        # Plot car
        self.plot_car(car)

        # Add alert info as text
        if alert_result.get("has_critical"):
            status_text = f"CRITICAL: Max deviation = {alert_result['max_deviation']:.2f}m"
            color = "red"
        elif alert_result.get("has_warning"):
            status_text = f"WARNING: Max deviation = {alert_result['max_deviation']:.2f}m"
            color = "orange"
        else:
            status_text = f"OK: Max deviation = {alert_result['max_deviation']:.2f}m"
            color = "green"

        self.ax_world.text(
            0.02,
            0.98,
            status_text,
            transform=self.ax_world.transAxes,
            fontsize=12,
            verticalalignment="top",
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.8, edgecolor=color, linewidth=2),
        )

        # Set limits
        all_points = np.vstack([self.track.inner_bound, self.track.outer_bound])
        padding = 10.0
        self.ax_world.set_xlim(
            np.min(all_points[:, 0]) - padding, np.max(all_points[:, 0]) + padding
        )
        self.ax_world.set_ylim(
            np.min(all_points[:, 1]) - padding, np.max(all_points[:, 1]) + padding
        )

        self.ax_car.set_xlim(-50, 50)
        self.ax_car.set_ylim(-50, 50)

        self.ax_world.legend(loc="upper right", fontsize=9)
        self.ax_car.legend(loc="upper right", fontsize=9)

        plt.tight_layout()

    def plot_alert_history(self, alert_history: list) -> None:
        """
        Plot alert history over time.

        Args:
            alert_history: List of alert history entries
        """
        if not alert_history:
            return

        fig, ax = plt.subplots(figsize=(12, 6))

        steps = [h["step"] for h in alert_history]
        deviations = [h["max_deviation"] for h in alert_history]
        warnings = [h.get("has_warning", False) for h in alert_history]
        criticals = [h.get("has_critical", False) for h in alert_history]

        ax.plot(steps, deviations, "b-", label="Max Deviation", linewidth=2)

        # Add threshold lines if available
        if hasattr(self, "warning_threshold"):
            ax.axhline(
                y=self.warning_threshold,
                color="orange",
                linestyle="--",
                label="Warning Threshold",
            )
        if hasattr(self, "critical_threshold"):
            ax.axhline(
                y=self.critical_threshold,
                color="red",
                linestyle="--",
                label="Critical Threshold",
            )

        # Fill warning and critical zones
        if any(warnings):
            ax.fill_between(
                steps,
                0,
                deviations,
                where=np.array(warnings),
                alpha=0.3,
                color="orange",
                label="Warning Zone",
            )
        if any(criticals):
            ax.fill_between(
                steps,
                0,
                deviations,
                where=np.array(criticals),
                alpha=0.3,
                color="red",
                label="Critical Zone",
            )

        ax.set_xlabel("Simulation Step")
        ax.set_ylabel("Max Deviation (m)")
        ax.set_title("Alert History - Deviation Over Time")
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()

    def show(self) -> None:
        """Display the figure."""
        if self.fig is not None:
            plt.show()
