"""Track-based path planner."""

from typing import TYPE_CHECKING, Any, Optional

import numpy as np

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.constants import (
    DEFAULT_PLAN_LINEWIDTH,
    DEFAULT_PLANNER_LOOKAHEAD_DISTANCE,
    DEFAULT_WAYPOINT_SPACING,
    MIN_SEGMENT_LENGTH,
)
from simple_autonomous_car.planning.base_planner import BasePlanner
from simple_autonomous_car.track.track import Track

if TYPE_CHECKING:
    from simple_autonomous_car.costmap.base_costmap import BaseCostmap


class TrackPlanner(BasePlanner):
    """
    Simple planner that follows the track centerline.

    This planner generates a path along the track centerline ahead of the car.
    It's useful for basic track following scenarios.

    Parameters
    ----------
    track : Track
        Track to plan on.
    lookahead_distance : float, default=50.0
        Distance ahead to plan (meters).
    waypoint_spacing : float, default=2.0
        Spacing between waypoints in meters.
    name : str, default="track_planner"
        Planner name.
    """

    def __init__(
        self,
        track: Track,
        lookahead_distance: float = DEFAULT_PLANNER_LOOKAHEAD_DISTANCE,
        waypoint_spacing: float = DEFAULT_WAYPOINT_SPACING,
        name: str = "track_planner",
    ):
        super().__init__(name=name)
        self.track = track
        self.lookahead_distance = lookahead_distance
        self.waypoint_spacing = waypoint_spacing

        # Precompute cumulative distances
        self._compute_cumulative_distances()

    def _compute_cumulative_distances(self) -> None:
        """Precompute cumulative distances along track."""
        self.cumulative_distances = np.zeros(len(self.track.centerline))
        for i in range(1, len(self.track.centerline)):
            dist = np.linalg.norm(self.track.centerline[i] - self.track.centerline[i - 1])
            self.cumulative_distances[i] = self.cumulative_distances[i - 1] + dist
        self.total_length = self.cumulative_distances[-1]

    def plan(
        self,
        car_state: CarState,
        perception_data: dict | None = None,
        costmap: Optional["BaseCostmap"] = None,
        goal: np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Generate plan along track centerline.

        Parameters
        ----------
        car_state : CarState
            Current car state.
        perception_data : dict, optional
            Not used by track planner.
        costmap : BaseCostmap, optional
            Costmap for obstacle avoidance (not used by basic track planner).
        goal : np.ndarray, optional
            Not used by track planner.

        Returns
        -------
        np.ndarray
            Planned path as array of shape (N, 2) with [x, y] waypoints.
        """
        if not self.enabled:
            return np.array([]).reshape(0, 2)

        # Find closest point on track
        car_pos = car_state.position()
        distances = np.linalg.norm(self.track.centerline - car_pos, axis=1)
        closest_idx = np.argmin(distances)
        s_start = self.cumulative_distances[closest_idx]

        # Generate waypoints ahead
        num_waypoints = int(self.lookahead_distance / self.waypoint_spacing) + 1

        waypoints = []
        for i in range(num_waypoints):
            s = s_start + (i / (num_waypoints - 1)) * self.lookahead_distance
            s = s % self.total_length  # Wrap around

            # Find point at distance s
            idx = np.searchsorted(self.cumulative_distances, s)
            if idx == 0:
                idx = 1
            if idx >= len(self.track.centerline):
                idx = len(self.track.centerline) - 1

            # Interpolate
            segment_dist = s - self.cumulative_distances[idx - 1]
            segment_length = self.cumulative_distances[idx] - self.cumulative_distances[idx - 1]
            if segment_length > MIN_SEGMENT_LENGTH:
                t = segment_dist / segment_length
            else:
                t = 0.0

            point = self.track.centerline[idx - 1] + t * (
                self.track.centerline[idx] - self.track.centerline[idx - 1]
            )
            waypoints.append(point)

        return np.array(waypoints)

    def visualize(
        self,
        ax: Any,
        car_state: CarState,
        plan: np.ndarray | None = None,
        frame: str = "global",
        **kwargs: Any,
    ) -> None:
        """
        Visualize planned path on the given axes.

        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Axes to plot on.
        car_state : CarState
            Current car state.
        plan : np.ndarray, optional
            Planned path (if already computed, otherwise will be computed).
        frame : str, default="global"
            Frame to plot in: "global" or "ego".
        **kwargs
            Additional visualization arguments:
            - color: str, color for plan line
            - linewidth: float, linewidth for plan
            - linestyle: str, linestyle for plan
            - show_waypoints: bool, whether to show waypoint markers
        """
        # Compute plan if not provided
        if plan is None:
            plan = self.plan(car_state)

        if len(plan) == 0:
            return

        # Transform plan to desired frame
        if frame == "ego":
            plan_plot = np.array([car_state.transform_to_car_frame(point) for point in plan])
        else:
            plan_plot = plan

        # Extract visualization parameters
        color = kwargs.pop("color", "green")
        linewidth = kwargs.pop("linewidth", DEFAULT_PLAN_LINEWIDTH)
        kwargs.pop("linestyle", "--")
        show_waypoints = kwargs.pop("show_waypoints", frame == "global")
        alpha = kwargs.pop("alpha", 0.8)

        # Plot plan
        if show_waypoints:
            ax.plot(
                plan_plot[:, 0],
                plan_plot[:, 1],
                "o-",
                color=color,
                linewidth=linewidth,
                markersize=4,
                label="Plan",
                alpha=alpha,
                **kwargs,
            )
        else:
            ax.plot(
                plan_plot[:, 0],
                plan_plot[:, 1],
                "-",
                color=color,
                linewidth=linewidth,
                label="Plan",
                alpha=alpha,
                **kwargs,
            )
