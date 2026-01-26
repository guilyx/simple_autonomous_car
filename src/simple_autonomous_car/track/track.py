"""Track generation with bounds and centerline."""

from typing import TYPE_CHECKING, Any, Optional

import numpy as np

if TYPE_CHECKING:
    from matplotlib.axes import Axes

    from simple_autonomous_car.car.car import CarState


class Track:
    """Represents a racing track with centerline and bounds."""

    def __init__(
        self,
        centerline: np.ndarray,
        track_width: float = 5.0,
        inner_bound: np.ndarray | None = None,
        outer_bound: np.ndarray | None = None,
    ):
        """
        Initialize track.

        Args:
            centerline: Array of shape (N, 2) with [x, y] coordinates of centerline
            track_width: Width of the track (used if bounds not provided)
            inner_bound: Array of shape (N, 2) with inner boundary points
            outer_bound: Array of shape (N, 2) with outer boundary points
        """
        self.centerline = np.asarray(centerline, dtype=np.float64)
        if self.centerline.ndim != 2 or self.centerline.shape[1] != 2:
            raise ValueError("centerline must be shape (N, 2)")

        self.track_width = track_width

        if inner_bound is None or outer_bound is None:
            self._generate_bounds()
        else:
            self.inner_bound = np.asarray(inner_bound, dtype=np.float64)
            self.outer_bound = np.asarray(outer_bound, dtype=np.float64)

        self._validate_bounds()

    def _generate_bounds(self) -> None:
        """Generate inner and outer bounds from centerline and width."""
        n_points = len(self.centerline)
        inner_bound = np.zeros_like(self.centerline)
        outer_bound = np.zeros_like(self.centerline)

        for i in range(n_points):
            # Get direction vector
            if i == 0:
                direction = self.centerline[1] - self.centerline[0]
            elif i == n_points - 1:
                direction = self.centerline[-1] - self.centerline[-2]
            else:
                direction = self.centerline[i + 1] - self.centerline[i - 1]

            # Normalize and get perpendicular
            direction_norm = np.linalg.norm(direction)
            if direction_norm > 1e-6:
                direction = direction / direction_norm
            else:
                direction = np.array([1.0, 0.0])

            # Perpendicular vector (rotate 90 degrees)
            perp = np.array([-direction[1], direction[0]])

            # Generate bounds
            half_width = self.track_width / 2.0
            inner_bound[i] = self.centerline[i] - perp * half_width
            outer_bound[i] = self.centerline[i] + perp * half_width

        self.inner_bound = inner_bound
        self.outer_bound = outer_bound

    def _validate_bounds(self) -> None:
        """Validate that bounds have correct shape."""
        if (
            self.inner_bound.shape != self.centerline.shape
            or self.outer_bound.shape != self.centerline.shape
        ):
            raise ValueError("Bounds must have same shape as centerline")

    @classmethod
    def create_oval_track(
        cls,
        length: float = 100.0,
        width: float = 30.0,
        track_width: float = 5.0,
        num_points: int = 200,
    ) -> "Track":
        """
        Create an oval-shaped track.

        Args:
            length: Length of the straight sections
            width: Width of the track (distance between straight sections)
            track_width: Width of the track boundaries
            num_points: Number of points along the centerline

        Returns:
            Track instance
        """
        # Generate oval centerline
        points = []
        straight_points = int(num_points * 0.3)

        # First straight
        for i in range(straight_points):
            x = -length / 2 + (i / straight_points) * length
            y = -width / 2
            points.append([x, y])

        # First curve
        curve_points = int(num_points * 0.2)
        for i in range(curve_points):
            angle = np.pi * (i / curve_points)
            x = length / 2 + (width / 2) * np.cos(angle)
            y = -width / 2 + (width / 2) * np.sin(angle)
            points.append([x, y])

        # Second straight
        for i in range(straight_points):
            x = length / 2 - (i / straight_points) * length
            y = width / 2
            points.append([x, y])

        # Second curve
        for i in range(curve_points):
            angle = np.pi + np.pi * (i / curve_points)
            x = -length / 2 + (width / 2) * np.cos(angle)
            y = width / 2 + (width / 2) * np.sin(angle)
            points.append([x, y])

        return cls(np.array(points), track_width=track_width)

    @classmethod
    def create_simple_track(
        cls,
        length: float = 100.0,
        width: float = 50.0,
        track_width: float = 5.0,
        num_points: int = 200,
    ) -> "Track":
        """
        Create a simple rounded rectangle track with gentle curves (no 180 or 360 degree turns).

        Args:
            length: Overall length of the track
            width: Overall width of the track
            track_width: Width of the track boundaries
            num_points: Number of points along the centerline

        Returns:
            Track instance
        """
        points = []

        # Create a rounded rectangle with very gentle corners
        # Use large radius for corners to avoid sharp turns
        corner_radius = min(length, width) * 0.3  # Large radius for gentle curves

        # Calculate dimensions
        straight_length = length - 2 * corner_radius
        straight_width = width - 2 * corner_radius

        # Distribute points: more on straights, fewer on gentle curves
        straight_points = int(num_points * 0.35)
        corner_points = int(num_points * 0.15)

        # Bottom straight (left to right)
        for i in range(straight_points):
            x = -length / 2 + corner_radius + (i / straight_points) * straight_length
            y = -width / 2
            points.append([x, y])

        # Bottom-right corner (gentle curve, max 90 degrees)
        for i in range(corner_points):
            angle = -np.pi / 2 + (i / corner_points) * (np.pi / 2)  # 0 to 90 degrees
            x = length / 2 - corner_radius + corner_radius * np.cos(angle)
            y = -width / 2 + corner_radius + corner_radius * np.sin(angle)
            points.append([x, y])

        # Right straight (bottom to top)
        for i in range(straight_points):
            x = length / 2
            y = -width / 2 + corner_radius + (i / straight_points) * straight_width
            points.append([x, y])

        # Top-right corner (gentle curve)
        for i in range(corner_points):
            angle = 0 + (i / corner_points) * (np.pi / 2)  # 0 to 90 degrees
            x = length / 2 - corner_radius + corner_radius * np.cos(angle)
            y = width / 2 - corner_radius + corner_radius * np.sin(angle)
            points.append([x, y])

        # Top straight (right to left)
        for i in range(straight_points):
            x = length / 2 - corner_radius - (i / straight_points) * straight_length
            y = width / 2
            points.append([x, y])

        # Top-left corner (gentle curve)
        for i in range(corner_points):
            angle = np.pi / 2 + (i / corner_points) * (np.pi / 2)  # 90 to 180 degrees
            x = -length / 2 + corner_radius + corner_radius * np.cos(angle)
            y = width / 2 - corner_radius + corner_radius * np.sin(angle)
            points.append([x, y])

        # Left straight (top to bottom)
        for i in range(straight_points):
            x = -length / 2
            y = width / 2 - corner_radius - (i / straight_points) * straight_width
            points.append([x, y])

        # Bottom-left corner (gentle curve) - close the loop
        for i in range(corner_points):
            angle = np.pi + (i / corner_points) * (np.pi / 2)  # 180 to 270 degrees
            x = -length / 2 + corner_radius + corner_radius * np.cos(angle)
            y = -width / 2 + corner_radius + corner_radius * np.sin(angle)
            points.append([x, y])

        return cls(np.array(points), track_width=track_width)

    @classmethod
    def create_figure8_track(
        cls,
        size: float = 60.0,
        track_width: float = 5.0,
        num_points: int = 300,
    ) -> "Track":
        """
        Create a figure-8 (lemniscate) shaped track.

        The track forms a smooth figure-8 pattern with two loops that cross in the middle.
        This creates a more challenging track with crossing paths and varying curvature.

        Args:
            size: Overall size of the track (controls the scale, approximately the diameter)
            track_width: Width of the track boundaries
            num_points: Number of points along the centerline

        Returns:
            Track instance
        """
        points = []

        # Use parametric equations for a lemniscate of Bernoulli (figure-8 curve)
        # Parametric form: x = a * sin(t) / (1 + cos^2(t)), y = a * sin(t) * cos(t) / (1 + cos^2(t))
        # This creates a smooth, symmetric figure-8

        # Generate parameter t from 0 to 2*pi
        t_values = np.linspace(0, 2 * np.pi, num_points)

        # Scale factor for the lemniscate (adjust to match desired size)
        a = size / 2.5  # Adjusted for better size control

        for t in t_values:
            # Lemniscate parametric equations
            # Denominator ensures smooth curve and proper figure-8 shape
            denom = 1 + np.cos(t) ** 2
            x = a * np.sin(t) / denom
            y = a * np.sin(t) * np.cos(t) / denom

            points.append([x, y])

        return cls(np.array(points), track_width=track_width)

    def get_point_at_distance(self, distance: float) -> tuple[np.ndarray, float]:
        """
        Get point on centerline at given distance along track.

        Args:
            distance: Distance along track from start

        Returns:
            Tuple of (point, heading_angle)
        """
        cumulative_distances = np.zeros(len(self.centerline))
        for i in range(1, len(self.centerline)):
            dist = np.linalg.norm(self.centerline[i] - self.centerline[i - 1])
            cumulative_distances[i] = cumulative_distances[i - 1] + dist

        total_length = cumulative_distances[-1]
        distance = distance % total_length  # Wrap around

        # Find segment
        idx_int = np.searchsorted(cumulative_distances, distance)
        idx: int = int(idx_int)
        if idx == 0:
            idx = 1
        if idx >= len(self.centerline):
            idx = len(self.centerline) - 1

        # Interpolate
        segment_dist = distance - cumulative_distances[idx - 1]
        segment_length = cumulative_distances[idx] - cumulative_distances[idx - 1]
        if segment_length > 1e-6:
            t = segment_dist / segment_length
        else:
            t = 0.0

        point = self.centerline[idx - 1] + t * (self.centerline[idx] - self.centerline[idx - 1])

        # Calculate heading
        direction = self.centerline[idx] - self.centerline[idx - 1]
        heading = np.arctan2(direction[1], direction[0])

        return point, heading

    def visualize(
        self,
        ax: "Axes",
        car_state: Optional["CarState"] = None,
        frame: str = "global",
        **kwargs: Any,
    ) -> None:
        """
        Visualize track boundaries on the given axes.

        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Axes to plot on.
        car_state : CarState, optional
            Current car state (for frame transformations).
        frame : str, default="global"
            Frame to plot in: "global" or "ego".
        **kwargs
            Additional visualization arguments:
            - show_centerline: bool, whether to show centerline
            - show_bounds: bool, whether to show track boundaries
            - centerline_color: str, color for centerline
            - bounds_color: str, color for boundaries
            - bounds_linewidth: float, linewidth for boundaries
            - horizon: float, visualization horizon (for ego frame filtering)
        """
        show_centerline = kwargs.pop("show_centerline", True)
        show_bounds = kwargs.pop("show_bounds", True)
        centerline_color = kwargs.pop("centerline_color", "b")
        bounds_color = kwargs.pop("bounds_color", "k")
        bounds_linewidth = kwargs.pop("bounds_linewidth", 2.5)
        horizon = kwargs.pop("horizon", None)

        if frame == "ego" and car_state is not None:
            # Transform track to ego frame
            if show_bounds:
                inner_bound_ego = np.array(
                    [car_state.transform_to_car_frame(point) for point in self.inner_bound]
                )
                outer_bound_ego = np.array(
                    [car_state.transform_to_car_frame(point) for point in self.outer_bound]
                )

                # Filter points within horizon for performance
                if horizon is not None:
                    mask_inner = (np.abs(inner_bound_ego[:, 0]) < horizon * 1.2) & (
                        np.abs(inner_bound_ego[:, 1]) < horizon * 1.2
                    )
                    mask_outer = (np.abs(outer_bound_ego[:, 0]) < horizon * 1.2) & (
                        np.abs(outer_bound_ego[:, 1]) < horizon * 1.2
                    )
                else:
                    mask_inner = np.ones(len(inner_bound_ego), dtype=bool)
                    mask_outer = np.ones(len(outer_bound_ego), dtype=bool)

                if np.any(mask_inner):
                    ax.plot(
                        inner_bound_ego[mask_inner, 0],
                        inner_bound_ego[mask_inner, 1],
                        "-",
                        color=bounds_color,
                        linewidth=bounds_linewidth,
                        label="Map",
                        alpha=0.9,
                        zorder=1,
                        **kwargs,
                    )
                if np.any(mask_outer):
                    ax.plot(
                        outer_bound_ego[mask_outer, 0],
                        outer_bound_ego[mask_outer, 1],
                        "-",
                        color=bounds_color,
                        linewidth=bounds_linewidth,
                        alpha=0.9,
                        zorder=1,
                        **kwargs,
                    )

            if show_centerline:
                centerline_ego = np.array(
                    [car_state.transform_to_car_frame(point) for point in self.centerline]
                )
                if horizon is not None:
                    mask = (np.abs(centerline_ego[:, 0]) < horizon * 1.2) & (
                        np.abs(centerline_ego[:, 1]) < horizon * 1.2
                    )
                else:
                    mask = np.ones(len(centerline_ego), dtype=bool)

                if np.any(mask):
                    ax.plot(
                        centerline_ego[mask, 0],
                        centerline_ego[mask, 1],
                        "--",
                        color=centerline_color,
                        linewidth=1.5,
                        alpha=0.5,
                        label="Centerline",
                        **kwargs,
                    )
        else:
            # Global frame
            if show_bounds:
                ax.plot(
                    self.inner_bound[:, 0],
                    self.inner_bound[:, 1],
                    "-",
                    color=bounds_color,
                    linewidth=bounds_linewidth,
                    label="Track Bounds",
                    **kwargs,
                )
                ax.plot(
                    self.outer_bound[:, 0],
                    self.outer_bound[:, 1],
                    "-",
                    color=bounds_color,
                    linewidth=bounds_linewidth,
                    **kwargs,
                )

            if show_centerline:
                ax.plot(
                    self.centerline[:, 0],
                    self.centerline[:, 1],
                    "--",
                    color=centerline_color,
                    linewidth=1.5,
                    alpha=0.5,
                    label="Centerline",
                    **kwargs,
                )

    def get_bounds_at_point(self, point: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """
        Get inner and outer boundary points closest to given point.

        Args:
            point: Point [x, y]

        Returns:
            Tuple of (inner_bound_point, outer_bound_point)
        """
        # Find closest centerline point
        distances = np.linalg.norm(self.centerline - point, axis=1)
        idx = np.argmin(distances)

        return self.inner_bound[idx], self.outer_bound[idx]
