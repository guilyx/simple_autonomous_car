"""Grid-based costmap implementation."""

import numpy as np
from typing import Dict, Optional, Tuple, Union

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.perception.perception import PerceptionPoints
from simple_autonomous_car.costmap.base_costmap import BaseCostmap
from simple_autonomous_car.costmap.inflation import inflate_obstacles
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from simple_autonomous_car.footprint.base_footprint import BaseFootprint
from simple_autonomous_car.constants import (
    DEFAULT_COSTMAP_WIDTH,
    DEFAULT_COSTMAP_HEIGHT,
    DEFAULT_COSTMAP_RESOLUTION,
    DEFAULT_INFLATION_RADIUS,
    COST_FREE,
    COST_OCCUPIED,
    COST_THRESHOLD,
    DEFAULT_COSTMAP_ALPHA,
    DEFAULT_COSTMAP_ZORDER,
)


class GridCostmap(BaseCostmap):
    """
    Grid-based costmap for obstacle representation.

    This costmap represents the environment as a 2D grid where each cell
    has a cost value (0.0 = free, 1.0 = occupied, 0.0-1.0 = inflated).

    Parameters
    ----------
    width : float
        Width of costmap in meters.
    height : float
        Height of costmap in meters.
    resolution : float, default=0.5
        Resolution in meters per cell.
    inflation_radius : float, optional
        Inflation radius for obstacles in meters.
        If None and footprint provided, uses footprint bounding radius + padding.
        If None and no footprint, uses DEFAULT_INFLATION_RADIUS.
    footprint : BaseFootprint, optional
        Vehicle footprint for accurate collision checking.
        If provided, inflation_radius is calculated from footprint.
    footprint_padding : float, default=0.0
        Additional safety padding around footprint in meters.
    origin : np.ndarray, optional
        Origin position [x, y] in global frame. If None, uses car position.
    frame : str, default="ego"
        Frame for costmap: "ego" (car-centered) or "global".
    enabled : bool, default=True
        Whether costmap is enabled.

    Attributes
    ----------
    costmap : np.ndarray
        2D array of cost values.
    origin : np.ndarray
        Origin position in global frame.
    width_pixels : int
        Width in pixels.
    height_pixels : int
        Height in pixels.
    """

    def __init__(
        self,
        width: float = DEFAULT_COSTMAP_WIDTH,
        height: float = DEFAULT_COSTMAP_HEIGHT,
        resolution: float = DEFAULT_COSTMAP_RESOLUTION,
        inflation_radius: Optional[float] = None,
        footprint: Optional["BaseFootprint"] = None,
        footprint_padding: float = 0.0,
        origin: Optional[np.ndarray] = None,
        frame: str = "ego",
        enabled: bool = True,
    ):
        # Determine inflation radius: use footprint if provided, otherwise use provided or default
        self.footprint = footprint
        self.footprint_padding = footprint_padding
        
        if inflation_radius is None:
            if footprint is not None:
                # Use footprint bounding radius + padding
                inflation_radius = footprint.get_inflation_radius(padding=footprint_padding)
            else:
                # Use default
                inflation_radius = DEFAULT_INFLATION_RADIUS
        
        super().__init__(resolution=resolution, inflation_radius=inflation_radius, enabled=enabled)
        self.width = width
        self.height = height
        self.width_pixels = int(width / resolution)
        self.height_pixels = int(height / resolution)
        self.frame = frame
        self.origin = origin if origin is not None else np.array([0.0, 0.0])

        # Initialize costmap (0.0 = free space)
        self.costmap = np.zeros((self.height_pixels, self.width_pixels), dtype=np.float32)

    def _world_to_grid(self, position: np.ndarray, car_state: Optional[CarState] = None) -> Tuple[int, int]:
        """
        Convert world position to grid coordinates.

        Parameters
        ----------
        position : np.ndarray
            Position in world frame [x, y].
        car_state : CarState, optional
            Car state for ego frame conversion.

        Returns
        -------
        Tuple[int, int]
            Grid coordinates (row, col).
        """
        if self.frame == "ego" and car_state is not None:
            # Convert to ego frame
            position_ego = car_state.transform_to_car_frame(position)
            x, y = position_ego[0], position_ego[1]
        else:
            # Global frame
            x = position[0] - self.origin[0]
            y = position[1] - self.origin[1]

        col = int((x + self.width / 2) / self.resolution)
        row = int((y + self.height / 2) / self.resolution)

        return row, col

    def _grid_to_world(self, row: int, col: int, car_state: Optional[CarState] = None) -> np.ndarray:
        """
        Convert grid coordinates to world position.

        Parameters
        ----------
        row : int
            Grid row.
        col : int
            Grid column.
        car_state : CarState, optional
            Car state for ego frame conversion.

        Returns
        -------
        np.ndarray
            Position in world frame [x, y].
        """
        x = (col * self.resolution) - (self.width / 2)
        y = (row * self.resolution) - (self.height / 2)

        if self.frame == "ego" and car_state is not None:
            # Convert from ego to global
            position_ego = np.array([x, y])
            position = car_state.transform_to_world_frame(position_ego)
        else:
            position = np.array([x + self.origin[0], y + self.origin[1]])

        return position

    def update(
        self,
        perception_data: Optional[Dict[str, PerceptionPoints]] = None,
        car_state: Optional[CarState] = None,
        static_obstacles: Optional[np.ndarray] = None,
        frame: str = "global",
    ) -> None:
        """
        Update costmap from perception data or static obstacles.

        Parameters
        ----------
        perception_data : dict, optional
            Dictionary of perception data from sensors.
            If None and static_obstacles provided, uses static obstacles.
        car_state : CarState, optional
            Current car state (required if using perception_data or ego frame).
        static_obstacles : np.ndarray, optional
            Static obstacles as array of shape (N, 2) with [x, y] positions in global frame.
            Used when perception_data is None (e.g., for static maps).
        frame : str, default="global"
            Frame to use (not used if costmap has fixed frame).
        """
        if not self.enabled:
            return

        if car_state is None and (self.frame == "ego" or perception_data is not None):
            raise ValueError("car_state is required for ego frame or perception_data updates")

        # Update origin if ego frame (but keep grid size fixed)
        if self.frame == "ego" and car_state is not None:
            # Store new origin but grid dimensions stay the same
            # The grid is always centered at origin in ego frame
            self.origin = car_state.position().copy()

        # Reset costmap (grid size is fixed, only data changes)
        self.costmap.fill(COST_FREE)

        # Mark obstacles from static obstacles (if provided)
        if static_obstacles is not None and len(static_obstacles) > 0:
            for obstacle_pos in static_obstacles:
                row, col = self._world_to_grid(obstacle_pos, car_state)
                if 0 <= row < self.height_pixels and 0 <= col < self.width_pixels:
                    self.costmap[row, col] = COST_OCCUPIED

        # Mark obstacles from perception data (if provided)
        if perception_data is not None:
            for sensor_name, perception_points in perception_data.items():
                if perception_points is None or len(perception_points.points) == 0:
                    continue

                # Convert to global frame if needed
                if perception_points.frame != "global":
                    points_global = perception_points.to_global_frame(car_state).points
                else:
                    points_global = perception_points.points

                # Mark obstacles in costmap
                for point in points_global:
                    row, col = self._world_to_grid(point, car_state)
                    if 0 <= row < self.height_pixels and 0 <= col < self.width_pixels:
                        self.costmap[row, col] = COST_OCCUPIED

        # Apply inflation (in-place to preserve array shape)
        if self.inflation_radius > 0:
            inflated = inflate_obstacles(
                self.costmap, self.inflation_radius, self.resolution, method="linear"
            )
            # Ensure we keep the same array shape (inflation should not change dimensions)
            if inflated.shape == self.costmap.shape:
                self.costmap[:] = inflated
            else:
                # Fallback: if shape changed, resize to original
                self.costmap.fill(0.0)
                min_rows = min(self.costmap.shape[0], inflated.shape[0])
                min_cols = min(self.costmap.shape[1], inflated.shape[1])
                self.costmap[:min_rows, :min_cols] = inflated[:min_rows, :min_cols]

    def get_cost(self, position: np.ndarray, frame: str = "global", car_state: Optional[CarState] = None) -> float:
        """
        Get cost at a specific position.

        Parameters
        ----------
        position : np.ndarray
            Position as [x, y] array.
        frame : str, default="global"
            Frame of the position.
        car_state : CarState, optional
            Car state for frame conversion.

        Returns
        -------
        float
            Cost value (0.0 = free, 1.0 = occupied).
        """
        if not self.enabled:
            return COST_FREE

        # Convert to global if needed
        if frame != "global" and car_state is not None:
            if frame == "ego":
                position = car_state.transform_to_world_frame(position)

        row, col = self._world_to_grid(position, car_state)
        if 0 <= row < self.height_pixels and 0 <= col < self.width_pixels:
            return float(self.costmap[row, col])
        else:
            return COST_FREE  # Outside costmap = free

    def get_cost_region(
        self,
        center: np.ndarray,
        size: float,
        frame: str = "global",
        car_state: Optional[CarState] = None,
    ) -> np.ndarray:
        """
        Get cost values in a region.

        Parameters
        ----------
        center : np.ndarray
            Center position as [x, y].
        size : float
            Size of the region in meters.
        frame : str, default="global"
            Frame of the center position.
        car_state : CarState, optional
            Car state for frame conversion.

        Returns
        -------
        np.ndarray
            2D array of cost values.
        """
        if not self.enabled:
            return np.zeros((1, 1))

        # Convert to global if needed
        if frame != "global" and car_state is not None:
            if frame == "ego":
                center = car_state.transform_to_world_frame(center)

        center_row, center_col = self._world_to_grid(center, car_state)
        half_size_pixels = int(size / (2 * self.resolution))

        row_start = max(0, center_row - half_size_pixels)
        row_end = min(self.height_pixels, center_row + half_size_pixels)
        col_start = max(0, center_col - half_size_pixels)
        col_end = min(self.width_pixels, center_col + half_size_pixels)

        return self.costmap[row_start:row_end, col_start:col_end].copy()

    def get_visualization_data(
        self,
        car_state: Optional[CarState] = None,
        **kwargs
    ) -> Dict:
        """
        Get visualization data for GridCostmap.
        
        Returns costmap array, bounds, origin, and frame information.
        
        Parameters
        ----------
        car_state : CarState, optional
            Current car state (for frame transformations).
        **kwargs
            Additional arguments.
        
        Returns
        -------
        Dict
            Dictionary with keys:
            - "costmap": np.ndarray, 2D costmap array
            - "width": float, width in meters
            - "height": float, height in meters
            - "resolution": float, resolution in meters per cell
            - "origin": np.ndarray, origin position [x, y]
            - "frame": str, frame type ("ego" or "global")
            - "enabled": bool, whether costmap is enabled
        """
        return {
            "costmap": self.get_full_costmap(),
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin": self.origin.copy(),
            "frame": self.frame,
            "enabled": self.enabled,
            "width_pixels": self.width_pixels,
            "height_pixels": self.height_pixels,
        }

    def get_full_costmap(self) -> np.ndarray:
        """
        Get full costmap array.

        Returns
        -------
        np.ndarray
            Full 2D costmap array.
        """
        return self.costmap.copy()
    
    def visualize(
        self,
        ax,
        car_state: Optional[CarState] = None,
        frame: str = "global",
        **kwargs
    ) -> None:
        """
        Visualize costmap on the given axes.
        
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
            - alpha: float, transparency of costmap
            - cmap: str, colormap name
            - show_car: bool, whether to show car position
        """
        if not self.enabled:
            return
        
        import numpy as np
        import matplotlib.pyplot as plt
        
        # Extract visualization parameters
        alpha = kwargs.pop("alpha", DEFAULT_COSTMAP_ALPHA)
        cmap = kwargs.pop("cmap", "RdYlGn_r")
        show_car = kwargs.pop("show_car", False)
        
        # Get costmap data
        costmap_data = self.get_full_costmap()
        
        # Mask out empty space (zero-cost cells) to save computation
        # Only mask if there's actual data to show
        if np.any(costmap_data > COST_FREE):
            costmap_data = np.ma.masked_where(costmap_data == COST_FREE, costmap_data)
        else:
            # Costmap is empty, nothing to visualize
            return
        
        # Use imshow for optimal performance
        if frame == "ego" and self.frame == "ego" and car_state is not None:
            # Costmap is in ego frame, plot in ego frame
            origin_x = -self.width / 2
            origin_y = -self.height / 2
            extent = [
                origin_x,
                origin_x + self.width,
                origin_y,
                origin_y + self.height,
            ]
            ax.imshow(
                costmap_data,
                extent=extent,
                origin="lower",
                cmap=cmap,
                vmin=COST_FREE,
                vmax=COST_OCCUPIED,
                interpolation="nearest",
                alpha=alpha,
                zorder=DEFAULT_COSTMAP_ZORDER,
                **kwargs
            )
        elif self.frame == "ego" and car_state is not None and frame == "global":
            # Costmap is in ego frame, but we want to plot in global frame
            # Use matplotlib transform instead of rotating array data for smoother visualization
            # This avoids jerky artifacts from rotating the array every frame
            car_pos = car_state.position()
            car_heading = car_state.heading
            
            from matplotlib.transforms import Affine2D
            
            # Use transform to rotate/translate at rendering time (smooth, no array manipulation)
            # This is the key difference: ego frame doesn't rotate array, it uses transforms
            # Transform order: rotate around origin (0,0) first, then translate to car position
            # This correctly transforms ego frame (centered at origin) to global frame
            trans = (Affine2D()
                    .rotate(car_heading)  # Rotate around origin (0,0) in ego frame
                    .translate(car_pos[0], car_pos[1])  # Then translate to car position
                    + ax.transData)
            
            # Extent in ego frame (centered at origin)
            extent = [
                -self.width / 2,
                self.width / 2,
                -self.height / 2,
                self.height / 2,
            ]
            
            # Plot with transform - matplotlib handles rotation smoothly at render time
            # No array rotation = no jerky artifacts
            ax.imshow(
                costmap_data,
                extent=extent,
                origin="lower",
                cmap=cmap,
                vmin=COST_FREE,
                vmax=COST_OCCUPIED,
                interpolation="nearest",
                alpha=alpha,
                zorder=DEFAULT_COSTMAP_ZORDER,
                transform=trans,
                **kwargs
            )
        else:
            # Global frame costmap in global frame
            origin = self.origin
            extent = [
                origin[0],
                origin[0] + self.width,
                origin[1],
                origin[1] + self.height,
            ]
            ax.imshow(
                costmap_data,
                extent=extent,
                origin="lower",
                cmap=cmap,
                vmin=COST_FREE,
                vmax=COST_OCCUPIED,
                interpolation="nearest",
                alpha=alpha,
                zorder=DEFAULT_COSTMAP_ZORDER,
                **kwargs
            )
        
        # Show car position if requested
        if show_car and car_state is not None:
            car_pos = car_state.position()
            if frame == "global":
                ax.plot(car_pos[0], car_pos[1], "bo", markersize=8, label="Car")
            else:
                ax.plot(0, 0, "bo", markersize=8, label="Car")
