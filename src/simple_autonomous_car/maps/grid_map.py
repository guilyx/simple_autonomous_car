"""Grid-based map with obstacles for goal-based navigation."""

import numpy as np
from typing import Tuple, Optional, TYPE_CHECKING
import matplotlib.pyplot as plt
import matplotlib.patches as patches

if TYPE_CHECKING:
    from simple_autonomous_car.car.car import CarState


class GridMap:
    """
    Represents a grid-based map with obstacles for goal-based navigation.
    
    This is similar to Track but for grid-based environments with obstacles.
    It provides a compatible interface for visualization and planning.
    
    Parameters
    ----------
    width : float
        Width of the map in meters.
    height : float
        Height of the map in meters.
    resolution : float, default=0.5
        Resolution in meters per cell.
    obstacles : np.ndarray, optional
        Array of obstacle positions [[x1, y1], [x2, y2], ...] or None.
    obstacle_size : float, default=1.0
        Size of each obstacle (radius or half-width).
    """
    
    def __init__(
        self,
        width: float,
        height: float,
        resolution: float = 0.5,
        obstacles: Optional[np.ndarray] = None,
        obstacle_size: float = 1.0,
    ):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.obstacle_size = obstacle_size
        
        # Store obstacles
        if obstacles is None:
            self.obstacles = np.array([]).reshape(0, 2)
        else:
            self.obstacles = np.asarray(obstacles, dtype=np.float64)
            if self.obstacles.ndim != 2 or self.obstacles.shape[1] != 2:
                raise ValueError("obstacles must be shape (N, 2)")
        
        # Create grid representation for fast collision checking
        self._create_grid()
    
    def _create_grid(self) -> None:
        """Create internal grid representation of obstacles."""
        self.width_cells = int(self.width / self.resolution)
        self.height_cells = int(self.height / self.resolution)
        self.grid = np.zeros((self.height_cells, self.width_cells), dtype=bool)
        
        # Mark obstacles in grid
        for obstacle in self.obstacles:
            x, y = obstacle
            # Convert to grid coordinates
            grid_x = int((x + self.width / 2) / self.resolution)
            grid_y = int((y + self.height / 2) / self.resolution)
            
            # Mark cells within obstacle_size
            radius_cells = int(self.obstacle_size / self.resolution)
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    gx = grid_x + dx
                    gy = grid_y + dy
                    if 0 <= gx < self.width_cells and 0 <= gy < self.height_cells:
                        dist = np.sqrt(dx**2 + dy**2) * self.resolution
                        if dist <= self.obstacle_size:
                            self.grid[gy, gx] = True
    
    def is_obstacle(self, position: np.ndarray) -> bool:
        """
        Check if a position is occupied by an obstacle.
        
        Parameters
        ----------
        position : np.ndarray
            Position [x, y] in world coordinates.
            
        Returns
        -------
        bool
            True if position is occupied, False otherwise.
        """
        # Convert to grid coordinates
        grid_x = int((position[0] + self.width / 2) / self.resolution)
        grid_y = int((position[1] + self.height / 2) / self.resolution)
        
        if 0 <= grid_x < self.width_cells and 0 <= grid_y < self.height_cells:
            return self.grid[grid_y, grid_x]
        return True  # Out of bounds is considered obstacle
    
    def is_valid_position(self, position: np.ndarray) -> bool:
        """
        Check if a position is valid (within bounds and not obstacle).
        
        Parameters
        ----------
        position : np.ndarray
            Position [x, y] in world coordinates.
            
        Returns
        -------
        bool
            True if position is valid, False otherwise.
        """
        x, y = position
        # Check bounds
        if x < -self.width/2 or x > self.width/2:
            return False
        if y < -self.height/2 or y > self.height/2:
            return False
        
        # Check obstacles
        return not self.is_obstacle(position)
    
    def get_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get map boundaries (for compatibility with Track interface).
        
        Returns
        -------
        Tuple[np.ndarray, np.ndarray]
            (inner_bound, outer_bound) - both are the same for grid maps.
        """
        bounds = np.array([
            [-self.width/2, -self.height/2],
            [self.width/2, -self.height/2],
            [self.width/2, self.height/2],
            [-self.width/2, self.height/2],
            [-self.width/2, -self.height/2],  # Close the loop
        ])
        return bounds, bounds.copy()
    
    def get_boundary_obstacles(self, spacing: Optional[float] = None) -> np.ndarray:
        """
        Get boundary points as obstacles for costmap.
        
        Samples points along the map boundaries at regular intervals.
        
        Parameters
        ----------
        spacing : float, optional
            Spacing between boundary points in meters.
            If None, uses resolution.
            
        Returns
        -------
        np.ndarray
            Array of boundary obstacle positions [[x1, y1], [x2, y2], ...]
        """
        if spacing is None:
            spacing = self.resolution
        
        boundary_points = []
        
        # Bottom edge (left to right)
        num_points_x = int(self.width / spacing) + 1
        for i in range(num_points_x):
            x = -self.width/2 + i * spacing
            if x > self.width/2:
                x = self.width/2
            boundary_points.append([x, -self.height/2])
        
        # Right edge (bottom to top)
        num_points_y = int(self.height / spacing) + 1
        for i in range(1, num_points_y):  # Skip corner already added
            y = -self.height/2 + i * spacing
            if y > self.height/2:
                y = self.height/2
            boundary_points.append([self.width/2, y])
        
        # Top edge (right to left)
        for i in range(1, num_points_x):  # Skip corner already added
            x = self.width/2 - i * spacing
            if x < -self.width/2:
                x = -self.width/2
            boundary_points.append([x, self.height/2])
        
        # Left edge (top to bottom)
        for i in range(1, num_points_y - 1):  # Skip both corners already added
            y = self.height/2 - i * spacing
            if y < -self.height/2:
                y = -self.height/2
            boundary_points.append([-self.width/2, y])
        
        return np.array(boundary_points)
    
    @classmethod
    def create_random_map(
        cls,
        width: float = 50.0,
        height: float = 50.0,
        resolution: float = 0.5,
        num_obstacles: int = 10,
        obstacle_size: float = 2.0,
        seed: Optional[int] = None,
    ) -> "GridMap":
        """
        Create a random grid map with obstacles.
        
        Parameters
        ----------
        width : float, default=50.0
            Width of map in meters.
        height : float, default=50.0
            Height of map in meters.
        resolution : float, default=0.5
            Resolution in meters per cell.
        num_obstacles : int, default=10
            Number of obstacles to place.
        obstacle_size : float, default=2.0
            Size of each obstacle.
        seed : int, optional
            Random seed for reproducibility.
            
        Returns
        -------
        GridMap
            Random grid map instance.
        """
        if seed is not None:
            np.random.seed(seed)
        
        # Generate random obstacle positions
        obstacles = []
        margin = obstacle_size + 2.0  # Keep obstacles away from edges
        
        for _ in range(num_obstacles):
            x = np.random.uniform(-width/2 + margin, width/2 - margin)
            y = np.random.uniform(-height/2 + margin, height/2 - margin)
            obstacles.append([x, y])
        
        return cls(
            width=width,
            height=height,
            resolution=resolution,
            obstacles=np.array(obstacles),
            obstacle_size=obstacle_size,
        )
    
    def visualize(
        self,
        ax,
        car_state: Optional["CarState"] = None,
        frame: str = "global",
        goal: Optional[np.ndarray] = None,
        horizon: Optional[float] = None,
        **kwargs
    ) -> None:
        """
        Visualize the grid map.
        
        Parameters
        ----------
        ax : matplotlib.axes.Axes
            Axes to plot on.
        car_state : CarState, optional
            Current car state (for frame transformation).
        frame : str, default="global"
            Frame to plot in: "global" or "ego".
        goal : np.ndarray, optional
            Goal position [x, y] to display.
        horizon : float, optional
            Maximum distance to display (for ego frame filtering).
        **kwargs
            Additional visualization arguments.
        """
        # Extract visualization parameters
        alpha = kwargs.pop("alpha", 0.3)
        obstacle_color = kwargs.pop("obstacle_color", "red")
        goal_color = kwargs.pop("goal_color", "green")
        goal_size = kwargs.pop("goal_size", 1.0)
        
        # Get car position for filtering
        car_pos = car_state.position() if car_state is not None else None
        
        # Draw map boundaries
        bounds, _ = self.get_bounds()
        if frame == "ego" and horizon is not None and car_state is not None:
            # Transform boundaries to ego frame and filter by horizon
            bounds_ego = np.array([car_state.transform_to_car_frame(b) for b in bounds])
            distances = np.linalg.norm(bounds_ego, axis=1)
            # Show boundaries that are within horizon OR form a visible segment
            # Keep at least 4 points to form a rectangle
            if len(bounds_ego) >= 4:
                # For a rectangle, show all points but clip to horizon if needed
                # Draw segments that are within horizon
                for i in range(len(bounds_ego)):
                    next_i = (i + 1) % len(bounds_ego)
                    if distances[i] <= horizon or distances[next_i] <= horizon:
                        # Draw segment if at least one endpoint is visible
                        ax.plot([bounds_ego[i, 0], bounds_ego[next_i, 0]], 
                               [bounds_ego[i, 1], bounds_ego[next_i, 1]], 
                               'k-', linewidth=2, label="Map Bounds" if i == 0 else "", **kwargs)
        else:
            ax.plot(bounds[:, 0], bounds[:, 1], 'k-', linewidth=2, label="Map Bounds", **kwargs)
        
        # Draw obstacles (filter by horizon in ego frame)
        for obstacle in self.obstacles:
            if frame == "ego" and car_state is not None:
                # Transform to ego frame
                obstacle_ego = car_state.transform_to_car_frame(obstacle)
                circle = patches.Circle(
                    obstacle_ego,
                    self.obstacle_size,
                    color=obstacle_color,
                    alpha=alpha,
                    **kwargs
                )
            else:
                circle = patches.Circle(
                    obstacle,
                    self.obstacle_size,
                    color=obstacle_color,
                    alpha=alpha,
                    **kwargs
                )
            ax.add_patch(circle)
        
        # Draw goal if provided (filter by horizon in ego frame)
        if goal is not None:
            if frame == "ego" and car_state is not None:
                goal_plot = car_state.transform_to_car_frame(goal)
                # Filter by horizon
                if horizon is not None:
                    distance = np.linalg.norm(goal_plot)
                    if distance > horizon:
                        return  # Goal outside horizon, don't draw
            else:
                goal_plot = goal
            
            ax.plot(goal_plot[0], goal_plot[1], 'o', color=goal_color, 
                   markersize=goal_size * 10, label="Goal", markeredgecolor='black', 
                   markeredgewidth=2, **kwargs)
        
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
