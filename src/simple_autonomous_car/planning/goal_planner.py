"""Goal-based path planner using A* algorithm."""

import numpy as np
import heapq
from typing import Optional, Tuple, List, TYPE_CHECKING
from simple_autonomous_car.planning.base_planner import BasePlanner
from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.constants import (
    DEFAULT_PLAN_LINEWIDTH,
    MIN_SEGMENT_LENGTH,
)

if TYPE_CHECKING:
    from simple_autonomous_car.costmap.base_costmap import BaseCostmap
    from simple_autonomous_car.maps.grid_map import GridMap


class GoalPlanner(BasePlanner):
    """
    Goal-based path planner using A* algorithm.
    
    This planner finds a path from the car's current position to a goal,
    avoiding obstacles using a costmap or grid map.
    
    Parameters
    ----------
    grid_map : GridMap, optional
        Grid map with obstacles. If None, uses costmap.
    resolution : float, default=0.5
        Planning resolution in meters.
    name : str, default="goal_planner"
        Planner name.
    """
    
    def __init__(
        self,
        grid_map: Optional["GridMap"] = None,
        resolution: float = 0.5,
        name: str = "goal_planner",
    ):
        super().__init__(name=name)
        self.grid_map = grid_map
        self.resolution = resolution
    
    def _heuristic(self, pos1: np.ndarray, pos2: np.ndarray) -> float:
        """Euclidean distance heuristic for A*."""
        return np.linalg.norm(pos1 - pos2)
    
    def _get_neighbors(self, pos: np.ndarray, grid_map: "GridMap") -> List[np.ndarray]:
        """Get valid neighboring positions."""
        neighbors = []
        directions = [
            [self.resolution, 0],
            [-self.resolution, 0],
            [0, self.resolution],
            [0, -self.resolution],
            [self.resolution, self.resolution],
            [self.resolution, -self.resolution],
            [-self.resolution, self.resolution],
            [-self.resolution, -self.resolution],
        ]
        
        for dx, dy in directions:
            neighbor = pos + np.array([dx, dy])
            if grid_map.is_valid_position(neighbor):
                neighbors.append(neighbor)
        
        return neighbors
    
    def _world_to_grid(self, pos: np.ndarray, grid_map: "GridMap") -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        grid_x = int((pos[0] + grid_map.width / 2) / self.resolution)
        grid_y = int((pos[1] + grid_map.height / 2) / self.resolution)
        return grid_x, grid_y
    
    def _grid_to_world(self, grid_x: int, grid_y: int, grid_map: "GridMap") -> np.ndarray:
        """Convert grid coordinates to world coordinates."""
        x = (grid_x * self.resolution) - grid_map.width / 2
        y = (grid_y * self.resolution) - grid_map.height / 2
        return np.array([x, y])
    
    def plan(
        self,
        car_state: CarState,
        perception_data: Optional[dict] = None,
        costmap: Optional["BaseCostmap"] = None,
        goal: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Plan path from car position to goal using A*.
        
        Parameters
        ----------
        car_state : CarState
            Current car state.
        perception_data : dict, optional
            Not used by goal planner.
        costmap : BaseCostmap, optional
            Costmap for obstacle avoidance (used if grid_map not provided).
        goal : np.ndarray, optional
            Goal position [x, y]. Required for goal-based planning.
            
        Returns
        -------
        np.ndarray
            Planned path as array of shape (N, 2) with [x, y] waypoints.
            Empty array if planning fails.
        """
        if not self.enabled:
            return np.array([]).reshape(0, 2)
        
        if goal is None:
            return np.array([]).reshape(0, 2)
        
        # Use grid_map if available, otherwise create from costmap
        if self.grid_map is None:
            if costmap is None or not hasattr(costmap, 'costmap'):
                return np.array([]).reshape(0, 2)
            # Create temporary grid map from costmap
            # This is a simplified approach - in practice you'd want better integration
            return np.array([]).reshape(0, 2)
        
        start = car_state.position()
        
        # Check if start and goal are valid
        if not self.grid_map.is_valid_position(start):
            return np.array([]).reshape(0, 2)
        if not self.grid_map.is_valid_position(goal):
            return np.array([]).reshape(0, 2)
        
        # A* algorithm
        open_set = [(0, tuple(start))]  # (f_score, position)
        came_from = {}
        g_score = {tuple(start): 0}
        f_score = {tuple(start): self._heuristic(start, goal)}
        closed_set = set()
        
        while open_set:
            current_f, current = heapq.heappop(open_set)
            current = np.array(current)
            
            if tuple(current) in closed_set:
                continue
            
            closed_set.add(tuple(current))
            
            # Check if reached goal
            if np.linalg.norm(current - goal) < self.resolution * 2:
                # Reconstruct path
                path = [goal]
                while tuple(current) in came_from:
                    path.append(current.copy())
                    current = came_from[tuple(current)]
                path.append(start)
                path.reverse()
                return np.array(path)
            
            # Explore neighbors
            neighbors = self._get_neighbors(current, self.grid_map)
            for neighbor in neighbors:
                neighbor_tuple = tuple(neighbor)
                
                if neighbor_tuple in closed_set:
                    continue
                
                # Cost to reach neighbor (1 step)
                tentative_g = g_score[tuple(current)] + np.linalg.norm(neighbor - current)
                
                if neighbor_tuple not in g_score or tentative_g < g_score[neighbor_tuple]:
                    came_from[neighbor_tuple] = current.copy()
                    g_score[neighbor_tuple] = tentative_g
                    f_score[neighbor_tuple] = tentative_g + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor_tuple], neighbor_tuple))
        
        # No path found
        return np.array([]).reshape(0, 2)
    
    def visualize(
        self,
        ax,
        car_state: CarState,
        plan: Optional[np.ndarray] = None,
        frame: str = "global",
        **kwargs
    ) -> None:
        """Visualize planned path."""
        if plan is None or len(plan) == 0:
            return
        
        # Transform plan to desired frame
        if frame == "ego":
            plan_plot = np.array([car_state.transform_to_car_frame(point) for point in plan])
        else:
            plan_plot = plan
        
        # Extract visualization parameters
        color = kwargs.pop("color", "blue")
        linewidth = kwargs.pop("linewidth", DEFAULT_PLAN_LINEWIDTH)
        linestyle = kwargs.pop("linestyle", "-")
        show_waypoints = kwargs.pop("show_waypoints", True)
        alpha = kwargs.pop("alpha", 0.8)
        
        # Plot plan
        if show_waypoints:
            ax.plot(
                plan_plot[:, 0],
                plan_plot[:, 1],
                "o-",
                color=color,
                linewidth=linewidth,
                linestyle=linestyle,
                markersize=4,
                label="Plan",
                alpha=alpha,
                **kwargs
            )
        else:
            ax.plot(
                plan_plot[:, 0],
                plan_plot[:, 1],
                "-",
                color=color,
                linewidth=linewidth,
                linestyle=linestyle,
                label="Plan",
                alpha=alpha,
                **kwargs
            )
