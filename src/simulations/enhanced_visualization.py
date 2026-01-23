"""
Enhanced visualization utilities for simulations.

This module provides comprehensive visualization of all simulation components
overlaid on two main views:
- World View (global frame)
- Ego View (car frame)

**Component-Based Visualization Architecture:**

Each component (Controller, Planner, Costmap, Track) now has its own `visualize()` method
that handles its own visualization logic. This makes visualization:
- Reusable: Any component can be visualized independently
- Maintainable: Visualization logic lives with the component
- Extensible: Easy to add new components with custom visualizations

**Usage in Simulation Loop:**

```python
# In your simulation loop:
for step in range(num_steps):
    # ... compute plan, control, etc ...
    
    # Update visualization - components handle their own plotting
    track.visualize(ax=ax_world, frame="global")
    costmap.visualize(ax=ax_world, car_state=car.state, frame="global")
    planner.visualize(ax=ax_world, car_state=car.state, plan=plan, frame="global")
    controller.visualize(ax=ax_world, car_state=car.state, plan=plan, frame="global")
    
    # Or use the orchestrator:
    enhanced_visualization.update_all_views(
        axes, track, car, plan, perception_points,
        costmap, controller, planner, view_bounds, horizon
    )
```

**Component Visualization Methods:**

- `Track.visualize(ax, car_state, frame, **kwargs)` - Plot track boundaries
- `Planner.visualize(ax, car_state, plan, frame, **kwargs)` - Plot planned path
- `Controller.visualize(ax, car_state, plan, frame, **kwargs)` - Plot lookahead, steering arc
- `Costmap.visualize(ax, car_state, frame, **kwargs)` - Plot costmap overlay
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Arc
from typing import Optional, Dict, Any

from simple_autonomous_car.track.track import Track
from simple_autonomous_car.car.car import Car
from simple_autonomous_car.perception.perception import PerceptionPoints
from simple_autonomous_car.visualization.utils import plot_perception, plot_car
from simple_autonomous_car.maps.grid_map import GridMap


def create_enhanced_visualization(
    figsize: tuple = (20, 10),
) -> tuple:
    """
    Create a two-panel visualization layout: World View and Ego View.

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure object
    axes : dict
        Dictionary of axes with keys: 'world', 'ego'
    cache : dict
        Cache for static elements to avoid redrawing
    """
    fig = plt.figure(figsize=figsize)
    gs = fig.add_gridspec(1, 2, hspace=0.3, wspace=0.3)

    axes = {
        "world": fig.add_subplot(gs[0, 0]),  # World frame view
        "ego": fig.add_subplot(gs[0, 1]),    # Ego frame view
    }
    
    # Cache for static elements (track, etc.)
    cache = {
        "world": {"track_plotted": False},
        "ego": {},
    }

    return fig, axes, cache


def _plot_costmap_overlay(
    ax,
    costmap,
    car: Car,
    frame: str = "global",
    alpha: float = 0.3,
) -> None:
    """
    Plot costmap overlay using costmap's visualize() method.
    
    This makes visualization reusable and component-specific.
    """
    if costmap is None:
        return
    
    # Use component's visualize method
    try:
        costmap.visualize(
            ax=ax,
            car_state=car.state,
            frame=frame,
            alpha=alpha,
            show_car=False,
        )
    except Exception:
        # Fallback if costmap doesn't implement visualize
        pass


def _plot_controller_overlay(
    ax,
    controller,
    car: Car,
    plan: np.ndarray,
    frame: str = "global",
) -> None:
    """
    Plot controller visualization overlay using controller's visualize() method.
    
    This makes visualization reusable and component-specific.
    """
    if controller is None or len(plan) == 0:
        return
    
    # Use component's visualize method
    try:
        controller.visualize(
            ax=ax,
            car_state=car.state,
            plan=plan,
            frame=frame,
            wheelbase=getattr(car, "wheelbase", 2.5),
        )
    except Exception as e:
        # Log error for debugging but don't crash
        import warnings
        warnings.warn(f"Controller visualization failed: {e}", RuntimeWarning)


def update_world_view(
    ax,
    track: Optional[Track],
    grid_map,
    goal: Optional[np.ndarray],
    car: Car,
    plan: np.ndarray,
    perception_points: Optional[PerceptionPoints],
    costmap,
    controller,
    planner,
    view_bounds: tuple,
    cache: Optional[dict] = None,
    is_grid_map: bool = False,
) -> None:
    """Update world frame visualization with all components overlaid."""
    ax.clear()
    ax.set_title("World Frame View", fontsize=12, fontweight="bold")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # Set view bounds
    view_x_min, view_x_max, view_y_min, view_y_max = view_bounds
    ax.set_xlim(view_x_min, view_x_max)
    ax.set_ylim(view_y_min, view_y_max)

    # 1. Plot environment (track or grid_map)
    if is_grid_map and grid_map is not None:
        grid_map.visualize(ax=ax, frame="global", goal=goal)
    elif track is not None:
        track.visualize(ax=ax, frame="global", show_centerline=True, show_bounds=True)

    # 2. Plot costmap overlay (re-enabled in world view) - plot early so it's behind other elements
    _plot_costmap_overlay(ax, costmap, car, frame="global", alpha=0.4)

    # 3. Plot plan using planner's visualize method
    if planner is not None and len(plan) > 0:
        planner.visualize(
            ax=ax,
            car_state=car.state,
            plan=plan,
            frame="global",
            color="green",
            linewidth=2.5,
            linestyle="--",
            show_waypoints=True,
        )

    # 4. Plot controller overlay (lookahead, steering arc)
    _plot_controller_overlay(ax, controller, car, plan, frame="global")

    # 5. Plot perception
    if perception_points is not None:
        plot_perception(
            perception_points,
            car.state,
            ax=ax,
            frame="global",
            color="crimson",
            label="Perception",
            alpha=0.7,
        )

    # 6. Plot car (top layer)
    plot_car(car, ax=ax, frame="global", color="navy", show_heading=True)

    ax.legend(loc="upper right", fontsize=8)


def update_ego_view(
    ax,
    car: Car,
    track: Optional[Track],
    grid_map,
    goal: Optional[np.ndarray],
    perception_points: Optional[PerceptionPoints],
    plan: np.ndarray,
    costmap,
    controller,
    planner,
    horizon: float,
    cache: Optional[dict] = None,
    is_grid_map: bool = False,
) -> None:
    """Update ego frame visualization with all components overlaid."""
    # Ego view has no static elements, clear is fine but optimize
    ax.clear()
    ax.set_title("Ego Frame View", fontsize=12, fontweight="bold")
    ax.set_xlabel("Forward (m)")
    ax.set_ylabel("Left (m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    # Set view bounds
    ax.set_xlim(-horizon * 1.1, horizon * 1.1)
    ax.set_ylim(-horizon * 1.1, horizon * 1.1)

    # Transform plan to car frame
    if len(plan) > 0:
        plan_ego = np.array([car.state.transform_to_car_frame(point) for point in plan])
    else:
        plan_ego = np.array([])

    # 0. Plot environment in ego frame
    if is_grid_map and grid_map is not None:
        grid_map.visualize(ax=ax, car_state=car.state, frame="ego", goal=goal, horizon=horizon)
    elif track is not None:
        track.visualize(
            ax=ax,
            car_state=car.state,
            frame="ego",
            show_centerline=False,
            show_bounds=True,
        horizon=horizon,
    )

    # 1. Plot costmap overlay in ego frame (if enabled) - plot early so it's behind other elements
    _plot_costmap_overlay(ax, costmap, car, frame="ego", alpha=0.6)

    # 2. Plot plan in car frame using planner's visualize method
    if planner is not None and len(plan) > 0:
        planner.visualize(
            ax=ax,
            car_state=car.state,
            plan=plan,
            frame="ego",
            color="green",
            linewidth=2.5,
            linestyle="--",
            show_waypoints=False,
        )

    # 3. Plot controller overlay (lookahead, steering arc) in ego frame
    _plot_controller_overlay(ax, controller, car, plan, frame="ego")

    # 4. Plot perception in car frame
    if perception_points is not None:
        plot_perception(
            perception_points,
            car.state,
            ax=ax,
            frame="ego",
            color="crimson",
            label="Perception",
            alpha=0.7,
        )

    # 5. Car at origin (top layer)
    car_corners_car = np.array([[-2, -0.9], [-2, 0.9], [2, 0.9], [2, -0.9]])
    car_poly = Polygon(car_corners_car, closed=True, color="navy", alpha=0.8, label="Car", zorder=10)
    ax.add_patch(car_poly)
    
    # Heading arrow (forward direction)
    ax.arrow(
        0, 0,
        3.0, 0,
        head_width=1.2,
        head_length=1.0,
        fc="navy",
        ec="navy",
        zorder=10,
    )

    ax.legend(loc="upper right", fontsize=8)


def update_all_views(
    axes: dict,
    track: Optional[Track],
    car: Car,
    plan: np.ndarray,
    perception_points: Optional[PerceptionPoints],
    costmap,
    controller,
    planner,
    view_bounds: tuple,
    horizon: float,
    control_history: Optional[list] = None,
    cache: Optional[dict] = None,
    grid_map=None,
    goal: Optional[np.ndarray] = None,
    is_grid_map: bool = False,
) -> None:
    """
    Update all visualization panels.

    Parameters
    ----------
    axes : dict
        Dictionary of axes from create_enhanced_visualization with keys: 'world', 'ego'
    track : Track
        Track object
    car : Car
        Car object
    plan : np.ndarray
        Planned path
    perception_points : PerceptionPoints, optional
        Perception data
    costmap : BaseCostmap
        Costmap object
    controller : BaseController
        Controller object
    planner : BasePlanner
        Planner object
    view_bounds : tuple
        (view_x_min, view_x_max, view_y_min, view_y_max)
    horizon : float
        Visualization horizon for ego frame
    control_history : list, optional
        History of control commands (not used in overlay visualization)
    """
    update_world_view(
        axes["world"],
        track,
        grid_map,
        goal,
        car,
        plan,
        perception_points,
        costmap,
        controller,
        planner,
        view_bounds,
        cache.get("world") if cache else None,
        is_grid_map,
    )
    update_ego_view(
        axes["ego"],
        car,
        track,
        grid_map,
        goal,
        perception_points,
        plan,
        costmap,
        controller,
        planner,
        horizon,
        cache.get("ego") if cache else None,
        is_grid_map,
    )

    # Only call tight_layout occasionally, not every frame
    # plt.tight_layout()  # Commented out for performance
