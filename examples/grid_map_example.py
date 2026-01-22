"""
Example: Grid Map with Goal-Based Navigation

This example demonstrates how to use the grid map environment
with obstacles and goal-based navigation.
"""

import sys
sys.path.insert(0, '../src')

import numpy as np
import matplotlib.pyplot as plt
from simple_autonomous_car import (
    GridMap,
    Car,
    CarState,
    GoalPlanner,
    PurePursuitController,
    GridCostmap,
)
from simple_autonomous_car.visualization import plot_car

def main():
    """Run grid map navigation example."""
    print("=" * 60)
    print("Grid Map Navigation Example")
    print("=" * 60)
    
    # Create grid map with obstacles
    print("\n1. Creating grid map with obstacles...")
    grid_map = GridMap.create_random_map(
        width=50.0,
        height=50.0,
        resolution=0.5,
        num_obstacles=15,
        obstacle_size=2.0,
        seed=42,  # For reproducibility
    )
    print(f"   Created {len(grid_map.obstacles)} obstacles")
    
    # Set start and goal positions
    start_pos = np.array([-20.0, -20.0])
    goal_pos = np.array([20.0, 20.0])
    
    print(f"\n2. Start position: {start_pos}")
    print(f"   Goal position: {goal_pos}")
    
    # Create car at start position
    car = Car(
        initial_state=CarState(
            x=start_pos[0],
            y=start_pos[1],
            heading=0.0,
            velocity=5.0
        ),
        wheelbase=2.5,
        max_velocity=15.0,
        max_steering_angle=np.pi / 4,
    )
    
    # Create goal-based planner
    print("\n3. Creating goal-based planner...")
    planner = GoalPlanner(
        grid_map=grid_map,
        resolution=0.5,
    )
    
    # Create controller
    controller = PurePursuitController(
        lookahead_distance=8.0,
        target_velocity=8.0,
    )
    
    # Create costmap (for visualization)
    costmap = GridCostmap(
        width=40.0,
        height=40.0,
        resolution=0.5,
        frame="ego",
    )
    
    # Generate initial plan
    print("\n4. Generating path to goal...")
    plan = planner.plan(car.state, goal=goal_pos)
    
    if len(plan) == 0:
        print("   ERROR: No path found to goal!")
        return
    
    print(f"   Generated path with {len(plan)} waypoints")
    print(f"   Path length: {np.sum(np.linalg.norm(np.diff(plan, axis=0), axis=1)):.2f}m")
    
    # Visualize
    print("\n5. Visualizing...")
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Plot grid map
    grid_map.visualize(ax=ax, frame="global", goal=goal_pos)
    
    # Plot plan
    if len(plan) > 0:
        ax.plot(plan[:, 0], plan[:, 1], 'b-', linewidth=2.5, label='Planned Path', alpha=0.8)
        ax.plot(plan[:, 0], plan[:, 1], 'bo', markersize=4, alpha=0.6)
    
    # Plot start and goal
    ax.plot(start_pos[0], start_pos[1], 'go', markersize=12, label='Start', markeredgecolor='black', markeredgewidth=2)
    ax.plot(goal_pos[0], goal_pos[1], 'ro', markersize=12, label='Goal', markeredgecolor='black', markeredgewidth=2)
    
    # Plot car
    plot_car(car, ax=ax, frame="global", color="navy", show_heading=True)
    
    ax.set_title("Grid Map Navigation - Planned Path", fontsize=14, fontweight="bold")
    ax.legend(loc="upper right")
    ax.set_xlim(-grid_map.width/2 - 5, grid_map.width/2 + 5)
    ax.set_ylim(-grid_map.height/2 - 5, grid_map.height/2 + 5)
    
    plt.tight_layout()
    plt.show()
    
    # Simulate a few steps
    print("\n6. Simulating navigation...")
    fig, ax = plt.subplots(figsize=(12, 12))
    
    trajectory = [car.state.position().copy()]
    
    for step in range(50):
        # Replan (in real scenario, you might replan less frequently)
        if step % 10 == 0:
            plan = planner.plan(car.state, goal=goal_pos)
        
        if len(plan) == 0:
            break
        
        # Compute control
        control = controller.compute_control(car.state, plan=plan)
        
        # Update car
        car.update(0.1, acceleration=control["acceleration"], steering_rate=control["steering_rate"])
        
        trajectory.append(car.state.position().copy())
        
        # Check if goal reached
        distance_to_goal = np.linalg.norm(car.state.position() - goal_pos)
        if distance_to_goal < 2.0:
            print(f"   Goal reached at step {step}!")
            break
    
    # Plot final visualization
    grid_map.visualize(ax=ax, frame="global", goal=goal_pos)
    
    if len(plan) > 0:
        ax.plot(plan[:, 0], plan[:, 1], 'b--', linewidth=2, label='Current Plan', alpha=0.6)
    
    trajectory = np.array(trajectory)
    ax.plot(trajectory[:, 0], trajectory[:, 1], 'g-', linewidth=2.5, label='Trajectory', alpha=0.8)
    
    ax.plot(start_pos[0], start_pos[1], 'go', markersize=12, label='Start', markeredgecolor='black', markeredgewidth=2)
    ax.plot(goal_pos[0], goal_pos[1], 'ro', markersize=12, label='Goal', markeredgecolor='black', markeredgewidth=2)
    
    plot_car(car, ax=ax, frame="global", color="navy", show_heading=True)
    
    ax.set_title("Grid Map Navigation - Simulation", fontsize=14, fontweight="bold")
    ax.legend(loc="upper right")
    ax.set_xlim(-grid_map.width/2 - 5, grid_map.width/2 + 5)
    ax.set_ylim(-grid_map.height/2 - 5, grid_map.height/2 + 5)
    
    plt.tight_layout()
    plt.show()
    
    print("\n" + "=" * 60)
    print("Example complete!")
    print("=" * 60)
    print("\nTo run the full simulation:")
    print("  python -m simulations.simulation grid_map")


if __name__ == "__main__":
    main()
