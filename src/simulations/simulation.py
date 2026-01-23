"""
Unified simulation runner that loads configurations and runs simulations.

Usage:
    python -m simulations.simulation simple_track
    python -m simulations.simulation race_track
    python -m simulations.simulation --config path/to/config.py
"""

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt

from simple_autonomous_car.track.track import Track
from simple_autonomous_car.car.car import Car, CarState
from simple_autonomous_car.maps.ground_truth_map import GroundTruthMap
from simple_autonomous_car.maps.perceived_map import PerceivedMap
from simple_autonomous_car.maps.grid_map import GridMap
from simple_autonomous_car.sensors.lidar_sensor import LiDARSensor
from simple_autonomous_car.control.pure_pursuit_controller import PurePursuitController
from simple_autonomous_car.planning.track_planner import TrackPlanner
from simple_autonomous_car.planning.goal_planner import GoalPlanner
from simple_autonomous_car.maps.frenet_map import FrenetMap
from simple_autonomous_car.alerts.track_bounds_alert import TrackBoundsAlert
from simple_autonomous_car.costmap.grid_costmap import GridCostmap

# Import enhanced visualization
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))
import enhanced_visualization

# Import configs
try:
    from .configs import SIMPLE_TRACK_CONFIG, RACE_TRACK_CONFIG, GRID_MAP_CONFIG
except ImportError:
    # Fallback for direct execution
    from configs import SIMPLE_TRACK_CONFIG, RACE_TRACK_CONFIG, GRID_MAP_CONFIG


def load_config(config_name: str):
    """Load configuration by name or path."""
    # Try built-in configs first
    if config_name == "simple_track" or config_name == "simple":
        return SIMPLE_TRACK_CONFIG
    elif config_name == "race_track" or config_name == "race":
        return RACE_TRACK_CONFIG
    elif config_name == "grid_map" or config_name == "grid":
        return GRID_MAP_CONFIG
    else:
        # Try to load from file
        import importlib.util
        spec = importlib.util.spec_from_file_location("config", config_name)
        if spec is None:
            raise ValueError(f"Config '{config_name}' not found. Use 'simple_track', 'race_track', or 'grid_map', or provide a path to a config file.")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.CONFIG


def run_simulation(config: dict) -> None:
    """Run simulation with provided configuration dictionary."""
    # Determine environment type (track or grid_map)
    is_grid_map = "map" in config and config["map"].get("type") == "grid"
    
    if is_grid_map:
        # Grid map environment
        map_config = config["map"]
        grid_map = GridMap.create_random_map(
            width=map_config["width"],
            height=map_config["height"],
            resolution=map_config["resolution"],
            num_obstacles=map_config["num_obstacles"],
            obstacle_size=map_config["obstacle_size"],
            seed=map_config.get("seed", None),
        )
        track = None  # No track for grid maps
        goal = np.array(config["goal"]["position"])
        
        # Create car at initial position
        car_config = config["car"]
        car = Car(
            initial_state=CarState(
                x=car_config["initial_position"][0],
                y=car_config["initial_position"][1],
                heading=car_config["initial_heading"],
                velocity=car_config["initial_velocity"]
            ),
            wheelbase=car_config["wheelbase"],
            max_velocity=car_config["max_velocity"],
            max_steering_angle=car_config["max_steering_angle"],
        )
        
        # For grid maps, create ground truth and perceived maps
        from simple_autonomous_car.maps.grid_ground_truth_map import GridGroundTruthMap
        ground_truth_map = GridGroundTruthMap(grid_map)
        perceived_map = PerceivedMap(
            ground_truth_map,
            position_noise_std=config["perception"]["position_noise_std"],
            orientation_noise_std=config["perception"]["orientation_noise_std"],
            measurement_noise_std=config["perception"]["measurement_noise_std"],
        )
        
    else:
        # Track environment (existing code)
        track_config = config["track"]
        if track_config["type"] == "simple":
            track = Track.create_simple_track(
                length=track_config["length"],
                width=track_config["width"],
                track_width=track_config["track_width"],
                num_points=track_config["num_points"],
            )
        elif track_config["type"] == "oval":
            track = Track.create_oval_track(
                length=track_config.get("length", 100.0),
                width=track_config.get("width", 30.0),
                track_width=track_config["track_width"],
                num_points=track_config["num_points"],
            )
        elif track_config["type"] == "figure8":
            track = Track.create_figure8_track(
                size=track_config["size"],
                track_width=track_config["track_width"],
                num_points=track_config["num_points"],
            )
        else:
            raise ValueError(f"Unknown track type: {track_config['type']}")
        
        grid_map = None
        goal = None
        
        # Create car
        start_point, start_heading = track.get_point_at_distance(0.0)
        car = Car(
            initial_state=CarState(
                x=start_point[0],
                y=start_point[1],
                heading=start_heading,
                velocity=config["car"]["initial_velocity"]
            ),
            wheelbase=config["car"]["wheelbase"],
            max_velocity=config["car"]["max_velocity"],
            max_steering_angle=config["car"]["max_steering_angle"],
        )
        
        # Create maps
        ground_truth_map = GroundTruthMap(track)
        perceived_map = PerceivedMap(
            ground_truth_map,
            position_noise_std=config["perception"]["position_noise_std"],
            orientation_noise_std=config["perception"]["orientation_noise_std"],
            measurement_noise_std=config["perception"]["measurement_noise_std"],
        )

    # Create sensors (works for both track and grid map environments)
    lidar = LiDARSensor(
        ground_truth_map=ground_truth_map,
        perceived_map=perceived_map,
        max_range=config["sensor"]["max_range"],
        angular_resolution=config["sensor"]["angular_resolution"],
        point_noise_std=config["sensor"]["point_noise_std"],
        name="lidar",
    )
    car.add_sensor(lidar)

    # Create planner
    if is_grid_map:
        planner = GoalPlanner(
            grid_map=grid_map,
            resolution=config["planner"]["resolution"],
        )
    else:
        planner = TrackPlanner(
            track=track,
            lookahead_distance=config["planner"]["lookahead_distance"],
            waypoint_spacing=config["planner"]["waypoint_spacing"],
        )

    # Create controller
    controller = PurePursuitController(
        lookahead_distance=config["controller"]["lookahead_distance"],
        lookahead_gain=config["controller"]["lookahead_gain"],
        max_steering_rate=config["controller"]["max_steering_rate"],
        target_velocity=config["controller"]["target_velocity"],
        velocity_gain=config["controller"]["velocity_gain"],
    )

    # Create costmap
    costmap_config = config["costmap"]
    resolution = max(costmap_config["resolution"], 1.0)
    costmap = GridCostmap(
        width=costmap_config["width"],
        height=costmap_config["height"],
        resolution=resolution,
        inflation_radius=costmap_config["inflation_radius"],
        frame=costmap_config["frame"],
    )

    # Create alert system (only for track environments)
    alert_system = None
    if not is_grid_map:
        frenet_map = FrenetMap(track)
        alert_system = TrackBoundsAlert(
            frenet_map,
            warning_threshold=config["alerts"]["warning_threshold"],
            critical_threshold=config["alerts"]["critical_threshold"],
            lookahead_distance=config["alerts"]["lookahead_distance"],
        )

    # Setup visualization
    fig, axes, cache = enhanced_visualization.create_enhanced_visualization()

    # Calculate view bounds
    sim_config = config["simulation"]
    horizon = sim_config["horizon"]
    if is_grid_map:
        view_x_min = -grid_map.width / 2 - 5
        view_x_max = grid_map.width / 2 + 5
        view_y_min = -grid_map.height / 2 - 5
        view_y_max = grid_map.height / 2 + 5
    else:
        track_bounds = np.concatenate([track.inner_bound, track.outer_bound])
        view_x_min = np.min(track_bounds[:, 0]) - 10
        view_x_max = np.max(track_bounds[:, 0]) + 10
        view_y_min = np.min(track_bounds[:, 1]) - 10
        view_y_max = np.max(track_bounds[:, 1]) + 10
    view_bounds = (view_x_min, view_x_max, view_y_min, view_y_max)

    # Run simulation
    _run_simulation_loop(
        car, track, grid_map, goal, ground_truth_map, lidar, planner, controller, costmap,
        alert_system, sim_config["dt"], sim_config["num_steps"],
        horizon, view_bounds, axes, cache, is_grid_map,
    )


def _run_simulation_loop(
    car, track, grid_map, goal, ground_truth_map, lidar, planner, controller, costmap,
    alert_system, dt, num_steps, horizon, view_bounds, axes, cache, is_grid_map, goal_tolerance=None,
) -> None:
    """Run the simulation loop with enhanced visualization."""
    control_history = []
    goal_reached = False

    for step in range(num_steps):
        # Check if goal reached (for grid maps) - check FIRST before any planning/control
        # Only check if car is very close and nearly stopped (controller should handle smooth stopping)
        if is_grid_map and goal is not None and not goal_reached:
            distance_to_goal = np.linalg.norm(car.state.position() - goal)
            if goal_tolerance is None:
                goal_tolerance = 1.0  # Reduced tolerance - controller handles most stopping
            # Only mark as reached if very close AND nearly stopped (controller did its job)
            if distance_to_goal < goal_tolerance and abs(car.state.velocity) < 0.5:
                print(f"Step {step:3d}: Goal reached! Distance: {distance_to_goal:.2f}m, Velocity: {car.state.velocity:.2f}m/s")
                goal_reached = True
        
        # If goal reached, stop the car and skip planning/control
        if goal_reached:
            # Stop the car (brake to zero velocity, no steering)
            # Use smooth braking to avoid sudden stops
            if car.state.velocity > 0.1:
                # Brake smoothly
                control = {
                    "acceleration": max(-car.state.velocity / dt, -5.0),  # Brake (cap at -5 m/s²)
                    "steering_rate": 0.0,
                }
            else:
                # Already stopped or nearly stopped - maintain zero velocity
                control = {
                    "acceleration": -car.state.velocity / dt,  # Maintain zero
                    "steering_rate": 0.0,
                }
            
            # Store control history
            control["time"] = step * dt
            control["velocity"] = car.state.velocity
            control_history.append(control.copy())
            
            # Update car to stop
            car.update(dt, acceleration=control["acceleration"], steering_rate=0.0)
            
            # No plan needed when stopped
            plan = np.array([]).reshape(0, 2)
            perception_points = None
        else:
            # Normal operation - plan and control
            # 1. Get perception from sensors (works for both environments now)
            perception_data = car.sense_all(environment_data={"ground_truth_map": ground_truth_map})

            # 2. Update costmap from perception data (if available)
            if perception_data:
                costmap.update(perception_data, car.state)

            # 3. Generate plan (use costmap for obstacle avoidance)
            if is_grid_map:
                plan = planner.plan(car.state, perception_data=perception_data, costmap=costmap, goal=goal)
            else:
                plan = planner.plan(car.state, perception_data=perception_data, costmap=costmap)

            # 4. Compute control (pass goal and tolerance for velocity adaptation)
            control = controller.compute_control(
                car_state=car.state,
                perception_data=perception_data,
                costmap=costmap,
                plan=plan,
                goal=goal if is_grid_map else None,
                goal_tolerance=goal_tolerance if is_grid_map else None,
                dt=dt,
            )

            # Store control history
            control["time"] = step * dt
            control["velocity"] = car.state.velocity
            control_history.append(control.copy())

            # 5. Update car with control commands
            car.update(
                dt,
                acceleration=control["acceleration"],
                steering_rate=control["steering_rate"],
            )
            
            # Get perception point cloud from LiDAR sensor
            perception_points = perception_data.get("lidar") if not is_grid_map else None


        # 6. Check alerts (only for track environments)
        if not is_grid_map and alert_system is not None and perception_points is not None:
            alert_result = alert_system.check(perception_points, car.state)

            if alert_result["has_critical"]:
                print(
                    f"Step {step:3d}: CRITICAL - Track bounds violation! "
                    f"Max deviation: {alert_result['max_deviation']:.2f}m"
                )
            elif alert_result["has_warning"]:
                print(
                    f"Step {step:3d}: WARNING - Track bounds deviation! "
                    f"Max deviation: {alert_result['max_deviation']:.2f}m"
                )

        # Update visualization every 5 steps for better performance
        if step % 5 == 0:
            enhanced_visualization.update_all_views(
                axes=axes,
                track=track,
                grid_map=grid_map,
                goal=goal,
                car=car,
                plan=plan,
                perception_points=perception_points,
                costmap=costmap,
                controller=controller,
                planner=planner,
                view_bounds=view_bounds,
                horizon=horizon,
                control_history=control_history[-50:] if len(control_history) > 50 else control_history,
                cache=cache,
                is_grid_map=is_grid_map,
            )
            plt.pause(0.01)

    print("=" * 60)
    if is_grid_map and goal is not None:
        final_distance = np.linalg.norm(car.state.position() - goal)
        print(f"Final distance to goal: {final_distance:.2f}m")
        if goal_reached:
            print(f"Goal successfully reached! Car stopped at step {step}.")
        else:
            print(f"Goal not reached within {num_steps} steps.")
        print(f"Final velocity: {car.state.velocity:.2f} m/s")
    print("Simulation complete!")
    plt.show()


def main():
    """Main entry point for simulation."""
    parser = argparse.ArgumentParser(
        description="Run autonomous car simulation with specified configuration"
    )
    parser.add_argument(
        "config",
        nargs="?",
        default="simple_track",
        help="Configuration name (simple_track, race_track, grid_map) or path to config file"
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available configurations"
    )
    
    args = parser.parse_args()
    
    if args.list:
        print("Available configurations:")
        print("  - simple_track (or simple)")
        print("  - race_track (or race)")
        print("  - grid_map (or grid) - Grid map with obstacles and goal")
        print("\nOr provide a path to a custom config file.")
        return
    
    try:
        config = load_config(args.config)
        print(f"Running simulation with config: {args.config}")
        run_simulation(config)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
