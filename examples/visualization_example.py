"""Example demonstrating easy-to-use visualization functions."""

import sys

import matplotlib.pyplot as plt

sys.path.insert(0, "../src")

from simple_autonomous_car import (
    Car,
    CarState,
    FrenetMap,
    GroundTruthMap,
    LiDARSensor,
    PerceivedMap,
    PurePursuitController,
    Track,
    TrackPlanner,
)
from simple_autonomous_car.visualization import (
    plot_car,
    plot_perception,
    plot_pure_pursuit_state,
)


def example_basic_visualizations():
    """Example of basic visualization functions."""
    print("=" * 60)
    print("Example: Basic Visualizations")
    print("=" * 60)

    # Create track
    track = Track.create_simple_track(length=80.0, width=40.0, track_width=5.0)

    # Create car
    start_point, start_heading = track.get_point_at_distance(0.0)
    car = Car(
        initial_state=CarState(
            x=start_point[0], y=start_point[1], heading=start_heading, velocity=8.0
        )
    )

    # Create maps and sensor
    ground_truth_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(ground_truth_map)
    lidar = LiDARSensor(ground_truth_map, perceived_map, max_range=40.0, name="lidar")
    car.add_sensor(lidar)

    # Create planner and controller
    planner = TrackPlanner(track, lookahead_distance=50.0, waypoint_spacing=2.0)
    controller = PurePursuitController(lookahead_distance=10.0, target_velocity=8.0)

    # Generate plan and get perception
    plan = planner.plan(car.state)
    perception_data = car.sense_all(environment_data={"ground_truth_map": ground_truth_map})
    perception_points = perception_data.get("lidar")

    # Example 1: Plot just the track
    fig, ax = plt.subplots(figsize=(10, 8))
    track.visualize(ax=ax, frame="global")
    plt.title("Example 1: Track Only")
    plt.show()

    # Example 2: Plot track with plan
    fig, ax = plt.subplots(figsize=(10, 8))
    track.visualize(ax=ax, frame="global")
    planner.visualize(
        ax=ax, car_state=car.state, plan=plan, frame="global", color="green", label="Plan"
    )
    plt.title("Example 2: Track + Plan")
    plt.legend()
    plt.show()

    # Example 3: Plot track with car and perception
    fig, ax = plt.subplots(figsize=(10, 8))
    track.visualize(ax=ax, frame="global")
    if perception_points is not None:
        plot_perception(perception_points, car.state, ax=ax, frame="global")
    plot_car(car, ax=ax, frame="global", show_heading=True)
    plt.title("Example 3: Track + Car + Perception")
    plt.legend()
    plt.show()

    # Example 4: Plot controller state with curvature
    fig, ax = plt.subplots(figsize=(12, 10))
    track.visualize(ax=ax, frame="global")
    controller.visualize(
        ax=ax,
        car_state=car.state,
        plan=plan,
        frame="global",
        wheelbase=getattr(car, "wheelbase", 2.5),
    )
    plot_car(car, ax=ax, frame="global")
    plt.title("Example 4: Controller State (with curvature)")
    plt.legend()
    plt.show()

    # Example 5: Detailed Pure Pursuit visualization
    if isinstance(controller, PurePursuitController):
        fig, ax = plt.subplots(figsize=(12, 10))
        track.visualize(ax=ax, frame="global")
        plot_pure_pursuit_state(
            controller,
            car.state,
            plan,
            ax=ax,
            show_curvature=True,
            show_steering_circle=True,
            show_lookahead=True,
        )
        plt.title("Example 5: Pure Pursuit Controller (with steering circle)")
        plt.legend()
        plt.show()

    # Example 6: Plot everything together
    FrenetMap(track)
    fig, ax = plt.subplots(figsize=(14, 12))
    track.visualize(ax=ax, frame="global")
    planner.visualize(ax=ax, car_state=car.state, plan=plan, frame="global")
    if perception_points is not None:
        plot_perception(perception_points, car.state, ax=ax, frame="global")
    controller.visualize(
        ax=ax,
        car_state=car.state,
        plan=plan,
        frame="global",
        wheelbase=getattr(car, "wheelbase", 2.5),
    )
    plot_car(car, ax=ax, frame="global", show_heading=True)
    plt.title("Example 6: Everything Together")
    plt.show()

    print("✓ All visualization examples completed!")


def example_controller_visualization():
    """Example showing controller-specific visualizations."""
    print("\n" + "=" * 60)
    print("Example: Controller Visualization")
    print("=" * 60)

    track = Track.create_simple_track(length=80.0, width=40.0, track_width=5.0)
    start_point, start_heading = track.get_point_at_distance(0.0)
    car = Car(
        initial_state=CarState(
            x=start_point[0], y=start_point[1], heading=start_heading, velocity=8.0
        )
    )

    planner = TrackPlanner(track, lookahead_distance=50.0, waypoint_spacing=2.0)
    controller = PurePursuitController(lookahead_distance=10.0, target_velocity=8.0)

    plan = planner.plan(car.state)

    # Show detailed Pure Pursuit visualization
    fig, ax = plt.subplots(figsize=(14, 12))
    track.visualize(ax=ax, frame="global")
    plot_pure_pursuit_state(
        controller,
        car.state,
        plan,
        ax=ax,
        show_curvature=True,
        show_steering_circle=True,
        show_lookahead=True,
    )
    plt.title("Pure Pursuit Controller - Detailed View")
    plt.legend()
    plt.show()

    print("✓ Controller visualization example completed!")


if __name__ == "__main__":
    # Run examples
    example_basic_visualizations()
    example_controller_visualization()
