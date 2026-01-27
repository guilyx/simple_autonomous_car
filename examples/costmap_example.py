"""Example demonstrating costmap usage with planners and controllers."""

import sys

import matplotlib.pyplot as plt

sys.path.insert(0, "../src")

from simple_autonomous_car import (
    Car,
    CarState,
    GridCostmap,
    GroundTruthMap,
    LiDARSensor,
    PerceivedMap,
    PurePursuitController,
    Track,
    TrackPlanner,
)
from simple_autonomous_car.visualization import plot_car


def example_costmap_basic():
    """Basic costmap example."""
    print("=" * 60)
    print("Example: Basic Costmap")
    print("=" * 60)

    # Create track and car
    track = Track.create_simple_track(length=80.0, width=40.0, track_width=5.0)
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

    # Create costmap
    costmap = GridCostmap(
        width=60.0,
        height=60.0,
        resolution=0.5,
        inflation_radius=2.0,
        frame="ego",  # Ego frame (moves with car)
    )

    # Get perception data
    perception_data = car.sense_all(environment_data={"ground_truth_map": ground_truth_map})

    # Update costmap
    costmap.update(perception_data, car.state)

    # Visualize
    fig, ax = plt.subplots(figsize=(12, 10))
    track.visualize(ax=ax, frame="global")
    costmap.visualize(ax=ax, car_state=car.state, frame="global")
    plot_car(car, ax=ax, show_heading=True)
    plt.title("Costmap from Perception Data")
    plt.legend()
    plt.show()

    print("✓ Basic costmap example completed!")


def example_costmap_with_planner():
    """Costmap with planner example."""
    print("\n" + "=" * 60)
    print("Example: Costmap with Planner")
    print("=" * 60)

    # Setup
    track = Track.create_simple_track(length=80.0, width=40.0, track_width=5.0)
    start_point, start_heading = track.get_point_at_distance(0.0)
    car = Car(
        initial_state=CarState(
            x=start_point[0], y=start_point[1], heading=start_heading, velocity=8.0
        )
    )

    ground_truth_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(ground_truth_map)
    lidar = LiDARSensor(ground_truth_map, perceived_map, max_range=40.0, name="lidar")
    car.add_sensor(lidar)

    # Create costmap
    costmap = GridCostmap(
        width=60.0,
        height=60.0,
        resolution=0.5,
        inflation_radius=2.0,
        frame="ego",
    )

    # Create planner
    planner = TrackPlanner(track, lookahead_distance=50.0, waypoint_spacing=2.0)

    # Get perception and update costmap
    perception_data = car.sense_all(environment_data={"ground_truth_map": ground_truth_map})
    costmap.update(perception_data, car.state)

    # Generate plan (planner can use costmap for obstacle avoidance)
    plan = planner.plan(car.state, perception_data=perception_data, costmap=costmap)

    # Visualize
    fig, ax = plt.subplots(figsize=(12, 10))
    track.visualize(ax=ax, frame="global")
    costmap.visualize(ax=ax, car_state=car.state, frame="global")
    planner.visualize(
        ax=ax, car_state=car.state, plan=plan, frame="global", color="green", label="Plan"
    )
    plot_car(car, ax=ax, show_heading=True)
    plt.title("Costmap with Planned Path")
    plt.legend()
    plt.show()

    print("✓ Costmap with planner example completed!")


def example_costmap_with_controller():
    """Costmap with controller example."""
    print("\n" + "=" * 60)
    print("Example: Costmap with Controller")
    print("=" * 60)

    # Setup
    track = Track.create_simple_track(length=80.0, width=40.0, track_width=5.0)
    start_point, start_heading = track.get_point_at_distance(0.0)
    car = Car(
        initial_state=CarState(
            x=start_point[0], y=start_point[1], heading=start_heading, velocity=8.0
        )
    )

    ground_truth_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(ground_truth_map)
    lidar = LiDARSensor(ground_truth_map, perceived_map, max_range=40.0, name="lidar")
    car.add_sensor(lidar)

    # Create costmap
    costmap = GridCostmap(
        width=60.0,
        height=60.0,
        resolution=0.5,
        inflation_radius=2.0,
        frame="ego",
    )

    # Create planner and controller
    planner = TrackPlanner(track, lookahead_distance=50.0, waypoint_spacing=2.0)
    controller = PurePursuitController(lookahead_distance=10.0, target_velocity=8.0)

    # Simulation loop
    dt = 0.1
    for step in range(10):
        # Get perception
        perception_data = car.sense_all(environment_data={"ground_truth_map": ground_truth_map})

        # Update costmap
        costmap.update(perception_data, car.state)

        # Generate plan (with costmap)
        plan = planner.plan(car.state, perception_data=perception_data, costmap=costmap)

        # Compute control (with costmap - controller adjusts velocity based on cost)
        control = controller.compute_control(
            car.state,
            perception_data=perception_data,
            costmap=costmap,
            plan=plan,
            dt=dt,
        )

        # Update car
        car.update(dt, acceleration=control["acceleration"], steering_rate=control["steering_rate"])

        if step % 5 == 0:
            print(
                f"Step {step}: Velocity={car.state.velocity:.2f} m/s, "
                f"Acceleration={control['acceleration']:.2f} m/s²"
            )

    # Final visualization
    perception_data = car.sense_all(environment_data={"ground_truth_map": ground_truth_map})
    costmap.update(perception_data, car.state)
    plan = planner.plan(car.state, perception_data=perception_data, costmap=costmap)

    fig, ax = plt.subplots(figsize=(12, 10))
    track.visualize(ax=ax, frame="global")
    costmap.visualize(ax=ax, car_state=car.state, frame="global")
    planner.visualize(
        ax=ax, car_state=car.state, plan=plan, frame="global", color="green", label="Plan"
    )
    plot_car(car, ax=ax, show_heading=True)
    plt.title("Costmap with Controller (Final State)")
    plt.legend()
    plt.show()

    print("✓ Costmap with controller example completed!")


if __name__ == "__main__":
    example_costmap_basic()
    example_costmap_with_planner()
    example_costmap_with_controller()
