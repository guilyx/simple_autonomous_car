"""Grid map simulation configuration with obstacles and goal-based navigation."""

import numpy as np

CONFIG = {
    "map": {
        "type": "grid",
        "width": 50.0,
        "height": 50.0,
        "resolution": 0.5,
        "num_obstacles": 15,
        "obstacle_size": 2.0,
        "seed": 42,  # For reproducibility
    },
    "car": {
        "initial_position": [-20.0, -20.0],  # Start position
        "initial_heading": 0.0,
        "initial_velocity": 5.0,
        "max_velocity": 15.0,
        "wheelbase": 2.5,
        "max_steering_angle": np.pi / 4,
    },
    "goal": {
        "position": [20.0, 20.0],  # Goal position
    },
    "sensor": {"max_range": 10.0, "angular_resolution": 0.1, "point_noise_std": 0.05},
    "perception": {
        "position_noise_std": 0.005,
        "orientation_noise_std": 0.001,
        "measurement_noise_std": 0.005,
    },
    "planner": {
        "type": "goal",
        "resolution": 0.5,
    },
    "controller": {
        "lookahead_distance": 2.0,
        "lookahead_gain": 0.8,
        "max_steering_rate": 0.8,
        "target_velocity": 2.0,
        "velocity_gain": 0.3,
    },
    "costmap": {
        "width": 40.0,
        "height": 40.0,
        "resolution": 1.0,
        "frame": "ego",
        # Footprint-based inflation (more accurate than arbitrary radius)
        "footprint": {
            "type": "rectangular",
            "length": 4.5,  # Vehicle length (meters)
            "width": 1.8,  # Vehicle width (meters)
            "padding": 0.3,  # Safety padding around footprint (meters)
        },
        # inflation_radius is now calculated from footprint automatically
        # Can still override manually if needed: "inflation_radius": 2.5
    },
    "simulation": {
        "dt": 0.1,
        "total_time": 60.0,
        "num_steps": 600,
        "horizon": 20.0,
        "goal_tolerance": 2.0,  # Distance to goal to consider reached
    },
}
