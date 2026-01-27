"""Simple track simulation configuration."""

import numpy as np

CONFIG = {
    "track": {
        "type": "simple",
        "length": 120.0,
        "width": 60.0,
        "track_width": 7.5,
        "num_points": 300,
    },
    "car": {
        "initial_velocity": 8.0,
        "max_velocity": 25.0,
        "wheelbase": 2.5,
        "max_steering_angle": np.pi / 4,
    },
    "sensor": {"max_range": 60.0, "angular_resolution": 0.1, "point_noise_std": 0.001},
    "perception": {
        "position_noise_std": 0.01,
        "orientation_noise_std": 0.0001,
        "measurement_noise_std": 0.01,
    },
    "planner": {"lookahead_distance": 75.0, "waypoint_spacing": 2.0},
    "controller": {
        "lookahead_distance": 10.0,
        "lookahead_gain": 1.5,
        "max_steering_rate": 0.8,
        "target_velocity": 8.0,
        "velocity_gain": 0.3,
    },
    "costmap": {
        "width": 90.0,
        "height": 90.0,
        "resolution": 0.5,
        "inflation_radius": 2.0,
        "frame": "ego",
    },
    "alerts": {"warning_threshold": 1.0, "critical_threshold": 2.0, "lookahead_distance": 30.0},
    "simulation": {"dt": 0.1, "total_time": 50.0, "num_steps": 500, "horizon": 60.0},
}
