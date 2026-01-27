"""Race track (figure-8) simulation configuration."""

import numpy as np

CONFIG = {
    "track": {"type": "figure8", "size": 105.0, "track_width": 9.0, "num_points": 450},
    "car": {
        "initial_velocity": 12.0,
        "max_velocity": 25.0,
        "wheelbase": 2.5,
        "max_steering_angle": np.pi / 6,
    },
    "sensor": {"max_range": 60.0, "angular_resolution": 0.1, "point_noise_std": 0.1},
    "perception": {
        "position_noise_std": 0.15,
        "orientation_noise_std": 0.08,
        "measurement_noise_std": 0.25,
    },
    "planner": {"lookahead_distance": 75.0, "waypoint_spacing": 2.0},
    "controller": {
        "lookahead_distance": 12.0,
        "lookahead_gain": 1.5,
        "max_steering_rate": 0.8,
        "target_velocity": 12.0,
        "velocity_gain": 0.3,
    },
    "costmap": {
        "width": 90.0,
        "height": 90.0,
        "resolution": 0.5,
        "inflation_radius": 2.0,
        "frame": "ego",
    },
    "alerts": {"warning_threshold": 1.5, "critical_threshold": 3.0, "lookahead_distance": 45.0},
    "simulation": {"dt": 0.1, "total_time": 30.0, "num_steps": 300, "horizon": 60.0},
}
