"""Centralized constants for the Simple Autonomous Car SDK.

This module contains all magic numbers and default values used throughout the codebase.
Using constants makes the code more maintainable and allows easy tuning of parameters.
"""

# ============================================================================
# Car Defaults
# ============================================================================

DEFAULT_WHEELBASE = 2.5  # meters
DEFAULT_INITIAL_VELOCITY = 10.0  # m/s
DEFAULT_MAX_VELOCITY = 25.0  # m/s
DEFAULT_MAX_STEERING_ANGLE = 0.7853981633974483  # π/4 radians (45 degrees)

# ============================================================================
# Controller Defaults
# ============================================================================

DEFAULT_LOOKAHEAD_DISTANCE = 10.0  # meters (controller default)
DEFAULT_LOOKAHEAD_GAIN = 2.0  # controller default (was 0.5, but controller uses 2.0)
DEFAULT_MAX_STEERING_RATE = 1.0  # rad/s
DEFAULT_TARGET_VELOCITY = 10.0  # m/s
DEFAULT_VELOCITY_GAIN = 0.5
DEFAULT_DT = 0.1  # seconds

# Controller Visualization
DEFAULT_ARC_LOOKAHEAD_DISTANCE = 20.0  # meters
DEFAULT_MIN_TURNING_RADIUS = 0.0  # meters
DEFAULT_MAX_TURNING_RADIUS = 1000.0  # meters
DEFAULT_ARC_LINEWIDTH = 3.5
DEFAULT_ARC_ALPHA = 0.95
DEFAULT_LOOKAHEAD_LINEWIDTH = 2.5

# Controller Thresholds
MIN_LOOKAHEAD_DISTANCE = 0.1  # meters
MIN_STEERING_ANGLE_THRESHOLD = 0.0  # radians
SMALL_STEERING_ANGLE_THRESHOLD = 1e-3  # radians
ADAPTIVE_GAIN_DENOMINATOR = 10.0  # used in adaptive gain calculation

# ============================================================================
# Planner Defaults
# ============================================================================

DEFAULT_PLANNER_LOOKAHEAD_DISTANCE = 50.0  # meters
DEFAULT_WAYPOINT_SPACING = 2.0  # meters
DEFAULT_PLAN_LINEWIDTH = 2.5

# Planner Thresholds
MIN_SEGMENT_LENGTH = 1e-6  # meters

# ============================================================================
# Costmap Defaults
# ============================================================================

DEFAULT_COSTMAP_WIDTH = 50.0  # meters
DEFAULT_COSTMAP_HEIGHT = 50.0  # meters
DEFAULT_COSTMAP_RESOLUTION = 0.5  # meters per cell
DEFAULT_INFLATION_RADIUS = 1.0  # meters

# Costmap Values
COST_FREE = 0.0
COST_OCCUPIED = 1.0
COST_THRESHOLD = 0.5  # threshold for occupied vs inflated

# Costmap Visualization
DEFAULT_COSTMAP_ALPHA = 0.6
DEFAULT_COSTMAP_ZORDER = 2  # Behind plan/controller but above track

# ============================================================================
# Sensor Defaults
# ============================================================================

DEFAULT_LIDAR_MAX_RANGE = 40.0  # meters
DEFAULT_LIDAR_ANGULAR_RESOLUTION = 0.1  # radians
DEFAULT_POINT_NOISE_STD = 0.1  # meters

# ============================================================================
# Track Defaults
# ============================================================================

DEFAULT_TRACK_LENGTH = 100.0  # meters
DEFAULT_TRACK_WIDTH = 50.0  # meters
DEFAULT_TRACK_WIDTH_INNER = 5.0  # meters
DEFAULT_TRACK_NUM_POINTS = 200

# ============================================================================
# Visualization Defaults
# ============================================================================

# Arrow visualization
DEFAULT_ARROW_HEAD_WIDTH = 1.0
DEFAULT_ARROW_HEAD_LENGTH = 1.0

# Perception visualization
DEFAULT_PERCEPTION_LINEWIDTH = 0.5

# ============================================================================
# Numerical Thresholds
# ============================================================================

EPSILON = 1e-6  # general small number threshold
EPSILON_SMALL = 1e-3  # slightly larger threshold
LARGE_NUMBER = 1e6  # used for "infinite" values

# ============================================================================
# Simulation Defaults
# ============================================================================

DEFAULT_SIMULATION_DT = 0.1  # seconds
DEFAULT_PLOT_PAUSE = 0.01  # seconds
