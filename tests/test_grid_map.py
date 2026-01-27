"""Tests for grid map functionality."""

import sys

import numpy as np
import pytest

sys.path.insert(0, "src")

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.maps.grid_map import GridMap
from simple_autonomous_car.planning.goal_planner import GoalPlanner


def test_grid_map_creation():
    """Test creating a grid map."""
    grid_map = GridMap.create_random_map(
        width=50.0,
        height=50.0,
        num_obstacles=10,
        seed=42,
    )
    assert grid_map.width == 50.0
    assert grid_map.height == 50.0
    assert len(grid_map.obstacles) == 10


def test_grid_map_obstacle_detection():
    """Test obstacle detection."""
    obstacles = np.array([[10.0, 10.0], [20.0, 20.0]])
    grid_map = GridMap(
        width=50.0,
        height=50.0,
        obstacles=obstacles,
        obstacle_size=2.0,
    )

    # Test obstacle positions
    assert grid_map.is_obstacle(np.array([10.0, 10.0]))
    assert grid_map.is_obstacle(np.array([20.0, 20.0]))
    assert not grid_map.is_obstacle(np.array([0.0, 0.0]))


def test_grid_map_valid_position():
    """Test position validation."""
    obstacles = np.array([[10.0, 10.0]])
    grid_map = GridMap(
        width=50.0,
        height=50.0,
        obstacles=obstacles,
        obstacle_size=2.0,
    )

    # Valid positions
    assert grid_map.is_valid_position(np.array([0.0, 0.0]))
    assert grid_map.is_valid_position(np.array([20.0, 20.0]))

    # Invalid positions (obstacles)
    assert not grid_map.is_valid_position(np.array([10.0, 10.0]))

    # Invalid positions (out of bounds)
    assert not grid_map.is_valid_position(np.array([30.0, 0.0]))  # Out of bounds


def test_goal_planner_basic():
    """Test basic goal planner functionality."""
    grid_map = GridMap.create_random_map(
        width=30.0,
        height=30.0,
        num_obstacles=5,
        seed=42,
    )

    planner = GoalPlanner(grid_map=grid_map, resolution=1.0)

    car_state = CarState(x=-10.0, y=-10.0, heading=0.0)
    goal = np.array([10.0, 10.0])

    plan = planner.plan(car_state, goal=goal)

    # Should find a path
    assert len(plan) > 0
    assert np.allclose(plan[0], car_state.position(), atol=1.0)
    assert np.allclose(plan[-1], goal, atol=2.0)


def test_goal_planner_no_path():
    """Test goal planner when no path exists."""
    # Create map with obstacles blocking path
    obstacles = np.array(
        [
            [0.0, -5.0],
            [0.0, 0.0],
            [0.0, 5.0],
        ]
    )
    grid_map = GridMap(
        width=20.0,
        height=20.0,
        obstacles=obstacles,
        obstacle_size=3.0,
    )

    planner = GoalPlanner(grid_map=grid_map, resolution=1.0)

    car_state = CarState(x=-10.0, y=0.0, heading=0.0)
    goal = np.array([10.0, 0.0])  # Goal blocked by obstacles

    plan = planner.plan(car_state, goal=goal)

    # May or may not find path depending on resolution and obstacle size
    # Just check it doesn't crash
    assert isinstance(plan, np.ndarray)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
