"""Tests for car module."""

import numpy as np
import pytest

import sys
sys.path.insert(0, 'src')

from simple_autonomous_car.car.car import Car, CarState


def test_car_state_creation():
    """Test car state creation."""
    state = CarState(x=1.0, y=2.0, heading=np.pi / 4, velocity=10.0)

    assert state.x == 1.0
    assert state.y == 2.0
    assert state.heading == np.pi / 4
    assert state.velocity == 10.0


def test_car_state_transforms():
    """Test coordinate frame transformations."""
    state = CarState(x=0.0, y=0.0, heading=0.0)

    # Point in world frame
    world_point = np.array([1.0, 0.0])

    # Transform to car frame (should be same since heading is 0)
    car_point = state.transform_to_car_frame(world_point)
    assert np.allclose(car_point, world_point, atol=1e-6)

    # Transform back
    world_point_back = state.transform_to_world_frame(car_point)
    assert np.allclose(world_point_back, world_point, atol=1e-6)


def test_car_creation():
    """Test car creation."""
    initial_state = CarState(x=0.0, y=0.0, heading=0.0, velocity=10.0)
    car = Car(initial_state=initial_state, wheelbase=2.5)

    assert car.state.x == 0.0
    assert car.state.velocity == 10.0


def test_car_update():
    """Test car state update."""
    car = Car(initial_state=CarState(x=0.0, y=0.0, heading=0.0, velocity=10.0))

    # Update with no steering
    car.update(dt=0.1, acceleration=0.0, steering_rate=0.0)

    # Should move forward
    assert car.state.x > 0.0
    assert np.allclose(car.state.y, 0.0, atol=1e-6)


def test_car_corners():
    """Test getting car corner points."""
    car = Car(initial_state=CarState(x=0.0, y=0.0, heading=0.0))
    corners = car.get_corners()

    assert corners.shape == (4, 2)
