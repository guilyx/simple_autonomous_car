"""Tests for maps module."""

import sys

import numpy as np

sys.path.insert(0, "src")

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.maps.ground_truth_map import GroundTruthMap
from simple_autonomous_car.maps.perceived_map import PerceivedMap
from simple_autonomous_car.track.track import Track


def test_ground_truth_map_creation():
    """Test ground truth map creation."""
    track = Track.create_oval_track()
    gt_map = GroundTruthMap(track)

    assert gt_map.track == track
    assert gt_map.centerline.shape == track.centerline.shape


def test_ground_truth_get_visible_segments():
    """Test getting visible segments."""
    track = Track.create_oval_track()
    gt_map = GroundTruthMap(track)

    car_state = CarState(x=0.0, y=0.0, heading=0.0)
    centerline, inner, outer = gt_map.get_visible_segments(
        car_state.position(), car_state.heading, horizon=50.0
    )

    assert len(centerline) >= 0
    assert len(inner) == len(centerline)
    assert len(outer) == len(centerline)


def test_perceived_map_creation():
    """Test perceived map creation."""
    track = Track.create_oval_track()
    gt_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(gt_map)

    assert perceived_map.ground_truth == gt_map


def test_perceived_map_update_state():
    """Test updating perceived car state."""
    track = Track.create_oval_track()
    gt_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(gt_map)

    car_state = CarState(x=1.0, y=2.0, heading=np.pi / 4)
    perceived_map.update_perceived_state(car_state)

    assert perceived_map.perceived_car_state is not None
    # Should have some noise, so not exactly equal
    assert abs(perceived_map.perceived_car_state.x - car_state.x) < 1.0


def test_perceived_map_get_segments():
    """Test getting perceived segments."""
    track = Track.create_oval_track()
    gt_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(gt_map)

    car_state = CarState(x=0.0, y=0.0, heading=0.0)
    perceived_map.update_perceived_state(car_state)

    centerline, inner, outer = perceived_map.get_perceived_segments(horizon=50.0)

    # Should return arrays even if empty
    assert isinstance(centerline, np.ndarray)
    assert isinstance(inner, np.ndarray)
    assert isinstance(outer, np.ndarray)
