"""Tests for track module."""

import sys

import numpy as np

sys.path.insert(0, "src")

from simple_autonomous_car.track.track import Track


def test_track_creation():
    """Test basic track creation."""
    centerline = np.array([[0, 0], [10, 0], [10, 10], [0, 10], [0, 0]])
    track = Track(centerline, track_width=5.0)

    assert track.centerline.shape == (5, 2)
    assert track.inner_bound.shape == (5, 2)
    assert track.outer_bound.shape == (5, 2)


def test_oval_track_creation():
    """Test oval track creation."""
    track = Track.create_oval_track(length=50.0, width=30.0, track_width=5.0, num_points=100)

    assert len(track.centerline) == 100
    assert track.track_width == 5.0


def test_get_point_at_distance():
    """Test getting point at distance along track."""
    centerline = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
    track = Track(centerline, track_width=5.0)

    point, heading = track.get_point_at_distance(0.0)
    assert np.allclose(point, centerline[0], atol=1e-6)


def test_get_bounds_at_point():
    """Test getting bounds at a point."""
    centerline = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
    track = Track(centerline, track_width=5.0)

    inner, outer = track.get_bounds_at_point(np.array([5.0, 5.0]))
    assert inner.shape == (2,)
    assert outer.shape == (2,)
