"""Tests for error detection module."""

import sys

sys.path.insert(0, "src")

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.detection.error_detector import LocalizationErrorDetector
from simple_autonomous_car.maps.ground_truth_map import GroundTruthMap
from simple_autonomous_car.maps.perceived_map import PerceivedMap
from simple_autonomous_car.track.track import Track


def test_error_detector_creation():
    """Test error detector creation."""
    track = Track.create_oval_track()
    gt_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(gt_map)

    detector = LocalizationErrorDetector(gt_map, perceived_map, error_threshold=1.0)

    assert detector.ground_truth_map == gt_map
    assert detector.perceived_map == perceived_map
    assert detector.error_threshold == 1.0


def test_compute_errors():
    """Test error computation."""
    track = Track.create_oval_track()
    gt_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(gt_map, measurement_noise_std=0.1)
    detector = LocalizationErrorDetector(gt_map, perceived_map, error_threshold=1.0)

    car_state = CarState(x=0.0, y=0.0, heading=0.0)
    perceived_map.update_perceived_state(car_state)

    errors = detector.compute_errors(car_state, horizon=50.0)

    assert "centerline_errors" in errors
    assert "inner_bound_errors" in errors
    assert "outer_bound_errors" in errors
    assert "max_error" in errors
    assert "mean_error" in errors


def test_detect_errors():
    """Test error detection."""
    track = Track.create_oval_track()
    gt_map = GroundTruthMap(track)
    perceived_map = PerceivedMap(gt_map, measurement_noise_std=0.1)
    detector = LocalizationErrorDetector(gt_map, perceived_map, error_threshold=1.0)

    car_state = CarState(x=0.0, y=0.0, heading=0.0)
    perceived_map.update_perceived_state(car_state)

    detections = detector.detect_errors(car_state, horizon=50.0)

    assert "has_error" in detections
    assert "centerline_has_error" in detections
    assert "inner_bound_has_error" in detections
    assert "outer_bound_has_error" in detections
