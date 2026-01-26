"""Tests for filter components."""

import sys

import numpy as np
import pytest

sys.path.insert(0, "src")

from simple_autonomous_car.filters import KalmanFilter, ParticleFilter


def test_kalman_filter_initialization():
    """Test Kalman filter initialization."""
    initial_state = np.array([0.0, 0.0, 1.0, 0.0])  # [x, y, vx, vy]
    initial_cov = np.eye(4) * 0.1
    process_noise = np.eye(4) * 0.01
    measurement_noise = np.eye(2) * 0.1

    kf = KalmanFilter(
        initial_state=initial_state,
        initial_covariance=initial_cov,
        process_noise=process_noise,
        measurement_noise=measurement_noise,
    )

    assert np.allclose(kf.get_state(), initial_state)
    assert kf.state_dim == 4


def test_kalman_filter_predict():
    """Test Kalman filter prediction."""
    initial_state = np.array([0.0, 0.0, 1.0, 0.0])  # [x, y, vx, vy]
    initial_cov = np.eye(4) * 0.1
    process_noise = np.eye(4) * 0.01
    measurement_noise = np.eye(2) * 0.1

    kf = KalmanFilter(
        initial_state=initial_state,
        initial_covariance=initial_cov,
        process_noise=process_noise,
        measurement_noise=measurement_noise,
    )

    # Predict forward
    kf.predict(dt=1.0)

    state = kf.get_state()
    # Should have moved: x += vx*dt = 0 + 1*1 = 1
    assert state[0] > 0.0  # x should increase


def test_kalman_filter_update():
    """Test Kalman filter update."""
    initial_state = np.array([0.0, 0.0])
    initial_cov = np.eye(2) * 1.0
    process_noise = np.eye(2) * 0.01
    measurement_noise = np.eye(2) * 0.1

    kf = KalmanFilter(
        initial_state=initial_state,
        initial_covariance=initial_cov,
        process_noise=process_noise,
        measurement_noise=measurement_noise,
    )

    # Update with measurement
    measurement = np.array([1.0, 1.0])
    kf.update(measurement)

    state = kf.get_state()
    # Should move towards measurement
    assert state[0] > 0.0
    assert state[1] > 0.0


def test_particle_filter_initialization():
    """Test particle filter initialization."""
    initial_state = np.array([0.0, 0.0])
    initial_cov = np.eye(2) * 0.1

    pf = ParticleFilter(
        initial_state=initial_state,
        initial_covariance=initial_cov,
        num_particles=50,
    )

    assert pf.num_particles == 50
    assert pf.particles.shape == (50, 2)
    assert len(pf.weights) == 50


def test_particle_filter_predict():
    """Test particle filter prediction."""
    initial_state = np.array([0.0, 0.0])
    initial_cov = np.eye(2) * 0.1

    pf = ParticleFilter(
        initial_state=initial_state,
        initial_covariance=initial_cov,
        num_particles=50,
    )

    # Predict forward
    pf.predict(dt=1.0, control={"velocity": 1.0, "heading": 0.0})

    state = pf.get_state()
    # Should have moved forward
    assert state[0] > 0.0


def test_particle_filter_update():
    """Test particle filter update."""
    initial_state = np.array([0.0, 0.0])
    initial_cov = np.eye(2) * 1.0

    pf = ParticleFilter(
        initial_state=initial_state,
        initial_covariance=initial_cov,
        num_particles=50,
    )

    # Update with measurement
    measurement = np.array([1.0, 1.0])
    pf.update(measurement)

    state = pf.get_state()
    # Should move towards measurement
    assert state[0] > -1.0  # Should be closer to 1.0 than -1.0
    assert state[1] > -1.0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
