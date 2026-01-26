"""Particle filter for state estimation."""

import numpy as np

from simple_autonomous_car.filters.base_filter import BaseFilter


class ParticleFilter(BaseFilter):
    """
    Particle filter for state estimation.

    Uses a set of particles to represent the probability distribution
    of the state. Good for non-linear systems and multi-modal distributions.

    Parameters
    ----------
    initial_state : np.ndarray
        Initial state vector (e.g., [x, y]).
    initial_covariance : np.ndarray
        Initial covariance matrix (used to initialize particles).
    num_particles : int, default=100
        Number of particles.
    process_noise : np.ndarray
        Process noise covariance matrix.
    measurement_noise : np.ndarray
        Measurement noise covariance matrix.
    name : str, default="particle_filter"
        Filter name.
    """

    def __init__(
        self,
        initial_state: np.ndarray,
        initial_covariance: np.ndarray,
        num_particles: int = 100,
        process_noise: np.ndarray | None = None,
        measurement_noise: np.ndarray | None = None,
        name: str = "particle_filter",
    ):
        super().__init__(name=name)
        self.state_dim = len(initial_state)
        self.num_particles = num_particles
        self.process_noise = (
            np.asarray(process_noise, dtype=np.float64)
            if process_noise is not None
            else np.eye(self.state_dim) * 0.1
        )
        self.measurement_noise = (
            np.asarray(measurement_noise, dtype=np.float64)
            if measurement_noise is not None
            else np.eye(self.state_dim) * 0.1
        )

        # Initialize particles
        self.particles = np.random.multivariate_normal(
            initial_state, initial_covariance, num_particles
        )
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, dt: float, control: dict[str, float] | None = None) -> None:
        """
        Predict state forward in time.

        Parameters
        ----------
        dt : float
            Time step in seconds.
        control : dict, optional
            Control inputs (e.g., {"velocity": 5.0, "heading": 0.5}).
        """
        if not self.enabled:
            return

        # Add process noise to particles
        noise = np.random.multivariate_normal(
            np.zeros(self.state_dim), self.process_noise * dt, self.num_particles
        )
        self.particles += noise

        # Apply control if provided (simple constant velocity model)
        if control is not None:
            if "velocity" in control and "heading" in control:
                v = control["velocity"]
                h = control["heading"]
                self.particles[:, 0] += v * np.cos(h) * dt  # x
                self.particles[:, 1] += v * np.sin(h) * dt  # y

    def update(
        self, measurement: np.ndarray, measurement_covariance: np.ndarray | None = None
    ) -> None:
        """
        Update state estimate with measurement.

        Parameters
        ----------
        measurement : np.ndarray
            Measurement vector.
        measurement_covariance : np.ndarray, optional
            Measurement noise covariance (uses default if not provided).
        """
        if not self.enabled:
            return

        measurement = np.asarray(measurement, dtype=np.float64)
        r_matrix = (
            measurement_covariance if measurement_covariance is not None else self.measurement_noise
        )

        # Update weights based on measurement likelihood
        for i in range(self.num_particles):
            # Compute likelihood: p(measurement | particle)
            residual = measurement - self.particles[i, : len(measurement)]
            likelihood = np.exp(-0.5 * residual.T @ np.linalg.inv(r_matrix) @ residual)
            self.weights[i] *= likelihood

        # Normalize weights
        self.weights /= np.sum(self.weights)

        # Resample if effective number of particles is too low
        effective_particles = 1.0 / np.sum(self.weights**2)
        if effective_particles < self.num_particles / 2:
            self._resample()

    def _resample(self) -> None:
        """Resample particles based on weights."""
        # Systematic resampling
        cumulative_weights = np.cumsum(self.weights)
        step = 1.0 / self.num_particles
        u = np.random.uniform(0, step)

        new_particles = np.zeros_like(self.particles)
        j = 0
        for i in range(self.num_particles):
            while u > cumulative_weights[j]:
                j += 1
            new_particles[i] = self.particles[j]
            u += step

        self.particles = new_particles
        self.weights = np.ones(self.num_particles) / self.num_particles

    def get_state(self) -> np.ndarray:
        """Get current state estimate (weighted mean of particles)."""
        result = np.average(self.particles, axis=0, weights=self.weights)
        return np.asarray(result, dtype=np.float64)

    def get_covariance(self) -> np.ndarray:
        """Get current state covariance."""
        state = self.get_state()
        diff = self.particles - state
        return np.cov(diff.T, aweights=self.weights)

    def reset(
        self, initial_state: np.ndarray, initial_covariance: np.ndarray | None = None
    ) -> None:
        """Reset filter to initial state."""
        if initial_covariance is None:
            initial_covariance = np.eye(self.state_dim) * 0.1
        self.particles = np.random.multivariate_normal(
            initial_state, initial_covariance, self.num_particles
        )
        self.weights = np.ones(self.num_particles) / self.num_particles
