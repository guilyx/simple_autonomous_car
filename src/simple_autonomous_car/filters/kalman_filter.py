"""Kalman filter for state estimation."""

import numpy as np
from typing import Optional, Dict

from simple_autonomous_car.filters.base_filter import BaseFilter


class KalmanFilter(BaseFilter):
    """
    Kalman filter for state estimation.
    
    Estimates state (position, velocity, etc.) from noisy measurements
    using the Kalman filter algorithm.
    
    Parameters
    ----------
    initial_state : np.ndarray
        Initial state vector (e.g., [x, y, vx, vy]).
    initial_covariance : np.ndarray
        Initial covariance matrix.
    process_noise : np.ndarray
        Process noise covariance matrix (Q).
    measurement_noise : np.ndarray
        Measurement noise covariance matrix (R).
    name : str, default="kalman_filter"
        Filter name.
    """
    
    def __init__(
        self,
        initial_state: np.ndarray,
        initial_covariance: np.ndarray,
        process_noise: np.ndarray,
        measurement_noise: np.ndarray,
        name: str = "kalman_filter",
    ):
        super().__init__(name=name)
        self.state = np.asarray(initial_state, dtype=np.float64).copy()
        self.covariance = np.asarray(initial_covariance, dtype=np.float64).copy()
        self.process_noise = np.asarray(process_noise, dtype=np.float64)
        self.measurement_noise = np.asarray(measurement_noise, dtype=np.float64)
        
        # State dimension
        self.state_dim = len(initial_state)
        self.measurement_dim = measurement_noise.shape[0]
        
        # State transition matrix (identity by default, can be overridden)
        self.F = np.eye(self.state_dim)
        # Measurement matrix (identity by default, can be overridden)
        self.H = np.eye(self.measurement_dim, self.state_dim)
    
    def predict(self, dt: float, control: Optional[Dict[str, float]] = None) -> None:
        """
        Predict state forward in time.
        
        Parameters
        ----------
        dt : float
            Time step in seconds.
        control : dict, optional
            Control inputs (not used in basic Kalman filter).
        """
        if not self.enabled:
            return
        
        # Update state transition matrix with dt if needed
        # For constant velocity model: x' = x + vx*dt, y' = y + vy*dt
        if self.state_dim >= 4:  # Has velocity components
            F = np.eye(self.state_dim)
            F[0, 2] = dt  # x += vx*dt
            F[1, 3] = dt  # y += vy*dt
            self.F = F
        
        # Predict state: x' = F * x
        self.state = self.F @ self.state
        
        # Predict covariance: P' = F * P * F^T + Q
        self.covariance = self.F @ self.covariance @ self.F.T + self.process_noise
    
    def update(self, measurement: np.ndarray, measurement_covariance: Optional[np.ndarray] = None) -> None:
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
        R = measurement_covariance if measurement_covariance is not None else self.measurement_noise
        
        # Innovation (measurement residual)
        y = measurement - self.H @ self.state
        
        # Innovation covariance
        S = self.H @ self.covariance @ self.H.T + R
        
        # Kalman gain
        K = self.covariance @ self.H.T @ np.linalg.inv(S)
        
        # Update state: x = x + K * y
        self.state = self.state + K @ y
        
        # Update covariance: P = (I - K*H) * P
        I = np.eye(self.state_dim)
        self.covariance = (I - K @ self.H) @ self.covariance
    
    def get_state(self) -> np.ndarray:
        """Get current state estimate."""
        return self.state.copy()
    
    def get_covariance(self) -> np.ndarray:
        """Get current state covariance."""
        return self.covariance.copy()
    
    def reset(self, initial_state: np.ndarray, initial_covariance: Optional[np.ndarray] = None) -> None:
        """Reset filter to initial state."""
        self.state = np.asarray(initial_state, dtype=np.float64).copy()
        if initial_covariance is not None:
            self.covariance = np.asarray(initial_covariance, dtype=np.float64).copy()
