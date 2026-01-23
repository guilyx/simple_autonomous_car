"""Base filter class for pose and object estimation."""

from abc import ABC, abstractmethod
import numpy as np
from typing import Optional, Dict, Any


class BaseFilter(ABC):
    """
    Base class for all filters.
    
    Filters estimate state (position, velocity, etc.) from noisy measurements.
    They can be used for:
    - Ego pose estimation (car's own position/heading)
    - Object tracking (obstacle positions/velocities)
    
    Attributes
    ----------
    name : str
        Filter name/identifier.
    enabled : bool
        Whether the filter is enabled.
    """
    
    def __init__(self, name: str = "filter", enabled: bool = True):
        """
        Initialize base filter.
        
        Parameters
        ----------
        name : str, default="filter"
            Filter name/identifier.
        enabled : bool, default=True
            Whether the filter is enabled.
        """
        self.name = name
        self.enabled = enabled
    
    @abstractmethod
    def predict(self, dt: float, control: Optional[Dict[str, float]] = None) -> None:
        """
        Predict state forward in time.
        
        Parameters
        ----------
        dt : float
            Time step in seconds.
        control : dict, optional
            Control inputs (e.g., {"acceleration": 0.5, "steering_rate": 0.1}).
        """
        pass
    
    @abstractmethod
    def update(self, measurement: np.ndarray, measurement_covariance: Optional[np.ndarray] = None) -> None:
        """
        Update state estimate with measurement.
        
        Parameters
        ----------
        measurement : np.ndarray
            Measurement vector (e.g., [x, y] for position).
        measurement_covariance : np.ndarray, optional
            Measurement noise covariance matrix.
        """
        pass
    
    @abstractmethod
    def get_state(self) -> np.ndarray:
        """
        Get current state estimate.
        
        Returns
        -------
        np.ndarray
            State vector (e.g., [x, y, vx, vy] for position and velocity).
        """
        pass
    
    @abstractmethod
    def get_covariance(self) -> np.ndarray:
        """
        Get current state covariance (uncertainty).
        
        Returns
        -------
        np.ndarray
            Covariance matrix.
        """
        pass
    
    def is_enabled(self) -> bool:
        """Check if filter is enabled."""
        return self.enabled
    
    def enable(self) -> None:
        """Enable the filter."""
        self.enabled = True
    
    def disable(self) -> None:
        """Disable the filter."""
        self.enabled = False
    
    def reset(self, initial_state: np.ndarray, initial_covariance: Optional[np.ndarray] = None) -> None:
        """
        Reset filter to initial state.
        
        Parameters
        ----------
        initial_state : np.ndarray
            Initial state vector.
        initial_covariance : np.ndarray, optional
            Initial covariance matrix.
        """
        # Default implementation - subclasses should override
        pass
