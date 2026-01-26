"""Inflation utilities for costmaps."""

import numpy as np
from scipy.ndimage import maximum_filter  # type: ignore[import-untyped]


def compute_inflation_kernel(radius: float, resolution: float) -> np.ndarray:
    """
    Compute inflation kernel for obstacle inflation.

    Parameters
    ----------
    radius : float
        Inflation radius in meters.
    resolution : float
        Costmap resolution in meters per cell.

    Returns
    -------
    np.ndarray
        2D kernel array for inflation.

    Examples
    --------
    >>> kernel = compute_inflation_kernel(radius=1.0, resolution=0.5)
    >>> print(kernel.shape)
    (5, 5)
    """
    kernel_size = int(2 * radius / resolution) + 1
    kernel = np.zeros((kernel_size, kernel_size))

    center = kernel_size // 2
    for i in range(kernel_size):
        for j in range(kernel_size):
            dx = (i - center) * resolution
            dy = (j - center) * resolution
            distance = np.sqrt(dx**2 + dy**2)
            if distance <= radius:
                # Linear decay from 1.0 at center to 0.0 at radius
                cost = 1.0 - (distance / radius)
                kernel[i, j] = cost

    return kernel


def inflate_obstacles(
    costmap: np.ndarray,
    inflation_radius: float,
    resolution: float,
    method: str = "linear",
) -> np.ndarray:
    """
    Inflate obstacles in a costmap.

    Parameters
    ----------
    costmap : np.ndarray
        2D costmap array (0.0 = free, 1.0 = occupied).
    inflation_radius : float
        Inflation radius in meters.
    resolution : float
        Costmap resolution in meters per cell.
    method : str, default="linear"
        Inflation method: "linear" (linear decay) or "binary" (binary expansion).

    Returns
    -------
    np.ndarray
        Inflated costmap.

    Examples
    --------
    >>> costmap = np.zeros((100, 100))
    >>> costmap[50, 50] = 1.0  # Obstacle
    >>> inflated = inflate_obstacles(costmap, inflation_radius=2.0, resolution=0.5)
    """
    if method == "linear":
        # More efficient linear inflation using distance transform
        from scipy.ndimage import distance_transform_edt

        # Create binary obstacle map
        obstacles = costmap >= 0.5

        # Compute distance to nearest obstacle
        distances = distance_transform_edt(~obstacles) * resolution

        # Apply linear decay
        result = np.zeros_like(costmap)
        mask = distances <= inflation_radius
        result[mask] = 1.0 - (distances[mask] / inflation_radius)
        result[~mask] = costmap[~mask]

        # Preserve original obstacles (set to max cost)
        result[obstacles] = 1.0

        return result
    else:  # binary
        kernel_size = int(2 * inflation_radius / resolution) + 1
        kernel = np.ones((kernel_size, kernel_size))
        inflated = maximum_filter(costmap, footprint=kernel)
        return np.asarray(inflated, dtype=costmap.dtype)


def compute_distance_transform(costmap: np.ndarray, resolution: float) -> np.ndarray:
    """
    Compute distance transform of costmap.

    Parameters
    ----------
    costmap : np.ndarray
        2D costmap array.
    resolution : float
        Costmap resolution in meters per cell.

    Returns
    -------
    np.ndarray
        Distance transform (distance to nearest obstacle in meters).
    """
    from scipy.ndimage import distance_transform_edt

    # Invert: obstacles become 0, free space becomes 1
    inverted = 1.0 - costmap
    # Compute distance transform
    distances = distance_transform_edt(inverted) * resolution
    return np.asarray(distances, dtype=np.float64)
