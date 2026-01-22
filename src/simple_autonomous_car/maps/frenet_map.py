"""Map representation in Frenet frame."""

import numpy as np
from typing import Tuple

from simple_autonomous_car.track.track import Track
from simple_autonomous_car.frames.frenet import FrenetFrame


class FrenetMap:
    """Map representation in Frenet frame coordinates."""

    def __init__(self, track: Track):
        """
        Initialize Frenet map.

        Args:
            track: Track object
        """
        self.track = track
        self.frenet_frame = FrenetFrame(track)
        self._compute_frenet_bounds()

    def _compute_frenet_bounds(self) -> None:
        """Compute track boundaries in Frenet frame."""
        # Convert inner and outer bounds to Frenet coordinates
        inner_frenet = []
        outer_frenet = []

        for i in range(len(self.track.centerline)):
            inner_point = self.track.inner_bound[i]
            outer_point = self.track.outer_bound[i]

            s_inner, d_inner = self.frenet_frame.global_to_frenet(inner_point)
            s_outer, d_outer = self.frenet_frame.global_to_frenet(outer_point)

            inner_frenet.append([s_inner, d_inner])
            outer_frenet.append([s_outer, d_outer])

        self.inner_bound_frenet = np.array(inner_frenet)
        self.outer_bound_frenet = np.array(outer_frenet)

    def get_bounds_at_s(self, s: float) -> Tuple[float, float]:
        """
        Get inner and outer boundary lateral offsets at distance s.

        Args:
            s: Distance along path

        Returns:
            Tuple of (d_inner, d_outer) lateral offsets
        """
        # Find closest point
        s_wrapped = s % self.frenet_frame.total_length

        # Find segment
        idx = np.searchsorted(self.frenet_frame.cumulative_distances, s_wrapped)
        if idx == 0:
            idx = 1
        if idx >= len(self.track.centerline):
            idx = len(self.track.centerline) - 1

        # Get bounds at this point
        d_inner = self.inner_bound_frenet[idx, 1]
        d_outer = self.outer_bound_frenet[idx, 1]

        return d_inner, d_outer

    def get_bounds_in_range(
        self, s_start: float, s_end: float
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get inner and outer bounds in a range of s values.

        Args:
            s_start: Start distance along path
            s_end: End distance along path

        Returns:
            Tuple of (inner_bounds, outer_bounds) as arrays of shape (N, 2) with [s, d]
        """
        # Find points in range
        s_start_wrapped = s_start % self.frenet_frame.total_length
        s_end_wrapped = s_end % self.frenet_frame.total_length

        # Get all points in range
        if s_start_wrapped <= s_end_wrapped:
            mask = (self.inner_bound_frenet[:, 0] >= s_start_wrapped) & (
                self.inner_bound_frenet[:, 0] <= s_end_wrapped
            )
        else:
            # Wraps around
            mask = (self.inner_bound_frenet[:, 0] >= s_start_wrapped) | (
                self.inner_bound_frenet[:, 0] <= s_end_wrapped
            )

        inner_bounds = self.inner_bound_frenet[mask]
        outer_bounds = self.outer_bound_frenet[mask]

        return inner_bounds, outer_bounds
