"""Track bounds alert system - detects when perceived lines are too far from map lines."""

import numpy as np
from typing import Dict, List, Optional, Tuple

from simple_autonomous_car.car.car import CarState
from simple_autonomous_car.perception.perception import PerceptionPoints
from simple_autonomous_car.maps.frenet_map import FrenetMap
from simple_autonomous_car.frames.frenet import FrenetFrame


class TrackBoundsAlert:
    """
    Alert system for detecting when perceived track boundaries deviate significantly
    from the ground truth map boundaries.

    This is the primary alert system for track bounds violations. It converts
    perception data to Frenet coordinates and compares against ground truth bounds
    to detect when the car is approaching or leaving the track.

    Parameters
    ----------
    frenet_map : FrenetMap
        Frenet map with ground truth boundaries.
    warning_threshold : float, default=1.0
        Distance threshold for warnings in meters. Deviations above this trigger warnings.
    critical_threshold : float, default=2.0
        Distance threshold for critical alerts in meters. Deviations above this trigger critical alerts.
        Must be greater than warning_threshold.
    lookahead_distance : float, default=20.0
        Distance ahead to check in meters. Only points within this distance are evaluated.

    Attributes
    ----------
    frenet_map : FrenetMap
        Frenet map with ground truth boundaries.
    warning_threshold : float
        Warning threshold in meters.
    critical_threshold : float
        Critical threshold in meters.
    lookahead_distance : float
        Lookahead distance in meters.
    alert_history : List[Dict]
        History of alert checks.

    Examples
    --------
    >>> from simple_autonomous_car import Track, FrenetMap, TrackBoundsAlert
    >>> track = Track.create_simple_track()
    >>> frenet_map = FrenetMap(track)
    >>> alert_system = TrackBoundsAlert(
    ...     frenet_map,
    ...     warning_threshold=1.0,
    ...     critical_threshold=2.0,
    ...     lookahead_distance=20.0
    ... )
    >>> result = alert_system.check(perception_points, car_state)
    >>> if result["has_critical"]:
    ...     print(f"CRITICAL: Max deviation = {result['max_deviation']:.2f}m")
    """

    def __init__(
        self,
        frenet_map: FrenetMap,
        warning_threshold: float = 1.0,
        critical_threshold: float = 2.0,
        lookahead_distance: float = 20.0,
    ):
        """
        Initialize track bounds alert system.

        Parameters
        ----------
        frenet_map : FrenetMap
            Frenet map with ground truth boundaries.
        warning_threshold : float, default=1.0
            Distance threshold for warnings (meters).
        critical_threshold : float, default=2.0
            Distance threshold for critical alerts (meters).
            Must be >= warning_threshold.
        lookahead_distance : float, default=20.0
            Distance ahead to check (meters). Must be positive.
        """
        if warning_threshold < 0:
            raise ValueError("warning_threshold must be non-negative")
        if critical_threshold < warning_threshold:
            raise ValueError("critical_threshold must be >= warning_threshold")
        if lookahead_distance <= 0:
            raise ValueError("lookahead_distance must be positive")

        self.frenet_map = frenet_map
        self.warning_threshold = warning_threshold
        self.critical_threshold = critical_threshold
        self.lookahead_distance = lookahead_distance
        self.alert_history: List[Dict] = []

    def check(
        self,
        perception_points: PerceptionPoints,
        car_state: CarState,
    ) -> Dict:
        """
        Check for track bounds violations.

        This method converts perception points to Frenet coordinates, filters them
        by lookahead distance, and compares against ground truth bounds to detect
        deviations.

        Parameters
        ----------
        perception_points : PerceptionPoints
            Perceived track boundaries. Can be in any frame (will be converted to ego).
        car_state : CarState
            Current car state for coordinate transformations.

        Returns
        -------
        Dict
            Dictionary with alert information containing:
            - has_warning : bool
                True if any deviation exceeds warning_threshold.
            - has_critical : bool
                True if any deviation exceeds critical_threshold.
            - max_deviation : float
                Maximum deviation from bounds in meters.
            - mean_deviation : float
                Mean deviation from bounds in meters.
            - deviations : np.ndarray
                Array of deviations for each point (meters).
            - alert_points : List[Dict]
                List of points with alerts, each containing:
                - s : float - Longitudinal position
                - d : float - Lateral position
                - deviation : float - Deviation from bounds

        Examples
        --------
        >>> result = alert_system.check(perception_points, car_state)
        >>> if result["has_critical"]:
        ...     print(f"CRITICAL: {result['max_deviation']:.2f}m deviation")
        >>> print(f"Found {len(result['alert_points'])} alert points")
        """
        # Convert perception points to Frenet frame
        # First ensure points are in ego frame
        if perception_points.frame != "ego":
            perception_points = perception_points.to_ego_frame(car_state)

        if len(perception_points.points) == 0:
            return self._empty_result()

        # Convert points to Frenet frame (vectorized where possible)
        frenet_points = []
        for point in perception_points.points:
            try:
                # Convert ego to global, then global to Frenet
                point_global = car_state.transform_to_world_frame(point)
                s, d = self.frenet_map.frenet_frame.global_to_frenet(point_global)
                # Only include valid points (s >= 0)
                if s >= 0:
                    frenet_points.append([s, d])
            except (ValueError, RuntimeError) as e:
                # Skip points that can't be converted (e.g., outside track)
                continue

        if len(frenet_points) == 0:
            return self._empty_result()

        frenet_points = np.array(frenet_points)

        # Get current position in Frenet frame
        car_pos_global = car_state.position()
        try:
            s_car, d_car = self.frenet_map.frenet_frame.global_to_frenet(car_pos_global)
        except (ValueError, RuntimeError):
            # Car position can't be converted (shouldn't happen, but handle gracefully)
            return self._empty_result()

        # Check points within lookahead distance (ahead of car)
        s_end = s_car + self.lookahead_distance
        mask = (frenet_points[:, 0] >= s_car) & (frenet_points[:, 0] <= s_end)
        relevant_points = frenet_points[mask]

        if len(relevant_points) == 0:
            return self._empty_result()

        # Get ground truth bounds for each point and calculate deviations
        deviations = []
        alert_points = []

        for s, d_perceived in relevant_points:
            try:
                d_inner, d_outer = self.frenet_map.get_bounds_at_s(s)
            except (ValueError, IndexError):
                # Skip if bounds can't be retrieved for this s
                continue

            # Check if perceived point is within bounds
            if d_perceived < d_inner:
                # Too far left (inside track) - negative d means left of centerline
                deviation = abs(d_perceived - d_inner)
            elif d_perceived > d_outer:
                # Too far right (outside track) - positive d means right of centerline
                deviation = abs(d_perceived - d_outer)
            else:
                # Within bounds
                deviation = 0.0

            deviations.append(deviation)

            # Store alert points that exceed warning threshold
            if deviation > self.warning_threshold:
                alert_points.append({"s": s, "d": d_perceived, "deviation": deviation})

        deviations = np.array(deviations)

        max_deviation = np.max(deviations) if len(deviations) > 0 else 0.0
        mean_deviation = np.mean(deviations) if len(deviations) > 0 else 0.0

        has_warning = max_deviation > self.warning_threshold
        has_critical = max_deviation > self.critical_threshold

        result = {
            "has_warning": has_warning,
            "has_critical": has_critical,
            "max_deviation": max_deviation,
            "mean_deviation": mean_deviation,
            "deviations": deviations,
            "alert_points": alert_points,
        }

        # Store in history
        self.alert_history.append(
            {
                "step": len(self.alert_history),
                "car_state": car_state,
                "result": result,
            }
        )

        return result

    def _empty_result(self) -> Dict:
        """
        Return empty result dictionary.

        Returns
        -------
        Dict
            Empty result with no alerts.
        """
        return {
            "has_warning": False,
            "has_critical": False,
            "max_deviation": 0.0,
            "mean_deviation": 0.0,
            "deviations": np.array([]),
            "alert_points": [],
        }

    def get_recent_alerts(self, num_recent: int = 10) -> List[Dict]:
        """
        Get recent alert history.

        Parameters
        ----------
        num_recent : int, default=10
            Number of recent alerts to return.

        Returns
        -------
        List[Dict]
            List of recent alert history entries.
        """
        return self.alert_history[-num_recent:] if num_recent > 0 else []

    def reset_history(self) -> None:
        """Clear alert history."""
        self.alert_history = []
