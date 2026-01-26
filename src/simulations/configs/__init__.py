"""Simulation configuration files."""

from .grid_map import CONFIG as GRID_MAP_CONFIG
from .race_track import CONFIG as RACE_TRACK_CONFIG
from .simple_track import CONFIG as SIMPLE_TRACK_CONFIG

__all__ = ["SIMPLE_TRACK_CONFIG", "RACE_TRACK_CONFIG", "GRID_MAP_CONFIG"]
