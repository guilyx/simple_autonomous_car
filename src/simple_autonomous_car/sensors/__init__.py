"""Sensor system for autonomous vehicles."""

from simple_autonomous_car.sensors.base_sensor import BaseSensor
from simple_autonomous_car.sensors.lidar_sensor import LiDARSensor

__all__ = ["BaseSensor", "LiDARSensor"]
