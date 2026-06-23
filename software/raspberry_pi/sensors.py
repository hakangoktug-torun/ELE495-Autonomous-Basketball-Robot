"""Sensor abstractions used by the robot controller."""

from dataclasses import dataclass


@dataclass
class DistanceReadings:
    front_clear: bool = True
    front_distance_cm: float | None = None


class SimulatedDistanceSensors:
    def __init__(self) -> None:
        self.readings = DistanceReadings()

    def read(self) -> DistanceReadings:
        return self.readings

    def set_front_obstacle(self, distance_cm: float | None) -> None:
        self.readings = DistanceReadings(
            front_clear=distance_cm is None,
            front_distance_cm=distance_cm,
        )
