"""Navigation helpers for an 80x120 cm basketball court."""

from dataclasses import dataclass
from math import hypot

from software.raspberry_pi.config import CourtConfig


@dataclass
class Pose:
    x_cm: float
    y_cm: float
    heading_deg: float = 0.0


@dataclass(frozen=True)
class ScoringZone:
    name: str
    x_cm: float
    y_cm: float


class Navigator:
    def __init__(self, court: CourtConfig) -> None:
        self.court = court
        self.scoring_zones = (
            ScoringZone("left", 20.0, 75.0),
            ScoringZone("center", 40.0, 70.0),
            ScoringZone("right", 60.0, 75.0),
        )

    def clamp_pose(self, pose: Pose) -> Pose:
        return Pose(
            x_cm=max(0.0, min(self.court.width_cm, pose.x_cm)),
            y_cm=max(0.0, min(self.court.length_cm, pose.y_cm)),
            heading_deg=pose.heading_deg % 360.0,
        )

    def distance_to_hoop(self, pose: Pose) -> float:
        return hypot(self.court.hoop_x_cm - pose.x_cm, self.court.hoop_y_cm - pose.y_cm)

    def nearest_scoring_zone(self, pose: Pose) -> ScoringZone:
        return min(
            self.scoring_zones,
            key=lambda zone: hypot(zone.x_cm - pose.x_cm, zone.y_cm - pose.y_cm),
        )
