"""Ping-pong ball shooting mechanism control."""

from dataclasses import dataclass


@dataclass
class ShotCommand:
    wheel_speed: float
    feeder_active: bool


class Shooter:
    def __init__(self, default_speed: float = 0.65) -> None:
        self.default_speed = default_speed
        self.armed = False
        self.last_command = ShotCommand(wheel_speed=0.0, feeder_active=False)

    def arm(self, speed: float | None = None) -> ShotCommand:
        self.armed = True
        self.last_command = ShotCommand(
            wheel_speed=self._clamp(speed if speed is not None else self.default_speed),
            feeder_active=False,
        )
        return self.last_command

    def fire(self) -> ShotCommand:
        if not self.armed:
            self.arm()
        self.last_command = ShotCommand(
            wheel_speed=self.last_command.wheel_speed,
            feeder_active=True,
        )
        return self.last_command

    def stop(self) -> ShotCommand:
        self.armed = False
        self.last_command = ShotCommand(wheel_speed=0.0, feeder_active=False)
        return self.last_command

    def speed_for_distance(self, distance_cm: float) -> float:
        return self._clamp(0.45 + (distance_cm / 120.0) * 0.35)

    @staticmethod
    def _clamp(value: float) -> float:
        return max(0.0, min(1.0, value))
