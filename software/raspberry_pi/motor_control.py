"""Drive base abstraction.

The current implementation is intentionally hardware-safe: it records commands
without touching GPIO. Raspberry Pi GPIO adapters can be added behind this
interface when the wiring is finalized.
"""

from dataclasses import dataclass


@dataclass
class DriveCommand:
    left_speed: float = 0.0
    right_speed: float = 0.0


class SimulatedDriveBase:
    def __init__(self, max_speed: float = 1.0) -> None:
        self.max_speed = max_speed
        self.command = DriveCommand()
        self.emergency_locked = False

    def move(self, left_speed: float, right_speed: float) -> DriveCommand:
        if self.emergency_locked:
            return self.stop()

        self.command = DriveCommand(
            left_speed=self._clamp(left_speed),
            right_speed=self._clamp(right_speed),
        )
        return self.command

    def stop(self) -> DriveCommand:
        self.command = DriveCommand()
        return self.command

    def emergency_stop(self) -> DriveCommand:
        self.emergency_locked = True
        return self.stop()

    def reset_emergency(self) -> None:
        self.emergency_locked = False

    def _clamp(self, value: float) -> float:
        return max(-self.max_speed, min(self.max_speed, value))
