"""Central configuration for the Raspberry Pi robot controller."""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class CourtConfig:
    width_cm: float = 80.0
    length_cm: float = 120.0
    hoop_x_cm: float = 40.0
    hoop_y_cm: float = 116.0


@dataclass(frozen=True)
class PinConfig:
    left_motor_pwm: int = 12
    left_motor_dir: int = 5
    right_motor_pwm: int = 13
    right_motor_dir: int = 6
    shooter_pwm: int = 18
    feeder_servo: int = 23
    front_distance_sensor: int = 24


@dataclass(frozen=True)
class RobotConfig:
    court: CourtConfig = field(default_factory=CourtConfig)
    pins: PinConfig = field(default_factory=PinConfig)
    dry_run: bool = True
    max_drive_speed: float = 1.0
    default_shooter_speed: float = 0.65


DEFAULT_CONFIG = RobotConfig()
