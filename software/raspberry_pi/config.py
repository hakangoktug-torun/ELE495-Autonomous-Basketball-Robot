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
    serial_port: str = "/dev/ttyUSB0"
    serial_baudrate: int = 115200
    dry_run: bool = True
    # If True (and dry_run is False): real sensors are read from Arduino,
    # but the drive base is simulated - wheels never actually turn.
    # Pose is instead integrated from commanded wheel speeds, so the GUI
    # can show the robot "moving" for bench testing without hardware risk.
    simulate_motion: bool = False
    max_drive_speed: float = 1.0
    default_shooter_speed: float = 0.65
    # Kinematics used only for pose simulation (dry_run or simulate_motion).
    # Tune sim_max_speed_cms once you know your real top speed at full PWM.
    sim_max_speed_cms: float = 25.0
    sim_wheelbase_cm: float = 15.0


DEFAULT_CONFIG = RobotConfig()

# Real sensors, fake wheels - the "sensors-in-the-loop" bench test mode.
SIMULATION_CONFIG = RobotConfig(dry_run=False, simulate_motion=True)

# Real sensors, real wheels - full hardware mode.
LIVE_CONFIG = RobotConfig(dry_run=False, simulate_motion=False)