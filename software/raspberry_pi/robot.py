"""High-level robot state and command orchestration."""

from dataclasses import asdict, dataclass
from datetime import datetime
from enum import Enum

from software.raspberry_pi.config import RobotConfig
from software.raspberry_pi.motor_control import SimulatedDriveBase
from software.raspberry_pi.navigation import Navigator, Pose
from software.raspberry_pi.sensors import SimulatedDistanceSensors
from software.raspberry_pi.shooting import Shooter
from software.raspberry_pi.vision import VisionSystem


class RobotMode(str, Enum):
    IDLE = "idle"
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    ERROR = "error"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class RobotStatus:
    mode: RobotMode
    pose: Pose
    dry_run: bool
    score: int
    task_history: list[str]
    front_clear: bool
    drive_left: float
    drive_right: float
    shooter_armed: bool
    shooter_speed: float


class RobotController:
    def __init__(self, config: RobotConfig) -> None:
        self.config = config
        self.mode = RobotMode.IDLE
        self.pose = Pose(config.court.width_cm / 2.0, 10.0, 0.0)
        self.drive = SimulatedDriveBase(max_speed=config.max_drive_speed)
        self.sensors = SimulatedDistanceSensors()
        self.navigator = Navigator(config.court)
        self.shooter = Shooter(default_speed=config.default_shooter_speed)
        self.vision = VisionSystem()
        self.score = 0
        self.task_history: list[str] = []
        self._record("Robot controller initialized")

    def set_mode(self, mode: RobotMode | str) -> RobotStatus:
        next_mode = RobotMode(mode)
        if self.mode == RobotMode.EMERGENCY_STOP and next_mode != RobotMode.IDLE:
            self._record("Mode change blocked by emergency stop")
            return self.status()
        self.mode = next_mode
        if self.mode == RobotMode.IDLE:
            self.drive.stop()
        self._record(f"Mode set to {self.mode.value}")
        return self.status()

    def manual_drive(self, left_speed: float, right_speed: float) -> RobotStatus:
        if self.mode != RobotMode.EMERGENCY_STOP:
            self.mode = RobotMode.MANUAL
            self.drive.move(left_speed, right_speed)
            self._record(f"Manual drive left={left_speed:.2f} right={right_speed:.2f}")
        return self.status()

    def autonomous_step(self) -> RobotStatus:
        if self.mode == RobotMode.EMERGENCY_STOP:
            return self.status()

        readings = self.sensors.read()
        self.mode = RobotMode.AUTONOMOUS
        if not readings.front_clear:
            self.drive.stop()
            self._record("Autonomous step stopped because front obstacle is detected")
            return self.status()

        zone = self.navigator.nearest_scoring_zone(self.pose)
        x_error = zone.x_cm - self.pose.x_cm
        turn = max(-0.35, min(0.35, x_error / self.config.court.width_cm))
        self.drive.move(0.35 - turn, 0.35 + turn)
        self._record(f"Autonomous step toward {zone.name} scoring zone")
        return self.status()

    def shoot(self) -> RobotStatus:
        if self.mode == RobotMode.EMERGENCY_STOP:
            return self.status()

        distance = self.navigator.distance_to_hoop(self.pose)
        self.shooter.arm(self.shooter.speed_for_distance(distance))
        self.shooter.fire()
        self._record(f"Shot command fired from {distance:.1f} cm")
        return self.status()

    def add_score(self, points: int) -> RobotStatus:
        if points <= 0:
            self._record("Score update ignored because points must be positive")
            return self.status()

        self.score += points
        self._record(f"Score updated by +{points}; total={self.score}")
        return self.status()

    def reset_score(self) -> RobotStatus:
        self.score = 0
        self._record("Score reset to 0")
        return self.status()

    def emergency_stop(self) -> RobotStatus:
        self.mode = RobotMode.EMERGENCY_STOP
        self.drive.emergency_stop()
        self.shooter.stop()
        self._record("Emergency stop activated")
        return self.status()

    def reset_emergency(self) -> RobotStatus:
        self.drive.reset_emergency()
        self.mode = RobotMode.IDLE
        self._record("Emergency stop reset; mode set to idle")
        return self.status()

    def status(self) -> RobotStatus:
        readings = self.sensors.read()
        return RobotStatus(
            mode=self.mode,
            pose=self.pose,
            dry_run=self.config.dry_run,
            score=self.score,
            task_history=list(self.task_history),
            front_clear=readings.front_clear,
            drive_left=self.drive.command.left_speed,
            drive_right=self.drive.command.right_speed,
            shooter_armed=self.shooter.armed,
            shooter_speed=self.shooter.last_command.wheel_speed,
        )

    def status_dict(self) -> dict:
        data = asdict(self.status())
        data["mode"] = self.mode.value
        return data

    def _record(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.task_history.insert(0, f"{timestamp} - {message}")
        self.task_history = self.task_history[:20]
