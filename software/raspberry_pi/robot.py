"""High-level robot state and command orchestration."""

import math
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from enum import Enum

from software.raspberry_pi.config import RobotConfig
from software.raspberry_pi.motor_control import RPiDriveBase, SimulatedDriveBase
from software.raspberry_pi.navigation import Navigator, Pose
from software.raspberry_pi.sensors import ArduinoDistanceSensors, SimulatedDistanceSensors
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
    simulated_motion: bool
    court_width_cm: float
    court_length_cm: float
    score: int
    battery_percent: int
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
        self.pose = Pose(config.court.width_cm / 2.0, 10.0, 90.0)

        # simulated_motion = True means: wheels never actually turn, pose is
        # instead integrated from commanded speeds. True for pure dry_run
        # bench testing AND for the "real sensors, fake wheels" test mode.
        self.simulated_motion = config.dry_run or config.simulate_motion

        if config.dry_run:
            self.drive = SimulatedDriveBase(max_speed=config.max_drive_speed)
            self.sensors = SimulatedDistanceSensors()
        elif config.simulate_motion:
            self.drive = SimulatedDriveBase(max_speed=config.max_drive_speed)
            self.sensors = ArduinoDistanceSensors(port=config.serial_port, baudrate=config.serial_baudrate)
        else:
            self.drive = RPiDriveBase(pins=config.pins, max_speed=config.max_drive_speed)
            self.sensors = ArduinoDistanceSensors(port=config.serial_port, baudrate=config.serial_baudrate)

        self.navigator = Navigator(config.court)
        self.shooter = Shooter(default_speed=config.default_shooter_speed)
        self.vision = VisionSystem()
        self.score = 0
        self.battery_percent = 92
        self.task_history: list[str] = []

        self._pose_lock = threading.Lock()
        self._sim_running = self.simulated_motion
        self._sim_thread = None
        if self.simulated_motion:
            self._sim_thread = threading.Thread(target=self._simulate_motion_loop, daemon=True)
            self._sim_thread.start()

        self._record(
            f"Robot controller initialized (dry_run={config.dry_run}, "
            f"simulate_motion={config.simulate_motion})"
        )

    def _simulate_motion_loop(self) -> None:
        tick_seconds = 0.1
        last = time.monotonic()
        while self._sim_running:
            time.sleep(tick_seconds)
            now = time.monotonic()
            dt = now - last
            last = now
            self._integrate_pose(dt)

    def _integrate_pose(self, dt: float) -> None:
        # Simple differential-drive kinematics. Purely for visualization -
        # not a substitute for real odometry once encoders/IMU exist.
        cmd = self.drive.command
        v_left = cmd.left_speed * self.config.sim_max_speed_cms
        v_right = cmd.right_speed * self.config.sim_max_speed_cms
        v = (v_left + v_right) / 2.0
        omega_deg_per_s = math.degrees(
            (v_right - v_left) / self.config.sim_wheelbase_cm
        )

        with self._pose_lock:
            heading_rad = math.radians(self.pose.heading_deg)
            new_x = self.pose.x_cm + v * math.cos(heading_rad) * dt
            new_y = self.pose.y_cm + v * math.sin(heading_rad) * dt
            new_heading = self.pose.heading_deg + omega_deg_per_s * dt
            self.pose = self.navigator.clamp_pose(Pose(new_x, new_y, new_heading))

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
        with self._pose_lock:
            pose = self.pose
        return RobotStatus(
            mode=self.mode,
            pose=pose,
            dry_run=self.config.dry_run,
            simulated_motion=self.simulated_motion,
            court_width_cm=self.config.court.width_cm,
            court_length_cm=self.config.court.length_cm,
            score=self.score,
            battery_percent=self.battery_percent,
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

    def close(self) -> None:
        self._sim_running = False
        self.drive.stop()
        self.sensors.close()

    def _record(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.task_history.insert(0, f"{timestamp} - {message}")
        self.task_history = self.task_history[:20]