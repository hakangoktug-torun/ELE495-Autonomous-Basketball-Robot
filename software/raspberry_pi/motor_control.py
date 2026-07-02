"""Drive base abstractions.

SimulatedDriveBase: hardware-safe, records commands without touching GPIO.
RPiDriveBase: real L298N driver over RPi.GPIO. Assumes each side has one
PWM (speed) pin and one direction pin — matches PinConfig as currently
defined. If your L298N wiring uses two direction pins per side (IN1+IN2)
instead of one, tell me and I'll adjust this class.
"""

from dataclasses import dataclass

from software.raspberry_pi.config import PinConfig


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

    def close(self) -> None:
        pass

    def _clamp(self, value: float) -> float:
        return max(-self.max_speed, min(self.max_speed, value))


class RPiDriveBase:
    def __init__(self, pins: PinConfig, max_speed: float = 1.0, pwm_freq_hz: int = 1000) -> None:
        import RPi.GPIO as GPIO  # imported here so this file still imports cleanly off-device

        self._GPIO = GPIO
        self.pins = pins
        self.max_speed = max_speed
        self.command = DriveCommand()
        self.emergency_locked = False

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pins.left_motor_pwm, GPIO.OUT)
        GPIO.setup(pins.left_motor_dir, GPIO.OUT)
        GPIO.setup(pins.right_motor_pwm, GPIO.OUT)
        GPIO.setup(pins.right_motor_dir, GPIO.OUT)

        self._left_pwm = GPIO.PWM(pins.left_motor_pwm, pwm_freq_hz)
        self._right_pwm = GPIO.PWM(pins.right_motor_pwm, pwm_freq_hz)
        self._left_pwm.start(0)
        self._right_pwm.start(0)

    def move(self, left_speed: float, right_speed: float) -> DriveCommand:
        if self.emergency_locked:
            return self.stop()

        left_speed = self._clamp(left_speed)
        right_speed = self._clamp(right_speed)
        GPIO = self._GPIO

        GPIO.output(self.pins.left_motor_dir, GPIO.HIGH if left_speed >= 0 else GPIO.LOW)
        GPIO.output(self.pins.right_motor_dir, GPIO.HIGH if right_speed >= 0 else GPIO.LOW)

        self._left_pwm.ChangeDutyCycle(abs(left_speed) * 100.0)
        self._right_pwm.ChangeDutyCycle(abs(right_speed) * 100.0)

        self.command = DriveCommand(left_speed=left_speed, right_speed=right_speed)
        return self.command

    def stop(self) -> DriveCommand:
        self._left_pwm.ChangeDutyCycle(0)
        self._right_pwm.ChangeDutyCycle(0)
        self.command = DriveCommand()
        return self.command

    def emergency_stop(self) -> DriveCommand:
        self.emergency_locked = True
        return self.stop()

    def reset_emergency(self) -> None:
        self.emergency_locked = False

    def close(self) -> None:
        self._left_pwm.stop()
        self._right_pwm.stop()
        self._GPIO.cleanup()

    def _clamp(self, value: float) -> float:
        return max(-self.max_speed, min(self.max_speed, value))