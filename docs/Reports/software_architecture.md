# Software Architecture

## Goal

The robot software controls a Raspberry Pi based autonomous ping-pong ball
shooting robot for an 80x120 cm basketball court.

## Runtime Layers

1. Flask web interface receives operator commands.
2. `RobotController` validates commands and owns the robot state.
3. Navigation, shooting, vision, sensors, and motor modules provide focused
   subsystem behavior.
4. GPIO-specific implementations can be added behind the existing interfaces
   after the final wiring is known.

## Robot Modes

- `idle`: motors are stopped and the robot waits for a command.
- `manual`: operator controls left and right drive speeds.
- `autonomous`: robot computes a simple navigation step toward a scoring zone.
- `error`: reserved for subsystem failures.
- `emergency_stop`: drive and shooter are stopped until reset.

## Near-Term Development Tasks

- Replace simulated drive and sensor classes with Raspberry Pi GPIO adapters.
- Add OpenCV hoop detection.
- Add camera calibration and court coordinate mapping.
- Tune shooter wheel speed with measured distance-to-hoop data.
- Add safety checks for low battery, obstacle detection, and stalled motors.
