# Hardware Notes

## Target Controller

The target controller is Raspberry Pi. Arduino and ESP32 are not part of the
target implementation.

## Initial Pin Map

| Function | GPIO |
| --- | ---: |
| Left motor PWM | 12 |
| Left motor direction | 5 |
| Right motor PWM | 13 |
| Right motor direction | 6 |
| Shooter PWM | 18 |
| Feeder servo | 23 |
| Front distance sensor | 24 |

The pin map is a starting point and must be checked against the final motor
driver, sensor voltage levels, and Raspberry Pi model before powering hardware.

## Safety Requirements

- Use a motor driver compatible with Raspberry Pi logic levels.
- Keep motor and Raspberry Pi power rails properly isolated or regulated.
- Add a physical emergency stop before full-speed testing.
- Test the shooter mechanism without balls before loading ping-pong balls.
