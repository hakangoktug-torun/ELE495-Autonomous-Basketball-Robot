# Hardware Overview

This document describes the physical components, wiring, and inter-board
communication used by the ELE495 autonomous basketball robot. It is meant to
sit alongside the top-level `README.md` and covers everything under
`hardware/` at a level of detail sufficient to rebuild or debug the wiring
without reading the firmware source.

## System Block Diagram (textual)

```text
                     ┌───────────────────────────┐
                     │   Raspberry Pi (main)      │
                     │   - navigation logic       │
                     │   - Flask web dashboard    │
                     └─────────┬─────────┬────────┘
                USB serial     │         │  GPIO (BCM)
              115200 baud      │         │
                     ┌─────────▼───┐     │
                     │ Arduino Uno │     ├── L298N motor driver (4WD chassis)
                     │ sensor      │     │
                     │ bridge      │     └── ESC (pigpio PWM) → brushless
                     └──┬───┬───┬──┘         shooter motors
                        │   │   │
                     BNO055 TCS34725 HC-SR04
                     (IMU)  (color)  (ultrasonic)

                     ┌───────────────────────────┐
                     │ Arduino Uno R4 WiFi        │
                     │ (mounted on backboard)     │
                     │ - 4x break-beam sensors    │
                     │ - reports made shots over  │
                     │   WiFi (HTTP GET) to the   │
                     │   Raspberry Pi's Flask     │
                     │   server                   │
                     └───────────────────────────┘
```

Two separate Arduinos are used, and they never talk to each other directly:

- The **Arduino Uno** is a dedicated sensor bridge wired to the Raspberry Pi
  over USB serial. It owns the IMU, color sensor, ultrasonic sensor, and (if
  populated) the IR pair, and streams them to the Pi as CSV. It never makes
  navigation decisions itself.
- The **Arduino Uno R4 WiFi** is mounted on the backboard/hoop assembly and
  is electrically independent from the drivetrain electronics. It only
  watches the break-beam sensors around the hoop and reports made shots to
  the Raspberry Pi over the local WiFi network — it has no wired connection
  to the rest of the robot.

## Bill of Materials

| Component | Notes |
|---|---|
| Raspberry Pi (3B) | Main controller, runs Python navigation code and the Flask GUI |
| Arduino Uno | Sensor bridge (IMU / color / ultrasonic / IR) |
| Arduino Uno R4 WiFi | Backboard-mounted scoring controller (break-beam + WiFi) |
| BNO055 9-DOF IMU | Absolute/relative heading feedback for turns and straight-line driving |
| TCS34725 RGB color sensor | Detects red (3-pt) vs. green (2-pt) court zone |
| HC-SR04 ultrasonic sensor | Distance-based stopping at shooting positions |
| 4x break-beam (IR emitter/receiver) pairs | Detect a ball passing through the hoop |
| 4WD DC motor chassis | Drivetrain |
| L298N dual H-bridge driver | Drives the two drivetrain motor channels |
| 2x brushless motors + ESC | Flywheel-style ball shooter, PWM-controlled via `pigpio` |
| 11.1 V 3S LiPo | Powers the chassis motors and the Arduino Uno sensor bridge |
| 7.4 V 2S LiPo | Powers the shooter ESC/motors |
| USB power bank | Powers the Raspberry Pi and the backboard Arduino Uno R4 WiFi |

## Power Distribution

- **Chassis / drive electronics** run from an **11.1 V 3S LiPo** through the
  L298N. The drivetrain motors are rated for 6 V, so they are deliberately
  run below their nameplate voltage headroom — duty cycles are tuned up
  (see the navigation code's speed constants) to keep them out of the
  marginal-torque region rather than lowering the supply voltage.
- **Shooter motors/ESC** run from a separate **7.4 V 2S LiPo**, chosen to
  keep flywheel speed (and therefore ball exit speed) inside the range
  needed for the court's shooting distances.
- **Raspberry Pi and the backboard Arduino Uno R4 WiFi** are powered from a
  USB power bank, independent of the drivetrain/shooter batteries — this
  keeps the backboard scoring unit fully decoupled from the chassis
  electrically, which matters because it only communicates over WiFi.
- **⚠️ ESC BEC warning:** the ESC's BEC (+5 V) output wire must **not** be
  connected to any Raspberry Pi GPIO or 5 V pin. Doing so back-feeds the
  Pi's 5 V rail from the ESC and can throttle or damage the Pi. Only the
  ESC's PWM signal wire and a common ground should be wired to the Pi;
  the BEC +5 V wire should be left disconnected/isolated.

## Raspberry Pi Wiring

All Raspberry Pi GPIO references use **BCM numbering**.

### Drivetrain (L298N)

| Signal | BCM GPIO |
|---|---|
| Motor A direction (IN1) | 5 |
| Motor A direction (IN2) | 6 |
| Motor B direction (IN3) | 13 |
| Motor B direction (IN4) | 26 |
| Motor A speed (ENA, PWM) | 12 |
| Motor B speed (ENB, PWM) | 16 |

Both motor channels use two direction pins each (IN1+IN2, IN3+IN4) rather
than a single direction pin — this matters if you ever try to reuse a
single-direction-pin drive abstraction, since it will not match this wiring.

### Shooter ESC

| Signal | BCM GPIO |
|---|---|
| ESC PWM signal | **17** (previously GPIO 23 in an earlier hardware revision — if you are debugging an older build, check which one is actually wired) |

The ESC is driven with a standard RC-style PWM pulse, **not** `RPi.GPIO`'s
software PWM — `RPi.GPIO`'s software-timed pulses were unstable enough that
the ESC repeatedly read them as an invalid signal (audible as a continuous/
rising arm-failure beep instead of one clean arm tone). The fix in use is
**`pigpio`**, which generates the pulse via DMA at microsecond precision
independent of CPU load:

```bash
sudo apt update
sudo apt install pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

`pigpiod` is a background daemon — `enable` only needs to be run once so it
survives reboots; if ESC control ever silently fails with a "could not
connect to pigpio" error, check that this daemon is running.

| Parameter | Value |
|---|---|
| Pulse width — motor stopped / minimum | 1000 µs |
| Pulse width — full speed | 2000 µs |
| Arm wait time (minimum-signal hold before first command) | 3.0 s |

The ESC must be **armed** before it will accept a speed command: hold the
pin at the minimum pulse width (1000 µs) for a few seconds first. Skipping
this, or sending a non-minimum pulse width on first signal, is what produces
the unstable-signal beep pattern mentioned above.

### Sensor bridge link

The Raspberry Pi talks to the Arduino Uno sensor bridge over **USB serial**
at **115200 baud**, appearing as `/dev/ttyUSB0` (confirm with `ls -l
/dev/ttyUSB*` — USB enumeration order can shift if other serial adapters are
plugged in).

## Arduino Uno Wiring (Sensor Bridge)

| Component | Pin(s) | Notes |
|---|---|---|
| BNO055 (IMU) | SDA → A4, SCL → A5 | I²C address `0x28`, shared bus |
| TCS34725 (color) | SDA → A4, SCL → A5 | I²C address `0x29`, same bus as BNO055 |
| HC-SR04 (ultrasonic) | TRIG → D9, ECHO → D10 | |
| IR sensor pins (reserved, **not populated**) | D4, D5 | These pins were set aside for a pair of IR sensors that was evaluated early in the project and **deliberately not used** on the final robot. The firmware still reserves D4/D5 for them and the CSV protocol still carries two IR fields for backward compatibility, but no physical IR sensors are wired to these pins — treat any IR values in the data stream as placeholders, not real readings. |

The Arduino streams all of these to the Raspberry Pi once per read cycle as
a single CSV line:

```text
IR1,IR2,Distance,Heading,R,G,B,C,VccMv
```

`IR1`/`IR2` are kept in the CSV purely so the format doesn't change size —
see the note above; there is no physical IR sensor behind these values on
the current robot, so don't rely on them for anything.

`VccMv` is not a battery voltage — it is the Arduino's own regulated **5 V
sensor-supply rail**, measured internally via the ATmega's ADC against its
1.1 V reference. It exists purely as a brownout/under-voltage diagnostic:
if it drops noticeably while the drivetrain motors are running, that
indicates the sensors are not getting clean power and readings (especially
the IMU) may become unreliable.

The Raspberry Pi can also send single-character commands back over the
same serial link:

| Command | Effect |
|---|---|
| `F` | Fast mode — shortens the read cycle for fresher heading data |
| `N` | Normal mode — restores the default read cycle |
| `C` | Reports current BNO055 calibration status (sys/gyro/accel/mag) |
| `S` | Saves the current BNO055 calibration offsets to EEPROM |
| `D` | Deletes the saved EEPROM calibration |
| `G` | Reports the calibration quality that was in effect when it was last saved |
| `R` | Resets/re-initializes the BNO055 (offsets and fusion mode are automatically reloaded afterward) |

### IMU fusion mode note

The BNO055 is run in **IMUPLUS** mode (gyroscope + accelerometer fusion,
magnetometer disabled) rather than the default NDOF mode. This is a
deliberate hardware/firmware co-decision: the drivetrain motors generate
enough electromagnetic interference to corrupt magnetometer-based heading
readings, and the robot only ever needs *relative* heading (turn by N
degrees from wherever it currently faces), not an absolute compass
direction. Disabling the magnetometer removes that interference path
entirely, at the cost of slow gyro drift over time (negligible over a
5-minute run). If you ever need absolute heading again, this is a one-line
change in the Arduino firmware, but be aware it reintroduces motor-EMI
sensitivity.

## Arduino Uno R4 WiFi Wiring (Scoring)

Mounted on the backboard, electrically separate from the chassis.

| Beam-break sensor | Pin | Mode |
|---|---|---|
| Sensor 1 | D2 | Interrupt (`FALLING`) |
| Sensor 2 | D3 | Interrupt (`FALLING`) |
| Sensor 3 | D4 | Polled |
| Sensor 4 | D5 | Polled |

Four break-beam pairs are arranged around the hoop opening so that a ball
passing through triggers at least one of them regardless of exact entry
angle. When any sensor detects a pass, the board sends:

```text
GET /skor?sensor=<N> HTTP/1.1
```

to the Raspberry Pi's Flask server (default `RPI_PORT = 5000`) over the
WiFi network both devices share (currently a mobile hotspot — see the
firmware's `WIFI_SSID`/`WIFI_PASS`/`RPI_IP` constants). Because the hotspot
assigns the Raspberry Pi's IP dynamically, `RPI_IP` must be re-checked and
updated in the firmware (`hostname -I` on the Pi) any time the hotspot
connection is re-established and a new address is handed out; a static IP
reservation on the hotspot, if available, removes this maintenance step
permanently.

A single shared cooldown window (1.0–1.5 s depending on firmware revision —
keep the Arduino and the Raspberry Pi's `skor_dinleyici.py` values in sync if
you change either) prevents one ball bouncing off the rim and re-triggering
a second sensor from being counted as two separate made shots.

## Court and Hoop Mechanical Reference

- Playing surface: 80 × 120 cm (cardboard or similar), divided into a red
  zone (3 points) and a green zone (2 points), with a marked start area.
- Hoop panel: 20 × 15 × 1 cm backboard panel, hoop diameter 15 cm.
- Hoop height: 20–30 cm above the playing surface, mounted level
  (parallel to the ground).
- The break-beam scoring board is mounted directly behind/around this hoop
  opening.

## Known Hardware Caveats

- **ESC BEC back-powering** — see the power distribution warning above;
  this has caused Raspberry Pi throttling in past testing and must not be
  re-introduced.
- **Motor voltage headroom** — the drivetrain motors are nominally rated
  well below the 11.1 V supply; this is intentional and compensated for in
  software (duty cycle tuning), not a wiring fault.
- **I²C bus sharing** — the BNO055 and TCS34725 share the same SDA/SCL
  lines at different addresses (`0x28` / `0x29`); if you add another I²C
  device, confirm its address doesn't collide with either.
- **USB serial enumeration** — the sensor-bridge Arduino Uno is assumed to
  appear as `/dev/ttyUSB0`. If other USB-serial devices are plugged into
  the Pi, this can shift; always verify with `ls -l /dev/ttyUSB*` after a
  reboot or replug.
- **Dynamic hotspot IP** — the backboard Arduino Uno R4 WiFi's target
  `RPI_IP` is hardcoded and must be updated whenever the hotspot hands the
  Raspberry Pi a new address (see the scoring wiring section above).
- **`pigpiod` must be running for the shooter to work** — the ESC is driven
  through the `pigpio` daemon, not directly through `RPi.GPIO`. If the
  shooter fails to arm or throws a "could not connect to pigpio" error,
  check `sudo systemctl status pigpiod` before assuming a wiring fault.
- **IR sensor pins are reserved but unpopulated** — D4/D5 on the sensor
  bridge Arduino and the corresponding CSV fields exist only for backward
  compatibility with an earlier design iteration; no IR sensors are
  actually installed on the robot. Don't spend debugging time on "IR
  readings look wrong" — they aren't connected to anything.
