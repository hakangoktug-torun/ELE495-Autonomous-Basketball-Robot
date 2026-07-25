# ELE495 Autonomous Basketball Robot

**TOBB University of Economics and Technology — Electrical-Electronics Engineering**
**ELE495 Capstone Design Project — Summer 2026 — Group 9**

An autonomous mobile robot that navigates a miniature basketball court and
shoots ping-pong balls through a hoop without any human control during the
run. The robot detects its position on the court through onboard sensors,
aims itself using a closed-loop heading controller, and reports every made
shot to a live web dashboard.

## Team

- Zeynep Sude Sezgin
- Ahmet Selim Gök
- Hakan Göktuğ Torun
- Kuzey Torçuk

## How It Works

The court is an 80×120 cm surface divided into a **red zone (3 points)** and
a **green zone (2 points)**, with a single hoop mounted on a backboard panel.
Within a 5-minute run, the robot autonomously:

1. Drives to a series of pre-mapped shooting positions using closed-loop
   heading control (BNO055 IMU) and ultrasonic distance checks instead of
   fixed timing, so each position is reached the same way every run.
2. Verifies which colored zone it has entered using an onboard color
   sensor, as a safety check on top of the planned route.
3. Aims by rotating in small increments ("sweep") around each shooting
   position to compensate for small heading errors, then spins up the
   flywheel shooter and releases a ball.
4. Detects made shots with a break-beam sensor pair mounted on the
   backboard and reports them wirelessly to the base station, which
   updates the score in real time.
5. Repeats for every position in the route until the time limit is
   reached or the operator triggers an emergency stop.

## Hardware

| Component | Role |
|---|---|
| Raspberry Pi (3B) | Main controller — runs all navigation, scoring, and GUI logic in Python |
| Arduino Uno | Sensor bridge — reads IMU/color/ultrasonic sensors and streams them to the Pi over serial |
| BNO055 9-DOF IMU | Closed-loop heading feedback for turns and straight-line driving |
| TCS34725 color sensor | Detects which scoring zone (red/green) the robot is in |
| HC-SR04 ultrasonic sensor | Distance-based stopping at shooting positions and obstacle/backboard proximity |
| 4WD DC drivetrain + L298N driver | Chassis propulsion |
| 2× brushless motors + ESC (pigpio PWM control) | Flywheel-style ball shooter |
| Arduino Uno R4 WiFi | Mounted on the backboard; its onboard WiFi co-processor reports break-beam "ball through hoop" events to the Raspberry Pi over the local network |
| 3S LiPo (11.1V) | Chassis and Arduino Uno power |
| 2S LiPo (7.4V) | Shooter motor power |

## Software Architecture

All robot-side code is Python, running on the Raspberry Pi, with a Flask
web application acting as the operator interface.

```text
software/raspberry_pi/
  robot_bridge.py                     Serial reader thread; exposes the latest
                                       heading, distance, color, and voltage
                                       readings from the Arduino sensor bridge
  kalibrasyon_kodlari/
    donus_hassas.py                   Closed-loop turn controller (BNO055
                                       feedback, early-stop + braking, small-
                                       angle pulse mode)
    donus_kapali_dongu.py             Shared low-level motor/GPIO setup and
                                       heading-diff helpers used across the
                                       calibration scripts
    test_surus.py                     Straight-line driving helpers, obstacle-
                                       distance stopping, PI heading correction
    ozel_navigasyon_testi_esc.py      Earlier manual-angle navigation route;
                                       still supplies shared color-zone and
                                       sensor-check helpers to the current route
    ozel_navigasyon_testi_esc_sweep_2.py
                                       Main autonomous shooting route in use:
                                       fixed shooting positions, angular sweep
                                       aiming, ultrasonic-based transitions,
                                       color verification
    atici_esc_kontrol_pigpio_2.py     Brushless shooter ESC control (pigpio)
    skor_dinleyici.py                 Bridges break-beam scoring events from
                                       the backboard Arduino into the
                                       navigation loop

    # Standalone calibration / diagnostic tools (not part of the live route;
    # used ad hoc during development and tuning)
    bno055_kalibrasyon_izle.py        Live BNO055 calibration status monitor
    donus_test_saga_90.py             Fixed single-turn test: 90° right
    donus_test_sola_90.py             Fixed single-turn test: 90° left
    donus_test_sola_180.py            Fixed single-turn test: 180° left
    donus_test_sola_360.py            Fixed single-turn test: 360° (full circle)
    kucuk_aci_sol_test.py             Small-angle left-turn accuracy test
    kalibrasyon_trim.py               Left/right motor trim (speed balance) tuning
    kalibrasyonu_kaydet.py            Saves BNO055 calibration offsets to
                                       Arduino EEPROM
    esc_hiz_kontrolcusu.py            ESC speed controller used by the
                                       interactive GUI (app_esc_interaktif.py)
                                       to accept and apply live shooter ESC
                                       speed changes during a run
    interaktif_kare_test.py           Interactive, step-through square-route test
    hareket_guvenilirlik_testi.py     Movement repeatability/reliability test
    ozel_navigasyon_testi_sweep.py    Earlier sweep-route iteration, superseded
                                       by ozel_navigasyon_testi_esc_sweep_2.py
    ozel_test_surus_esc.py            Combined driving + shooter ESC test script
    renk_izle.py                      Live color sensor monitor
    ultrasonik_test.py                Live ultrasonic sensor monitor
    voltaj_izle.py                    Monitors the 5V rail the Arduino supplies
                                       to the sensors, to catch under-voltage
                                       conditions before they cause bad readings

software/flask_gui/
  app.py                              Flask server: runs the navigation route
                                       in a background thread, exposes REST
                                       endpoints for status polling, starting
                                       the run, and emergency stop
  app_esc_interaktif.py               Alternate server variant that pauses at
                                       each shooting position to accept a
                                       live shooter ESC speed value from the
                                       operator, instead of using fixed speeds
  templates/, static/                 Live operator dashboard: score, current
                                       robot action (turning / driving /
                                       shooting), position progress, and full
                                       action history
```

The Arduino Uno firmware (`arduino_combined_bridge.ino`) polls all sensors on
a fixed cycle and streams them to the Raspberry Pi as CSV over USB serial at
115200 baud. The Raspberry Pi never talks to the sensors directly — it only
talks to the Arduino, and the Arduino never makes navigation decisions.

## Operator Interface

The Flask dashboard is the only interface used during a run. It shows,
updated live:

- **Score** and number of successful shots detected
- **Current robot action** — turning, driving forward/backward, or actively
  shooting/sweeping — derived from the live event stream
- **Current shooting position** and which scoring zone the robot is in
- **Full action history** — a timestamped log of every command and
  measurement the robot has acted on
- **Remaining time** in the 5-minute run, with an automatic stop when it
  expires
- An **emergency stop** button that halts all motors and the shooter
  immediately, from any point in the route

## Repository Structure

```text
docs/
  Gantt/           Project planning documents
  Presentations/   Presentation files
  Reports/         Reports and architecture notes
  WBS/             Work breakdown structure

hardware/          Wiring, pin map, and mechanical notes
media/             Photos and videos

software/
  flask_gui/       Flask web control panel
  raspberry_pi/    Robot control, navigation, vision, and shooting modules
    kalibrasyon_kodlari/   Live navigation/shooting route plus the standalone
                           calibration and diagnostic scripts used during
                           development — see Software Architecture above for
                           the full breakdown of what each file does
```

## Requirements Traceability

The official project criteria and instructor Q&A are tracked against the
current implementation in:

```text
docs/Reports/requirements_compliance.md
```

That checklist records which requirements are already covered by the
repository and which are still open.

## Running the Flask Interface

```bash
pip install -r requirements.txt
python -m software.flask_gui.app
```

Then open:

```text
http://localhost:5000
```

On machines other than the Raspberry Pi (i.e. without the real motor drivers,
sensors, and shooter mechanism connected), the interface will load but the
autonomous run cannot be started until a live sensor connection is available.

## Demo Constraints

- 5-minute time limit per run, with at most 2 attempts.
- Material cost capped at 20,000 TL (incl. VAT).
- All autonomous decision-making runs on the robot's own onboard processor —
  no external PC/Mac is used during a run.
