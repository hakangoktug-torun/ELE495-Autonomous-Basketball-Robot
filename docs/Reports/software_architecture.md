# Software Architecture

## Goal

The robot software controls a Raspberry Pi based autonomous ping-pong ball
shooting robot for an 80x120 cm basketball court.

## Runtime Layers

1. Flask web interface receives operator commands.
2. `AtisTestiKontrolcusu` owns the GUI-facing test state, score, task log,
   emergency-stop flag, and 5-minute demo timer.
3. `RobotBridge` reads the Arduino sensor stream over serial and exposes
   heading, distance, IR, RGB, and voltage data to the Raspberry Pi code.
4. `ozel_navigasyon_testi_esc_sweep_2.py` runs the autonomous route, closed-loop
   turns, sweep shooting behavior, and scoring-window coordination.
5. `SkorDinleyici` receives break-beam score events from Arduino R4 WiFi through
   the Flask `/skor` route.
6. ESC and motor control modules drive the shooter and L298N motor driver on the
   Raspberry Pi.

## Robot Modes

- `beklemede`: Flask is running and waits for the operator to start the test.
- `baslatiliyor`: the GUI requested a new autonomous test run.
- `calisiyor`: the autonomous route is active.
- `girdi bekleniyor`: the interactive ESC GUI is waiting for a shooter speed.
- `hata`: sensor, serial, or hardware initialization failed.
- `acil durdur`: motors, scoring, and ESC are being stopped as quickly as possible.

## Near-Term Development Tasks

- Keep Flask `app.py` and `app_esc_interaktif.py` aligned where they share route
  and emergency-stop behavior.
- Move WiFi credentials and robot IP values out of Arduino source files.
- Document the active Arduino/Raspberry Pi wiring and serial protocol.
- Add lightweight smoke-check commands for Flask imports and GUI routes.
- Tune RGB thresholds, ESC speeds, and sweep angles with measured demo data.
