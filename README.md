# ELE495 Autonomous Basketball Robot

## Project Description

This repository contains the software and documentation for an ELE495 senior
design project: a Raspberry Pi based autonomous mobile robot that moves on an
80x120 cm miniature basketball court and shoots ping-pong balls toward a hoop.

## Core Rules

- ESP32 is used within Arduino Uno R4 Wifi Module to connect beam break sensors to RPi via Wifi. 
- Arduino Uno is used for ultrasonic distance, BNO055 IMU and RGB detection sensors
- The main processor is Raspberry Pi.
- Robot software is written in Python.
- The operator interface is built with Flask.
- The existing repository structure is preserved.

## Technologies

- Arduino Uno
- Arduino Uno R4 Wifi
- Raspberry Pi
- Python
- Flask
- GPIO-compatible motor, sensor, and shooting mechanism control

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
  raspberry_pi/    
  arduino/         Beam break sensors, bridge connection between RPi and Uno.
```

## Requirements Traceability

The project criteria and question-answer PDFs are summarized in:

```text
docs/Reports/requirements_compliance.md
```

That checklist tracks which requirements are covered by the current repository
and which items still need implementation or documentation.

## Running the Flask Interface

```bash
pip install -r requirements.txt
python -m software.flask_gui.app
```

Then open:

```text
http://localhost:5000
```

## Running Tests

```bash
pytest
```

## Implementation Notes

GPIO access is intentionally abstracted behind Python classes. The default
configuration runs in `dry_run` mode so the project can be developed and tested
on non-Raspberry Pi machines before real motor drivers, sensors, and the shooter
mechanism are connected.
