# ELE495 Requirements Compliance Checklist

This checklist summarizes the requirements extracted from the 2026 Summer ELE
495 project criteria document and the published question-answer document.

## Project Scope

| Requirement | Current repo status | Notes |
| --- | --- | --- |
| Robot must play on an 80x120 cm miniature basketball court | Covered | `CourtConfig` defines the court as 80x120 cm. |
| Robot must shoot ping-pong balls or same-size balls | Partially covered | Shooter software interface exists; mechanical validation is still required. |
| Robot must be autonomous | Partially covered | Autonomous state and navigation step exist; real sensing and calibration remain. |
| Robot must not use an external PC/Mac for autonomous control | Covered by design | Control logic is in Raspberry Pi Python modules. |
| Main controller for this repository is Raspberry Pi | Covered | Repository architecture is Raspberry Pi + Python focused. |
| Arduino and ESP32 are not used in the target implementation | Covered by current design | Existing Arduino folder is left untouched as legacy material and is not referenced by the Python app. |

## Demo Rules

| Requirement | Current repo status | Notes |
| --- | --- | --- |
| Demo has two attempts of 5 minutes each | Needs implementation | Add a countdown timer to the Flask interface. |
| Minimum passing target is 2 points in 5 minutes | Partially covered | Score tracking exists; pass/fail indicator can be added. |
| 4 points and more give additional demo score | Partially covered | Score tracking supports +2 and +4 entries. |
| Same successful shooting location can be used only one more time | Needs implementation | Store successful shot poses and enforce 10 cm relocation rule. |
| Next successful shot after repeat must be at least 10 cm away | Needs implementation | Requires pose tracking accuracy. |
| Balls wait in an onboard storage area before dropping to the shooting platform | Mechanical requirement | Add feeder mechanism design and test documentation. |
| Ball must stop on the shooting platform before launch | Mechanical/software requirement | Add feeder timing and shooter interlock. |

## User Interface

| Requirement | Current repo status | Notes |
| --- | --- | --- |
| Interface must show real-time robot state | Covered | Flask UI shows mode, pose, sensor state, motors, and shooter state. |
| Interface must show score | Covered | Score is shown and can be updated from the UI. |
| Interface must show commands if commands exist | Covered | Manual drive, autonomous step, shoot, reset, score, and emergency stop commands exist. |
| Interface must show task history | Covered | `task_history` is shown in the Flask UI. |
| Separate scoreboard is not required | Covered | Score is integrated into the web interface. |
| Interface must be functional and user friendly | In progress | Current UI is functional; visual polish and mobile testing are still recommended. |

## Communication

| Requirement | Current repo status | Notes |
| --- | --- | --- |
| Robot interface must communicate over Bluetooth or Wi-Fi | Partially covered | Flask is suitable for Wi-Fi/LAN use on Raspberry Pi. |
| Standard transport protocol must be identified | Covered by design | HTTP over TCP is used by Flask endpoints. |
| Protocol choice must be justified in final report | Needs documentation | Add final report section explaining Flask/HTTP choice. |

## Documentation

| Requirement | Current repo status | Notes |
| --- | --- | --- |
| GitHub must include complete code and documentation | In progress | README, architecture notes, hardware notes, and tests are started. |
| WBS must be prepared | Needs content | `docs/WBS` exists but still needs real project files. |
| Gantt chart must be prepared | Needs content | `docs/Gantt` exists but still needs real project files. |
| Development reports and presentations must be included | Needs content | `docs/Reports` and `docs/Presentations` exist. |
| External libraries must be referenced | Partially covered | `requirements.txt` lists libraries; final report should explain them. |
| Realistic constraints must be discussed in final report | Needs documentation | Address at least five, including innovation. |
| Material cost must stay under 20000 TL including VAT | Needs documentation | Add bill of materials and invoice references. |

## Hardware

| Requirement | Current repo status | Notes |
| --- | --- | --- |
| Hoop plane must be parallel to ground | Mechanical requirement | Document after platform construction. |
| Hoop height must be 20-30 cm from ground | Mechanical requirement | Document measured final height. |
| Platform walls are allowed | Design option | Hardware notes should state chosen wall layout. |
| Zone colors are free | Design option | Document selected zone colors. |
| Hoop diameter from Q&A is relaxed up to 10 cm | Mechanical requirement | Confirm final hoop diameter in hardware docs. |

## Recommended Next Actions

1. Add a 5-minute demo timer to the Flask UI.
2. Add shot-attempt records with zone, pose, result, and points.
3. Add the 10 cm relocation rule for repeated successful shots.
4. Add OpenCV-based hoop/zone detection.
5. Replace simulated motor and sensor classes with Raspberry Pi GPIO adapters.
6. Prepare WBS, Gantt, bill of materials, and final report sections.
