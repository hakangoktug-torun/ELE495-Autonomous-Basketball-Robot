"""Command-line orientation entry point for the Raspberry Pi software.

The current robot runtime is driven through the Flask GUI and calibration
scripts under ``kalibrasyon_kodlari``. This module intentionally avoids
starting motors or opening serial ports.
"""

from software.raspberry_pi import DEFAULT_CONFIG, LIVE_CONFIG, SIMULATION_CONFIG


def main() -> None:
    print("ELE495 Raspberry Pi software package")
    print(f"Default serial port: {DEFAULT_CONFIG.serial_port}")
    print(f"Default dry-run mode: {DEFAULT_CONFIG.dry_run}")
    print(f"Simulation config: dry_run={SIMULATION_CONFIG.dry_run}, simulate_motion={SIMULATION_CONFIG.simulate_motion}")
    print(f"Live config: dry_run={LIVE_CONFIG.dry_run}, simulate_motion={LIVE_CONFIG.simulate_motion}")
    print("Run the hardware GUI with: python -m software.flask_gui.app")


if __name__ == "__main__":
    main()
