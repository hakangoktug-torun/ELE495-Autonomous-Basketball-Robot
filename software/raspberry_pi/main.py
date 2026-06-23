"""Command-line entry point for the Raspberry Pi robot controller."""

from software.raspberry_pi import DEFAULT_CONFIG, RobotController


def main() -> None:
    robot = RobotController(DEFAULT_CONFIG)
    print("ELE495 robot controller initialized")
    print(robot.status_dict())


if __name__ == "__main__":
    main()
