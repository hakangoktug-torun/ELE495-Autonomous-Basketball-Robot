import unittest

from software.raspberry_pi import DEFAULT_CONFIG, RobotController, RobotMode


class RobotControllerTests(unittest.TestCase):
    def test_manual_drive_updates_mode_and_drive_command(self):
        robot = RobotController(DEFAULT_CONFIG)

        status = robot.manual_drive(0.5, 0.25)

        self.assertEqual(status.mode, RobotMode.MANUAL)
        self.assertEqual(status.drive_left, 0.5)
        self.assertEqual(status.drive_right, 0.25)

    def test_emergency_stop_locks_drive_until_reset(self):
        robot = RobotController(DEFAULT_CONFIG)

        robot.emergency_stop()
        locked_status = robot.manual_drive(0.5, 0.5)

        self.assertEqual(locked_status.mode, RobotMode.EMERGENCY_STOP)
        self.assertEqual(locked_status.drive_left, 0.0)
        self.assertEqual(locked_status.drive_right, 0.0)

        reset_status = robot.reset_emergency()

        self.assertEqual(reset_status.mode, RobotMode.IDLE)

    def test_status_dict_is_json_ready(self):
        robot = RobotController(DEFAULT_CONFIG)

        status = robot.status_dict()

        self.assertEqual(status["mode"], "idle")
        self.assertEqual(status["pose"]["x_cm"], 40.0)
        self.assertEqual(status["score"], 0)
        self.assertGreaterEqual(len(status["task_history"]), 1)

    def test_score_updates_are_tracked_in_history(self):
        robot = RobotController(DEFAULT_CONFIG)

        status = robot.add_score(2)

        self.assertEqual(status.score, 2)
        self.assertIn("Score updated by +2", status.task_history[0])


if __name__ == "__main__":
    unittest.main()
