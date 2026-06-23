import unittest

from software.raspberry_pi.config import CourtConfig
from software.raspberry_pi.navigation import Navigator, Pose


class NavigationTests(unittest.TestCase):
    def test_pose_is_clamped_to_court_boundaries(self):
        navigator = Navigator(CourtConfig())

        pose = navigator.clamp_pose(Pose(-5.0, 140.0, 370.0))

        self.assertEqual(pose.x_cm, 0.0)
        self.assertEqual(pose.y_cm, 120.0)
        self.assertEqual(pose.heading_deg, 10.0)

    def test_nearest_scoring_zone_selects_center_from_middle(self):
        navigator = Navigator(CourtConfig())

        zone = navigator.nearest_scoring_zone(Pose(42.0, 68.0))

        self.assertEqual(zone.name, "center")


if __name__ == "__main__":
    unittest.main()
