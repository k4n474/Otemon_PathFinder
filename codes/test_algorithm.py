import math
import unittest

from algorithm import detect_corners, detect_side_walls, detect_walls


def point(x, y):
    return {
        "angle": math.degrees(math.atan2(x, y)) % 360.0,
        "distance": math.hypot(x, y)
    }


class WallAxisDetectionTest(unittest.TestCase):
    def setUp(self):
        self.points = []
        self.points.extend(
            point(x, 1000.0)
            for x in range(-600, 601, 50)
        )
        self.points.extend(
            point(-600.0, y)
            for y in range(100, 1001, 50)
        )
        self.points.extend(
            point(600.0, y)
            for y in range(100, 1001, 50)
        )

    def test_classifies_walls_by_axis(self):
        walls = detect_walls(self.points)

        self.assertTrue(any(wall["is_front_wall"] for wall in walls))
        self.assertTrue(
            any(
                wall["is_side_wall"] and wall["side"] == "left"
                for wall in walls
            )
        )
        self.assertTrue(
            any(
                wall["is_side_wall"] and wall["side"] == "right"
                for wall in walls
            )
        )

    def test_detects_left_and_right_corners(self):
        corners = detect_corners(detect_walls(self.points))

        self.assertEqual(
            {"left", "right"},
            {corner["side"] for corner in corners}
        )

    def test_side_detection_does_not_use_angle_ranges(self):
        sides = detect_side_walls(self.points)

        self.assertIsNotNone(sides["left"])
        self.assertIsNotNone(sides["right"])


if __name__ == "__main__":
    unittest.main()
