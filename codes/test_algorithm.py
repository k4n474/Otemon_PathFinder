import math
import unittest

from algorithm import (
    _classify_walls_by_normal_angle,
    detect_corners,
    detect_front_and_side_walls,
    detect_side_walls,
    detect_walls,
    measure_front_distance,
)


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

    def test_front_and_sides_use_the_same_detected_walls(self):
        walls = detect_walls(self.points)
        front, sides = detect_front_and_side_walls(
            self.points,
            detected_walls=walls
        )

        self.assertIsNotNone(front)
        self.assertIsNotNone(sides["left"])
        self.assertIsNotNone(sides["right"])
        self.assertAlmostEqual(
            1000.0,
            front["front_distance"],
            delta=10.0
        )
        self.assertAlmostEqual(
            600.0,
            sides["right"]["wall_distance"],
            delta=10.0
        )

    def test_short_wall_is_not_classified_as_front_wall(self):
        wall = {
            "normal_angle": 90.0,
            "inlier_count": 20,
            "length": 499.0,
            "wall_distance": 300.0,
            "role": None,
            "is_front_wall": False,
            "is_side_wall": False,
            "side": None,
            "front_distance": None,
        }

        _classify_walls_by_normal_angle([wall])

        self.assertEqual("front", wall["role"])
        self.assertFalse(wall["is_front_wall"])
        self.assertIsNone(wall["front_distance"])

    def test_measures_front_distance_without_wall_classification(self):
        front_points = [
            point(x, 500.0)
            for x in (-100.0, -50.0, 0.0, 50.0, 100.0)
        ]

        self.assertAlmostEqual(
            500.0,
            measure_front_distance(front_points),
            delta=1.0,
        )

    def test_front_distance_ignores_single_close_noise_point(self):
        front_points = [
            point(x, 500.0)
            for x in (-100.0, -50.0, 0.0, 50.0, 100.0)
        ]
        front_points.append(point(0.0, 60.0))

        self.assertAlmostEqual(
            500.0,
            measure_front_distance(front_points),
            delta=1.0,
        )

    def test_classifies_walls_when_robot_is_rotated_45_degrees(self):
        for rotation in (-45.0, 45.0):
            radians = math.radians(rotation)
            rotated_points = []

            for item in self.points:
                distance = item["distance"]
                angle = math.radians(item["angle"])
                x = math.sin(angle) * distance
                y = math.cos(angle) * distance
                rotated_x = (
                    math.cos(radians) * x
                    - math.sin(radians) * y
                )
                rotated_y = (
                    math.sin(radians) * x
                    + math.cos(radians) * y
                )
                rotated_points.append(point(rotated_x, rotated_y))

            walls = detect_walls(rotated_points)

            self.assertEqual(
                {"right", "front", "left"},
                {
                    wall["role"]
                    for wall in walls
                    if wall["role"] is not None
                }
            )
            self.assertEqual(
                2,
                len(detect_corners(walls))
            )

    def test_three_walls_are_ordered_from_robot_rear(self):
        examples = (
            (
                (49.3, 227.6, 319.6),
                {319.6: "right", 49.3: "front", 227.6: "left"}
            ),
            (
                (87.0, 177.4, 359.5),
                {359.5: "right", 87.0: "front", 177.4: "left"}
            )
        )

        for angles, expected_roles in examples:
            walls = [
                {
                    "normal_angle": angle,
                    "inlier_count": 20,
                    "length": 1000.0,
                    "wall_distance": 500.0
                }
                for angle in angles
            ]

            _classify_walls_by_normal_angle(walls)

            self.assertEqual(
                expected_roles,
                {
                    wall["normal_angle"]: wall["role"]
                    for wall in walls
                }
            )


if __name__ == "__main__":
    unittest.main()
