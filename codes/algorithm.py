"""Detect walls in LiDAR point clouds using RANSAC."""

import math
import random
from typing import Optional


# Minimum detected segment length for a front wall, in millimeters.
MIN_FRONT_WALL_LENGTH = 500.0


def detect_walls(
    points: list[dict[str, float | int]],
    *,
    distance_threshold: float = 50.0,
    min_inliers: int = 10,
    min_wall_length: float = 100.0,
    max_point_gap: float = 250.0,
    max_walls: int = 4,
    iterations: int = 100,
    maximum_distance: float = 3000.0
) -> list[dict]:
    """Detect multiple walls by fitting lines to nearby groups of points.

    Coordinates are in millimeters: +Y is forward and +X is right.
    RANSAC samples candidate lines and keeps points close to the best line.
    """

    remaining = _to_cartesian(
        points,
        maximum_distance
    )
    walls = []
    generator = random.Random(0)

    while (
        len(remaining) >= min_inliers
        and len(walls) < max_walls
    ):
        best_points: list[tuple[float, float]] = []

        for _ in range(iterations):
            first, second = generator.sample(
                remaining,
                2
            )
            line = _line_from_two_points(
                first,
                second
            )

            if line is None:
                continue

            inliers = [
                point
                for point in remaining
                if _distance_to_line(point, line)
                <= distance_threshold
            ]
            inliers = _largest_group(
                inliers,
                line,
                max_point_gap
            )

            if len(inliers) > len(best_points):
                best_points = inliers

        if len(best_points) < min_inliers:
            break

        line = _fit_line(best_points)

        if line is None:
            break

        # Recompute inliers after refining the line with least squares.
        inliers = [
            point
            for point in remaining
            if _distance_to_line(point, line)
            <= distance_threshold
        ]
        inliers = _largest_group(
            inliers,
            line,
            max_point_gap
        )

        if len(inliers) < min_inliers:
            inliers = best_points
            line = _fit_line(inliers)

        if line is None:
            break

        start, end = _segment_endpoints(
            inliers,
            line
        )
        length = math.dist(start, end)

        # Remove points already assigned to this line before searching for the next wall.
        inlier_ids = {id(point) for point in inliers}
        remaining = [
            point
            for point in remaining
            if id(point) not in inlier_ids
        ]

        if length < min_wall_length:
            continue

        a, b, c = line
        # The coefficients a and b are normalized, so the perpendicular distance
        # from the origin to the wall is |a*0 + b*0 + c| = |c|.
        perpendicular_distance = abs(c)
        closest_x = -a * c
        closest_y = -b * c
        normal_angle = math.degrees(
            math.atan2(closest_y, closest_x)
        ) % 360.0
        walls.append({
            "start": {
                "x": round(start[0], 1),
                "y": round(start[1], 1)
            },
            "end": {
                "x": round(end[0], 1),
                "y": round(end[1], 1)
            },
            "length": round(length, 1),
            "inlier_count": len(inliers),
            "closest_point": {
                "x": round(closest_x, 1),
                "y": round(closest_y, 1)
            },
            # Normal angles: right = 0 degrees, forward = 90, left = 180.
            "normal_angle": round(normal_angle, 1),
            "role": None,
            "is_front_wall": False,
            "is_side_wall": False,
            "side": None,
            "front_distance": None,
            "wall_distance": round(perpendicular_distance, 1),
            "line": {
                "a": round(a, 6),
                "b": round(b, 6),
                "c": round(c, 3)
            }
        })

    walls.sort(
        key=lambda wall: wall["inlier_count"],
        reverse=True
    )
    _classify_walls_by_normal_angle(walls)
    return walls


def detect_front_wall(
    points: list[dict[str, float | int]],
    **wall_detection_options
) -> Optional[dict]:
    """Return the nearest wall classified as a front wall by normal-angle order.

    front_distance is the perpendicular distance from the robot center to
    the wall line, in millimeters. Return None if no front wall is found.
    """

    front_walls = [
        wall
        for wall in detect_walls(
            points,
            **wall_detection_options
        )
        if wall["is_front_wall"]
    ]

    if not front_walls:
        return None

    return min(
        front_walls,
        key=lambda wall: wall["front_distance"]
    )


def measure_front_distance(
    points: list[dict[str, float | int]],
    *,
    half_angle: float = 15.0,
    sample_count: int = 5,
) -> Optional[float]:
    """Measure the distance to an object directly ahead from the point cloud.

    Use forward components of points within +/- half_angle degrees without
    relying on RANSAC wall labels. Return the median of the nearest
    sample_count points to reduce stops caused by isolated noise.
    """
    if not 0.0 < half_angle < 90.0:
        raise ValueError("half_angleは0より大きく90未満にしてください")
    if sample_count < 1:
        raise ValueError("sample_countは1以上にしてください")

    front_distances = []

    for point in points:
        try:
            angle = float(point["angle"]) % 360.0
            distance = float(point["distance"])
        except (KeyError, TypeError, ValueError):
            continue

        signed_angle = (
            angle if angle <= 180.0 else angle - 360.0
        )

        if (
            not math.isfinite(distance)
            or abs(signed_angle) > half_angle
            or distance < 50.0
        ):
            continue

        forward_distance = distance * math.cos(
            math.radians(signed_angle)
        )

        if forward_distance > 0.0:
            front_distances.append(forward_distance)

    if len(front_distances) < sample_count:
        return None

    nearest = sorted(front_distances)[:sample_count]
    middle = len(nearest) // 2

    if len(nearest) % 2:
        return round(nearest[middle], 1)

    return round(
        (nearest[middle - 1] + nearest[middle]) / 2.0,
        1,
    )


def detect_side_walls(
    points: list[dict[str, float | int]],
    *,
    detected_walls: Optional[list[dict]] = None,
    **wall_detection_options
) -> dict[str, Optional[dict]]:
    """Return the longest wall on each side and the wall selected for following.

    Classify walls as right, front, and left by increasing normal angle,
    using the robot's rear as zero. Do not restrict wall angles to the axes.
    """

    result: dict[str, Optional[dict]] = {
        "left": None,
        "right": None,
        "trace": None
    }
    if detected_walls is None:
        detected_walls = detect_walls(
            points,
            **wall_detection_options
        )

    for side in ("left", "right"):
        walls = [
            wall
            for wall in detected_walls
            if wall["role"] == side
        ]

        if walls:
            longest_wall = max(
                walls,
                key=lambda wall: wall["length"]
            )
            result[side] = {
                **longest_wall,
                "side": side,
                # The line is normalized, so |c| is the perpendicular distance
                # from the robot center to the wall.
                "wall_distance": longest_wall["wall_distance"]
            }

    candidates = [
        wall
        for wall in (result["left"], result["right"])
        if wall is not None
    ]

    if candidates:
        result["trace"] = max(
            candidates,
            key=lambda wall: wall["length"]
        )

    return result


def detect_front_and_side_walls(
    points: list[dict[str, float | int]],
    *,
    detected_walls: Optional[list[dict]] = None,
    **wall_detection_options
) -> tuple[Optional[dict], dict[str, Optional[dict]]]:
    """Return the nearest front wall and side walls from one detection result.

    Read distances from front_wall["front_distance"] and
    side_walls[side]["wall_distance"]. All distances are perpendicular
    distances from the robot center to a wall line, in millimeters.

    Passing detected_walls skips another RANSAC run and ensures that front
    and side walls come from the same frame and detection result.
    """

    if detected_walls is None:
        detected_walls = detect_walls(
            points,
            **wall_detection_options
        )

    front_wall = min(
        (
            wall
            for wall in detected_walls
            if wall.get("is_front_wall")
        ),
        key=lambda wall: float(wall["front_distance"]),
        default=None
    )
    side_walls = detect_side_walls(
        points,
        detected_walls=detected_walls
    )
    return front_wall, side_walls


def detect_corners(
    walls: list[dict],
    *,
    endpoint_tolerance: float = 180.0
) -> list[dict]:
    """Return intersections of nearly perpendicular walls as corners.

    Allow intersections within endpoint_tolerance of both segment ends so
    small gaps caused by measurement noise do not hide a corner.
    """

    corners = []

    for first_index, first_wall in enumerate(walls):
        for second_wall in walls[first_index + 1:]:
            if not _walls_are_perpendicular(
                first_wall,
                second_wall
            ):
                continue

            intersection = _line_intersection(
                first_wall["line"],
                second_wall["line"]
            )

            if intersection is None:
                continue

            if not (
                _point_is_near_segment(
                    intersection,
                    first_wall,
                    endpoint_tolerance
                )
                and _point_is_near_segment(
                    intersection,
                    second_wall,
                    endpoint_tolerance
                )
            ):
                continue

            x, y = intersection
            corners.append({
                "x": round(x, 1),
                "y": round(y, 1),
                "side": "right" if x > 0.0 else "left",
                "walls": [first_wall, second_wall]
            })

    return corners


def _classify_walls_by_normal_angle(
    walls: list[dict]
) -> None:
    """Assign right, front, and left roles by normal angle, starting at the rear.

    First select up to three walls with the most inliers, then sort by angle
    so short false detections do not displace the main walls.
    """

    candidates = sorted(
        walls,
        key=lambda wall: (
            wall["inlier_count"],
            wall["length"]
        ),
        reverse=True
    )[:3]
    candidates.sort(
        key=lambda wall: (
            float(wall["normal_angle"]) - 270.0
        ) % 360.0
    )

    if len(candidates) == 3:
        # normal_angle places the rear at 270 degrees. Shift that to zero
        # for classification so walls are ordered right, front, then left.
        roles = ("right", "front", "left")
    else:
        target_angles = {
            "right": 0.0,
            "front": 90.0,
            "left": 180.0
        }
        available_roles = set(target_angles)
        roles = []

        for wall in candidates:
            role = min(
                available_roles,
                key=lambda name: _circular_angle_distance(
                    wall["normal_angle"],
                    target_angles[name]
                )
            )
            roles.append(role)
            available_roles.remove(role)

    for wall, role in zip(candidates, roles):
        wall["role"] = role
        wall["is_front_wall"] = bool(
            role == "front"
            and wall["length"] >= MIN_FRONT_WALL_LENGTH
        )
        wall["is_side_wall"] = role in ("left", "right")
        wall["side"] = (
            role if role in ("left", "right") else None
        )

        if wall["is_front_wall"]:
            wall["front_distance"] = wall["wall_distance"]


def _circular_angle_distance(
    first: float,
    second: float
) -> float:
    difference = abs(first - second) % 360.0
    return min(difference, 360.0 - difference)


def _walls_are_perpendicular(
    first: dict,
    second: dict,
    maximum_error: float = math.pi / 6
) -> bool:
    first_line = first["line"]
    second_line = second["line"]
    dot = abs(
        first_line["a"] * second_line["a"]
        + first_line["b"] * second_line["b"]
    )
    dot = max(0.0, min(1.0, dot))
    angle = math.acos(dot)
    return abs(angle - math.pi / 2) <= maximum_error


def _line_intersection(
    first: dict[str, float],
    second: dict[str, float]
) -> Optional[tuple[float, float]]:
    a1, b1, c1 = first["a"], first["b"], first["c"]
    a2, b2, c2 = second["a"], second["b"], second["c"]
    determinant = a1 * b2 - a2 * b1

    if abs(determinant) < 1e-9:
        return None

    return (
        (b1 * c2 - b2 * c1) / determinant,
        (c1 * a2 - c2 * a1) / determinant
    )


def _point_is_near_segment(
    point: tuple[float, float],
    wall: dict,
    tolerance: float
) -> bool:
    start = (wall["start"]["x"], wall["start"]["y"])
    end = (wall["end"]["x"], wall["end"]["y"])
    segment_length = math.dist(start, end)

    return (
        math.dist(start, point)
        + math.dist(point, end)
        <= segment_length + 2.0 * tolerance
    )


def _to_cartesian(
    points: list[dict[str, float | int]],
    maximum_distance: float
) -> list[tuple[float, float]]:
    converted = []

    for point in points:
        try:
            angle = float(point["angle"])
            distance = float(point["distance"])
        except (KeyError, TypeError, ValueError):
            continue

        if (
            not math.isfinite(angle)
            or not math.isfinite(distance)
            or distance < 50.0
            or distance > maximum_distance
        ):
            continue

        radians = math.radians(angle)
        converted.append((
            math.sin(radians) * distance,
            math.cos(radians) * distance
        ))

    return converted


def _line_from_two_points(
    first: tuple[float, float],
    second: tuple[float, float]
) -> Optional[tuple[float, float, float]]:
    dx = second[0] - first[0]
    dy = second[1] - first[1]
    length = math.hypot(dx, dy)

    if length < 1.0:
        return None

    a = dy / length
    b = -dx / length
    c = -(a * first[0] + b * first[1])
    return a, b, c


def _distance_to_line(
    point: tuple[float, float],
    line: tuple[float, float, float]
) -> float:
    a, b, c = line
    return abs(a * point[0] + b * point[1] + c)


def _largest_group(
    points: list[tuple[float, float]],
    line: tuple[float, float, float],
    max_point_gap: float
) -> list[tuple[float, float]]:
    """Treat separated groups of collinear points as distinct walls."""

    if not points:
        return []

    a, b, _ = line
    direction_x = -b
    direction_y = a

    def projection(
        point: tuple[float, float]
    ) -> float:
        return (
            point[0] * direction_x
            + point[1] * direction_y
        )

    ordered = sorted(points, key=projection)
    groups = [[ordered[0]]]
    previous = projection(ordered[0])

    for point in ordered[1:]:
        current = projection(point)

        if current - previous > max_point_gap:
            groups.append([])

        groups[-1].append(point)
        previous = current

    return max(groups, key=len)


def _fit_line(
    points: list[tuple[float, float]]
) -> Optional[tuple[float, float, float]]:
    """Refine the wall line using total least squares."""

    if len(points) < 2:
        return None

    center_x = sum(p[0] for p in points) / len(points)
    center_y = sum(p[1] for p in points) / len(points)
    covariance_xx = sum(
        (p[0] - center_x) ** 2
        for p in points
    )
    covariance_yy = sum(
        (p[1] - center_y) ** 2
        for p in points
    )
    covariance_xy = sum(
        (p[0] - center_x) * (p[1] - center_y)
        for p in points
    )

    if covariance_xx + covariance_yy < 1.0:
        return None

    direction_angle = 0.5 * math.atan2(
        2.0 * covariance_xy,
        covariance_xx - covariance_yy
    )
    direction_x = math.cos(direction_angle)
    direction_y = math.sin(direction_angle)
    a = direction_y
    b = -direction_x
    c = -(a * center_x + b * center_y)
    return a, b, c


def _segment_endpoints(
    points: list[tuple[float, float]],
    line: tuple[float, float, float]
) -> tuple[tuple[float, float], tuple[float, float]]:
    a, b, c = line
    direction_x = -b
    direction_y = a

    # Find the point on the normalized line closest to the origin.
    origin_x = -a * c
    origin_y = -b * c
    projections = [
        (
            (point[0] - origin_x) * direction_x
            + (point[1] - origin_y) * direction_y
        )
        for point in points
    ]
    minimum = min(projections)
    maximum = max(projections)

    return (
        (
            origin_x + minimum * direction_x,
            origin_y + minimum * direction_y
        ),
        (
            origin_x + maximum * direction_x,
            origin_y + maximum * direction_y
        )
    )


def _is_in_front_view(
    start: tuple[float, float],
    end: tuple[float, float],
    half_view_angle: float
) -> bool:
    """Return whether any part of a wall segment lies in the forward field of view.

    Include intersections with the viewing boundaries so long walls crossing
    the field of view can also be recognized as front walls.
    """

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    tangent = math.tan(
        math.radians(half_view_angle)
    )
    candidates = [0.0, 1.0]

    # Project the origin perpendicularly onto the segment line.
    squared_length = dx * dx + dy * dy

    if squared_length > 0.0:
        closest_t = -(
            start[0] * dx
            + start[1] * dy
        ) / squared_length
        candidates.append(
            max(0.0, min(1.0, closest_t))
        )

    # Check intersections with the forward-view boundaries x = +/- y*tan(30 degrees).
    for side in (-1.0, 1.0):
        denominator = dx - side * tangent * dy

        if abs(denominator) < 1e-9:
            continue

        intersection_t = (
            side * tangent * start[1]
            - start[0]
        ) / denominator

        if 0.0 <= intersection_t <= 1.0:
            candidates.append(intersection_t)

    for position in candidates:
        x = start[0] + position * dx
        y = start[1] + position * dy

        if y <= 0.0:
            continue

        if abs(x) <= y * tangent + 1e-6:
            return True

    return False


def _is_aligned_with_x_axis(
    start: tuple[float, float],
    end: tuple[float, float],
    maximum_angle: float
) -> bool:
    """Return whether the wall segment is nearly parallel to the robot's X axis.

    A segment has no direction, so treat the +X and -X axes as equivalent.
    """

    dx = end[0] - start[0]
    dy = end[1] - start[1]

    if math.hypot(dx, dy) < 1.0:
        return False

    angle = abs(math.atan2(dy, dx))
    angle_from_x_axis = min(
        angle,
        math.pi - angle
    )
    return angle_from_x_axis <= maximum_angle + 1e-9


def _is_aligned_with_y_axis(
    start: tuple[float, float],
    end: tuple[float, float],
    maximum_angle: float
) -> bool:
    """Return whether the wall segment is nearly parallel to the robot's Y axis."""

    dx = end[0] - start[0]
    dy = end[1] - start[1]

    if math.hypot(dx, dy) < 1.0:
        return False

    angle_from_y_axis = abs(
        math.atan2(dx, dy)
    )
    angle_from_y_axis = min(
        angle_from_y_axis,
        math.pi - angle_from_y_axis
    )
    return angle_from_y_axis <= maximum_angle + 1e-9
