"""LiDAR点群からRANSACで壁を検出する。"""

import math
import random
from typing import Optional


def detect_walls(
    points: list[dict[str, float | int]],
    *,
    distance_threshold: float = 50.0,
    min_inliers: int = 10,
    min_wall_length: float = 100.0,
    max_point_gap: float = 250.0,
    max_walls: int = 8,
    iterations: int = 150,
    maximum_distance: float = 3000.0
) -> list[dict]:
    """
    点群から複数の壁を検出する。

    座標系:
        LiDAR正面: +Y
        LiDAR右側: +X
        単位: mm
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

        # 最小二乗法で直線を補正した後、内点を取り直す。
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

        # 検出済みの点を除去し、次の壁を探す。
        inlier_ids = {id(point) for point in inliers}
        remaining = [
            point
            for point in remaining
            if id(point) not in inlier_ids
        ]

        if length < min_wall_length:
            continue

        a, b, c = line
        is_x_axis_wall = _is_aligned_with_x_axis(
            start,
            end,
            maximum_angle=math.pi / 6
        )
        is_y_axis_wall = _is_aligned_with_y_axis(
            start,
            end,
            maximum_angle=math.pi / 6
        )
        center_x = (start[0] + end[0]) / 2.0
        center_y = (start[1] + end[1]) / 2.0
        is_front_wall = is_x_axis_wall and center_y > 0.0
        is_side_wall = is_y_axis_wall
        wall_side = (
            "right"
            if is_side_wall and center_x > 0.0
            else "left"
            if is_side_wall and center_x < 0.0
            else None
        )
        # a, bは正規化済みなので、原点から壁直線への
        # 垂線距離は |a*0 + b*0 + c| = |c| になる。
        perpendicular_distance = abs(c)
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
            "axis": (
                "x"
                if is_x_axis_wall
                else "y"
                if is_y_axis_wall
                else None
            ),
            "is_front_wall": is_front_wall,
            "is_side_wall": is_side_wall,
            "side": wall_side,
            "front_distance": (
                round(perpendicular_distance, 1)
                if is_front_wall
                else None
            ),
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
    return walls


def detect_front_wall(
    points: list[dict[str, float | int]],
    **wall_detection_options
) -> Optional[dict]:
    """
    X軸方向でロボットより前にある壁のうち、最短の壁を返す。

    front_distanceはロボット中心から壁直線へ下ろした垂線の長さ。
    前方壁が存在しない場合はNoneを返す。
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


def detect_side_walls(
    points: list[dict[str, float | int]],
    **wall_detection_options
) -> dict[str, Optional[dict]]:
    """
    左右それぞれの最長壁と、追従対象にする壁を返す。

    側壁はY軸方向の壁とし、壁中心のX座標で左右を決める。
    LiDAR角度による視野の切り分けは行わない。
    """

    result: dict[str, Optional[dict]] = {
        "left": None,
        "right": None,
        "trace": None
    }
    detected_walls = detect_walls(
        points,
        **wall_detection_options
    )

    for side in ("left", "right"):
        walls = [
            wall
            for wall in detected_walls
            if (
                wall["is_side_wall"]
                and wall["side"] == side
            )
        ]

        if walls:
            longest_wall = max(
                walls,
                key=lambda wall: wall["length"]
            )
            result[side] = {
                **longest_wall,
                "side": side,
                # lineは正規化済みなので、|c|がロボット中心から
                # 壁へ下ろした垂線距離になる。
                "wall_distance": abs(
                    longest_wall["line"]["c"]
                )
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


def detect_corners(
    walls: list[dict],
    *,
    endpoint_tolerance: float = 180.0
) -> list[dict]:
    """
    X軸方向の壁とY軸方向の壁の交点を角として返す。

    ノイズで線分端が少し欠けても検出できるよう、交点が両線分の
    端からendpoint_tolerance以内にあれば同じ角とみなす。
    """

    x_walls = [wall for wall in walls if wall.get("axis") == "x"]
    y_walls = [wall for wall in walls if wall.get("axis") == "y"]
    corners = []

    for front_wall in x_walls:
        for side_wall in y_walls:
            intersection = _line_intersection(
                front_wall["line"],
                side_wall["line"]
            )

            if intersection is None:
                continue

            if not (
                _point_is_near_segment(
                    intersection,
                    front_wall,
                    endpoint_tolerance
                )
                and _point_is_near_segment(
                    intersection,
                    side_wall,
                    endpoint_tolerance
                )
            ):
                continue

            x, y = intersection
            corners.append({
                "x": round(x, 1),
                "y": round(y, 1),
                "side": "right" if x > 0.0 else "left",
                "front_wall": front_wall,
                "side_wall": side_wall
            })

    return corners


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
    """同一直線上でも離れている点群は別の壁として扱う。"""

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
    """全最小二乗法で直線を補正する。"""

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

    # 正規化された直線上で原点に最も近い点。
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
    """
    壁線分の一部が前方視野内に入っているかを返す。

    境界線との交点も調べるため、長い壁が視野を横切る場合も
    前方壁として判定できる。
    """

    dx = end[0] - start[0]
    dy = end[1] - start[1]
    tangent = math.tan(
        math.radians(half_view_angle)
    )
    candidates = [0.0, 1.0]

    # 原点から線分への垂線の足
    squared_length = dx * dx + dy * dy

    if squared_length > 0.0:
        closest_t = -(
            start[0] * dx
            + start[1] * dy
        ) / squared_length
        candidates.append(
            max(0.0, min(1.0, closest_t))
        )

    # 前方視野の左右境界 x = ±y*tan(30°) との交点
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
    """
    壁線分がロボットのX軸に対してほぼ平行かを返す。

    線分には向きがないため、+X軸と-X軸を同一として扱う。
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
    """壁線分がロボットのY軸に対してほぼ平行かを返す。"""

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
