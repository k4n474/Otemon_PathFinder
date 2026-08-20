"""LiDARを使った再利用可能な壁追従走行。"""

import time
from collections.abc import Callable

from algorithm import (
    detect_front_and_side_walls,
    detect_walls,
    measure_front_distance,
)


class WallPIDController:
    """側壁との距離を一定に保つPID制御器。"""

    def __init__(
        self,
        target_distance=250.0,
        kp=0.1,
        ki=0.01,
        kd=0.12,
        max_steering_angle=35.0,
        integral_limit=800.0,
    ):
        self.target_distance = target_distance
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_steering_angle = max_steering_angle
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_error = None

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, wall, dt, derivative_enabled=True):
        error = wall["wall_distance"] - self.target_distance
        self.integral += error * dt
        self.integral = max(
            -self.integral_limit,
            min(self.integral_limit, self.integral),
        )
        derivative = 0.0

        if self.previous_error is not None and dt > 0.0:
            derivative = (error - self.previous_error) / dt

        self.previous_error = error
        output = (
            self.kp * error
            + self.ki * self.integral
            + (self.kd * derivative if derivative_enabled else 0.0)
        )
        side_sign = 1 if wall["side"] == "right" else -1
        steering = side_sign * output
        return max(
            -self.max_steering_angle,
            min(self.max_steering_angle, steering),
        )


def follow_wall_until_front_distance(
    lidar,
    *,
    front_stop_distance: float,
    drive_motor: Callable[[float], None],
    set_steering: Callable[[float], None],
    stop_motor: Callable[[], None],
    target_side_distance: float = 250.0,
    motor_speed: float = 42.0,
    trace_side: str | None = None,
    interval: float = 0.05,
    timeout: float = 30.0,
    wall_lost_timeout: float = 1.0,
) -> dict:
    """側壁を追従し、前壁が指定距離に達したら停止する。

    LiDARの開始・停止は呼び出し側が管理する。戻り値には停止時の
    前壁距離、追従した側、走行時間を格納する。
    """
    if front_stop_distance <= 0.0:
        raise ValueError("front_stop_distanceは0より大きくしてください")
    if trace_side not in (None, "left", "right"):
        raise ValueError("trace_sideはleft、right、Noneのいずれかです")
    if timeout <= 0.0 or wall_lost_timeout <= 0.0:
        raise ValueError("timeoutは0より大きくしてください")

    pid = WallPIDController(target_distance=target_side_distance)
    started_at = time.monotonic()
    previous_time = started_at
    wall_missing_since = None
    last_trace_wall = None

    try:
        while True:
            now = time.monotonic()

            if now - started_at >= timeout:
                raise TimeoutError("壁追従走行がタイムアウトしました")

            points = lidar.get_points()
            measured_front_distance = measure_front_distance(points)
            walls = detect_walls(points)
            front_wall, side_walls = detect_front_and_side_walls(
                points,
                detected_walls=walls,
            )

            front_distance = measured_front_distance

            if front_distance is None and front_wall is not None:
                front_distance = front_wall["front_distance"]

            if (
                front_distance is not None
                and front_distance <= front_stop_distance
            ):
                return {
                    "front_distance": front_distance,
                    "trace_side": trace_side,
                    "elapsed": now - started_at,
                }

            if trace_side is None:
                candidates = [
                    side
                    for side in ("left", "right")
                    if side_walls[side] is not None
                ]
                if candidates:
                    trace_side = max(
                        candidates,
                        key=lambda side: side_walls[side]["length"],
                    )
                    pid.reset()

            trace_wall = (
                side_walls[trace_side]
                if trace_side is not None
                else None
            )

            if trace_wall is None:
                wall_missing_since = wall_missing_since or now

                # RANSACが1～数フレームだけ壁を落とした場合は、最後に
                # 正常検出した壁を短時間使い、不要な停止を防ぐ。
                if (
                    last_trace_wall is not None
                    and now - wall_missing_since < wall_lost_timeout
                ):
                    trace_wall = last_trace_wall
                else:
                    stop_motor()
                    set_steering(0)
                    pid.reset()
            else:
                wall_missing_since = None

            if trace_wall is not None:
                last_trace_wall = trace_wall
                dt = now - previous_time
                set_steering(pid.update(trace_wall, dt))
                drive_motor(motor_speed)

            previous_time = now
            time.sleep(interval)
    finally:
        stop_motor()
        set_steering(0)
