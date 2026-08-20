"""LiDARの前壁に対して車体を垂直にするサーボ制御。"""

import atexit
import threading
import time

from algorithm import detect_walls
from gyro import close_gyro, get_angle, reset_angle
from lidar_read import LidarReader

# GPIOを使うnewobotは、インポート時ではなく制御開始時に読み込む。
brake = dc_motor = set_angle = stop = None


DETECTION_RANGE = 1500.0
TARGET_NORMAL_ANGLE = 86.0
ALIGN_KP = 1
ALIGN_KD = 0.3
MAX_STEERING_ANGLE = 40.0
ANGLE_DEADBAND = 1.5
FINE_CONTROL_DISTANCE = 400.0
FINE_ALIGN_KP = 1
FINE_ALIGN_KD = 0.4
FINE_MAX_STEERING_ANGLE = 15.0
CONTROL_INTERVAL = 0.05
FRONT_STOP_DISTANCE = 150.0
BACKUP_TARGET_DISTANCE = 390.0
CLOCKWISE_BACKUP_TARGET_DISTANCE = 200.0
BACKUP_SPEED = 35.0##
FINAL_BACKUP_TARGET_DISTANCE = 420.0
FINAL_BACKUP_SLOWDOWN_DISTANCE = 200.0
FINAL_BACKUP_SPEED = 42.0##
FINAL_BACKUP_SLOW_SPEED = 41.0##
FINAL_REVERSE_TURN_TARGET_ANGLE = 80.0
BRAKE_BEFORE_REVERSE_DURATION = 0.4
RUN_DIRECTION = 1
COUNTERCLOCKWISE = 1
CLOCKWISE = 0
LEFT_TURN_TARGET_ANGLE = 90.0
LEFT_TURN_STEERING_ANGLE = -40.0
RIGHT_TURN_STEERING_ANGLE = 40.0
INITIAL_RIGHT_TURN_ANGLE = 80.0
INITIAL_RIGHT_REVERSE_TURN_ANGLE = 90.0
LEFT_TURN_SPEED = 52.0##
LEFT_TURN_TIMEOUT = 10.0
STRAIGHT_TARGET_WALL_DISTANCE = 1200.0
SECOND_TURN_STOP_SECONDS = 1.0
STRAIGHT_DISTANCE_IGNORE_SECONDS = 1.0
STRAIGHT_DISTANCE_CONFIRM_SAMPLES = 3
STRAIGHT_DRIVE_SPEED = 40.0##
COUNTERCLOCKWISE_REVERSE_SLOWDOWN_DISTANCE = 1000.0
STRAIGHT_SLOWDOWN_DISTANCE = 1600.0
STRAIGHT_SLOW_SPEED = 30.0##
GYRO_STRAIGHT_KP = 1
GYRO_STRAIGHT_MAX_STEERING = 30.0
SECOND_LEFT_TURN_ANGLE = 80.0
FINAL_TARGET_NORMAL_ANGLE = 90.0
FINAL_TARGET_ANGLE_TOLERANCE = 40.0
ALIGN_START_SPEED = 50.0##
ALIGN_START_DURATION = 0.4
ALIGN_DRIVE_SPEED = 30.0##

STEERING_SIGN = -1.0

FINAL_WALL_STATES = {
    "second_turn_braking", "turning_left_to_180", "final_forward",
    "final_braking", "final_reversing",
}
TURN_STATES = {
    "initial_turning_right", "initial_turning_right_reverse",
    "turning_left", "turning_left_to_180",
    "final_turning_left_reverse", "final_turning_right_reverse",
}
WALL_DRIVE_STATES = {"forward", "reversing", "final_forward", "final_reversing"}
BRAKE_STATES = {
    "braking", "complete", "turn_error", "second_turn_braking",
    "final_braking", "final_complete",
}


def _load_hardware() -> None:
    global brake, dc_motor, set_angle, stop
    if brake is not None:
        return
    from newobot import brake as motor_brake
    from newobot import dc_motor as motor_drive
    from newobot import set_angle as steer
    from newobot import stop as motor_stop

    brake, dc_motor, set_angle, stop = (
        motor_brake, motor_drive, steer, motor_stop,
    )


def select_final_target_wall(walls: list[dict]) -> dict | None:
    """法線がY軸±40°以内の最短壁を、長さに関係なく返す。"""
    candidates = (
        wall for wall in walls
        if abs(signed_angle_error(
            wall["normal_angle"], FINAL_TARGET_NORMAL_ANGLE,
        )) <= FINAL_TARGET_ANGLE_TOLERANCE
    )
    return min(candidates, key=lambda wall: float(wall["wall_distance"]), default=None)


def signed_angle_error(angle: float, target: float = TARGET_NORMAL_ANGLE) -> float:
    """targetからangleまでの最短の符号付き角度差を返す。"""
    return (float(angle) - float(target) + 180.0) % 360.0 - 180.0


class FrontWallAlignmentController:
    """前壁の傾きからサーボ操舵角を求めるPD制御器。"""

    def __init__(self) -> None:
        self.previous_error = None
        self.previous_time = None

    def reset(self) -> None:
        self.previous_error = None
        self.previous_time = None

    def update(
        self,
        front_wall: dict | None,
        now: float | None = None,
        front_distance: float | None = None,
    ) -> dict:
        now = time.monotonic() if now is None else now
        if front_wall is None:
            self.reset()
            return {
                "active": False, "aligned": False, "normal_angle": None,
                "angle_error": None, "steering_angle": 0.0,
            }

        normal_angle = float(front_wall["normal_angle"])
        error = signed_angle_error(normal_angle)
        derivative = 0.0
        if self.previous_error is not None and self.previous_time is not None:
            dt = now - self.previous_time
            if dt > 0.0:
                derivative = (error - self.previous_error) / dt

        fine_control = bool(front_distance is not None
                            and front_distance <= FINE_CONTROL_DISTANCE)
        kp = FINE_ALIGN_KP if fine_control else ALIGN_KP
        kd = FINE_ALIGN_KD if fine_control else ALIGN_KD
        maximum_steering = FINE_MAX_STEERING_ANGLE if fine_control else MAX_STEERING_ANGLE
        aligned = abs(error) <= ANGLE_DEADBAND
        steering = 0.0 if aligned else STEERING_SIGN * (
            kp * error + kd * derivative
        )
        steering = max(-maximum_steering, min(maximum_steering, steering))
        self.previous_error = error
        self.previous_time = now

        return {
            "active": True, "aligned": aligned,
            "normal_angle": round(normal_angle, 1),
            "target_normal_angle": TARGET_NORMAL_ANGLE,
            "angle_error": round(error, 1),
            "steering_angle": round(steering, 1),
            "fine_control": fine_control,
            "maximum_steering_angle": maximum_steering,
        }


lidar = LidarReader(
    port="/dev/serial0",
    baudrate=230400,
    scan_frequency_increase_hz=6,
)
controller = FrontWallAlignmentController()
control_running = False
control_thread = None
drive_started_at = None
motion_state = (
    "initial_turning_right"
    if RUN_DIRECTION == COUNTERCLOCKWISE
    else "forward"
)
state_started_at = None
gyro_reset_attempted = False
gyro_ready = False
gyro_error = None
gyro_yaw = None
straight_target_yaw = None
left_yaw_sign = None
turn_start_yaw = None
final_reverse_turn_start_yaw = None
final_reverse_turn_target_yaw = None
straight_near_samples = 0
clockwise_initial_reverse_turn_complete = False


def _control_loop() -> None:
    global drive_started_at, motion_state, state_started_at
    global gyro_reset_attempted, gyro_ready, gyro_error, gyro_yaw
    global straight_target_yaw, left_yaw_sign, turn_start_yaw
    global final_reverse_turn_start_yaw, final_reverse_turn_target_yaw
    global straight_near_samples
    global clockwise_initial_reverse_turn_complete
    while control_running:
        started_at = time.monotonic()
        points = lidar.get_points()
        walls = detect_walls(points, maximum_distance=DETECTION_RANGE)
        classified_front_wall = min(
            (wall for wall in walls if wall.get("is_front_wall")),
            key=lambda wall: float(wall["front_distance"]),
            default=None,
        )
        final_target_wall = select_final_target_wall(walls)
        front_wall = (
            final_target_wall
            if motion_state in FINAL_WALL_STATES
            else classified_front_wall
        )
        # 現在の工程で選ばれた目標壁だけを、姿勢制御と
        # 停止距離判定に使用する。
        front_distance = (
            float(front_wall["wall_distance"])
            if front_wall is not None
            else None
        )
        if motion_state == "driving_straight":
            # 直進区間ではLiDARの壁角度を操舵に使わない。
            # LiDARはfront_distanceによる減速・停止判定だけに使用する。
            alignment = controller.update(None, started_at, front_distance)
        else:
            alignment = controller.update(
                front_wall,
                started_at,
                front_distance,
            )

        if gyro_ready:
            try:
                gyro_yaw = round(get_angle("z"), 1)
            except Exception as error:
                gyro_ready = False
                gyro_error = str(error)
                motion_state = "turn_error"
                gyro_yaw = None

        if motion_state == "forward" and (
            front_distance is not None
            and front_distance <= FRONT_STOP_DISTANCE
        ):
            state_started_at = started_at
            controller.reset()
            motion_state = "braking"
        elif motion_state == "braking" and (
            state_started_at is not None
            and started_at - state_started_at >= BRAKE_BEFORE_REVERSE_DURATION
        ):
            motion_state = "reversing"
            state_started_at = started_at
            controller.reset()
        elif motion_state == "reversing" and (
            front_distance is not None
            and front_distance >= (
                CLOCKWISE_BACKUP_TARGET_DISTANCE
                if (
                    RUN_DIRECTION == CLOCKWISE
                    and not clockwise_initial_reverse_turn_complete
                )
                else BACKUP_TARGET_DISTANCE
            )
        ):
            motion_state = "complete"
            state_started_at = started_at
        elif motion_state == "initial_turning_right" and (
            gyro_yaw is not None
            and abs(gyro_yaw) >= INITIAL_RIGHT_TURN_ANGLE
        ):
            motion_state = "forward"
            state_started_at = started_at
            drive_started_at = None
            controller.reset()
        elif motion_state == "initial_turning_right_reverse" and (
            gyro_yaw is not None
            and abs(gyro_yaw) >= INITIAL_RIGHT_REVERSE_TURN_ANGLE
        ):
            clockwise_initial_reverse_turn_complete = True
            gyro_reset_attempted = False
            motion_state = "forward"
            state_started_at = started_at
            drive_started_at = None
            controller.reset()
        elif motion_state == "turning_left" and (
            gyro_yaw is not None
            and abs(gyro_yaw) >= LEFT_TURN_TARGET_ANGLE
        ):
            left_yaw_sign = 1.0 if gyro_yaw >= 0.0 else -1.0
            # 旋回終了時の実測角度をそのまま保持目標にする。
            # 固定90°へ戻そうとする切替直後の急な反対舵を防ぐ。
            straight_target_yaw = gyro_yaw
            motion_state = "driving_straight"
            state_started_at = started_at
            straight_near_samples = 0
        elif motion_state == "driving_straight":
            straight_elapsed = (
                started_at - state_started_at
                if state_started_at is not None
                else 0.0
            )
            if straight_elapsed < STRAIGHT_DISTANCE_IGNORE_SECONDS:
                straight_near_samples = 0
            elif (
                front_distance is not None
                and (
                    front_distance >= STRAIGHT_TARGET_WALL_DISTANCE
                    if RUN_DIRECTION == COUNTERCLOCKWISE
                    else front_distance <= STRAIGHT_TARGET_WALL_DISTANCE
                )
            ):
                straight_near_samples += 1
                if (
                    straight_near_samples
                    >= STRAIGHT_DISTANCE_CONFIRM_SAMPLES
                ):
                    # 時計回りは接近、反時計回りは後進で離れて
                    # 目標距離へ達した場合だけ停止する。
                    motion_state = "second_turn_braking"
                    state_started_at = started_at
            else:
                straight_near_samples = 0
        elif motion_state == "second_turn_braking" and (
            state_started_at is not None
            and started_at - state_started_at >= SECOND_TURN_STOP_SECONDS
        ):
            motion_state = "turning_left_to_180"
            state_started_at = started_at
            turn_start_yaw = gyro_yaw
        elif motion_state == "turning_left_to_180" and (
            gyro_yaw is not None
            and turn_start_yaw is not None
            and abs(gyro_yaw - turn_start_yaw) >= SECOND_LEFT_TURN_ANGLE
        ):
            # 旋回完了後、余分な直進を挟まず直ちに姿勢制御へ移る。
            motion_state = "final_forward"
            state_started_at = started_at
            drive_started_at = None
            controller.reset()
        elif motion_state == "final_forward" and (
            front_distance is not None
            and front_distance <= FRONT_STOP_DISTANCE
        ):
            state_started_at = started_at
            controller.reset()
            try:
                # 最後の10 cm到達時を、後退旋回用ジャイロの0°にする。
                reset_angle("z")
                gyro_yaw = round(get_angle("z"), 1)
                gyro_ready = True
                gyro_error = None
                motion_state = "final_braking"
            except Exception as error:
                gyro_ready = False
                gyro_error = str(error)
                motion_state = "turn_error"
        elif motion_state == "final_braking" and (
            state_started_at is not None
            and started_at - state_started_at >= BRAKE_BEFORE_REVERSE_DURATION
        ):
            motion_state = "final_reversing"
            state_started_at = started_at
            controller.reset()
        elif motion_state == "final_reversing" and (
            front_distance is not None
            and front_distance >= FINAL_BACKUP_TARGET_DISTANCE
        ):
            final_reverse_turn_start_yaw = gyro_yaw
            if RUN_DIRECTION == COUNTERCLOCKWISE:
                motion_state = "final_turning_right_reverse"
            else:
                final_reverse_turn_target_yaw = (
                    -left_yaw_sign * FINAL_REVERSE_TURN_TARGET_ANGLE
                    if left_yaw_sign is not None
                    else -FINAL_REVERSE_TURN_TARGET_ANGLE
                )
                motion_state = "final_turning_left_reverse"
            state_started_at = started_at
            controller.reset()
        elif motion_state == "final_turning_right_reverse" and (
            gyro_yaw is not None
            and final_reverse_turn_start_yaw is not None
            and abs(gyro_yaw - final_reverse_turn_start_yaw)
            >= FINAL_REVERSE_TURN_TARGET_ANGLE
        ):
            motion_state = "final_complete"
            state_started_at = started_at
            controller.reset()
        elif motion_state == "final_turning_left_reverse" and (
            gyro_yaw is not None
            and final_reverse_turn_target_yaw is not None
            and final_reverse_turn_start_yaw is not None
            and (
                (
                    final_reverse_turn_start_yaw
                    >= final_reverse_turn_target_yaw
                    and gyro_yaw <= final_reverse_turn_target_yaw
                )
                or (
                    final_reverse_turn_start_yaw
                    < final_reverse_turn_target_yaw
                    and gyro_yaw >= final_reverse_turn_target_yaw
                )
            )
        ):
            motion_state = "final_complete"
            state_started_at = started_at
            controller.reset()
        elif motion_state in TURN_STATES and (
            state_started_at is not None
            and started_at - state_started_at >= LEFT_TURN_TIMEOUT
        ):
            motion_state = "turn_error"
            gyro_error = "旋回がタイムアウトしました"

        driving = bool(
            (
                front_wall is not None
                and front_distance is not None
                and motion_state in WALL_DRIVE_STATES
            )
            or (motion_state in TURN_STATES and gyro_ready)
            or (motion_state == "driving_straight" and gyro_ready)
        )
        if driving and drive_started_at is None:
            drive_started_at = started_at
        drive_elapsed = (
            started_at - drive_started_at
            if driving and drive_started_at is not None
            else 0.0
        )
        forward_speed = (
            ALIGN_START_SPEED
            if motion_state in ("forward", "final_forward")
            and driving
            and drive_elapsed < ALIGN_START_DURATION
            else ALIGN_DRIVE_SPEED
            if motion_state in ("forward", "final_forward")
            and driving
            else 0.0
        )
        drive_speed = (
            LEFT_TURN_SPEED
            if motion_state in (
                "initial_turning_right", "turning_left",
                "turning_left_to_180",
            )
            and driving
            else -BACKUP_SPEED
            if motion_state == "initial_turning_right_reverse" and driving
            else -STRAIGHT_SLOW_SPEED
            if (
                motion_state == "driving_straight"
                and driving
                and RUN_DIRECTION == COUNTERCLOCKWISE
                and front_distance is not None
                and front_distance
                >= COUNTERCLOCKWISE_REVERSE_SLOWDOWN_DISTANCE
            )
            else -STRAIGHT_DRIVE_SPEED
            if motion_state == "driving_straight"
            and driving
            and RUN_DIRECTION == COUNTERCLOCKWISE
            else STRAIGHT_SLOW_SPEED
            if (
                motion_state == "driving_straight"
                and driving
                and front_distance is not None
                and front_distance <= STRAIGHT_SLOWDOWN_DISTANCE
            )
            else STRAIGHT_DRIVE_SPEED
            if motion_state == "driving_straight" and driving
            else
            -BACKUP_SPEED
            if motion_state == "reversing" and driving
            else
            -FINAL_BACKUP_SLOW_SPEED
            if (
                motion_state == "final_reversing"
                and driving
                and front_distance is not None
                and front_distance >= FINAL_BACKUP_SLOWDOWN_DISTANCE
            )
            else
            -FINAL_BACKUP_SPEED
            if motion_state == "final_reversing" and driving
            else
            -FINAL_BACKUP_SLOW_SPEED
            if motion_state in (
                "final_turning_left_reverse", "final_turning_right_reverse",
            ) and driving
            else forward_speed
        )
        commanded_steering = (
            RIGHT_TURN_STEERING_ANGLE
            if motion_state in (
                "initial_turning_right", "initial_turning_right_reverse",
                "final_turning_right_reverse",
            )
            else
            LEFT_TURN_STEERING_ANGLE
            if motion_state in (
                "turning_left", "turning_left_to_180",
                "final_turning_left_reverse",
            )
            else max(
                -GYRO_STRAIGHT_MAX_STEERING,
                min(
                    GYRO_STRAIGHT_MAX_STEERING,
                    (1.0 if RUN_DIRECTION == COUNTERCLOCKWISE else -1.0)
                    * left_yaw_sign
                    * GYRO_STRAIGHT_KP
                    * (straight_target_yaw - gyro_yaw),
                ),
            )
            if (
                motion_state == "driving_straight"
                and left_yaw_sign is not None
                and straight_target_yaw is not None
                and gyro_yaw is not None
            )
            else
            -alignment["steering_angle"]
            if motion_state in ("reversing", "final_reversing")
            else alignment["steering_angle"]
        )

        try:
            if driving:
                set_angle(commanded_steering)
                dc_motor(drive_speed)
            elif motion_state in BRAKE_STATES:
                brake()
                set_angle(0)
            else:
                stop()
                set_angle(0)
        except Exception as error:
            try:
                stop()
            except Exception:
                pass
            alignment["servo_error"] = str(error)

        # 後退完了後、ブレーキ状態を一定時間維持して車体が停止してから、
        # 一度だけヨー角を0°へリセットする。
        if (
            motion_state == "complete"
            and not gyro_reset_attempted
            and state_started_at is not None
            and started_at - state_started_at >= BRAKE_BEFORE_REVERSE_DURATION
        ):
            gyro_reset_attempted = True
            try:
                reset_angle("z")
                gyro_ready = True
                gyro_error = None
                gyro_yaw = round(get_angle("z"), 1)
                turn_start_yaw = gyro_yaw
                motion_state = (
                    "initial_turning_right_reverse"
                    if (
                        RUN_DIRECTION == CLOCKWISE
                        and not clockwise_initial_reverse_turn_complete
                    )
                    else "turning_left"
                )
                state_started_at = time.monotonic()
            except Exception as error:
                gyro_ready = False
                gyro_error = str(error)
                motion_state = "turn_error"
        remaining = CONTROL_INTERVAL - (time.monotonic() - started_at)
        if remaining > 0.0:
            time.sleep(remaining)


def _reset_runtime(run_direction: int) -> None:
    """新しい駐車走行用に、制御状態を初期値へ戻す。"""
    global RUN_DIRECTION, drive_started_at, motion_state
    global state_started_at, gyro_reset_attempted, gyro_ready, gyro_error
    global gyro_yaw, straight_target_yaw, left_yaw_sign, turn_start_yaw
    global final_reverse_turn_start_yaw, final_reverse_turn_target_yaw
    global straight_near_samples
    global clockwise_initial_reverse_turn_complete

    RUN_DIRECTION = run_direction
    drive_started_at = state_started_at = None
    gyro_reset_attempted = gyro_ready = False
    gyro_error = gyro_yaw = straight_target_yaw = left_yaw_sign = None
    turn_start_yaw = final_reverse_turn_start_yaw = None
    final_reverse_turn_target_yaw = None
    straight_near_samples = 0
    clockwise_initial_reverse_turn_complete = False
    motion_state = (
        "initial_turning_right"
        if run_direction == COUNTERCLOCKWISE
        else "forward"
    )
    controller.reset()


def start_parking(run_direction: int) -> threading.Thread:
    """方向（0=時計回り、1=反時計回り）を指定して駐車制御を開始する。"""
    global control_running, control_thread, gyro_ready, gyro_error, gyro_yaw
    global state_started_at
    if run_direction not in (CLOCKWISE, COUNTERCLOCKWISE):
        raise ValueError("run_direction must be 0 (clockwise) or 1 (counterclockwise)")
    if control_running:
        raise RuntimeError("parking control is already running")

    _load_hardware()
    _reset_runtime(run_direction)
    if run_direction == COUNTERCLOCKWISE:
        # 最初の右80°旋回用に、制御開始前のヨー角を0°へ合わせる。
        reset_angle("z")
        gyro_ready = True
        gyro_error = None
        gyro_yaw = round(get_angle("z"), 1)
        state_started_at = time.monotonic()
    lidar.start()
    control_running = True
    control_thread = threading.Thread(
        target=_control_loop,
        daemon=True,
        name="front-wall-alignment",
    )
    control_thread.start()
    return control_thread


def stop_parking() -> None:
    """駐車制御を止め、モーター・ジャイロ・LiDARを安全に終了する。"""
    global control_running, control_thread
    if not control_running and control_thread is None and stop is None:
        return
    control_running = False
    if control_thread is not None:
        control_thread.join(timeout=1.0)
        control_thread = None
    if stop is not None:
        try:
            stop()
            set_angle(0)
        except Exception:
            pass
    close_gyro()
    lidar.stop()


@atexit.register
def cleanup() -> None:
    stop_parking()


if __name__ == "__main__":
    try:
        start_parking(1).join()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"Control start error: {error}")
