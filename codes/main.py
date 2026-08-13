"""障害物競技で使うロボットの走行制御。

処理の流れは大きく次の3段階に分かれる。
1. カメラで走行対象を探す
2. 対象物と黒壁を避けながら青線の検出回数を数える
3. 規定回数の青線を検出したらすぐに停止する
"""

import time

from camera_detector import PiColorDetector, build_primary_target_line
from gyro import get_angle, reset_angle, close_gyro
# from ultrasound import us_get, dis_get, us_back_get, dis_back_get

from buzzer import buzzer_start, buzzer_stop, buzzer_sleep, hurt_beats
from lidar_read import LidarReader
from lidar_wall_follow import follow_wall_until_front_distance
from park import start_parking
# 機器の初期化
# ---------------------------------------------------------------------------

lidar = LidarReader(scan_frequency_increase_hz=6)

# ---------------------------------------------------------------------------
# 走行設定
# ---------------------------------------------------------------------------

# 基本設定
TIMEOUT_SECONDS = 30
FAR_OBJECT_AREA_MAX = 3500

# ジャイロを使った旋回走行
GYRO_TURN_TIMEOUT_SECONDS = 20.0

# 壁と床の境界線に合わせるPD制御
WALL_STRAIGHT_TARGET_ANGLE = 0.0
WALL_STRAIGHT_KP = 3.5
WALL_STRAIGHT_KD = 0.4
WALL_STRAIGHT_MAX_STEERING = 45

# 障害物・壁回避のPD制御
AVOID_GREEN_TARGET_ANGLE = -35
AVOID_RED_TARGET_ANGLE = 35
AVOID_STEERING_MAX = 45
AVOID_WALL_STEERING_MAX = 50.0
AVOID_KP = 1.5
AVOID_KD = 0.2
AVOID_GO_STRAIGHT_BELOW_Y = 200
AVOID_WALL_STEERING_UPDATE_MIN = 0.5

# 青線の検出判定
BLUE_LINE_COOLDOWN_SECONDS = 2.5
BLUE_LINE_CROSSING_TARGET = 5

detector = PiColorDetector(enable_recording=True, detect_boundary_enabled=False)
from newobot import dc_motor, set_angle, stop, cleanup

# ---------------------------------------------------------------------------
# 走行中に引き継ぐ状態
# ---------------------------------------------------------------------------

blue_line_crossing_count = 0
blue_line_was_detected = False
blue_line_ignore_until = 0.0


def drive_along_wall_until_front_distance(
    stop_distance,
    *,
    target_side_distance=250.0,
    duty_cycle=42.0,
    trace_side=None,
    timeout=30.0,
):
    """側壁との距離を保ち、前壁が指定距離に達するまで走行する。"""
    started_lidar_here = not lidar.running

    try:
        if started_lidar_here:
            lidar.start()

        return follow_wall_until_front_distance(
            lidar,
            front_stop_distance=stop_distance,
            target_side_distance=target_side_distance,
            motor_speed=duty_cycle,
            trace_side=trace_side,
            timeout=timeout,
            drive_motor=dc_motor,
            set_steering=set_angle,
            stop_motor=stop,
        )
    finally:
        if started_lidar_here:
            lidar.stop()


def set_boundary_detection(enabled):
    """カメラの境界線検出を有効または無効にする。"""
    detector.detect_boundary_enabled = enabled


def set_object_detection(enabled):
    """カメラの物体検出を有効または無効にする。"""
    detector.detect_objects_enabled = enabled


def gyro_turn(
    target_angle,
    steering_angle,
    duty_cycle,
    timeout=GYRO_TURN_TIMEOUT_SECONDS,
):
    """指定した操舵角で走り、ヨー角が目標に達したら停止する。

    ``target_angle`` の符号でジャイロの旋回方向を指定する。
    ``steering_angle`` の符号は到達判定には使用しない。
    ジャイロはこの関数内ではリセットせず、最後にリセットした時点からの
    ヨー角を ``target_angle`` と比較する。
    """
    if timeout <= 0:
        raise ValueError("timeout は0より大きくしてください。")

    started_at = time.monotonic()
    turned_angle = get_angle("z")
    first_angle = turned_angle

    def target_reached():
        if target_angle >= first_angle:
            return turned_angle >= target_angle
        return turned_angle <= target_angle

    try:
        set_angle(steering_angle)
        dc_motor(duty_cycle)

        while not target_reached():
            turned_angle = get_angle("z")
            print(
                f"ジャイロ旋回: {turned_angle:.1f}°"
                f" / 目標: {target_angle:.1f}°",
                end="\r",
                flush=True,
            )

            if time.monotonic() - started_at >= timeout:
                raise RuntimeError(
                    f"{target_angle:.1f}度のジャイロ旋回がタイムアウトしました"
                )

            time.sleep(0.01)
    finally:
        stop()
        set_angle(0)

    print(f"\nジャイロ旋回完了: {turned_angle:.1f}°")
    return turned_angle


def update_blue_line_crossing(result):
    """
    青線が見えていない状態から見えたとき、検出回数を増やす。
    カウント後は一定時間青線を無視し、同じ線の再検出による二重計上を防ぐ。

    カウンターはavoid_objを抜けても保持されるため、走行開始からの合計になる。
    """
    global blue_line_crossing_count
    global blue_line_was_detected
    global blue_line_ignore_until

    if blue_line_crossing_count >= BLUE_LINE_CROSSING_TARGET:
        return blue_line_crossing_count

    current_time = time.monotonic()
    if current_time < blue_line_ignore_until:
        return blue_line_crossing_count

    blue_line_is_detected = result.get("blue_line") is not None
    if blue_line_is_detected and not blue_line_was_detected:
        blue_line_crossing_count += 1
        blue_line_was_detected = True
        blue_line_ignore_until = current_time + BLUE_LINE_COOLDOWN_SECONDS
        detector.set_blue_line_crossing_count(blue_line_crossing_count)
        print(f"\nBLUE LINE DETECTED: {blue_line_crossing_count}")
    elif not blue_line_is_detected:
        blue_line_was_detected = False

    return blue_line_crossing_count


def select_front_object(result):
    """検出結果から、画面の一番下に映っている物体を選ぶ。"""
    selected_color = None
    selected_object = None

    candidates = []
    for red_object in result["red_objects"]:
        candidates.append(("red", red_object))

    for green_object in result["green_objects"]:
        candidates.append(("green", green_object))

    for color_name, obj in candidates:
        if selected_object is None:
            selected_color = color_name
            selected_object = obj
            continue

        object_y = obj["center"][1]
        selected_y = selected_object["center"][1]

        is_lower = object_y > selected_y
        is_same_height = object_y == selected_y
        is_larger = obj["area"] > selected_object["area"]

        if is_lower or (is_same_height and is_larger):
            selected_color = color_name
            selected_object = obj

    return selected_color, selected_object


def select_front_magenta_object(result):
    """検出したマゼンタのうち、画面上で最も手前の物体を選ぶ。"""
    selected_object = None

    for obj in result.get("magenta_objects", []):
        if selected_object is None:
            selected_object = obj
            continue

        object_y = obj["center"][1]
        selected_y = selected_object["center"][1]

        is_lower = object_y > selected_y
        is_same_height = object_y == selected_y
        is_larger = obj["area"] > selected_object["area"]

        if is_lower or (is_same_height and is_larger):
            selected_object = obj

    return selected_object


def find_obj(duty_cycle, rd):
    """
    最初に停止状態でカメラを確認し、オブジェクトが映っていない場合だけ
    ステアリングを切って前進しながら探索する。

    Returns:
        str | None: 見つけた色名。中断されたら None
    """
    steering_angle = AVOID_STEERING_MAX
    search_started = False
    try:
        while True:
            result = detector.process_once()
            color_name, obj = select_front_object(result)

            if obj is not None:
                center_x, center_y = obj["center"]
                width, height = obj["size"]
                stop()
                set_angle(0)
                return color_name

            if not search_started:
                if rd == 0:
                    set_angle(steering_angle)
                else:
                    set_angle(steering_angle * -1)
                dc_motor(duty_cycle)
                search_started = True

    except KeyboardInterrupt:
        stop()
        set_angle(0)
        raise


def wall_straight(duty_cycle, stop_y):
    """壁と床の境界線が水平になるようPD制御しながら前進する。

    境界線の角度を0°へ近づけるように操舵する。境界線を見失った場合、
    または境界線中央のY座標が ``stop_y`` 以上になった場合は、
    安全な状態で停止して終了する。
    """
    previous_boundary_detection = detector.detect_boundary_enabled
    previous_error = None
    previous_time = None
    motor_started = False
    detector.detect_boundary_enabled = True

    try:
        while True:
            result = detector.process_once()

            # 録画スレッドに残っている、境界検出を有効にする前のフレームは
            # 使用しない。検出設定が反映された新しいフレームを待つ。
            if result.get("boundary_status") == "boundary: disabled":
                continue

            boundary = result.get("boundary")
            if boundary is None:
                return
            if boundary["y_at_center"] >= stop_y:
                return

            if not motor_started:
                dc_motor(duty_cycle)
                motor_started = True

            current_time = time.monotonic()
            error = boundary["angle_deg"] - WALL_STRAIGHT_TARGET_ANGLE

            if previous_error is None or previous_time is None:
                derivative = 0.0
            else:
                elapsed = current_time - previous_time
                derivative = (
                    (error - previous_error) / elapsed
                    if elapsed > 0.0
                    else 0.0
                )

            steering = WALL_STRAIGHT_KP * error + WALL_STRAIGHT_KD * derivative
            steering = max(
                -WALL_STRAIGHT_MAX_STEERING,
                min(WALL_STRAIGHT_MAX_STEERING, steering),
            )
            set_angle(steering)

            previous_error = error
            previous_time = current_time
    finally:
        stop()
        set_angle(0)
        detector.detect_boundary_enabled = previous_boundary_detection


def avoid_obj(duty_cycle, direction):
    """
    物体のPD回避と黒壁回避を、1回のカメラ取得ループ内で実行する。

    direction=1は現在の検査線位置と右操舵、direction=0は検査線位置を
    左右反転して左操舵する。壁検知が消えたら物体のPD制御へ戻る。
    """
    if direction not in (0, 1):
        raise ValueError("avoid_obj の direction は0または1にしてください。")

    detector.set_black_wall_probe_direction(direction)
    wall_steering_sign = 1.0 if direction == 1 else -1.0
    previous_error = None
    previous_time = None
    previous_color = None
    previous_wall_steering = None
    dc_motor(duty_cycle)

    while True:
        result = detector.process_once()
        crossing_count = update_blue_line_crossing(result)
        primary = result.get("primary_detection")

        if crossing_count >= BLUE_LINE_CROSSING_TARGET:
            stop()
            set_angle(0)
            return

        # 黒壁が見えている間は、物体より壁の回避を優先する。
        if result.get("black_wall_on_probe", False):
            black_ratio = max(
                0.0,
                min(1.0, result.get("black_wall_ratio", 0.0)),
            )
            wall_steering = (
                wall_steering_sign
                * AVOID_WALL_STEERING_MAX
                * black_ratio
            )
            if (
                previous_wall_steering is None
                or abs(wall_steering - previous_wall_steering)
                >= AVOID_WALL_STEERING_UPDATE_MIN
            ):
                set_angle(wall_steering)
                previous_wall_steering = wall_steering

            # 壁回避後に微分項が急増しないよう、PD履歴をリセットする。
            previous_error = None
            previous_time = None
            previous_color = None
            continue

        previous_wall_steering = None

        control_target = primary
        target_line = build_primary_target_line(control_target, result["frame"])
        line_angle = target_line["angle_deg"] if target_line is not None else None
        if control_target is None or line_angle is None:
            stop()
            set_angle(0)
            return

        color_name, obj = control_target
        object_y = obj["center"][1]
        if object_y > AVOID_GO_STRAIGHT_BELOW_Y:
            set_angle(0)
            previous_error = None
            previous_time = None
            previous_color = None
            continue

        target_angle = (
            AVOID_GREEN_TARGET_ANGLE
            if color_name == "green"
            else AVOID_RED_TARGET_ANGLE
        )
        current_time = time.monotonic()
        error = line_angle - target_angle

        if (
            previous_error is None
            or previous_time is None
            or previous_color != color_name
        ):
            derivative = 0.0
        else:
            elapsed = current_time - previous_time
            derivative = (
                (error - previous_error) / elapsed
                if elapsed > 0.0
                else 0.0
            )

        steering = AVOID_KP * error + AVOID_KD * derivative
        steering = max(-AVOID_STEERING_MAX, min(AVOID_STEERING_MAX, steering))
        set_angle(steering)
        previous_error = error
        previous_time = current_time
        previous_color = color_name


def obstacle_challenge(power, direction):
    """カメラを開始し、探索と回避を終了条件まで繰り返す。"""
    try:
        detector.start()
        # lidar.start()

        while True:
            # 対象物が見つかるまで探索する。
            find_obj(power, direction)

            # 見つけた対象物と壁を回避する。
            avoid_obj(power, direction)

            if blue_line_crossing_count >= BLUE_LINE_CROSSING_TARGET:
                break

        stop()
        set_angle(0)
    except KeyboardInterrupt:
        print("\nCtrl+C を受け付けたため終了します")
        stop()
        set_angle(0)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"エラー: {exc}")
    finally:
        detector.stop()
        # close_gyro()
        if detector.recording_path is not None:
            print(
                f"録画保存: scp otm@10.129.219.239:{detector.recording_path} "
                "~/workspace/pivideos"
            )
        # cleanup()


def main():
    obstacle_challenge(45, 1)
    stop()
    dc_motor(-45)
    time.sleep(1)
    stop()
    start_parking(1).join()
if __name__ == "__main__":
    main()
