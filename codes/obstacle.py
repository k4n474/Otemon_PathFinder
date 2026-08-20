"""障害物競技で使うロボットの走行制御。

処理の流れは大きく次の3段階に分かれる。
1. カメラで走行対象を探す
2. 対象物と黒壁を避けながら青線の検出回数を数える
3. 規定回数の青線と競技ごとの追加条件を満たしたら停止する
"""

import time

import RPi.GPIO as GPIO

from camera_detector import PiColorDetector, build_primary_target_line
from gyro import get_angle, reset_angle, close_gyro
# from ultrasound import us_get, dis_get, us_back_get, dis_back_get

from buzzer import buzzer_start, buzzer_stop, buzzer_sleep, hurt_beats
from button import button_sleep
from lidar_read import LidarReader
from lidar_wall_follow import follow_wall_until_front_distance
from park import start_parking
# 機器の初期化
# ---------------------------------------------------------------------------

lidar = LidarReader(scan_frequency_increase_hz=6)

# ---------------------------------------------------------------------------
# 走行設定
# ---------------------------------------------------------------------------


# ジャイロを使った旋回走行
GYRO_TURN_TIMEOUT_SECONDS = 20.0  # ジャイロ旋回を強制終了するまでの最大秒数


# 障害物・壁回避のPD制御
AVOID_GREEN_TARGET_ANGLE = -40  # 緑オブジェクト回避時の目標線角度（度）
AVOID_RED_TARGET_ANGLE = 40  # 赤オブジェクト回避時の目標線角度（度）
AVOID_STEERING_MAX = 37  # オブジェクト回避で許可する最大操舵角（度）
AVOID_WALL_STEERING_ANGLE = 30.0  # 黒壁を検知したときの固定操舵角（度）
AVOID_KP = 1.5  # 目標線の角度ずれに対する比例補正の強さ
AVOID_KD = 0.2  # 目標線の角度変化に対する微分補正の強さ
AVOID_GO_STRAIGHT_BELOW_Y = 200  # 物体中心がこのY座標より下なら直進する
AVOID_WALL_STEERING_UPDATE_MIN = 0.5  # 壁回避の操舵を更新する最小角度差（度）
AVOID_POWER_BOOST_STEERING_THRESHOLD = 30  # パワーを上げる操舵角の境界値（度）
AVOID_POWER_BOOST = 5  # 急操舵時にモーターパワーへ加える値

# 後退確認
BACK_CHECK_AREA_THRESHOLD = 2000  # 後退が必要と判定する物体の最小面積
BACK_CHECK_SECONDS = 1.25  # 後退を継続する秒数

# 青線の検出判定
BLUE_LINE_COOLDOWN_SECONDS = 2.5  # 同じ青線の二重計上を防ぐ無視時間（秒）
BLUE_LINE_CROSSING_TARGET = 4  # 終了判定を開始する青線の目標通過回数
BLUE_LINE_LOST_CONFIRM_SECONDS = 1.0  # direction=0で青線消失を確定する秒数
BLUE_LINE_LOST_CONFIRM_SECONDS_DIRECTION_ONE = 1.5  # direction=1で青線消失を確定する秒数

# 障害物競技中に常時点灯する後方ライト
REAR_LIGHT_PIN = 21  # 後方ライトを接続するGPIO番号（BCM）

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


def blue_line_finish_reached(
    finish_state=None,
    finish_delay_seconds=0.0,
    result=None,
    require_magenta_absent=False,
):
    """青線が目標回数に達し、追加の終了条件も満たしたか判定する。

    ``require_magenta_absent`` が有効な場合は、周回方向ごとに決めた時間、
    最後の青線が連続して見えなくなった後、マゼンタの個数を判定する。
    """
    if blue_line_crossing_count < BLUE_LINE_CROSSING_TARGET:
        return False

    if require_magenta_absent:
        if finish_state is None:
            return False

        if result is not None:
            current_time = time.monotonic()
            blue_line_is_detected = result.get("blue_line") is not None
            direction = finish_state.get("direction", 0)
            blue_line_lost_confirm_seconds = (
                BLUE_LINE_LOST_CONFIRM_SECONDS_DIRECTION_ONE
                if direction == 1
                else BLUE_LINE_LOST_CONFIRM_SECONDS
            )

            if not finish_state.get("blue_line_cleared", False):
                if blue_line_is_detected:
                    finish_state["blue_line_absent_since"] = None
                elif finish_state.get("blue_line_absent_since") is None:
                    finish_state["blue_line_absent_since"] = current_time
                elif (
                    current_time - finish_state["blue_line_absent_since"]
                    >= blue_line_lost_confirm_seconds
                ):
                    finish_state["blue_line_cleared"] = True
                    print(
                        "\n青線が"
                        f"{blue_line_lost_confirm_seconds:.1f}秒以上"
                        "見えなくなりました"
                    )

            if finish_state.get("blue_line_cleared", False):
                magenta_count = len(result.get("magenta_objects", []))
                magenta_limit = 1 if direction == 1 else 0
                finish_state["magenta_condition_met"] = (
                    magenta_count <= magenta_limit
                )

        if not finish_state.get("blue_line_cleared", False):
            return False
        if not finish_state.get("magenta_condition_met", False):
            return False

    if finish_state is None or finish_delay_seconds <= 0.0:
        return True

    current_time = time.monotonic()
    if finish_state["finish_at"] is None:
        finish_state["finish_at"] = current_time + finish_delay_seconds
        print(
            f"\n青線を目標回数検出: あと{finish_delay_seconds:.1f}秒走行します"
        )

    return current_time >= finish_state["finish_at"]


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


def back_check(power, keep_camera_running=False):
    """1フレーム確認し、必要なら後退してマゼンタの左右を返す。

    Returns:
        int | None: マゼンタが画面左側なら0、右側なら1、未検出なら0。
    """
    started_detector_here = detector.camera is None
    try:
        if started_detector_here:
            detector.start()

        result = detector.process_once()
        _color_name, obj = select_front_object(result)
        magenta_obj = select_front_magenta_object(result)

        magenta_direction = 0
        if magenta_obj is not None:
            frame_center_x = result["frame"].shape[1] / 2.0
            magenta_direction = 0 if magenta_obj["center"][0] < frame_center_x else 1

        if obj is not None and obj["area"] >= BACK_CHECK_AREA_THRESHOLD:
            print("Need to back")
            set_angle(0)
            dc_motor(power)
            time.sleep(BACK_CHECK_SECONDS)

        return magenta_direction
    finally:
        stop()
        set_angle(0)
        if started_detector_here and not keep_camera_running:
            detector.stop()


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


def find_obj(
    duty_cycle,
    rd,
    finish_state=None,
    finish_delay_seconds=0.0,
    require_magenta_absent=False,
):
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
            crossing_count = update_blue_line_crossing(result)
            if blue_line_finish_reached(
                finish_state,
                finish_delay_seconds,
                result,
                require_magenta_absent,
            ):
                stop()
                set_angle(0)
                return None

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


def avoid_obj(
    duty_cycle,
    direction,
    finish_state=None,
    finish_delay_seconds=0.0,
    require_magenta_absent=False,
):
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
    current_duty_cycle = duty_cycle
    buzzer_active = False
    buzzer_stop()
    dc_motor(duty_cycle)

    while True:
        result = detector.process_once()
        crossing_count = update_blue_line_crossing(result)
        primary = result.get("primary_detection")

        if blue_line_finish_reached(
            finish_state,
            finish_delay_seconds,
            result,
            require_magenta_absent,
        ):
            buzzer_stop()
            stop()
            set_angle(0)
            return

        # 黒壁が見えている間は、物体より壁の回避を優先する。
        if result.get("black_wall_on_probe", False):
            if buzzer_active:
                buzzer_stop()
                buzzer_active = False
            if current_duty_cycle != duty_cycle:
                dc_motor(duty_cycle)
                current_duty_cycle = duty_cycle
            wall_steering = wall_steering_sign * AVOID_WALL_STEERING_ANGLE
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
            buzzer_stop()
            stop()
            set_angle(0)
            return

        color_name, obj = control_target
        object_y = obj["center"][1]
        if object_y > AVOID_GO_STRAIGHT_BELOW_Y:
            if buzzer_active:
                buzzer_stop()
                buzzer_active = False
            if current_duty_cycle != duty_cycle:
                dc_motor(duty_cycle)
                current_duty_cycle = duty_cycle
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
        steering_over_threshold = (
            abs(steering) > AVOID_POWER_BOOST_STEERING_THRESHOLD
        )
        if steering_over_threshold and not buzzer_active:
            buzzer_start()
            buzzer_active = True
        elif not steering_over_threshold and buzzer_active:
            buzzer_stop()
            buzzer_active = False

        boosted_duty_cycle = min(100.0, duty_cycle + AVOID_POWER_BOOST)
        desired_duty_cycle = (
            boosted_duty_cycle
            if steering_over_threshold
            else duty_cycle
        )
        if current_duty_cycle != desired_duty_cycle:
            dc_motor(desired_duty_cycle)
            current_duty_cycle = desired_duty_cycle
            if desired_duty_cycle == boosted_duty_cycle:
                print(
                    "[avoid_obj] ステアリング角が"
                    f"±{AVOID_POWER_BOOST_STEERING_THRESHOLD:g}°を超えました: "
                    f"steering={steering:+.1f}°, power={desired_duty_cycle:.1f}"
                )
            else:
                print(
                    "[avoid_obj] ステアリング角が"
                    f"±{AVOID_POWER_BOOST_STEERING_THRESHOLD:g}°以内に戻りました: "
                    f"steering={steering:+.1f}°, power={desired_duty_cycle:.1f}"
                )
        set_angle(steering)
        previous_error = error
        previous_time = current_time
        previous_color = color_name


def _run_obstacle_challenge(
    power,
    direction,
    finish_delay_seconds=0.0,
    require_magenta_absent=False,
):
    """指定した終了遅延で障害物競技を実行する。"""
    finish_state = {
        "finish_at": None,
        "direction": direction,
        "blue_line_absent_since": None,
        "blue_line_cleared": False,
        "magenta_condition_met": False,
    }
    try:
        # back_checkから動作中のカメラを引き継いだ場合は再初期化しない。
        if detector.camera is None:
            detector.start()
        # lidar.start()

        while True:
            # 対象物が見つかるまで探索する。
            find_obj(
                power + 20,
                direction,
                finish_state,
                finish_delay_seconds,
                require_magenta_absent,
            )

            if blue_line_finish_reached(
                finish_state,
                finish_delay_seconds,
                require_magenta_absent=require_magenta_absent,
            ):
                break

            # 見つけた対象物と壁を回避する。
            avoid_obj(
                power,
                direction,
                finish_state,
                finish_delay_seconds,
                require_magenta_absent,
            )

            if blue_line_finish_reached(
                finish_state,
                finish_delay_seconds,
                require_magenta_absent=require_magenta_absent,
            ):
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
        buzzer_stop()
        detector.stop()
        # close_gyro()
        if detector.recording_path is not None:
            print(
                f"録画保存: scp otm@192.168.137.213:{detector.recording_path} "
                "~/workspace/pivideos"
            )
        # cleanup()


def obstacle_challenge(power, direction):
    """青線を目標回数検出したら、すぐに停止する。"""
    _run_obstacle_with_rear_light(
        power,
        direction,
    )


def _run_obstacle_with_rear_light(
    power,
    direction,
    finish_delay_seconds=0.0,
):
    """後方ライトを点灯し、障害物競技終了時に必ず消灯する。"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(REAR_LIGHT_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(REAR_LIGHT_PIN, GPIO.HIGH)
    try:
        _run_obstacle_challenge(
            power,
            direction,
            finish_delay_seconds=finish_delay_seconds,
        )
    finally:
        GPIO.output(REAR_LIGHT_PIN, GPIO.LOW)


def obstacle_challenge_np(power):
    """周回方向別の青線消失時間とマゼンタ個数で停止する。"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(REAR_LIGHT_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.output(REAR_LIGHT_PIN, GPIO.HIGH)
    # button_sleep()
    try:
        direction = back_check(
            (power + 10) * -1,
            keep_camera_running=True,
        )
        _run_obstacle_challenge(
            power,
            direction,
            require_magenta_absent=True,
        )
    finally:
        GPIO.output(REAR_LIGHT_PIN, GPIO.LOW)


def main():
    obstacle_challenge_np(62)
    stop()
    # dc_motor(35)
    # time.sleep(1)
    # stop()
    # start_parking(1).join()
if __name__ == "__main__":
    main()
