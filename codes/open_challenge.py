"""左右で最も長い壁を選び、200 mm離れて走る。"""
import time
import RPi.GPIO as GPIO
from threading import Event, Lock, Thread

from flask import Flask, jsonify, send_from_directory
from flask_cors import CORS

from algorithm import (
    detect_corners,
    detect_front_and_side_walls,
    detect_walls
)
from gyro import close_gyro, get_angle, reset_angle
from lidar_read import LidarReader
from lidar_wall_follow import WallPIDController
from newobot import cleanup, dc_motor, set_angle, stop


TARGET_DISTANCE = 250
MOTOR_SPEED = 38
TURN_MOTOR_SPEED = 42
STEERING_KP = 0.1
STEERING_KI = 0.01
STEERING_KD = 0.1
MAX_STEERING_ANGLE = 30.0
INTEGRAL_LIMIT = 800
INTERVAL = 0.1
FRONT_WALL_TURN_DISTANCE = 360
FRONT_WALL_PLAN_DISTANCE = 800
TURN_STEERING_ANGLE = 40
TURN_ANGLE_REDUCTION = 5.0
MIN_TURN_TARGET_ANGLE = 40.0
TURN_TIMEOUT = 20.0
TURN_END_STOP_SECONDS = 1.0
TRACE_SELECTION_SAMPLES = 0
MAX_TURN_COUNT = 12
FINAL_RUN_STRAIGHT_SECONDS = 2.0
FINAL_STOP_FRONT_DISTANCE = 1500
WALL_ROLE_LOCK_DELAY = 2.0
FRONT_WALL_IGNORE_AFTER_TURN_SECONDS = 2.0


viewer_app = Flask(__name__)
CORS(viewer_app)
viewer_lock = Lock()
viewer_previous_update = None
viewer_data = {
    "count": 0,
    "fps": 0.0,
    "points": [],
    "wall_count": 0,
    "walls": [],
    "corner_count": 0,
    "corners": [],
    "front_wall_detected": False,
    "front_wall": None,
    "side_walls": {
        "left": None,
        "right": None,
        "trace": None
    },
    "gyro": {
        "ready": False,
        "yaw": None,
        "error": None
    },
    "turn": {
        "status": "waiting",
        "active": False,
        "direction": None,
        "target_angle": None,
        "turned_angle": 0.0
    }
}


@viewer_app.get("/")
def viewer_home():
    """Live Viewer本体をAPIと同じサーバーから配信する。"""
    return send_from_directory(viewer_app.root_path, "index.html")


@viewer_app.get("/api/points")
def api_points():
    with viewer_lock:
        return jsonify(viewer_data)


def start_viewer_api():
    """Live Serverのindex.htmlへ走行中のLiDAR判定結果を配信する。"""

    viewer_app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        threaded=True,
        use_reloader=False
    )


def update_viewer(points, walls, side_walls):
    global viewer_previous_update

    updated_at = time.monotonic()
    fps = 0.0

    if viewer_previous_update is not None:
        elapsed = updated_at - viewer_previous_update

        if elapsed > 0.0:
            fps = 1.0 / elapsed

    viewer_previous_update = updated_at
    front_wall = min(
        (
            wall
            for wall in walls
            if wall.get("is_front_wall")
        ),
        key=lambda wall: wall["front_distance"],
        default=None
    )
    corners = detect_corners(walls)

    with viewer_lock:
        viewer_data.update({
            "count": len(points),
            "fps": round(fps, 1),
            "points": points,
            "wall_count": len(walls),
            "walls": walls,
            "corner_count": len(corners),
            "corners": corners,
            "front_wall_detected": front_wall is not None,
            "front_wall": front_wall,
            "side_walls": side_walls
        })


def update_turn_viewer(
    active,
    direction,
    target_angle,
    turned_angle,
    status=None
):
    """旋回状態をLive Viewerへ配信する。"""

    with viewer_lock:
        viewer_data["turn"] = {
            "status": status or (
                "turning" if active else "completed"
            ),
            "active": active,
            "direction": direction,
            "target_angle": target_angle,
            "turned_angle": turned_angle
        }


def update_gyro_viewer():
    """現在のYaw角と読み取り状態をLive Viewerへ配信する。"""
    try:
        yaw = round(get_angle("z"), 1)
        gyro_state = {"ready": True, "yaw": yaw, "error": None}
    except RuntimeError as error:
        gyro_state = {"ready": False, "yaw": None, "error": str(error)}

    with viewer_lock:
        viewer_data["gyro"] = gyro_state


def choose_turn_direction(_walls, _side_walls, trace_side=None):
    """最初に固定した追従壁の反対へ曲がる。"""

    if trace_side == "right":
        return "left"

    if trace_side == "left":
        return "right"

    # 追従側が未決定なら、安全のため旋回方向を推測しない。
    return None


def lock_wall_role_angles(walls):
    """現在の分類名と壁の向きを、追従区間用に記録する。"""

    role_angles = {
        wall["role"]: float(wall["normal_angle"])
        for wall in walls
        if wall.get("role") in ("left", "front", "right")
    }
    if not role_angles:
        return role_angles

    canonical_angles = {
        "right": 0.0,
        "front": 90.0,
        "left": 180.0
    }
    anchor_role = next(iter(role_angles))
    rotation = (
        role_angles[anchor_role] - canonical_angles[anchor_role]
    )
    for role, canonical_angle in canonical_angles.items():
        role_angles.setdefault(
            role,
            (canonical_angle + rotation) % 360.0
        )

    return role_angles


def apply_locked_wall_roles(walls, role_angles):
    """固定した壁の向きに従って分類名を維持する。"""

    if not role_angles:
        return

    for wall in walls:
        normal_angle = float(wall["normal_angle"])
        role = min(
            role_angles,
            key=lambda name: min(
                abs(normal_angle - role_angles[name]) % 360.0,
                360.0 - abs(normal_angle - role_angles[name]) % 360.0
            )
        )
        wall["role"] = role
        wall["is_front_wall"] = role == "front"
        wall["is_side_wall"] = role in ("left", "right")
        wall["side"] = role if role in ("left", "right") else None
        wall["front_distance"] = (
            wall["wall_distance"] if role == "front" else None
        )


def turn_angle_for_front_wall(front_wall):
    """前向き軸と前壁の直線が作る角度を、旋回補正込みで返す。"""

    normal_angle = float(front_wall["normal_angle"]) % 180.0
    detected_angle = min(normal_angle, 180.0 - normal_angle)
    return max(0.0, detected_angle - TURN_ANGLE_REDUCTION)


def turn_by_front_wall(front_wall, direction, target_angle=None):
    """旋回前の前壁情報とジャイロだけを使って旋回する。"""

    steering = (
        TURN_STEERING_ANGLE
        if direction == "right"
        else -TURN_STEERING_ANGLE
    )
    if target_angle is None:
        target_angle = turn_angle_for_front_wall(front_wall)
    set_angle(steering)
    start_yaw = get_angle("z")
    started_at = time.monotonic()
    update_turn_viewer(
        True,
        direction,
        target_angle,
        0.0
    )
    dc_motor(TURN_MOTOR_SPEED)

    while True:
        current_yaw = get_angle("z")
        turned_angle = abs(current_yaw - start_yaw)
        update_turn_viewer(
            True,
            direction,
            target_angle,
            turned_angle
        )

        print(
            f"turning {direction}: {turned_angle:.1f}°"
            f" / target: {target_angle:.1f}°",
            end="\r",
            flush=True
        )

        if turned_angle >= target_angle:
            break

        if time.monotonic() - started_at >= TURN_TIMEOUT:
            stop()
            set_angle(0)
            update_turn_viewer(
                False,
                direction,
                target_angle,
                turned_angle
            )
            raise RuntimeError(
                f"{target_angle:.1f}度旋回がタイムアウトしました"
            )

        time.sleep(0.01)

    stop()
    set_angle(0)
    update_turn_viewer(
        False,
        direction,
        target_angle,
        turned_angle
    )
    print(
        f"\n旋回完了: {turned_angle:.1f}°"
        f" / 目標: {target_angle:.1f}°"
        f" / {TURN_END_STOP_SECONDS:.1f}秒停止"
    )
    # time.sleep(TURN_END_STOP_SECONDS)


BUZZER = 19
BLINK_LED = 21
PULSE_LEDS = (16, 20)
LED_TURN_INTERVAL = 0.04
BLINK_LED_BRIGHT_DUTY = 100.0
BLINK_LED_DIM_DUTY = 20.0
PULSE_LED_ON_SECONDS = 0.1
PULSE_LED_OFF_SECONDS = 0.8
PULSE_LED_BRIGHT_DUTY = 100.0
PULSE_LED_DIM_DUTY = 40.0
PULSE_LED_PWM_FREQUENCY = 200
LED_UPDATE_INTERVAL = 0.01
START_FADE_SECONDS = (1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2)
START_FULL_BRIGHT_SECONDS = 2.0
COMPLETE_SIGNAL_SECONDS = 4.0
COMPLETE_FADE_SECONDS = 1.0
COMPLETE_BUZZER_INTERVAL = 0.2
COMPLETE_BUZZER_ON_SECONDS = 0.05

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)


def run_leds(stop_event, turning_event):
    """GPIO21を状態表示し、GPIO16・20の輝度を周期的に変える。"""

    turn_led_on = True
    pulse_led_on = True
    pulse_pwms = [
        GPIO.PWM(pin, PULSE_LED_PWM_FREQUENCY)
        for pin in PULSE_LEDS
    ]
    blink_pwm = GPIO.PWM(BLINK_LED, PULSE_LED_PWM_FREQUENCY)
    blink_pwm.start(BLINK_LED_BRIGHT_DUTY)
    for pwm in pulse_pwms:
        pwm.start(PULSE_LED_BRIGHT_DUTY)

    now = time.monotonic()
    next_turn_toggle_at = now + LED_TURN_INTERVAL
    next_pulse_toggle_at = now + PULSE_LED_ON_SECONDS

    try:
        while not stop_event.is_set():
            now = time.monotonic()
            if turning_event.is_set():
                if now >= next_turn_toggle_at:
                    turn_led_on = not turn_led_on
                    blink_pwm.ChangeDutyCycle(
                        BLINK_LED_BRIGHT_DUTY
                        if turn_led_on
                        else BLINK_LED_DIM_DUTY
                    )
                    next_turn_toggle_at = now + LED_TURN_INTERVAL
            else:
                turn_led_on = True
                blink_pwm.ChangeDutyCycle(BLINK_LED_BRIGHT_DUTY)
                next_turn_toggle_at = now + LED_TURN_INTERVAL

            if now >= next_pulse_toggle_at:
                pulse_led_on = not pulse_led_on
                duty = (
                    PULSE_LED_BRIGHT_DUTY
                    if pulse_led_on
                    else PULSE_LED_DIM_DUTY
                )
                for pwm in pulse_pwms:
                    pwm.ChangeDutyCycle(duty)
                pulse_duration = (
                    PULSE_LED_ON_SECONDS
                    if pulse_led_on
                    else PULSE_LED_OFF_SECONDS
                )
                next_pulse_toggle_at = now + pulse_duration

            if stop_event.wait(LED_UPDATE_INTERVAL):
                break
    finally:
        blink_pwm.stop()
        for pwm in pulse_pwms:
            pwm.stop()
        GPIO.output((BLINK_LED, *PULSE_LEDS), GPIO.LOW)

def buzzer_stop():
    GPIO.output(BUZZER, GPIO.LOW)   # 止める
    
    
def buzzer_sleep():
    GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
    time.sleep(0.1)
    GPIO.output(BUZZER, GPIO.LOW)   # 止める


def signal_run_complete():
    """全LEDのフェードとブザーで正常終了を通知する。"""

    complete_leds = (*PULSE_LEDS, BLINK_LED)
    complete_pwms = [
        GPIO.PWM(pin, PULSE_LED_PWM_FREQUENCY)
        for pin in complete_leds
    ]
    for pwm in complete_pwms:
        pwm.start(100.0)

    started_at = time.monotonic()
    buzzer_on = False
    try:
        while True:
            elapsed = time.monotonic() - started_at
            if elapsed >= COMPLETE_SIGNAL_SECONDS:
                break

            fade_progress = (
                elapsed % COMPLETE_FADE_SECONDS
            ) / COMPLETE_FADE_SECONDS
            duty = 100.0 * (1.0 - fade_progress)
            for pwm in complete_pwms:
                pwm.ChangeDutyCycle(duty)

            should_buzz = (
                elapsed % COMPLETE_BUZZER_INTERVAL
                < COMPLETE_BUZZER_ON_SECONDS
            )
            if should_buzz != buzzer_on:
                buzzer_on = should_buzz
                GPIO.output(
                    BUZZER,
                    GPIO.HIGH if buzzer_on else GPIO.LOW
                )

            time.sleep(LED_UPDATE_INTERVAL)
    finally:
        GPIO.output(BUZZER, GPIO.LOW)
        for pwm in complete_pwms:
            pwm.stop()
        GPIO.output(complete_leds, GPIO.LOW)


def signal_run_start():
    """全LEDのフェードを加速させ、全点灯後に走行開始を通知する。"""

    start_leds = (*PULSE_LEDS, BLINK_LED)
    start_pwms = [
        GPIO.PWM(pin, PULSE_LED_PWM_FREQUENCY)
        for pin in start_leds
    ]
    for pwm in start_pwms:
        pwm.start(100.0)

    try:
        for fade_seconds in START_FADE_SECONDS:
            fade_started_at = time.monotonic()
            while True:
                elapsed = time.monotonic() - fade_started_at
                if elapsed >= fade_seconds:
                    break

                duty = 100.0 * (1.0 - elapsed / fade_seconds)
                for pwm in start_pwms:
                    pwm.ChangeDutyCycle(duty)
                time.sleep(LED_UPDATE_INTERVAL)

            for pwm in start_pwms:
                pwm.ChangeDutyCycle(0.0)

        for pwm in start_pwms:
            pwm.ChangeDutyCycle(100.0)
        time.sleep(START_FULL_BRIGHT_SECONDS)
    finally:
        for pwm in start_pwms:
            pwm.stop()
        GPIO.output(start_leds, GPIO.HIGH)


def run():
    lidar = LidarReader()
    led_stop_event = Event()
    turning_event = Event()
    led_thread = Thread(
        target=run_leds,
        args=(led_stop_event, turning_event),
        daemon=True,
        name="status-led"
    )
    trace_side = None
    trace_length_totals = {
        "left": 0.0,
        "right": 0.0
    }
    trace_selection_count = 0
    planned_turn_direction = None
    planned_turn_angle = None
    turn_count = 0
    final_run_active = False
    final_distance_check_ready_at = None
    locked_role_angles = None
    role_lock_ready_at = None
    front_wall_ignore_until = None
    run_completed = False
    pid = WallPIDController(
        target_distance=TARGET_DISTANCE,
        kp=STEERING_KP,
        ki=STEERING_KI,
        kd=STEERING_KD,
        max_steering_angle=MAX_STEERING_ANGLE,
        integral_limit=INTEGRAL_LIMIT,
    )
    viewer_thread = Thread(
        target=start_viewer_api,
        daemon=True,
        name="lidar-viewer-api"
    )

    try:
        stop()
        set_angle(0)

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUZZER, GPIO.OUT)
        GPIO.setup(BLINK_LED, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(PULSE_LEDS, GPIO.OUT, initial=GPIO.LOW)

        lidar.start()

        print("ジャイロを初期化中です。ロボットを動かさないでください")
        reset_angle("z")
        update_gyro_viewer()
        print("ジャイロ角度を0°にリセットしました")
        viewer_thread.start()
        print(
            "Live Viewer API: "
            "http://<Raspberry PiのIP>:5000/api/points"
        )
        print("起動ライト演出を開始します")
        signal_run_start()
        led_thread.start()
        print("走行を開始します")
        previous_time = time.monotonic()
        # time.sleep(4)
        role_lock_ready_at = time.monotonic() + WALL_ROLE_LOCK_DELAY
        while True:
            current_time = time.monotonic()
            update_gyro_viewer()
            dt = current_time - previous_time
            previous_time = current_time
            points = lidar.get_points()
            detected_walls = detect_walls(points)
            if locked_role_angles is not None:
                apply_locked_wall_roles(
                    detected_walls,
                    locked_role_angles
                )
            elif (
                role_lock_ready_at is not None
                and current_time >= role_lock_ready_at
            ):
                role_angles = lock_wall_role_angles(
                    detected_walls
                )
                if role_angles:
                    locked_role_angles = role_angles
                    apply_locked_wall_roles(
                        detected_walls,
                        locked_role_angles
                    )
                    print(
                        "\n壁の分類名を固定: "
                        f"{locked_role_angles}"
                    )

            front_wall, side_walls = detect_front_and_side_walls(
                points,
                detected_walls=detected_walls
            )
            ignoring_front_wall = (
                front_wall_ignore_until is not None
                and current_time < front_wall_ignore_until
            )
            control_front_wall = None if ignoring_front_wall else front_wall
            update_viewer(
                points,
                detected_walls,
                side_walls
            )
            detected_turn_angle = (
                turn_angle_for_front_wall(control_front_wall)
                if control_front_wall is not None
                else None
            )
            target_turn_angle = (
                planned_turn_angle
                if planned_turn_angle is not None
                else detected_turn_angle
            )
            turn_target_is_valid = (
                target_turn_angle is not None
                and target_turn_angle > MIN_TURN_TARGET_ANGLE
            )
            
            if (
                final_run_active
                and final_distance_check_ready_at is not None
                and current_time >= final_distance_check_ready_at
                and control_front_wall is not None
                and control_front_wall["front_distance"]
                <= FINAL_STOP_FRONT_DISTANCE
            ):
                stop()
                set_angle(0)
                print(
                    "\n3周完了後の停止位置に到達: "
                    f"前壁まで {control_front_wall['front_distance']:.1f} mm"
                )
                run_completed = True
                break

            if (
                not final_run_active
                and control_front_wall is not None
                and turn_target_is_valid
                and control_front_wall["front_distance"]
                <= FRONT_WALL_TURN_DISTANCE
            ):
                turn_direction = (
                    planned_turn_direction
                    or choose_turn_direction(
                        detected_walls,
                        side_walls,
                        trace_side
                    )
                )
                if turn_direction is None:
                    stop()
                    set_angle(0)
                    print(
                        "\n旋回方向を判定できないため停止します"
                    )
                    time.sleep(INTERVAL)
                    continue

                print(
                    "\n前方壁を検出: "
                    f"{control_front_wall['front_distance']:.1f} mm"
                    f" / 壁垂線角度: "
                    f"{control_front_wall['normal_angle']:.1f}°"
                    f" / 旋回方向: {turn_direction}"
                    f" / 目標旋回角度: {target_turn_angle:.1f}°"
                )
                pid.reset()
                locked_role_angles = None
                role_lock_ready_at = None
                turning_event.set()
                try:
                    turn_by_front_wall(
                        control_front_wall,
                        turn_direction,
                        target_turn_angle
                    )
                finally:
                    turning_event.clear()
                front_wall_ignore_until = (
                    time.monotonic()
                    + FRONT_WALL_IGNORE_AFTER_TURN_SECONDS
                )
                role_lock_ready_at = (
                    time.monotonic() + WALL_ROLE_LOCK_DELAY
                )
                turn_count += 1
                print(
                    f"旋回回数: {turn_count} / {MAX_TURN_COUNT}"
                )

                if turn_count >= MAX_TURN_COUNT:
                    final_run_active = True
                    final_distance_check_ready_at = (
                        time.monotonic() + FINAL_RUN_STRAIGHT_SECONDS
                    )
                    print(
                        "3周完了: 旋回後に"
                        f"{FINAL_RUN_STRAIGHT_SECONDS:.1f}秒間"
                        "壁トレースを続けた後、"
                        "前壁との距離による停止判定を開始します: "
                        f"停止距離 {FINAL_STOP_FRONT_DISTANCE} mm"
                    )

                planned_turn_direction = None
                planned_turn_angle = None
                previous_time = time.monotonic()
                continue

            if (
                not final_run_active
                and control_front_wall is not None
                and detected_turn_angle is not None
                and detected_turn_angle > MIN_TURN_TARGET_ANGLE
                and control_front_wall["front_distance"]
                <= FRONT_WALL_PLAN_DISTANCE
            ):
                if planned_turn_angle is None:
                    planned_turn_direction = choose_turn_direction(
                        detected_walls,
                        side_walls,
                        trace_side
                    )
                    if planned_turn_direction is None:
                        stop()
                        set_angle(0)
                        print(
                            "\n旋回方向を計画できないため停止します"
                        )
                        time.sleep(INTERVAL)
                        continue

                    planned_turn_angle = detected_turn_angle
                    # 旋回開始の予告として、計画確定時点から高速点滅する。
                    turning_event.set()
                    update_turn_viewer(
                        False,
                        planned_turn_direction,
                        planned_turn_angle,
                        0.0,
                        status="planned"
                    )
                    print(
                        "\n旋回計画を確定: "
                        f"{planned_turn_direction}"
                        f" / {planned_turn_angle:.1f}°"
                        f" / 前壁まで"
                        f" {control_front_wall['front_distance']:.1f} mm"
                    )

            left_wall = side_walls["left"]
            right_wall = side_walls["right"]

            if trace_side is None:
                if left_wall is not None and right_wall is not None:
                    trace_length_totals["left"] += left_wall["length"]
                    trace_length_totals["right"] += right_wall["length"]
                    trace_selection_count += 1

                    if trace_selection_count >= TRACE_SELECTION_SAMPLES:
                        trace_side = max(
                            ("left", "right"),
                            key=lambda side: trace_length_totals[side]
                        )
                        pid.reset()
                        print(
                            "\n追従壁を確定: "
                            f"{trace_side}"
                            " / 平均長 left="
                            f"{trace_length_totals['left'] / trace_selection_count:.1f} mm"
                            " / right="
                            f"{trace_length_totals['right'] / trace_selection_count:.1f} mm"
                        )
                else:
                    trace_length_totals = {
                        "left": 0.0,
                        "right": 0.0
                    }
                    trace_selection_count = 0

            trace_wall = (
                side_walls[trace_side]
                if trace_side is not None
                else None
            )

            if trace_wall is None:
                pid.reset()
                stop()
                set_angle(0)
                status = (
                    f"trace: {trace_side or 'None'}"
                    "（未検出）/ motor: STOP"
                )
            else:
                steering = pid.update(trace_wall, dt)
                # カーブ直前は角を側壁として拾ってPID出力が反転することが
                # ある。旋回計画後は予定方向と逆の操舵だけを抑止する。
                if planned_turn_direction == "right":
                    steering = max(0.0, steering)
                elif planned_turn_direction == "left":
                    steering = min(0.0, steering)
                set_angle(steering)
                dc_motor(MOTOR_SPEED)
                status = (
                    f"trace: {trace_wall['side']} / "
                    f"length: {trace_wall['length']:.1f} mm / "
                    f"distance: {trace_wall['wall_distance']:.1f} mm / "
                    f"steering: {steering:+.1f}°"
                )

            left_text = (
                "None"
                if left_wall is None
                else (
                    f"{left_wall['wall_distance']:.0f} mm"
                    f" (長さ {left_wall['length']:.0f} mm)"
                )
            )
            right_text = (
                "None"
                if right_wall is None
                else (
                    f"{right_wall['wall_distance']:.0f} mm"
                    f" (長さ {right_wall['length']:.0f} mm)"
                )
            )
            print(
                f"left: {left_text} / right: {right_text} / {status}",
                end="\r",
                flush=True
            )
            time.sleep(INTERVAL)

    except KeyboardInterrupt:
        print("\n終了します")

    finally:
        led_stop_event.set()
        if led_thread.is_alive():
            led_thread.join(timeout=LED_TURN_INTERVAL + 0.1)
        if run_completed:
            signal_run_complete()
        GPIO.output((BLINK_LED, *PULSE_LEDS), GPIO.LOW)
        stop()
        set_angle(0)
        lidar.stop()
        close_gyro()
        cleanup()
        GPIO.cleanup((
            BUZZER,
            BLINK_LED,
            *PULSE_LEDS
        ))


if __name__ == "__main__":
    run()
