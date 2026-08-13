"""左右で最も長い壁を選び、200 mm離れて走る。"""
import time
import RPi.GPIO as GPIO
from threading import Lock, Thread

from flask import Flask, jsonify
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
MOTOR_SPEED = 42
TURN_MOTOR_SPEED = 46
STEERING_KP = 0.1
STEERING_KI = 0.01
STEERING_KD = 0.12
MAX_STEERING_ANGLE = 35
INTEGRAL_LIMIT = 800
INTERVAL = 0.1
FRONT_WALL_TURN_DISTANCE = 420
FRONT_WALL_PLAN_DISTANCE = 420
TURN_STEERING_ANGLE = 40
TURN_ANGLE_REDUCTION = 0.0
MIN_TURN_TARGET_ANGLE = 30.0
TURN_TIMEOUT = 20.0
TURN_END_STOP_SECONDS = 1.0
TRACE_SELECTION_SAMPLES = 0
MAX_TURN_COUNT = 12
FINAL_STOP_FRONT_DISTANCE = 1600
WALL_ROLE_LOCK_DELAY = 2.0


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
    "turn": {
        "status": "waiting",
        "active": False,
        "direction": None,
        "target_angle": None,
        "turned_angle": 0.0
    }
}


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

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUZZER, GPIO.OUT)

def buzzer_stop():
    GPIO.output(BUZZER, GPIO.LOW)   # 止める
    
    
def buzzer_sleep():
    GPIO.output(BUZZER, GPIO.HIGH)  # 鳴らす
    time.sleep(0.1)
    GPIO.output(BUZZER, GPIO.LOW)   # 止める


def run():
    lidar = LidarReader()
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
    locked_role_angles = None
    role_lock_ready_at = None
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
        lidar.start()
        BUZZER = 19

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUZZER, GPIO.OUT)

        print("ジャイロを初期化中です。ロボットを動かさないでください")
        reset_angle("z")
        print("ジャイロ角度を0°にリセットしました")
        viewer_thread.start()
        print(
            "Live Viewer API: "
            "http://<Raspberry PiのIP>:5000/api/points"
        )
        previous_time = time.monotonic()
        # time.sleep(4)
        role_lock_ready_at = time.monotonic() + WALL_ROLE_LOCK_DELAY
        while True:
            current_time = time.monotonic()
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
            update_viewer(
                points,
                detected_walls,
                side_walls
            )
            detected_turn_angle = (
                turn_angle_for_front_wall(front_wall)
                if front_wall is not None
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
                and front_wall is not None
                and front_wall["front_distance"]
                <= FINAL_STOP_FRONT_DISTANCE
            ):
                stop()
                set_angle(0)
                print(
                    "\n3周完了後の停止位置に到達: "
                    f"前壁まで {front_wall['front_distance']:.1f} mm"
                )
                break

            if (
                front_wall is not None
                and turn_target_is_valid
                and front_wall["front_distance"]
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
                    f"{front_wall['front_distance']:.1f} mm"
                    f" / 壁垂線角度: "
                    f"{front_wall['normal_angle']:.1f}°"
                    f" / 旋回方向: {turn_direction}"
                    f" / 目標旋回角度: {target_turn_angle:.1f}°"
                )
                pid.reset()
                locked_role_angles = None
                role_lock_ready_at = None
                turn_by_front_wall(
                    front_wall,
                    turn_direction,
                    target_turn_angle
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
                    print(
                        "3周完了: 前壁まで1300 mmの位置へ走行します"
                    )

                planned_turn_direction = None
                planned_turn_angle = None
                previous_time = time.monotonic()
                continue

            if (
                front_wall is not None
                and detected_turn_angle is not None
                and detected_turn_angle > MIN_TURN_TARGET_ANGLE
                and front_wall["front_distance"]
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
                        f" {front_wall['front_distance']:.1f} mm"
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
        stop()
        set_angle(0)
        lidar.stop()
        close_gyro()
        cleanup()


if __name__ == "__main__":
    run()
