"""障害物走行用のLiDAR壁判定結果をLive Viewerへ配信する。"""

import atexit
import math
import time
from threading import Event, Lock, Thread

from flask import Flask, jsonify
from flask_cors import CORS

from algorithm import detect_walls
from lidar_read import LidarReader
from newobot import cleanup as motor_cleanup
from newobot import dc_motor, set_angle, stop
from buzzer import buzzer_start, buzzer_stop, buzzer_sleep, hurt_beats


app = Flask(__name__)
CORS(app)

lidar = LidarReader(
    port="/dev/serial0",
    baudrate=230400
)

GUIDE_LINES = [
    {
        "x": -100.0,
        "start": {"x": -100.0, "y": -3000.0},
        "end": {"x": -100.0, "y": 3000.0}
    },
    {
        "x": 100.0,
        "start": {"x": 100.0, "y": -3000.0},
        "end": {"x": 100.0, "y": 3000.0}
    }
]
MOTOR_SPEED = 50
AVOIDANCE_STEERING_ANGLE = 40
OBSTACLE_X_LIMIT = 100.0
OBSTACLE_HALF_VIEW_ANGLE = 60.0
PERPENDICULAR_WALL_TOLERANCE = 30.0
CONTROL_INTERVAL = 0.05
# 0: 反時計回り、1: 時計回り
which_direction = 1

state_lock = Lock()
stop_event = Event()
controller_thread = None
latest_state = {
    "count": 0,
    "points": [],
    "wall_count": 0,
    "walls": [],
    "short_wall": None,
    "long_wall": None,
    "target_wall": None,
    "obstacle_point": None,
    "avoidance_active": False,
    "avoidance_direction": None,
    "which_direction": which_direction,
    "guide_lines": GUIDE_LINES
}


def classify_wall_lengths(walls):
    """検出壁へshort/longの長さ分類を付けて返す。"""

    classified = [
        {**wall, "length_type": None}
        for wall in walls
    ]

    if not classified:
        return classified, None, None

    short_wall = min(
        classified,
        key=lambda wall: wall["length"]
    )
    long_wall = max(
        classified,
        key=lambda wall: wall["length"]
    )
    short_wall["length_type"] = "short"
    long_wall["length_type"] = (
        "short_and_long"
        if long_wall is short_wall
        else "long"
    )
    return classified, short_wall, long_wall


def mark_walls_perpendicular_to_front(walls):
    """前壁と垂直方向にある壁へ表示用フラグを付ける。"""

    marked_walls = [
        {**wall, "is_perpendicular_to_front": False}
        for wall in walls
    ]
    front_walls = [
        wall
        for wall in marked_walls
        if wall.get("is_front_wall")
    ]

    if not front_walls:
        return marked_walls

    front_wall = min(
        front_walls,
        key=lambda wall: wall["wall_distance"]
    )
    front_line = front_wall["line"]
    maximum_perpendicular_dot = math.sin(
        math.radians(PERPENDICULAR_WALL_TOLERANCE)
    )

    for wall in marked_walls:
        if wall is front_wall:
            continue

        line = wall["line"]
        normal_dot = abs(
            float(line["a"]) * float(front_line["a"])
            + float(line["b"]) * float(front_line["b"])
        )
        wall["is_perpendicular_to_front"] = (
            normal_dot <= maximum_perpendicular_dot
        )

    return marked_walls


def target_wall_for_course_direction(walls, direction):
    """
    前壁をx軸として、垂直壁のx座標からP判定用の壁を返す。
    """

    if direction not in (0, 1):
        raise ValueError("which_directionは0または1にしてください")

    front_walls = [
        wall
        for wall in walls
        if wall.get("is_front_wall")
    ]

    if not front_walls:
        return None

    front_wall = min(
        front_walls,
        key=lambda wall: wall["wall_distance"]
    )
    front_line = front_wall["line"]
    # 前壁の直線方向を新しいx軸とし、正方向をロボット右側に揃える。
    axis_x = -float(front_line["b"])
    axis_y = float(front_line["a"])

    if axis_x < 0.0:
        axis_x *= -1.0
        axis_y *= -1.0

    candidates = [
        {
            **wall,
            "selection_x": round(
                float(wall["closest_point"]["x"]) * axis_x
                + float(wall["closest_point"]["y"]) * axis_y,
                1
            )
        }
        for wall in walls
        if wall.get("is_perpendicular_to_front")
    ]

    if not candidates:
        return None

    key = lambda wall: wall["selection_x"]

    if direction == 0:
        return min(candidates, key=key)

    return max(candidates, key=key)


def obstacle_point_from_wall(target_wall):
    """対象壁の端点から前方±60°内の回避判定点Pを選ぶ。"""

    if target_wall is None:
        return None

    candidates = []

    for endpoint_name in ("start", "end"):
        endpoint = target_wall.get(endpoint_name)

        if endpoint is None:
            continue

        x = float(endpoint["x"])
        y = float(endpoint["y"])
        angle = math.degrees(math.atan2(x, y))

        if abs(angle) > OBSTACLE_HALF_VIEW_ANGLE:
            continue

        candidates.append({
            "x": x,
            "y": y,
            "angle": round(angle, 1),
            "distance": round(math.hypot(x, y), 1),
            "endpoint": endpoint_name
        })

    if not candidates:
        return None

    points_in_path = [
        point
        for point in candidates
        if -OBSTACLE_X_LIMIT < point["x"] < OBSTACLE_X_LIMIT
    ]
    selectable = points_in_path or candidates
    return min(selectable, key=lambda point: point["distance"])


def update_avoidance(point, course_direction):
    """点Pと周回方向に応じて操舵し、現在の回避状態を返す。"""

    if course_direction not in (0, 1):
        raise ValueError("which_directionは0または1にしてください")

    if (
        point is None
        or not -OBSTACLE_X_LIMIT < point["x"] < OBSTACLE_X_LIMIT
    ):
        set_angle(0)
        stop()
        return False, None

    # 反時計回りは右へ、時計回りは左へ回避する。
    direction = "right" if course_direction == 0 else "left"

    steering = (
        AVOIDANCE_STEERING_ANGLE
        if direction == "right"
        else -AVOIDANCE_STEERING_ANGLE
    )
    set_angle(steering)
    dc_motor(MOTOR_SPEED)
    return True, direction


def process_obstacle_frame(lidar_reader, course_direction):
    """LiDARの現在値を1回処理し、回避状態を返す。"""

    points = lidar_reader.get_points()
    walls = detect_walls(points)
    walls = mark_walls_perpendicular_to_front(walls)
    walls, short_wall, long_wall = classify_wall_lengths(walls)
    target_wall = target_wall_for_course_direction(
        walls,
        course_direction
    )
    point = obstacle_point_from_wall(target_wall)
    avoidance_active, avoidance_direction = update_avoidance(
        point,
        course_direction
    )
    return {
        "count": len(points),
        "points": points,
        "wall_count": len(walls),
        "walls": walls,
        "short_wall": short_wall,
        "long_wall": long_wall,
        "target_wall": target_wall,
        "obstacle_point": point,
        "avoidance_active": avoidance_active,
        "avoidance_direction": avoidance_direction,
        "which_direction": course_direction,
        "guide_lines": GUIDE_LINES
    }


def publish_state(state):
    """最新状態をLive Viewer API用に保存する。"""

    with state_lock:
        latest_state.update(state)


def avoid_obstacle(lidar_reader, course_direction):
    """
    障害物回避を実行し、完了時の判定結果を返す。

    Pが最初から±100 mm領域内にない場合は走行せずに戻る。
    回避開始後は、Pが領域からなくなるまでこの関数内で制御する。
    """

    state = process_obstacle_frame(
        lidar_reader,
        course_direction
    )
    publish_state(state)

    if not state["avoidance_active"]:
        return state
    else:
        buzzer_start()

    while state["avoidance_active"]:
        time.sleep(CONTROL_INTERVAL)
        state = process_obstacle_frame(
            lidar_reader,
            course_direction
        )
        publish_state(state)
    
    buzzer_stop()
    return state


def control_loop():
    """LiDAR判定と回避操舵をViewerとは独立して繰り返す。"""

    while not stop_event.is_set():
        state = process_obstacle_frame(
            lidar,
            which_direction
        )
        publish_state(state)

        stop_event.wait(CONTROL_INTERVAL)


try:
    lidar.start()
except Exception as error:
    print(f"LiDAR start error: {error}")


@app.get("/")
def home():
    return jsonify({
        "message": "Obstacle LiDAR API is running",
        "points_url": "/api/points",
        "status_url": "/api/status"
    })


@app.get("/api/points")
def api_points():
    with state_lock:
        return jsonify(latest_state)


@app.get("/api/status")
def api_status():
    return jsonify(lidar.get_status())


@atexit.register
def cleanup() -> None:
    stop_event.set()
    stop()
    set_angle(0)
    lidar.stop()
    motor_cleanup()


if __name__ == "__main__":
    controller_thread = Thread(
        target=control_loop,
        daemon=True,
        name="obstacle-lidar-controller"
    )
    controller_thread.start()
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        threaded=True
    )
