"""左右で最も長い壁を選び、200 mm離れて走る。"""
import time
from threading import Lock, Thread

from flask import Flask, jsonify
from flask_cors import CORS

from algorithm import (
    detect_corners,
    detect_side_walls,
    detect_walls
)
from lidar_read import LidarReader
from newobot import cleanup, dc_motor, set_angle, stop


TARGET_DISTANCE = 200
MOTOR_SPEED = 60
STEERING_KP = 0.12
MAX_STEERING_ANGLE = 35
INTERVAL = 0.1


viewer_app = Flask(__name__)
CORS(viewer_app)
viewer_lock = Lock()
viewer_data = {
    "count": 0,
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
            "points": points,
            "wall_count": len(walls),
            "walls": walls,
            "corner_count": len(corners),
            "corners": corners,
            "front_wall_detected": front_wall is not None,
            "front_wall": front_wall,
            "side_walls": side_walls
        })


def steering_for_wall(wall):
    """壁との距離を200 mmに保つステアリング角を返す。"""

    distance_error = wall["wall_distance"] - TARGET_DISTANCE
    side_sign = 1 if wall["side"] == "right" else -1
    steering = side_sign * STEERING_KP * distance_error
    return max(
        -MAX_STEERING_ANGLE,
        min(MAX_STEERING_ANGLE, steering)
    )


def run():
    lidar = LidarReader()
    trace_side = None
    viewer_thread = Thread(
        target=start_viewer_api,
        daemon=True,
        name="lidar-viewer-api"
    )

    try:
        stop()
        set_angle(0)
        lidar.start()
        viewer_thread.start()
        print(
            "Live Viewer API: "
            "http://<Raspberry PiのIP>:5000/api/points"
        )

        while True:
            points = lidar.get_points()
            detected_walls = detect_walls(points)
            side_walls = detect_side_walls(points)
            update_viewer(
                points,
                detected_walls,
                side_walls
            )
            left_wall = side_walls["left"]
            right_wall = side_walls["right"]

            if (
                trace_side is None
                and side_walls["trace"] is not None
            ):
                trace_side = side_walls["trace"]["side"]

            trace_wall = (
                side_walls[trace_side]
                if trace_side is not None
                else None
            )

            if trace_wall is None:
                stop()
                set_angle(0)
                status = (
                    f"trace: {trace_side or 'None'}"
                    "（未検出）/ motor: STOP"
                )
            else:
                steering = steering_for_wall(trace_wall)
                set_angle(steering * -1)
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
        cleanup()


if __name__ == "__main__":
    run()
