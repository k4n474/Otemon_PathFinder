import atexit

from flask import Flask, jsonify, send_from_directory
from flask_cors import CORS

from algorithm import detect_corners, detect_side_walls, detect_walls
from lidar_read import LidarReader


app = Flask(__name__)
CORS(app)


lidar = LidarReader(
    port="/dev/serial0",
    baudrate=230400,
    scan_frequency_increase_hz=6
)


try:
    lidar.start()

except Exception as error:
    print(f"LiDAR start error: {error}")


@app.get("/")
def home():
    return send_from_directory(
        app.root_path,
        "index.html"
    )


@app.get("/api/points")
def api_points():
    points = lidar.get_points()
    walls = detect_walls(points)
    corners = detect_corners(walls)
    side_walls = detect_side_walls(points)
    front_walls = [
        wall
        for wall in walls
        if wall.get("is_front_wall")
    ]
    front_wall = min(
        front_walls,
        key=lambda wall: wall["front_distance"],
        default=None
    )

    return jsonify({
        "count": len(points),
        "fps": round(lidar.get_fps(), 1),
        "points": points,
        "wall_count": len(walls),
        "walls": walls,
        "corner_count": len(corners),
        "corners": corners,
        "front_wall_detected": front_wall is not None,
        "front_wall": front_wall,
        "side_walls": side_walls
    })


@app.get("/api/status")
def api_status():
    return jsonify(
        lidar.get_status()
    )


@atexit.register
def cleanup() -> None:
    lidar.stop()


if __name__ == "__main__":
    app.run(
        host="0.0.0.0",
        port=5000,
        debug=False,
        threaded=True
    )
