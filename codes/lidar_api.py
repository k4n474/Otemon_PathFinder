"""LiDAR viewer API. 単体起動、または駐車中の共有データを配信する。"""
from flask import Flask, jsonify, send_from_directory
from flask_cors import CORS
from werkzeug.serving import make_server
from threading import Thread

from algorithm import detect_corners, detect_front_and_side_walls, detect_walls


def build_payload(points, walls, status):
    front, sides = detect_front_and_side_walls(points, detected_walls=walls)
    corners = detect_corners(walls)
    return {
        "count": len(points), "fps": status.get("fps", status.get("scan_fps", 0)),
        "points": points, "walls": walls, "wall_count": len(walls),
        "corners": corners, "corner_count": len(corners),
        "front_wall_detected": front is not None, "front_wall": front,
        "side_walls": sides, "lidar_status": status,
    }


def create_app(read_frame):
    """read_frameは(points, walls, status)を返す。importでは機器を起動しない。"""
    app = Flask(__name__)
    CORS(app)

    @app.get("/")
    def home():
        return send_from_directory(app.root_path, "index.html")

    @app.get("/api/points")
    def points():
        try:
            return jsonify(build_payload(*read_frame()))
        except Exception as error:
            return jsonify({"error": str(error)}), 503

    @app.get("/api/status")
    def status():
        try:
            return jsonify(read_frame()[2])
        except Exception as error:
            return jsonify({"error": str(error)}), 503

    return app


class LidarViewerServer:
    def __init__(self, read_frame, host="0.0.0.0", port=5000):
        self.server = make_server(host, port, create_app(read_frame), threaded=True)
        self.thread = Thread(target=self.server.serve_forever, daemon=True,
                             name="lidar-viewer")

    def start(self):
        self.thread.start()

    def stop(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()


def main():
    # 観察だけをしたい場合。モーターは初期化しない。
    from lidar_read import LidarReader
    lidar = LidarReader(port="/dev/serial0", baudrate=230400,
                        scan_frequency_increase_hz=6)

    def read_frame():
        points = lidar.get_points()
        return points, detect_walls(points, maximum_distance=1500), lidar.get_status()

    try:
        lidar.start()
        create_app(read_frame).run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    finally:
        lidar.stop()


if __name__ == "__main__":
    main()
