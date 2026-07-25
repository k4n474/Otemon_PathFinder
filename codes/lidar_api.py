import json
import serial
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PORT = "/dev/serial0"
BAUDRATE = 230400
HTTP_PORT = 8000

latest_points = []
points_lock = threading.Lock()


def angle_from_raw(raw: int) -> float:
    """LiDAR内部の角度値を度へ変換する。"""
    return (raw >> 1) / 64.0


def parse_packet(packet: bytes) -> list[dict]:
    """
    T-mini Plusの1パケットを解析して、
    [{"angle": 10.2, "distance": 830.0, "intensity": 100}, ...]
    の形で返す。
    """
    if len(packet) < 10:
        return []

    if packet[0] != 0xAA or packet[1] != 0x55:
        return []

    sample_count = packet[3]

    first_raw = packet[4] | (packet[5] << 8)
    last_raw = packet[6] | (packet[7] << 8)

    first_angle = angle_from_raw(first_raw)
    last_angle = angle_from_raw(last_raw)

    # 359°→0°をまたぐ場合
    if last_angle < first_angle:
        last_angle += 360.0

    points = []
    data_start = 10

    for i in range(sample_count):
        index = data_start + i * 3

        if index + 2 >= len(packet):
            break

        intensity = packet[index]

        distance_raw = (
            packet[index + 1]
            | (packet[index + 2] << 8)
        )

        # T-mini Plusの距離値をmmへ変換
        distance_mm = distance_raw / 4.0

        if sample_count > 1:
            angle = first_angle + (
                (last_angle - first_angle)
                * i
                / (sample_count - 1)
            )
        else:
            angle = first_angle

        angle = (360.0 - angle) % 360.0
        
        # 後ろ130°～230°は無視
        if 120 <= angle <= 240:
            continue

        if 50 <= distance_mm <= 12000:
            points.append({
                "angle": round(angle, 2),
                "distance": round(distance_mm, 1),
                "intensity": intensity
            })

    return points


def lidar_reader():
    global latest_points

    ser = serial.Serial(
        PORT,
        BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1
    )

    buffer = bytearray()

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # スキャン開始
        ser.write(b"\xA5\x60")
        ser.flush()

        print("T-mini Plusのスキャンを開始しました")
        time.sleep(0.2)

        # 角度ごとの最新値を保存
        scan_map = {}

        while True:
            received = ser.read(4096)

            if received:
                buffer.extend(received)

            while True:
                header_index = buffer.find(b"\xAA\x55")

                if header_index == -1:
                    if len(buffer) > 1:
                        del buffer[:-1]
                    break

                if header_index > 0:
                    del buffer[:header_index]

                if len(buffer) < 10:
                    break

                sample_count = buffer[3]
                packet_length = 10 + sample_count * 3

                if len(buffer) < packet_length:
                    break

                packet = bytes(buffer[:packet_length])
                del buffer[:packet_length]

                parsed_points = parse_packet(packet)

                for point in parsed_points:
                    # 1度単位で最新値を保存
                    angle_key = int(round(point["angle"])) % 360
                    scan_map[angle_key] = point

                # ブラウザへ渡す点群を更新
                with points_lock:
                    latest_points = list(scan_map.values())

    except Exception as error:
        print(f"LiDARエラー: {error}")

    finally:
        try:
            ser.write(b"\xA5\x65")
            ser.flush()
            ser.close()
        except Exception:
            pass


class LidarAPIHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/scan":
            with points_lock:
                response = {
                    "timestamp": time.time(),
                    "count": len(latest_points),
                    "points": latest_points
                }

            body = json.dumps(response).encode("utf-8")

            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/":
            body = b"T-mini Plus API is running"

            self.send_response(200)
            self.send_header("Content-Type", "text/plain")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        # アクセスログを表示しない
        return


def main():
    lidar_thread = threading.Thread(
        target=lidar_reader,
        daemon=True
    )
    lidar_thread.start()

    server = ThreadingHTTPServer(
        ("0.0.0.0", HTTP_PORT),
        LidarAPIHandler
    )

    print(f"LiDAR API: http://localhost:{HTTP_PORT}/scan")
    print("Ctrl+Cで終了")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n終了します")
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()