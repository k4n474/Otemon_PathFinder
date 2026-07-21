"""
ブラウザから見られる簡易MJPEGプレビューサーバー。

- `/` でプレビュー画面
- `/stream.mjpg` で映像ストリーム
- `/action/<name>` でボタン操作
"""

from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock, Thread
from urllib.parse import urlparse

import cv2


class PreviewServer:
    def __init__(self, host="0.0.0.0", port=8000, title="Pi Camera Preview"):
        self.host = host
        self.port = port
        self.title = title
        self._frame_lock = Lock()
        self._status_lock = Lock()
        self._action_lock = Lock()
        self._latest_jpeg = None
        self._status_lines = []
        self._actions = []
        self._server = None
        self._thread = None

    def start(self):
        server_ref = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    self._serve_index()
                    return
                if parsed.path == "/stream.mjpg":
                    self._serve_stream()
                    return
                if parsed.path.startswith("/action/"):
                    action = parsed.path.split("/action/", 1)[1]
                    server_ref.push_action(action)
                    self.send_response(HTTPStatus.SEE_OTHER)
                    self.send_header("Location", "/")
                    self.end_headers()
                    return

                self.send_error(HTTPStatus.NOT_FOUND)

            def log_message(self, format, *args):
                return

            def _serve_index(self):
                body = server_ref.render_index().encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_stream(self):
                self.send_response(HTTPStatus.OK)
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()

                try:
                    while True:
                        frame = server_ref.get_latest_jpeg()
                        if frame is None:
                            continue

                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    return

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None

    def publish_frame(self, frame):
        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        if not ok:
            return
        with self._frame_lock:
            self._latest_jpeg = encoded.tobytes()

    def get_latest_jpeg(self):
        with self._frame_lock:
            return self._latest_jpeg

    def set_status(self, *lines):
        with self._status_lock:
            self._status_lines = [line for line in lines if line]

    def get_status(self):
        with self._status_lock:
            return list(self._status_lines)

    def push_action(self, action):
        with self._action_lock:
            self._actions.append(action)

    def pop_actions(self):
        with self._action_lock:
            actions = list(self._actions)
            self._actions.clear()
            return actions

    def render_index(self):
        status_html = "".join(f"<li>{line}</li>" for line in self.get_status())
        return f"""<!doctype html>
<html lang="ja">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{self.title}</title>
  <style>
    body {{
      margin: 0;
      font-family: -apple-system, BlinkMacSystemFont, sans-serif;
      background: #0f172a;
      color: #e2e8f0;
    }}
    .wrap {{
      max-width: 980px;
      margin: 0 auto;
      padding: 20px;
    }}
    img {{
      width: 100%;
      border-radius: 12px;
      background: #020617;
      border: 1px solid #334155;
    }}
    .panel {{
      margin-top: 16px;
      padding: 16px;
      border-radius: 12px;
      background: #111827;
      border: 1px solid #334155;
    }}
    .buttons {{
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      margin-top: 12px;
    }}
    a.button {{
      text-decoration: none;
      color: white;
      background: #2563eb;
      padding: 10px 14px;
      border-radius: 10px;
    }}
    li {{
      margin: 6px 0;
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <h1>{self.title}</h1>
    <img src="/stream.mjpg" alt="camera preview">
    <div class="panel">
      <strong>Status</strong>
      <ul>{status_html}</ul>
      <div class="buttons">
        <a class="button" href="/action/red">Red</a>
        <a class="button" href="/action/green">Green</a>
        <a class="button" href="/action/other">Other</a>
        <a class="button" href="/action/save">Save Sample</a>
        <a class="button" href="/action/clear">Clear Current</a>
        <a class="button" href="/action/quit">Quit</a>
      </div>
    </div>
  </div>
</body>
</html>
"""
