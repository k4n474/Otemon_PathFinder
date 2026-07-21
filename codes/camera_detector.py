"""
メイン制御から読み込んで使うためのカメラ検知モジュール。
"""

from pathlib import Path
from threading import Condition, Thread
from datetime import datetime
import time

import cv2
import numpy as np
from picamera2 import Picamera2

from preview_server import PreviewServer


BASE_DIR = Path(__file__).resolve().parent
DEFAULT_SAMPLE_DIR = BASE_DIR / "samples"
DEFAULT_RECORDING_DIR = BASE_DIR / "recordings"

FRAME_SIZE = (480,270)# (320, 180)(640, 360)(960, 540)
DEFAULT_IGNORE_BELOW_Y = FRAME_SIZE[1] * 2 // 3
# main は処理用の軽い出力サイズ、raw は広い画角を保つためのセンサー読み出しサイズ。
RAW_SENSOR_SIZE = (4608, 2592)
RECORDING_FPS = 20.0
MIN_AREA = 50
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
PREVIEW_PORT = 8000
GUIDE_COLOR = (0, 255, 255)
GUIDE_THICKNESS = 1
FPS_COLOR = (255, 255, 255)
GUIDE_CROSS_OFFSET_Y = 0
GUIDE_TOP_LINE_Y = 60
DETECTION_LIMIT_COLOR = (255, 0, 255)
BOUNDARY_COLOR = (255, 255, 0)
BOUNDARY_ROI_TOP_RATIO = 0.12
BOUNDARY_ROI_BOTTOM_RATIO = 0.90
BOUNDARY_MIN_LINE_LENGTH_RATIO = 0.20
BOUNDARY_MAX_LINE_GAP = 35
BOUNDARY_MAX_ANGLE_DEG = 30.0
BOUNDARY_DARK_VALUE_MAX = 120
BOUNDARY_FLOOR_VALUE_MIN = 45
BOUNDARY_CONTRAST_MIN = 12.0
BOUNDARY_SCAN_X_MARGIN_RATIO = 0.12
BOUNDARY_SCAN_WINDOW = 10

# ガイド枠はこの2つの数値を変えるだけで調整できます。
# 左枠の左下頂点、右枠の右下頂点は必ず画面の角に固定されます。
GUIDE_BOX_WIDTH = 140
GUIDE_BOX_HEIGHT = 240

COLOR_RULES = {
    "red": {
        "max_distance": 14.0,
        "distance_margin": 2.5,
        "min_saturation": 30,
        "min_value": 40,
        "value_weight": 0.6,
        "hue_ranges": ((0, 8), (170, 179)),
    },
    "green": {
        "max_distance": 12.5,
        "distance_margin": 1.6,
        "min_saturation": 35,
        "min_value": 22,
        "value_weight": 0.45,
        "hue_margin": 6,
    },
}


def load_hsv_samples(sample_dir: Path, color: str):
    path = sample_dir / f"{color}.npy"
    if not path.exists():
        raise FileNotFoundError(f"HSVサンプルが見つかりません: {path}")

    data = np.load(path).astype(np.float32)
    if data.ndim != 2 or data.shape[1] != 3:
        raise ValueError(f"HSVサンプルの形式が不正です: {path}")
    return data


def build_model(samples):
    hue = samples[:, 0].astype(np.float32)
    hue_angles = hue / 180.0 * 2.0 * np.pi
    hue_sin_mean = np.mean(np.sin(hue_angles))
    hue_cos_mean = np.mean(np.cos(hue_angles))
    hue_mean_angle = np.arctan2(hue_sin_mean, hue_cos_mean)
    if hue_mean_angle < 0:
        hue_mean_angle += 2.0 * np.pi
    circular_hue_mean = hue_mean_angle / (2.0 * np.pi) * 180.0

    linear_hue_mean = np.mean(hue)
    linear_hue_std = np.std(hue)
    circular_hue_diff = hue_distance(hue, circular_hue_mean)
    circular_hue_std = np.std(circular_hue_diff)

    if circular_hue_std < linear_hue_std and circular_hue_std < 20.0:
        hue_mean = circular_hue_mean
        hue_std = circular_hue_std
    else:
        hue_mean = linear_hue_mean
        hue_std = linear_hue_std

    return {
        "mean": np.array([hue_mean, np.mean(samples[:, 1]), np.mean(samples[:, 2])], dtype=np.float32),
        "std": np.clip(
            np.array([hue_std, np.std(samples[:, 1]), np.std(samples[:, 2])], dtype=np.float32),
            [4.0, 18.0, 18.0],
            [40.0, 80.0, 80.0],
        ),
    }


def hue_distance(hue_channel, center):
    diff = np.abs(hue_channel.astype(np.float32) - float(center))
    return np.minimum(diff, 180.0 - diff)


def class_distance(hsv, model, value_weight=1.0):
    hue_diff = hue_distance(hsv[:, :, 0], model["mean"][0]) / model["std"][0]
    sat_diff = (hsv[:, :, 1].astype(np.float32) - model["mean"][1]) / model["std"][1]
    val_diff = (hsv[:, :, 2].astype(np.float32) - model["mean"][2]) / model["std"][2]
    return hue_diff * hue_diff + sat_diff * sat_diff + value_weight * val_diff * val_diff


def create_mask(hsv, color_name, color_model, other_models):
    rule = COLOR_RULES[color_name]
    saturation_min = int(rule["min_saturation"])
    value_min = int(rule["min_value"])

    if "hue_ranges" in rule:
        hue_ranges = rule["hue_ranges"]
    else:
        hue_center = float(color_model["mean"][0])
        hue_margin = float(rule["hue_margin"])
        lower = hue_center - hue_margin
        upper = hue_center + hue_margin
        if lower < 0:
            hue_ranges = ((0, int(upper)), (int(180 + lower), 179))
        elif upper > 179:
            hue_ranges = ((0, int(upper - 180)), (int(lower), 179))
        else:
            hue_ranges = ((int(lower), int(upper)),)

    mask = None
    for hue_lower, hue_upper in hue_ranges:
        lower = np.array([hue_lower, saturation_min, value_min], dtype=np.uint8)
        upper = np.array([hue_upper, 255, 255], dtype=np.uint8)
        range_mask = cv2.inRange(hsv, lower, upper)
        mask = range_mask if mask is None else cv2.bitwise_or(mask, range_mask)

    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


def find_objects(mask, min_area=MIN_AREA, min_height=1):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    objects = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w == 0 or h == 0:
            continue
        if h < min_height:
            continue

        extent = area / float(w * h)
        aspect_ratio = w / float(h)
        if extent < 0.35 or not 0.25 <= aspect_ratio <= 4.0:
            continue

        m = cv2.moments(contour)
        if m["m00"] != 0:
            cx = int(m["m10"] / m["m00"])
            cy = int(m["m01"] / m["m00"])
        else:
            cx = x + w // 2
            cy = y + h // 2

        objects.append(
            {
                "bbox": (x, y, w, h),
                "center": (cx, cy),
                "area": int(area),
                "extent": round(extent, 2),
                "size": (w, h),
            }
        )

    return sorted(objects, key=lambda item: item["area"], reverse=True)


def draw_objects(frame, color_name, detections, bgr):
    for obj in detections:
        x, y, w, h = obj["bbox"]
        cx, cy = obj["center"]
        area = obj["area"]
        extent = obj["extent"]

        cv2.rectangle(frame, (x, y), (x + w, y + h), bgr, 2)
        cv2.circle(frame, (cx, cy), 5, bgr, -1)
        cv2.putText(
            frame,
            f"{color_name} ({cx},{cy}) W={w} H={h}",
            (x, max(y - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            bgr,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"A={area} E={extent}",
            (x, min(y + h + 18, FRAME_SIZE[1] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            bgr,
            2,
            cv2.LINE_AA,
        )


def draw_guide_boxes(frame, ignore_below_y=None):
    frame_h, frame_w = frame.shape[:2]
    guide_box_width = min(GUIDE_BOX_WIDTH, frame_w)
    guide_box_height = min(GUIDE_BOX_HEIGHT, frame_h)
    center_x = frame_w // 2
    center_y = (frame_h // 2) + GUIDE_CROSS_OFFSET_Y

    left_top = (0, frame_h - guide_box_height)
    left_bottom = (guide_box_width, frame_h)
    right_top = (frame_w - guide_box_width, frame_h - guide_box_height)
    right_bottom = (frame_w, frame_h)

    cv2.rectangle(frame, left_top, left_bottom, GUIDE_COLOR, GUIDE_THICKNESS)
    cv2.rectangle(frame, right_top, right_bottom, GUIDE_COLOR, GUIDE_THICKNESS)
    cv2.line(frame, (0, center_y), (frame_w, center_y), GUIDE_COLOR, GUIDE_THICKNESS)
    cv2.line(frame, (center_x, 0), (center_x, frame_h), GUIDE_COLOR, GUIDE_THICKNESS)
    cv2.line(frame, (0, GUIDE_TOP_LINE_Y), (frame_w, GUIDE_TOP_LINE_Y), GUIDE_COLOR, GUIDE_THICKNESS)
    if ignore_below_y is not None:
        limit_y = max(0, min(int(ignore_below_y), frame_h - 1))
        cv2.line(frame, (0, limit_y), (frame_w, limit_y), DETECTION_LIMIT_COLOR, 2)


def build_boundary_result(x1, y1, x2, y2, contrast, length, frame_w, method):
    angle = np.degrees(np.arctan2(y2 - y1, x2 - x1))
    if x2 != x1:
        y_at_center = y1 + (frame_w / 2 - x1) * (y2 - y1) / (x2 - x1)
    else:
        y_at_center = (y1 + y2) / 2

    confidence = min(1.0, (length / frame_w) * (contrast / 70.0))
    return {
        "line": (int(x1), int(y1), int(x2), int(y2)),
        "angle_deg": round(float(angle), 2),
        "y_at_center": round(float(y_at_center), 1),
        "contrast": round(float(contrast), 1),
        "confidence": round(float(confidence), 2),
        "method": method,
    }


def detect_boundary_by_row_scan(gray, roi_top, frame_w):
    window = BOUNDARY_SCAN_WINDOW
    x_margin = int(frame_w * BOUNDARY_SCAN_X_MARGIN_RATIO)
    scan = gray[:, x_margin:frame_w - x_margin]
    if scan.size == 0 or gray.shape[0] <= window * 3:
        return None

    profile = np.median(scan, axis=1).astype(np.float32)
    profile = cv2.GaussianBlur(profile.reshape(-1, 1), (1, 9), 0).reshape(-1)

    best = None
    best_score = 0.0
    for y in range(window * 2, len(profile) - window * 2):
        above = float(np.mean(profile[y - window * 2:y - window]))
        below = float(np.mean(profile[y + window:y + window * 2]))
        contrast = below - above
        if (
            above > BOUNDARY_DARK_VALUE_MAX
            or below < BOUNDARY_FLOOR_VALUE_MIN
            or contrast < BOUNDARY_CONTRAST_MIN
        ):
            continue

        local_stability = 1.0 / (1.0 + abs(profile[y] - (above + below) / 2.0) / 40.0)
        score = contrast * local_stability
        if score > best_score:
            best_score = score
            best = (y, contrast)

    if best is None:
        return None

    y, contrast = best
    global_y = y + roi_top
    length = frame_w - x_margin * 2
    return build_boundary_result(
        x_margin,
        global_y,
        frame_w - x_margin,
        global_y,
        contrast,
        length,
        frame_w,
        "row_scan",
    )


def detect_wall_floor_boundary(frame):
    """
    正面の黒い壁と床の境界線を推定する。

    戻り値は画像座標の line=(x1, y1, x2, y2)、傾き angle_deg、
    中央付近の y 座標 y_at_center、confidence を含む dict。見つからない場合は None。
    """
    frame_h, frame_w = frame.shape[:2]
    roi_top = int(frame_h * BOUNDARY_ROI_TOP_RATIO)
    roi_bottom = int(frame_h * BOUNDARY_ROI_BOTTOM_RATIO)
    if roi_bottom <= roi_top:
        return None

    roi = frame[roi_top:roi_bottom, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # 黒い壁と床の明るさ差を主な手がかりにする。柱の赤/緑には依存しない。
    edges = cv2.Canny(gray, 40, 120)
    min_line_length = int(frame_w * BOUNDARY_MIN_LINE_LENGTH_RATIO)
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        threshold=40,
        minLineLength=min_line_length,
        maxLineGap=BOUNDARY_MAX_LINE_GAP,
    )

    best = None
    best_score = 0.0
    if lines is not None:
        for line in lines[:, 0, :]:
            x1, y1, x2, y2 = [int(v) for v in line]
            dx = x2 - x1
            dy = y2 - y1
            length = float(np.hypot(dx, dy))
            if length < min_line_length:
                continue

            angle = np.degrees(np.arctan2(dy, dx))
            if abs(angle) > BOUNDARY_MAX_ANGLE_DEG:
                continue

            y_mid = int((y1 + y2) / 2)
            band = 8
            above_top = max(0, y_mid - band * 3)
            above_bottom = max(0, y_mid - band)
            below_top = min(gray.shape[0], y_mid + band)
            below_bottom = min(gray.shape[0], y_mid + band * 3)
            if above_bottom <= above_top or below_bottom <= below_top:
                continue

            above_value = float(np.mean(gray[above_top:above_bottom, :]))
            below_value = float(np.mean(gray[below_top:below_bottom, :]))
            contrast = below_value - above_value
            if (
                above_value > BOUNDARY_DARK_VALUE_MAX
                or below_value < BOUNDARY_FLOOR_VALUE_MIN
                or contrast < BOUNDARY_CONTRAST_MIN
            ):
                continue

            score = length * contrast * (1.0 - abs(angle) / BOUNDARY_MAX_ANGLE_DEG)
            if score > best_score:
                best_score = score
                best = (x1, y1 + roi_top, x2, y2 + roi_top, contrast, length)

    if best is not None:
        x1, y1, x2, y2, contrast, length = best
        return build_boundary_result(x1, y1, x2, y2, contrast, length, frame_w, "hough")

    return detect_boundary_by_row_scan(gray, roi_top, frame_w)


def draw_boundary(frame, boundary):
    if boundary is None:
        return

    x1, y1, x2, y2 = boundary["line"]
    cv2.line(frame, (x1, y1), (x2, y2), BOUNDARY_COLOR, 3)
    cv2.putText(
        frame,
        f"BOUNDARY y={boundary['y_at_center']} angle={boundary['angle_deg']} conf={boundary['confidence']}",
        (10, min(max(int(boundary["y_at_center"]) - 12, 24), FRAME_SIZE[1] - 12)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        BOUNDARY_COLOR,
        2,
        cv2.LINE_AA,
    )


def draw_fps(frame, fps):
    cv2.putText(
        frame,
        f"FPS: {fps:.1f}",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        FPS_COLOR,
        2,
        cv2.LINE_AA,
    )


def format_detection(color_name, detections):
    if not detections:
        return f"{color_name}: not found"

    obj = detections[0]
    cx, cy = obj["center"]
    w, h = obj["size"]
    x, y, _, _ = obj["bbox"]
    return f"{color_name}: center=({cx},{cy}) left_top=({x},{y}) width={w} height={h}"


def format_boundary(boundary):
    if boundary is None:
        return "boundary: not found"

    return (
        "boundary: "
        f"y={boundary['y_at_center']} "
        f"angle={boundary['angle_deg']} "
        f"confidence={boundary['confidence']} "
        f"method={boundary['method']}"
    )


def choose_primary_detection(red_objects, green_objects):
    candidates = []
    if red_objects:
        candidates.append(("red", red_objects[0]))
    if green_objects:
        candidates.append(("green", green_objects[0]))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[1]["area"])


class PiColorDetector:
    def __init__(
        self,
        sample_dir=DEFAULT_SAMPLE_DIR,
        preview_port=PREVIEW_PORT,
        enable_preview=False,
        enable_recording=False,
        detect_objects_enabled=True,
        detect_boundary_enabled=True,
        recording_path=None,
        recording_fps=RECORDING_FPS,
    ):
        self.sample_dir = Path(sample_dir)
        self.preview_port = preview_port
        self.enable_preview = enable_preview
        self.enable_recording = enable_recording
        self.detect_objects_enabled = detect_objects_enabled
        self.detect_boundary_enabled = detect_boundary_enabled
        self.recording_path = Path(recording_path) if recording_path is not None else None
        self.recording_fps = recording_fps
        self.camera = None
        self.preview = None
        self.models = {}
        self._recording_writer = None
        self._capture_thread = None
        self._capture_running = False
        self._result_condition = Condition()
        self._latest_result = None
        self._latest_result_id = 0
        self._last_seen_result_id = 0
        self._last_fps_time = None
        self._fps = 0.0
        # 画面の下1/3は物体検出の対象外にする。
        self.ignore_below_y = DEFAULT_IGNORE_BELOW_Y

    def load_models(self):
        red_samples = load_hsv_samples(self.sample_dir, "red")
        green_samples = load_hsv_samples(self.sample_dir, "green")
        other_samples = load_hsv_samples(self.sample_dir, "other")

        self.models = {
            "red": build_model(red_samples),
            "green": build_model(green_samples),
            "other": build_model(other_samples),
        }

    def start(self):
        self.load_models()
        try:
            self.camera = Picamera2()
        except IndexError as exc:
            raise RuntimeError(
                "Pi Camera が見つかりません。カメラの接続、CSI ケーブルの向き、"
                "カメラ有効化、再起動を確認してください。"
            ) from exc
        self.camera.configure(
            self.camera.create_preview_configuration(
                main={"size": FRAME_SIZE, "format": "BGR888"},
                raw={"size": RAW_SENSOR_SIZE},
            )
        )
        self.camera.start()

        if self.enable_preview:
            self.preview = PreviewServer(port=self.preview_port, title="Pi Detect")
            self.preview.start()

        if self.enable_recording:
            self._start_recording()

    def stop(self):
        self._stop_recording()
        if self.preview is not None:
            self.preview.stop()
            self.preview = None
        if self.camera is not None:
            self.camera.stop()
            self.camera = None

    def _build_recording_path(self):
        if self.recording_path is not None:
            return self.recording_path

        DEFAULT_RECORDING_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return DEFAULT_RECORDING_DIR / f"robot_{timestamp}.mp4"

    def _start_recording(self):
        self.recording_path = self._build_recording_path()
        self.recording_path.parent.mkdir(parents=True, exist_ok=True)

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self._recording_writer = cv2.VideoWriter(
            str(self.recording_path),
            fourcc,
            self.recording_fps,
            FRAME_SIZE,
        )
        if not self._recording_writer.isOpened():
            self._recording_writer = None
            raise RuntimeError(f"録画ファイルを開けませんでした: {self.recording_path}")

        self._capture_running = True
        self._capture_thread = Thread(target=self._record_loop, daemon=True)
        self._capture_thread.start()

    def _stop_recording(self):
        self._capture_running = False
        if self._capture_thread is not None:
            self._capture_thread.join(timeout=2.0)
            self._capture_thread = None
        if self._recording_writer is not None:
            self._recording_writer.release()
            self._recording_writer = None

    def set_ignore_below_y(self, y):
        self.ignore_below_y = y

    def _record_loop(self):
        frame_interval = 1.0 / self.recording_fps if self.recording_fps > 0 else 0
        while self._capture_running:
            started_at = time.time()
            try:
                result = self._capture_and_detect()
            except RuntimeError:
                break

            if self._recording_writer is not None:
                self._recording_writer.write(result["annotated_frame"])

            with self._result_condition:
                self._latest_result = result
                self._latest_result_id += 1
                self._result_condition.notify_all()

            elapsed = time.time() - started_at
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)

    def _capture_and_detect(self):
        if self.camera is None:
            raise RuntimeError("PiColorDetector.start() を先に呼んでください。")

        captured_at = time.time()
        if self._last_fps_time is not None:
            elapsed = captured_at - self._last_fps_time
            if elapsed > 0:
                current_fps = 1.0 / elapsed
                if self._fps <= 0:
                    self._fps = current_fps
                else:
                    self._fps = self._fps * 0.8 + current_fps * 0.2
        self._last_fps_time = captured_at

        frame = self.camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        if self.detect_objects_enabled:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            red_mask = create_mask(hsv, "red", self.models["red"], [self.models["green"], self.models["other"]])
            green_mask = create_mask(hsv, "green", self.models["green"], [self.models["red"], self.models["other"]])
            red_mask[:GUIDE_TOP_LINE_Y, :] = 0
            green_mask[:GUIDE_TOP_LINE_Y, :] = 0
            # if self.ignore_below_y is not None:
            #     limit_y = max(0, min(int(self.ignore_below_y), frame.shape[0]))
            #     red_mask[limit_y:, :] = 0
            #     green_mask[limit_y:, :] = 0
            red_objects = find_objects(red_mask)
            green_objects = find_objects(green_mask)
        else:
            red_objects = []
            green_objects = []
        boundary = detect_wall_floor_boundary(frame) if self.detect_boundary_enabled else None

        if self.enable_preview or self.enable_recording:
            annotated_frame = frame.copy()
            draw_fps(annotated_frame, self._fps)
            draw_guide_boxes(annotated_frame, self.ignore_below_y)
            draw_boundary(annotated_frame, boundary)
            draw_objects(annotated_frame, "RED", red_objects, (0, 0, 255))
            draw_objects(annotated_frame, "GREEN", green_objects, (0, 200, 0))
        else:
            annotated_frame = frame

        red_status = format_detection("RED", red_objects) if self.detect_objects_enabled else "RED: disabled"
        green_status = format_detection("GREEN", green_objects) if self.detect_objects_enabled else "GREEN: disabled"
        boundary_status = format_boundary(boundary) if self.detect_boundary_enabled else "boundary: disabled"
        primary = choose_primary_detection(red_objects, green_objects)

        return {
            "frame": frame,
            "annotated_frame": annotated_frame,
            "red_objects": red_objects,
            "green_objects": green_objects,
            "boundary": boundary,
            "red_status": red_status,
            "green_status": green_status,
            "boundary_status": boundary_status,
            "primary_detection": primary,
            "fps": self._fps,
        }

    def process_once(self):
        if not self.enable_recording:
            return self._capture_and_detect()

        with self._result_condition:
            self._result_condition.wait_for(
                lambda: self._latest_result_id != self._last_seen_result_id,
                timeout=2.0,
            )
            if self._latest_result is None:
                raise RuntimeError("カメラ映像を取得できませんでした。")

            self._last_seen_result_id = self._latest_result_id
            return self._latest_result

    def update_preview(self, result, extra_lines=None):
        if self.preview is None:
            return []

        lines = [
            "ブラウザから映像を確認できます",
            f"FPS: {result.get('fps', 0.0):.1f}",
            result["red_status"],
            result["green_status"],
            result["boundary_status"],
        ]
        if extra_lines:
            lines.extend(extra_lines)
        lines.append("終了するときは Quit ボタンか Ctrl+C を使ってください")
        self.preview.set_status(*lines)
        self.preview.publish_frame(result["annotated_frame"])
        return self.preview.pop_actions()
