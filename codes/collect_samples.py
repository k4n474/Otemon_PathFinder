"""
Pi Camera2 で赤・緑の HSV サンプルを集めて pidetect/samples に保存する。

操作:
- `1`: 赤モード
- `2`: 緑モード
- `3`: その他モード
- `space`: 画面中央のサンプルを追加
- `c`: 現在の色のサンプルを消去
- `q`: 保存して終了
"""

from pathlib import Path

import cv2
import numpy as np
from picamera2 import Picamera2

from preview_server import PreviewServer


BASE_DIR = Path(__file__).resolve().parent
SAVE_DIR = BASE_DIR / "samples"
SAVE_DIR.mkdir(exist_ok=True)

SAMPLE_FILES = {
    "red": SAVE_DIR / "red.npy",
    "green": SAVE_DIR / "green.npy",
    "other": SAVE_DIR / "other.npy",
}

FRAME_SIZE = (640, 480)
BOX_HALF_SIZE = 10
PREVIEW_PORT = 8001


def load_existing_samples():
    samples = {}
    for color, path in SAMPLE_FILES.items():
        if path.exists():
            samples[color] = list(np.load(path))
            print(f"既存サンプル読み込み: {color} {len(samples[color])}件")
        else:
            samples[color] = []
    return samples


def get_mode_color(current_color):
    if current_color == "red":
        return (0, 0, 220)
    if current_color == "green":
        return (0, 180, 0)
    return (180, 180, 180)


def main():
    print("新しいカメラ用のHSVサンプル収集を開始します。")
    print(f"保存先: {SAVE_DIR}")

    samples = load_existing_samples()
    current_color = "red"

    camera = Picamera2()
    camera.configure(
        camera.create_preview_configuration(
            main={"size": FRAME_SIZE, "format": "BGR888"}
        )
    )
    camera.start()
    preview = PreviewServer(port=PREVIEW_PORT, title="Sample Collector")
    preview.start()

    print(f"ブラウザ表示: http://localhost:{PREVIEW_PORT}")
    print("VS Code の Port Forwarding で 8001 番を Mac 側に転送して開いてください。")
    print("ブラウザのボタンからモード切替・保存・終了ができます。")

    try:
        while True:
            frame = camera.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            h, w = frame.shape[:2]
            cx, cy = w // 2, h // 2
            region_hsv = hsv[
                cy - BOX_HALF_SIZE:cy + BOX_HALF_SIZE,
                cx - BOX_HALF_SIZE:cx + BOX_HALF_SIZE,
            ]
            avg_hsv = np.mean(region_hsv, axis=(0, 1)).astype(int)
            box_color = get_mode_color(current_color)

            cv2.rectangle(
                frame,
                (cx - BOX_HALF_SIZE, cy - BOX_HALF_SIZE),
                (cx + BOX_HALF_SIZE, cy + BOX_HALF_SIZE),
                box_color,
                2,
            )
            cv2.line(frame, (cx - 10, cy), (cx + 10, cy), (255, 255, 255), 1)
            cv2.line(frame, (cx, cy - 10), (cx, cy + 10), (255, 255, 255), 1)

            cv2.putText(
                frame,
                f"H:{avg_hsv[0]} S:{avg_hsv[1]} V:{avg_hsv[2]}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"mode:{current_color} [browser buttons]",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                box_color,
                2,
            )
            cv2.putText(
                frame,
                f"red:{len(samples['red'])} green:{len(samples['green'])} other:{len(samples['other'])}",
                (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (220, 220, 220),
                1,
            )

            preview.set_status(
                f"mode: {current_color}",
                f"HSV center: {avg_hsv.tolist()}",
                f"red={len(samples['red'])} green={len(samples['green'])} other={len(samples['other'])}",
                "ブラウザのボタンで操作してください",
            )
            preview.publish_frame(frame)

            for action in preview.pop_actions():
                if action == "quit":
                    return
                if action == "red":
                    current_color = "red"
                    print("赤モードに切り替え")
                elif action == "green":
                    current_color = "green"
                    print("緑モードに切り替え")
                elif action == "other":
                    current_color = "other"
                    print("その他モードに切り替え")
                elif action == "clear":
                    samples[current_color] = []
                    print(f"{current_color} のサンプルを消去しました")
                elif action == "save":
                    pixels = region_hsv.reshape(-1, 3)
                    samples[current_color].extend(pixels.tolist())
                    print(f"[{current_color}] サンプル追加 -> 合計{len(samples[current_color])}件")
    finally:
        for color, path in SAMPLE_FILES.items():
            if samples[color]:
                np.save(path, np.array(samples[color]))
                print(f"保存完了: {path} ({len(samples[color])}件)")
            elif path.exists():
                path.unlink()
                print(f"空になったため削除: {path}")

        preview.stop()
        camera.stop()
        print("終了しました。")


if __name__ == "__main__":
    main()
