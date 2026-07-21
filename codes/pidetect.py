"""
カメラ検知だけを確認するための実行ファイル。

実際のメイン制御は main_program.py から camera_detector を読み込んで使う。
"""

import argparse

import numpy as np

from camera_detector import FRAME_SIZE, PiColorDetector


FAR_OBJECT_AREA_MAX = 3500


def classify_obj_position(obj):
    """
    検出したオブジェクトを6箇所のどれかに分類する。
    """
    frame_width, frame_height = FRAME_SIZE
    center_x, center_y = obj["center"]
    area = obj["area"]

    side = "右" if center_x >= frame_width / 2 else "左"

    if center_y >= frame_height / 2:
        depth = "手前"
    elif center_y <= frame_height * 3 / 4 and area <= FAR_OBJECT_AREA_MAX:
        depth = "奥"
    else:
        depth = "真ん中"

    return side + depth


def build_position_lines(result, max_objects=2):
    candidates = []
    for color_name, objects in (
        ("RED", result["red_objects"]),
        ("GREEN", result["green_objects"]),
    ):
        for color_index, obj in enumerate(objects, start=1):
            candidates.append((color_name, color_index, obj))

    candidates.sort(key=lambda item: item[2]["area"], reverse=True)
    if not candidates:
        return ["位置: not found"]

    lines = []
    for color_name, color_index, obj in candidates[:max_objects]:
        lines.append(
            f"位置: {color_name}{color_index} {classify_obj_position(obj)} "
            f"center={obj['center']} area={obj['area']}"
        )
    return lines


def format_console_detection(name, objects):
    if not objects:
        return f"{name}: not found"

    obj = objects[0]
    return (
        f"{name}: center={obj['center']} "
        f"left_top=({obj['bbox'][0]},{obj['bbox'][1]}) "
        f"width={obj['size'][0]} height={obj['size'][1]}"
    )


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--wall-only", action="store_true", help="色検出を止めて境界線だけ検出する")
    mode.add_argument("--objects-only", action="store_true", help="境界線検出を止めて色だけ検出する")
    args = parser.parse_args()

    detector = PiColorDetector(
        enable_preview=True,
        detect_objects_enabled=not args.wall_only,
        detect_boundary_enabled=not args.objects_only,
    )

    try:
        detector.start()
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(exc)
        print("サンプル不足なら FE/samples を作り直してください。")
        print("例: python3 /home/kanata/workspace/FE/collect_samples.py")
        return

    print(f"HSV sample dir: {detector.sample_dir}")
    print(f"RED mean={np.round(detector.models['red']['mean'], 1)} std={np.round(detector.models['red']['std'], 1)}")
    print(f"GREEN mean={np.round(detector.models['green']['mean'], 1)} std={np.round(detector.models['green']['std'], 1)}")
    print(f"OTHER mean={np.round(detector.models['other']['mean'], 1)} std={np.round(detector.models['other']['std'], 1)}")
    print(f"色検出: {'disabled' if args.wall_only else 'enabled'}")
    print(f"境界線検出: {'disabled' if args.objects_only else 'enabled'}")
    print(f"ブラウザ表示: http://localhost:{detector.preview_port}")
    print("VS Code の Port Forwarding で 8000 番を Mac 側に転送して開いてください。")

    try:
        while True:
            result = detector.process_once()
            position_lines = build_position_lines(result)
            status = " | ".join(
                (
                    format_console_detection("RED", result["red_objects"]),
                    format_console_detection("GREEN", result["green_objects"]),
                    " / ".join(position_lines),
                    result["boundary_status"],
                )
            )

            print(status, end="   \r")
            actions = detector.update_preview(result, extra_lines=position_lines)
            if "quit" in actions:
                break
    finally:
        detector.stop()
        print("\n終了しました。")


if __name__ == "__main__":
    main()
