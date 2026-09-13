"""
カメラ検知だけを確認するための実行ファイル。

実際のメイン制御は main_program.py から camera_detector を読み込んで使う。
"""

import cv2

from camera_detector import BOTTOM_EXCLUSION_SIZE, FRAME_SIZE, PiColorDetector, object_front_priority


FAR_OBJECT_AREA_MAX = 3500
# False: 赤・緑・マゼンタの検知枠、白コート、青ラインだけを描画。
# True: オブジェクトの座標・サイズ、補助線、FPS、除外範囲の枠なども描画。
SHOW_DEBUG_OVERLAYS = True
# True: コート内の赤・緑・マゼンタだけを採用。False: コートによる絞り込みをしない。
# Falseではコートの検出・表示も無効にする。
FILTER_OBJECTS_ON_COURT = True

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
        ("MAGENTA", result["magenta_objects"]),
    ):
        for color_index, obj in enumerate(objects, start=1):
            candidates.append((color_name, color_index, obj))

    candidates.sort(
        key=lambda item: object_front_priority(item[2]),
        reverse=True,
    )
    if not candidates:
        return ["位置: not found"]

    lines = []
    for color_name, color_index, obj in candidates[:max_objects]:
        lines.append(
            f"位置: {color_name}{color_index} {classify_obj_position(obj)} "
            f"center={obj['center']} area={obj['area']}"
        )
    return lines


def main():
    detector = PiColorDetector(
        enable_preview=True,
        detect_objects_enabled=True,
        detect_boundary_enabled=False,
        detect_court_enabled=FILTER_OBJECTS_ON_COURT,
        bottom_exclusion_size=BOTTOM_EXCLUSION_SIZE,
        show_debug_overlays=SHOW_DEBUG_OVERLAYS,
        filter_objects_on_court_enabled=FILTER_OBJECTS_ON_COURT,
    )

    try:
        detector.start()
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(exc)
        print("サンプル不足なら FE/samples を作り直してください。")
        print("例: python3 /home/kanata/workspace/FE/collect_samples.py")
        return

    print(f"プレビュー: http://<Raspberry PiのIP>:{detector.preview_port}")

    try:
        while True:
            result = detector.process_once()
            line_angle_deg = result["line_angle_deg"]
            status = (
                f"{line_angle_deg:.1f} deg"
                if line_angle_deg is not None
                else "not found"
            )
            position_lines = build_position_lines(result)

            print(status, end="   \r")
            # 検出結果の画像を変更せず、プレビュー用のコピーに描画する。
            preview_frame = result["annotated_frame"].copy()
            height, width = preview_frame.shape[:2]
            rectangle_width = min(BOTTOM_EXCLUSION_SIZE[0], width)
            rectangle_height = min(BOTTOM_EXCLUSION_SIZE[1], height)
            left = (width - rectangle_width) // 2
            top = height - rectangle_height
            if SHOW_DEBUG_OVERLAYS:
                cv2.rectangle(
                    preview_frame,
                    (left, top),
                    (left + rectangle_width - 1, height - 1),
                    (0, 0, 0),
                    2,
                )
            preview_result = {**result, "annotated_frame": preview_frame}
            actions = detector.update_preview(preview_result, extra_lines=position_lines)
            if "quit" in actions:
                break
    finally:
        detector.stop()
        print()


if __name__ == "__main__":
    main()
