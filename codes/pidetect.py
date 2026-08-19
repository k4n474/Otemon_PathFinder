"""
カメラ検知だけを確認するための実行ファイル。

実際のメイン制御は main_program.py から camera_detector を読み込んで使う。
"""

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
        ("MAGENTA", result["magenta_objects"]),
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


def main():
    detector = PiColorDetector(
        enable_preview=True,
        detect_objects_enabled=True,
        detect_boundary_enabled=False,
    )

    try:
        detector.start()
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(exc)
        print("サンプル不足なら FE/samples を作り直してください。")
        print("例: python3 /home/kanata/workspace/FE/collect_samples.py")
        return

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
            actions = detector.update_preview(result, extra_lines=position_lines)
            if "quit" in actions:
                break
    finally:
        detector.stop()
        print()


if __name__ == "__main__":
    main()
