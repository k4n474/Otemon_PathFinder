"""カメラ中央の画素のHSV値を継続表示する確認プログラム。"""

import cv2

from camera_detector import PiColorDetector


CENTER_POINT_OUTER_COLOR = (0, 0, 0)
CENTER_POINT_INNER_COLOR = (255, 255, 255)
CENTER_POINT_OUTER_RADIUS = 7
CENTER_POINT_INNER_RADIUS = 3
HSV_TEXT_POSITION = (10, 125)
HSV_TEXT_COLOR = (255, 255, 255)


def read_center_hsv(frame):
    """フレーム中央の座標と、その画素のOpenCV形式HSV値を返す。"""
    frame_height, frame_width = frame.shape[:2]
    center = (frame_width // 2, frame_height // 2)
    center_x, center_y = center
    center_bgr = frame[center_y, center_x].reshape(1, 1, 3)
    hue, saturation, value = cv2.cvtColor(
        center_bgr,
        cv2.COLOR_BGR2HSV,
    )[0, 0]
    return center, (int(hue), int(saturation), int(value))


def draw_center_hsv(frame, center, hsv):
    """中央点とHSV値をフレームへ描画する。"""
    hue, saturation, value = hsv
    cv2.circle(
        frame,
        center,
        CENTER_POINT_OUTER_RADIUS,
        CENTER_POINT_OUTER_COLOR,
        -1,
        cv2.LINE_AA,
    )
    cv2.circle(
        frame,
        center,
        CENTER_POINT_INNER_RADIUS,
        CENTER_POINT_INNER_COLOR,
        -1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"CENTER HSV: H={hue} S={saturation} V={value}",
        HSV_TEXT_POSITION,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (0, 0, 0),
        4,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        f"CENTER HSV: H={hue} S={saturation} V={value}",
        HSV_TEXT_POSITION,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        HSV_TEXT_COLOR,
        1,
        cv2.LINE_AA,
    )


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
        return

    try:
        while True:
            result = detector.process_once()
            center, hsv = read_center_hsv(result["frame"])
            annotated_frame = result["annotated_frame"].copy()
            draw_center_hsv(annotated_frame, center, hsv)

            preview_result = dict(result)
            preview_result["annotated_frame"] = annotated_frame
            hue, saturation, value = hsv
            status = (
                f"CENTER {center}: "
                f"H={hue} S={saturation} V={value}"
            )
            print(status, end="   \r", flush=True)

            actions = detector.update_preview(
                preview_result,
                extra_lines=[status],
            )
            if "quit" in actions:
                break
    except KeyboardInterrupt:
        pass
    finally:
        detector.stop()
        print()


if __name__ == "__main__":
    main()
