# from robot import robot
from camera_detector import FRAME_SIZE, PiColorDetector
import time
# from gyro import get_angle, reset_angle, close_gyro
# from ultrasound import us_get, dis_get, us_back_get, dis_back_get
from time import sleep

from buzzer import buzzer_start, buzzer_stop, buzzer_sleep, hurt_beats


TIMEOUT_SECONDS = 30
FAR_OBJECT_AREA_MAX = 3500
detector = PiColorDetector(enable_recording=True, detect_boundary_enabled=False)
from newobot import dc_motor, set_angle, stop, cleanup


def set_boundary_detection(enabled):
    detector.detect_boundary_enabled = enabled


def set_object_detection(enabled):
    detector.detect_objects_enabled = enabled

class PIDController:
    def __init__(self, kp=0.1, ki=0.01, kd=0.05, integral_limit=None):
        """
        PID制御器
        
        Args:
            kp: 比例ゲイン
            ki: 積分ゲイン
            kd: 微分ゲイン
            integral_limit: 積分値の上限。Noneなら制限なし
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error, dt=None):
        """
        エラーに基づいて制御出力を計算
        
        Args:
            error: 目標値との誤差
            dt: 前回更新からの経過時間。Noneなら従来通り1ステップとして扱う
            
        Returns:
            float: 制御出力
        """
        if dt is None:
            self.integral += error
            derivative = error - self.prev_error
        else:
            dt = max(dt, 0.001)
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt

        if self.integral_limit is not None:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))

        self.prev_error = error
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output
    
    def reset(self):
        """PIDコントローラーをリセット"""
        self.prev_error = 0
        self.integral = 0

# def detect_obj(item="color"):
#     """
#     今の画面で一番大きく見えているオブジェクトを1回だけ検知して返す。

#     Args:
#         item: 返したい情報。
#               "color", "width", "height", "center", "area", "bbox", "object", "all"

#     Returns:
#         見つからない場合は None
#     """
#     result = detector.process_once()
#     primary = result["primary_detection"]

#     if primary is None:
#         return None

#     color_name, obj = primary

#     if item == "color":
#         return color_name
#     if item == "width":
#         return obj["size"][0]
#     if item == "height":
#         return obj["size"][1]
#     if item == "center":
#         return obj["center"]
#     if item == "area":
#         return obj["area"]
#     if item == "bbox":
#         return obj["bbox"]
#     if item == "object":
#         return obj
#     if item == "all":
#         return color_name, obj

#     raise ValueError('item must be "color", "width", "height", "center", "area", "bbox", "object", or "all"')

# def classify_obj_position(obj):
#     """
#     検出したオブジェクトを6箇所のどれかに分類する。

#     戻り値:
#         "右手前", "左手前", "右真ん中", "左真ん中", "右奥", "左奥"
#     """
#     frame_width, frame_height = FRAME_SIZE
#     center_x, center_y = obj["center"]
#     area = obj["area"]

#     side = "右" if center_x >= frame_width / 2 else "左"

#     if center_y >= frame_height / 2:
#         depth = "手前"
#     elif center_y <= frame_height * 3 / 4 and area <= FAR_OBJECT_AREA_MAX:
#         depth = "奥"
#     else:
#         depth = "真ん中"

#     return side + depth

# def detect_obj_positions(max_objects=2):
#     """
#     今の画面で見えている赤/緑オブジェクトの位置を最大 max_objects 個返す。

#     Returns:
#         [
#             {
#                 "color": "red" or "green",
#                 "position": "右手前" など,
#                 "center": (x, y),
#                 "area": 面積,
#                 "size": (w, h),
#                 "object": 検出情報全体,
#             },
#             ...
#         ]
#         見つからない場合は [] を返す。
#     """
#     result = detector.process_once()
#     candidates = []
#     for color_name, objects in (
#         ("red", result["red_objects"]),
#         ("green", result["green_objects"]),
#     ):
#         for color_index, obj in enumerate(objects, start=1):
#             candidates.append((color_name, color_index, obj))

#     candidates.sort(key=lambda item: item[2]["area"], reverse=True)
#     positions = []
#     for color_name, color_index, obj in candidates[:max_objects]:
#         positions.append(
#             {
#                 "color": color_name,
#                 "color_index": color_index,
#                 "position": classify_obj_position(obj),
#                 "center": obj["center"],
#                 "area": obj["area"],
#                 "size": obj["size"],
#                 "object": obj,
#             }
#         )

#     return positions

# def position_to_number(position):
#     """
#     位置の文字を試合用の番号に変換する。

#     0: なし
#     1: 左手前
#     2: 右手前
#     3: 左真ん中
#     4: 右真ん中
#     5: 左奥
#     6: 右奥
#     """
#     position_numbers = {
#         "左手前": 1,
#         "右手前": 2,
#         "左真ん中": 3,
#         "右真ん中": 4,
#         "左奥": 5,
#         "右奥": 6,
#     }
#     return position_numbers.get(position, 0)

# def detect_obj_position_numbers():
#     """
#     赤2個、緑2個の位置を番号で返す。

#     Returns:
#         tuple[int, int, int, int]:
#             (赤1の位置, 赤2の位置, 緑1の位置, 緑2の位置)

#     位置番号:
#         0: なし
#         1: 左手前
#         2: 右手前
#         3: 左真ん中
#         4: 右真ん中
#         5: 左奥
#         6: 右奥

#     例:
#         右手前に緑、左奥に緑がある場合は (0, 0, 2, 5)
#     """
#     result = detector.process_once()
#     red_positions = [0, 0]
#     green_positions = [0, 0]

#     for index, obj in enumerate(result["red_objects"][:2]):
#         red_positions[index] = position_to_number(classify_obj_position(obj))

#     for index, obj in enumerate(result["green_objects"][:2]):
#         green_positions[index] = position_to_number(classify_obj_position(obj))

#     return (
#         red_positions[0],
#         red_positions[1],
#         green_positions[0],
#         green_positions[1],
#     )


def select_front_object(result):
    """検出結果から、画面の一番下に映っている物体を選ぶ。"""
    selected_color = None
    selected_object = None

    candidates = []
    for red_object in result["red_objects"]:
        candidates.append(("red", red_object))

    for green_object in result["green_objects"]:
        candidates.append(("green", green_object))

    for color_name, obj in candidates:
        if selected_object is None:
            selected_color = color_name
            selected_object = obj
            continue

        object_y = obj["center"][1]
        selected_y = selected_object["center"][1]

        is_lower = object_y > selected_y
        is_same_height = object_y == selected_y
        is_larger = obj["area"] > selected_object["area"]

        if is_lower or (is_same_height and is_larger):
            selected_color = color_name
            selected_object = obj

    return selected_color, selected_object

def avoid_obj(duty_cycle, straight_seconds=0.6, back_seconds=1.5):
    """
    一番手前に見えているオブジェクトに合わせてPID制御しながら前進する。

    赤なら中心Xが左から80px、緑なら中心Xが右から80pxになるように曲がる。
    オブジェクトの縦幅が130px以上になった時に端から90px以上離れていたら、
    back_seconds 秒だけバックしてPID制御をやり直す。
    十分端に寄っていたらステアリングを戻し、straight_seconds 秒だけ直進する。
    """
    frame_width = FRAME_SIZE[0]
    red_target_x = 30
    green_target_x = frame_width - 30
    max_angle = 50
    kp = 0.1
    ki = 0
    kd = 0.01
    pid = PIDController(kp=kp, ki=ki, kd=kd, integral_limit=300)
    last_time = time.monotonic()
    ab = 0

    try:
        set_angle(0)
        dc_motor(duty_cycle)
        while True:
            result = detector.process_once()
            color_name, obj = select_front_object(result)

            if obj is None:
                # オブジェクトが映らなくなったため停止
                stop()
                set_angle(0)
                return None

            center_x, center_y = obj["center"]
            current_height = obj["size"][1]

            if current_height >= 90:
                if color_name == "red":
                    too_far_from_edge = center_x > 135
                else:
                    too_far_from_edge = center_x < frame_width - 135

                if too_far_from_edge:
                    # バックする
                    ab = 1
                    set_angle(0)
                    dc_motor(-70)
                    time.sleep(back_seconds)
                        

                    pid.reset()
                    last_time = time.monotonic()
                    set_angle(0)
                    dc_motor(duty_cycle)
                    continue

                # 通り過ぎる
                set_angle(0)
                dc_motor(duty_cycle)
                time.sleep(back_seconds)

                # 関数終了
                stop()
                set_angle(0)
                return color_name

            if color_name == "red":
                if ab == 0:
                    target_x = red_target_x  # 35.8+6800/(current_height+1.2)
                else:
                    target_x = red_target_x
            else:
                if ab == 0:
                    target_x = green_target_x  # 480-(35.8+6800/(current_height+1.2))
                else:
                    target_x = green_target_x

            error = center_x - target_x

            now = time.monotonic()
            dt = now - last_time
            last_time = now

            servo_angle = pid.update(error, dt)
            if servo_angle > max_angle:
                servo_angle = max_angle
            elif servo_angle < -max_angle:
                servo_angle = -max_angle

            set_angle(servo_angle)
            dc_motor(duty_cycle)


    except KeyboardInterrupt:
        print("\n中断されました")
        stop()
        set_angle(0)
        raise

def find_obj(duty_cycle, rd):
    """
    オブジェクトが画面に映るまで、ステアリングを右に切って前進し続ける。

    Returns:
        str | None: 見つけた色名。中断されたら None
    """
    steering_angle=40
    try:
        if rd == 0:
            set_angle(steering_angle)
        else:
            set_angle(steering_angle * -1)
        dc_motor(duty_cycle)

        while True:
            result = detector.process_once()
            primary = result["primary_detection"]

            if primary is not None:
                color_name, obj = primary
                center_x, center_y = obj["center"]
                width, height = obj["size"]
                stop()
                set_angle(0)
                return color_name

    except KeyboardInterrupt:
        print("\n中断されました")
        stop()
        set_angle(0)
        raise


def main():
    print("robot init")
    try:
        detector.start()
        if detector.recording_path is not None:
            print(f"録画開始: {detector.recording_path}")


        # while True:
            # find_obj(70, 1)
            # avoid_obj(70)
        
        
        
        
       


        

        
            


            







    

    except KeyboardInterrupt:
        print("\nCtrl+C を受け付けたため終了します")
        stop()
        set_angle(0)
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(f"エラー: {exc}")
    finally:
        detector.stop()
        if detector.recording_path is not None:
            print(f"録画保存: scp 10.178.179.239:{detector.recording_path} ~/workspace/pivideos")
        cleanup()
        print("cleanup")


if __name__ == "__main__":
    main()
