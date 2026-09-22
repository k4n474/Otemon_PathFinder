"""駐車の動作を上から順に書くファイル。

編集する場所は parking_sequence_0()〜parking_sequence_7()。番号はpoに対応。
各パターンの関数内に動作を書く。各動作関数は終わるまで待つ。
距離:mm / 角度:度 / 時間:秒 / モーター出力:%（正:前進、負:後退）
ハンドル: 負が左、正が右。
"""
import threading
import time
from park_control import (
    ParkingCancelled, cancel, prepare, shutdown, start_lidar_viewer,
    dc_motor, set_angle, stop, brake, wait, pause,
    gyro_turn, gyro_pd, read_front_wall, read_front_and_side_walls,
    read_front_wall_angle, read_left_wall_distance,
    lidar_forward, lidar_backward, lidar_front, gyro_forward, gyro_backward,
    get_angle, reset_angle,
)

CLOCKWISE = 0
COUNTERCLOCKWISE = 1
RUN_DIRECTION = COUNTERCLOCKWISE
RUN_PATTERN = 2 # 直接実行する駐車パターン（0〜7）
TRACE_SIDE = "left"  # 横壁PDで使う壁: "left" / "right"
SIDE_DISTANCE = None  # mm。Noneなら各動作の開始時の横距離を維持


def parking_sequence_0(direction):
    """po=0: 方向0・初回近接なし、追加判定でも対象なし。ここに駐車動作を書く。"""
    
    pause(0.2)
    angle = read_front_wall_angle()
    print(angle)
    # angle = (angle + 5) - 87

    reset_angle("z")
    gyro_turn(-1 * angle, -36, 35)
    pause(0.1)
    
    reset_angle("z")
    
    gyro_backward(0, 30, 400)
    
    pause(0.1)
    
    # 時計回りだけ、最初に壁寄せと後退旋回を行う。
    
    lidar_front(180, 23)
    # lidar_forward(180, 23, "right", SIDE_DISTANCE)
    pause(1)
    reset_angle("z") 
    
    pause(1)

    # angle = read_front_wall_angle()
    # print(angle)
    # angle = 90 - (angle - 5)
    gyro_turn(89, -40, -30)

    pause(0.6)

    gyro_backward(89, 30, 1200)
    pause(1)
    reset_angle("z")

    gyro_turn(87,-40,-30)
    pause(0.4)
    
    lidar_front(90,27)
    pause(0.4)
    reset_angle("z")
    
    lidar_front(300,-25)
    lidar_front(480, -20)
    pause(0.4)
    gyro_turn(-82,40,-28)
    pause(1)
    
    dc_motor(30)
    time.sleep(0.2)
    brake()
    
    
    
    # set_angle / dc_motor / wait / gyro_turn などを追加。
    stop()


def parking_sequence_1(direction):
    """po=1: 方向0・追加判定で赤。ここに駐車動作を書く。"""
    # set_angle / dc_motor / wait / gyro_turn などを追加。
    
    pause(0.2)
    angle = read_front_wall_angle()
    print(angle)
    # angle = (angle + 5) - 87

    reset_angle("z")
    gyro_turn(-1 * angle, -30, 35)
    pause(0.1)
    
    reset_angle("z")
    
    gyro_backward(0, 30, 400)
    
    pause(0.1)
    
    # 時計回りだけ、最初に壁寄せと後退旋回を行う。
    
    lidar_front(180, 23)
    # lidar_forward(180, 23, "right", SIDE_DISTANCE)
    pause(1)
    reset_angle("z") 
    
    pause(1)
    
    gyro_backward(0, 30, 330)
    pause(0.4)
    # angle = read_front_wall_angle()
    # print(angle)
    # angle = 90 - (angle - 5)
    gyro_turn(89, -40, -30)

    pause(0.6)

    gyro_backward(89, 30, 1250)
    pause(1)
    reset_angle("z")

    gyro_turn(87,-40,-30)
    pause(0.4)
    
    lidar_front(90,23)
    pause(0.4)
    reset_angle("z")
    
    lidar_front(300,-25)
    lidar_front(480, -20)
    pause(0.4)
    gyro_turn(-82,40,-28)
    pause(1)
    
    dc_motor(30)
    time.sleep(0.6)
    brake()
    
    
    
    # set_angle / dc_motor / wait / gyro_turn などを追加。
    stop()
    
    
    stop()
    
    
    


def parking_sequence_2(direction):
    """po=2: 方向0・追加判定で緑。ここに駐車動作を書く。"""
    pause(0.2)
    angle = read_front_wall_angle()
    print(angle)
    # angle = (angle + 5) - 87

    reset_angle("z")
    gyro_turn(-1 * angle, -36, 35)
    pause(0.1)
    
    reset_angle("z")
    
    gyro_backward(0, 30, 400)
    
    pause(0.1)
    
    lidar_front(130, 25)
    # lidar_forward(130, 25, "right", SIDE_DISTANCE)
    pause(1)
    reset_angle("z") 
    
    pause(1)

    # angle = read_front_wall_angle()
    # print(angle)
    # angle = 90 - (angle - 5)
    gyro_turn(89, -40, -30)

    pause(0.6)

    gyro_backward(89, 30, 1260)
    pause(1)
    reset_angle("z")

    gyro_turn(87,-40,-30)
    pause(0.4)
    
    lidar_front(90,27)
    pause(0.4)
    reset_angle("z")
    
    lidar_front(300,-25)
    lidar_front(480, -20)
    pause(0.4)
    gyro_turn(-82,40,-28)
    pause(1)
    
    dc_motor(30)
    time.sleep(0.2)
    brake()
    
    
    
    # set_angle / dc_motor / wait / gyro_turn などを追加。
    stop()


def parking_sequence_3(direction):
    """po=3: 方向1・近接物体なし。既存の駐車手順。

    例（必要な場所へコピー）:
        set_angle(0)
        dc_motor(30)
        wait(1.0)
        pause(0.4)
        gyro_turn(80, 40, 35)
        gyro_pd(30, duration=2.0, target_angle=0, kp=1, kd=0.1)
        lidar_forward(150, 30, "left", 250)
        lidar_backward(450, 30, "left", 250)
        gyro_forward(0, 30, 150)
        gyro_backward(0, 30, 450)
        wall_angle = read_front_wall_angle()  # 正対時90°、未検出はNone
        if wall_angle is not None:
            print(f"前壁の角度: {wall_angle:.1f}°")
        left_distance = read_left_wall_distance()  # mm、未検出はNone
        if left_distance is not None:
            print(f"左壁との距離: {left_distance:.1f}mm")

    reset_angle("z") は停止中に呼ぶ。指定角度はすべて最後のリセット時の0°が基準。
    """
    

    pause(0.2)
    angle = read_front_wall_angle()
    print(angle)
    angle = 90 - angle

    reset_angle("z")
    gyro_turn(82 + angle, 30, 35)
    pause(0.1)
    
    # 時計回りだけ、最初に壁寄せと後退旋回を行う。
    lidar_forward(100, 25, "left", SIDE_DISTANCE)
    pause(0.5)
    reset_angle("z")
    
    left_angle = read_left_wall_distance()
    
    pause(1)

    gyro_turn(-85, 40, -30)

    pause(0.6)

    gyro_backward(-85, 30, 1150)
    # lidar_backward(450, 30, "right", 370)
    pause(1)
    reset_angle("z")

    gyro_turn(-85,-35,30)
    pause(0.4)
    
    lidar_front(90,27)
    pause(0.4)
    reset_angle("z")
    
    lidar_front(300,-25)
    lidar_front(480, -20)
    pause(0.4)
    gyro_turn(-82,40,-28)
    pause(1)
    
    dc_motor(30)
    time.sleep(0.2)
    brake()
    
    

def parking_sequence_4(direction):
    """po=4: 方向1・近接物体が赤。ここに駐車動作を書く。"""
    # set_angle / dc_motor / wait / gyro_turn などを追加。
    
    
    pause(0.2)
    angle = read_front_wall_angle()
    print(angle)
    angle = 90 - angle

    reset_angle("z")
    gyro_turn(40 + angle, 30, 35)
    pause(0.1)
    gyro_turn(85 + angle, -40, -35)
    pause(0.2)
    
    # 時計回りだけ、最初に壁寄せと後退旋回を行う。
    lidar_forward(100, 25, "left", SIDE_DISTANCE)
    pause(0.5)
    reset_angle("z")
    
    gyro_backward(0, 30, 300)
    pause(0.2)
    
    lidar_forward(100, 25, "left", SIDE_DISTANCE)
    pause(0.5)
    reset_angle("z")
    
    left_angle = read_left_wall_distance()

    pause(1)

    gyro_turn(-85, 40, -30)

    pause(0.6)

    gyro_backward(-85, 30, 1150)
    # lidar_backward(450, 30, "right", 370)
    pause(1)
    reset_angle("z")

    gyro_turn(-85,-35,30)
    pause(0.4)

    lidar_front(90,27)
    pause(0.4)
    reset_angle("z")

    lidar_front(300,-25)
    lidar_front(480, -20)
    pause(0.4)
    gyro_turn(-82,40,-28)
    pause(1)

    dc_motor(30)
    time.sleep(0.2)
    brake()
    
    
    
    
    
    stop()


def parking_sequence_5(direction):
    """po=5: 方向1・近接物体が緑。ここに駐車動作を書く。"""
    # set_angle / dc_motor / wait / gyro_turn などを追加。
    
    
    pause(0.2)
    angle = read_front_wall_angle()
    print(angle)
    angle = 90 - angle

    reset_angle("z")
    gyro_turn(82 + angle, 30, 35)
    pause(0.1)
    
    # 時計回りだけ、最初に壁寄せと後退旋回を行う。
    lidar_forward(100, 25, "left", SIDE_DISTANCE)
    pause(0.5)
    reset_angle("z")
    
    left_angle = read_left_wall_distance()
    
    pause(1)
    
    gyro_backward(0, 30, 350)
    
    pause(0.2)

    gyro_turn(-85, 40, -30)

    pause(0.6)

    gyro_backward(-85, 30, 880)
    # lidar_backward(450, 30, "right", 370)
    pause(0.5)
    reset_angle("z")

    gyro_turn(-85,40,-30)
    pause(0.2)
    
    lidar_front(90,27)
    pause(0.4)
    reset_angle("z")
    
    lidar_front(300,-24)
    lidar_front(480, -19)
    
    pause(0.4)
    gyro_turn(-82,40,-28)
    pause(1)
    
    dc_motor(30)
    time.sleep(0.2)
    brake()
    
    
    
    stop()


def parking_sequence_6(direction):
    """po=6: 方向0・最初の近接物体が赤。ここに駐車動作を書く。"""
    
    
    pause(0.2)
    angle = read_front_wall_angle()
    print(angle)
    # angle = (angle + 5) - 87

    reset_angle("z")
    gyro_turn(-1 * angle, -30, 35)
    pause(0.1)
    
    reset_angle("z")
    
    gyro_backward(0, 30, 400)
    
    pause(0.1)
    
    # 時計回りだけ、最初に壁寄せと後退旋回を行う。
    
    lidar_front(180, 23)
    # lidar_forward(180, 23, "right", SIDE_DISTANCE)
    pause(1)
    reset_angle("z") 
    
    pause(1)
    
    gyro_backward(0, 30, 320)
    pause(0.4)
    # angle = read_front_wall_angle()
    # print(angle)
    # angle = 90 - (angle - 5)
    gyro_turn(90, -40, -30)

    pause(0.6)

    gyro_backward(90, 30, 1270)
    pause(1)
    reset_angle("z")

    gyro_turn(87,-40,-30)
    pause(0.4)
    
    lidar_front(90,23)
    pause(0.4)
    reset_angle("z")
    
    lidar_front(300,-25)
    lidar_front(480, -20)
    pause(0.4)
    gyro_turn(-82,40,-28)
    pause(1)
    
    dc_motor(30)
    time.sleep(0.6)
    brake()
    
    
    # set_angle / dc_motor / wait / gyro_turn などを追加。
    stop()
    


def parking_sequence_7(direction):
    """po=7: 方向0・最初の近接物体が緑。ここに駐車動作を書く。"""
    
    pause(0.2)
    angle = read_front_wall_angle()
    print(angle)
    # angle = (angle + 5) - 87

    reset_angle("z")
    gyro_turn(-1 * angle, -36, 35)
    pause(0.1)
    
    reset_angle("z")
    
    gyro_backward(0, 30, 400)
    
    pause(0.1)
    
    lidar_front(120, 25)
    # lidar_forward(130, 25, "right", SIDE_DISTANCE)
    pause(1)
    reset_angle("z") 
    
    pause(1)

    # angle = read_front_wall_angle()
    # print(angle)
    # angle = 90 - (angle - 5)
    gyro_turn(89, -40, -30)

    pause(0.6)

    gyro_backward(89, 30, 1260)
    pause(1)
    reset_angle("z")

    gyro_turn(87,-40,-30)
    pause(0.4)
    
    lidar_front(90,27)
    pause(0.4)
    reset_angle("z")
    
    lidar_front(300,-25)
    lidar_front(480, -20)
    pause(0.4)
    gyro_turn(-82,40,-28)
    pause(1)
    
    dc_motor(30)
    time.sleep(0.2)
    brake()
    
    
    
    # set_angle / dc_motor / wait / gyro_turn などを追加。
    stop()


def parking_sequence(direction):
    """従来の呼び出し用。パターン3を実行する。"""
    parking_sequence_3(direction)


# 以下は共通の開始・停止処理。通常は上の8つの動作関数だけ編集する。
control_thread = None
control_running = False
parking_error = None
_lifecycle_lock = threading.Lock()


def _run(direction, pattern=3):
    global control_running, parking_error
    try:
        start_lidar_viewer()
        sequences = (parking_sequence_0, parking_sequence_1, parking_sequence_2,
                     parking_sequence_3, parking_sequence_4, parking_sequence_5,
                     parking_sequence_6, parking_sequence_7)
        sequences[pattern](direction)
    except ParkingCancelled:
        pass
    except Exception as error:
        parking_error = error
        raise
    finally:
        try:
            shutdown()
        finally:
            control_running = False


def _begin(direction):
    global control_running, parking_error
    if direction not in (CLOCKWISE, COUNTERCLOCKWISE):
        raise ValueError("direction は0（時計回り）または1（反時計回り）")
    if control_running:
        raise RuntimeError("駐車プログラムは既に実行中です")
    prepare()
    parking_error = None
    control_running = True


def _start_parking(run_direction, pattern):
    """他のプログラムから非同期実行。従来通りThreadを返す。

    終了後、parking_error で失敗内容を確認できる。
    """
    global control_thread
    with _lifecycle_lock:
        _begin(run_direction)
        control_thread = threading.Thread(target=_run, args=(run_direction, pattern),
                                          daemon=True, name=f"parking_{pattern}")
        control_thread.start()
        return control_thread


def start_parking(run_direction):
    """従来の呼び出し用。パターン3を開始する。"""
    return start_parking_3(run_direction)


def start_parking_0(run_direction):
    """parking_sequence_0を別スレッドで開始し、Threadを返す。"""
    return _start_parking(run_direction, 0)


def start_parking_1(run_direction):
    """parking_sequence_1を別スレッドで開始し、Threadを返す。"""
    return _start_parking(run_direction, 1)


def start_parking_2(run_direction):
    """parking_sequence_2を別スレッドで開始し、Threadを返す。"""
    return _start_parking(run_direction, 2)


def start_parking_3(run_direction):
    """parking_sequence_3を別スレッドで開始し、Threadを返す。"""
    return _start_parking(run_direction, 3)


def start_parking_4(run_direction):
    """parking_sequence_4を別スレッドで開始し、Threadを返す。"""
    return _start_parking(run_direction, 4)


def start_parking_5(run_direction):
    """parking_sequence_5を別スレッドで開始し、Threadを返す。"""
    return _start_parking(run_direction, 5)


def start_parking_6(run_direction):
    """parking_sequence_6を別スレッドで開始し、Threadを返す。"""
    return _start_parking(run_direction, 6)


def start_parking_7(run_direction):
    """parking_sequence_7を別スレッドで開始し、Threadを返す。"""
    return _start_parking(run_direction, 7)


def stop_parking():
    """走行関数・waitを中断して、終了処理が終わるまで待つ。"""
    with _lifecycle_lock:
        cancel()
        if control_thread is not None and control_thread is not threading.current_thread():
            control_thread.join()


def main():
    if RUN_PATTERN not in range(8):
        raise ValueError("RUN_PATTERN は0〜7にしてください")
    with _lifecycle_lock:
        _begin(RUN_DIRECTION)
    try:
        _run(RUN_DIRECTION, RUN_PATTERN)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
