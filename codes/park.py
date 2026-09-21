"""駐車の動作を上から順に書くファイル。

編集する場所は parking_sequence()。各動作関数は終わるまで待つ。
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
TRACE_SIDE = "left"  # 横壁PDで使う壁: "left" / "right"
SIDE_DISTANCE = None  # mm。Noneなら各動作の開始時の横距離を維持


def parking_sequence(direction):
    """ここに動作を追加・並べ替えして駐車手順を作る。

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
    time.sleep(0.5)
    brake()
    
    
    

    


# 以下は開始・停止処理。通常は上の parking_sequence() だけ編集すればよい。
control_thread = None
control_running = False
parking_error = None
_lifecycle_lock = threading.Lock()


def _run(direction):
    global control_running, parking_error
    try:
        start_lidar_viewer()
        parking_sequence(direction)
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


def start_parking(run_direction):
    """他のプログラムから非同期実行。従来通りThreadを返す。

    終了後、parking_error で失敗内容を確認できる。
    """
    global control_thread
    with _lifecycle_lock:
        _begin(run_direction)
        control_thread = threading.Thread(target=_run, args=(run_direction,),
                                          daemon=True, name="parking")
        control_thread.start()
        return control_thread


def stop_parking():
    """走行関数・waitを中断して、終了処理が終わるまで待つ。"""
    with _lifecycle_lock:
        cancel()
        if control_thread is not None and control_thread is not threading.current_thread():
            control_thread.join()


def main():
    with _lifecycle_lock:
        _begin(RUN_DIRECTION)
    try:
        _run(RUN_DIRECTION)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
