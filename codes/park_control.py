"""park.py から使う動作部品。距離:mm、角度:度、時間:秒、出力:%。

各走行関数は完了まで待ち、終了・例外時に停止する。
GPIO / LiDAR は使うときに初期化するため、import だけでは動かない。
"""
import math
import threading
import time
from contextlib import contextmanager

from algorithm import detect_walls, detect_front_and_side_walls
from gyro import close_gyro, get_angle, reset_angle

CONTROL_INTERVAL = 0.05
_cancel = threading.Event()
_hardware = None
_lidar = None
_lidar_lock = threading.Lock()
_viewer = None


class ParkingCancelled(Exception):
    """stop_parking() による中断。"""


def _motor():
    global _hardware
    if _hardware is None:
        import newobot
        _hardware = newobot
    return _hardware


def dc_motor(power):
    """newobot.dc_motor と同じ。正:前進、負:後退。"""
    _check_cancel()
    if not math.isfinite(power) or not -100 <= power <= 100:
        raise ValueError("モーター出力は -100〜100 にしてください")
    _motor().dc_motor(power)


def set_angle(angle):
    """newobot.set_angle と同じ。負:左、正:右。"""
    _motor().set_angle(angle)


def stop():
    _motor().stop()


def brake():
    _motor().brake()


def _check_cancel():
    if _cancel.is_set():
        raise ParkingCancelled()


def wait(seconds):
    """直接モーター操作の後にも使える、中断可能な待機。"""
    if not math.isfinite(seconds) or seconds < 0:
        raise ValueError("待機時間は0以上にしてください")
    if _cancel.wait(seconds):
        raise ParkingCancelled()


def pause(seconds=0.4):
    """ブレーキをかけ、ハンドルを中央に戻して待つ。"""
    brake()
    set_angle(0)
    wait(seconds)


@contextmanager
def _movement():
    try:
        _check_cancel()
        yield
    finally:
        try:
            stop()
        finally:
            set_angle(0)


def _positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} は0より大きくしてください")


def _yaw():
    angle = get_angle("z")
    if not math.isfinite(angle):
        raise RuntimeError("ジャイロ角度が不正です")
    return angle


class _PD:
    def __init__(self):
        self.error = self.time = None

    def update(self, error, kp, kd, limit, sign=1.0, deadband=0.0):
        now = time.monotonic()
        derivative = 0.0
        if self.time is not None and now > self.time:
            derivative = (error - self.error) / (now - self.time)
        self.error, self.time = error, now
        steering = sign * (kp * error + kd * derivative)
        return 0.0 if abs(error) <= deadband else max(-limit, min(limit, steering))


GYRO_TURN_TIMEOUT = 10.0
GYRO_TURN_MIN_POWER = 22.0       # 発進・目標付近の出力（指定出力を上限にする）
GYRO_TURN_ACCEL_SECONDS = 0.4    # 指定出力まで上げる時間
GYRO_TURN_SLOWDOWN_ANGLE = 25.0  # 残り何度から減速するか
GYRO_TURN_BRAKE_SECONDS = 0.3    # 到達後の電気ブレーキ保持時間
GYRO_TURN_INTERVAL = 0.01       # 旋回時の角度確認周期


def gyro_turn(target_angle, steering_angle, power):
    """固定舵で加減速しながら旋回し、ブレーキ保持後のyawを返す。

    target_angleは最後にreset_angle("z")した時の0°を基準とする符号付き角度。
    現在角度への加算はしない。回る方向はsteering_angleとpowerで決まる。
    ジャイロはこの関数内ではリセットしない。
    """
    _positive(GYRO_TURN_TIMEOUT, "GYRO_TURN_TIMEOUT")
    if not math.isfinite(target_angle):
        raise ValueError("目標角度は有限値にしてください")
    if not math.isfinite(power) or not 0 < abs(power) <= 100:
        raise ValueError("power は0以外の -100〜100 にしてください")
    for value, name in ((GYRO_TURN_ACCEL_SECONDS, "加速時間"),
                        (GYRO_TURN_SLOWDOWN_ANGLE, "減速開始角度"),
                        (GYRO_TURN_INTERVAL, "制御周期")):
        _positive(value, name)
    with _movement():
        start = _yaw()
        started_at = time.monotonic()
        deadline = started_at + GYRO_TURN_TIMEOUT
        maximum = abs(power)
        minimum = min(GYRO_TURN_MIN_POWER, maximum)
        travel_sign = 1 if power > 0 else -1
        while True:
            _check_cancel()
            yaw = _yaw()
            reached = (
                yaw >= target_angle if target_angle >= start else yaw <= target_angle
            )
            if reached:
                # stop()だけでは惰性が残るため、舵を保ったまま電気ブレーキ。
                brake()
                wait(GYRO_TURN_BRAKE_SECONDS)
                return _yaw()
            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError("ジャイロ旋回がタイムアウトしました")
            remaining = abs(target_angle - yaw)
            acceleration = min(1.0, (now - started_at) / GYRO_TURN_ACCEL_SECONDS)
            deceleration = min(1.0, remaining / GYRO_TURN_SLOWDOWN_ANGLE)
            output = minimum + (maximum - minimum) * min(acceleration, deceleration)
            set_angle(steering_angle)
            dc_motor(travel_sign * output)
            wait(GYRO_TURN_INTERVAL)


def gyro_pd(power, duration, target_angle=None, *, kp=1.0, kd=0.1,
            max_steering=30.0, steering_sign=1.0):
    """指定秒数、目標yawをPD制御で保って走る。省略時は開始時の向き。

    指定角度は最後のreset_angle("z")基準。現在角度への加算はしない。
    後退時の操舵反転は自動。steering_sign は機体のジャイロ符号補正用。
    """
    _positive(duration, "duration")
    pd = _PD()
    with _movement():
        target = _yaw() if target_angle is None else target_angle
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            yaw = _yaw()
            sign = steering_sign * (1 if power > 0 else -1)
            set_angle(pd.update(target - yaw, kp, kd, max_steering, sign))
            dc_motor(power)
            wait(min(CONTROL_INTERVAL, max(0, deadline - time.monotonic())))
        return _yaw()


def _read_lidar_frame(detection_range=1500.0):
    """制御とブラウザが同じLiDAR接続を共有する。"""
    global _lidar
    with _lidar_lock:
        _check_cancel()
        if _lidar is None:
            from lidar_read import LidarReader
            _lidar = LidarReader(port="/dev/serial0", baudrate=230400,
                                 scan_frequency_increase_hz=6)
        if not _lidar.running:
            _lidar.start()
        points = _lidar.get_points()
        status = _lidar.get_status()
    walls = detect_walls(points, maximum_distance=detection_range)
    return points, walls, status


def _read_walls(detection_range=1500.0):
    return _read_lidar_frame(detection_range)[1]


def _select_front_wall(walls, final_wall=False):
    if final_wall:
        walls = [
            w for w in walls
            if abs((float(w.get("normal_angle", math.nan)) - 90 + 180) % 360 - 180) <= 40
        ]
    else:
        walls = [w for w in walls if w.get("is_front_wall")]
    return min(walls, key=lambda w: w["wall_distance"], default=None)


def read_front_and_side_walls(*, final_wall=False, detection_range=1500.0):
    """algorithm.pyで同じスキャンから前壁・左壁・右壁を取得する。

    戻り値: (front_wall, side_walls)。未検出の壁はNone。
    side_walls["left"] / side_walls["right"]で横壁を参照する。
    """
    walls = _read_walls(detection_range)
    front_wall, side_walls = detect_front_and_side_walls([], detected_walls=walls)
    if final_wall:
        front_wall = _select_front_wall(walls, final_wall=True)
    return front_wall, side_walls


def read_front_wall(*, final_wall=False, detection_range=1500.0):
    """algorithm.pyで前壁・横壁を検出し、前壁を返す。未検出はNone。"""
    front_wall, _ = read_front_and_side_walls(
        final_wall=final_wall, detection_range=detection_range)
    return front_wall


def read_front_wall_angle(*, final_wall=False, detection_range=1500.0, timeout=2.0):
    """前壁の法線角度を度で返す。正対時90°、右方向0°、左方向180°。

    前壁が検出されるまで最大timeout秒読み直し、未検出ならNoneを返す。
    timeout=0なら1回だけ読む。ジャイロ角度とは別の基準。
    detection_rangeは検出範囲(mm)。モーターは操作しない。
    """
    if not math.isfinite(timeout) or timeout < 0:
        raise ValueError("timeout は0以上の有限値にしてください")
    deadline = time.monotonic() + timeout
    while True:
        _check_cancel()
        wall = read_front_wall(
            final_wall=final_wall, detection_range=detection_range)
        if wall is not None:
            return float(wall["normal_angle"])
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        wait(min(CONTROL_INTERVAL, remaining))


def read_left_wall_distance(*, detection_range=1500.0, timeout=2.0):
    """LiDAR原点から左壁への垂直距離をmmで返す。未検出ならNone。

    左壁が検出されるまで最大timeout秒読み直す。timeout=0なら1回だけ読む。
    detection_rangeは検出範囲(mm)。モーターは操作しない。
    """
    if not math.isfinite(timeout) or timeout < 0:
        raise ValueError("timeout は0以上の有限値にしてください")
    deadline = time.monotonic() + timeout
    while True:
        _check_cancel()
        _, side_walls = read_front_and_side_walls(detection_range=detection_range)
        wall = side_walls["left"]
        if wall is not None:
            return float(wall["wall_distance"])
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        wait(min(CONTROL_INTERVAL, remaining))


def _power_magnitude(power):
    if not math.isfinite(power) or not 0 < power <= 100:
        raise ValueError("power は前進・後退とも正の値（0より大きく100以下）を指定してください")
    return power


# 横壁走行の調整値。呼び出し側の引数は4つだけにする。
SIDE_KP = 0.2
SIDE_KD = 0.12
SIDE_MAX_STEERING = 30.0


def lidar_forward(target_distance, power=30, side="left", target_side_distance=None):
    """lidar_forward(停止する前壁距離mm, 出力%, 左右, 横壁の目標距離mm)。

    例: lidar_forward(150, 30, "left", 250)
    横壁距離をPD制御し、前壁距離が目標以下になるまで前進する。
    sideは"left"または"right"。横距離Noneは開始時の距離を維持。
    """
    return _lidar_drive(target_distance, _power_magnitude(power), steering="side",
                        side=side, target_side_distance=target_side_distance,
                        kp=SIDE_KP, kd=SIDE_KD, max_steering=SIDE_MAX_STEERING)


def lidar_backward(target_distance, power=30, side="left", target_side_distance=None):
    """lidar_backward(停止する前壁距離mm, 出力%, 左右, 横壁の目標距離mm)。

    前壁距離が目標以上になるまで後退。powerは正の大きさ。
    後退による操舵反転は自動。その他はlidar_forwardと同じ。
    """
    return _lidar_drive(target_distance, -_power_magnitude(power), steering="side",
                        side=side, target_side_distance=target_side_distance,
                        kp=SIDE_KP, kd=SIDE_KD, max_steering=SIDE_MAX_STEERING)


# 前壁に正対する法線角度。機体の取り付け誤差を含めて85°に補正する。
FRONT_TARGET_ANGLE = 85.0
FRONT_KP = 0.25
FRONT_KD = 0.1
FRONT_MAX_STEERING = 30.0
FRONT_ANGLE_DEADBAND = 1.5


def lidar_front(target_distance, power=30, side="left", target_side_distance=None):
    """前壁の傾きをPD補正して前後進する。正出力は前進、負出力は後退。

    lidar_front(150, 30, "left", 250)
    引数の順序はlidar_forwardと同じ。sideとtarget_side_distanceは
    呼び出し互換用で、制御には使わない（横壁の検出も不要）。
    壁の並びによる役割分類は使わず、機体前方（90±40°）の壁を直接選ぶ。
    前壁の法線角をFRONT_TARGET_ANGLEに合わせる。
    前進は前壁距離が目標以下、後退は目標以上で停止。
    例: lidar_front(450, -30) は前壁から450mmまで後退する。
    """
    return _lidar_drive(target_distance, power, steering="front",
                        target_angle=FRONT_TARGET_ANGLE,
                        kp=FRONT_KP, kd=FRONT_KD, max_steering=FRONT_MAX_STEERING,
                        final_wall=True)


# ジャイロ距離走行のPD調整値。
GYRO_DRIVE_KP = 1.0
GYRO_DRIVE_KD = 0.1
GYRO_DRIVE_MAX_STEERING = 30.0
GYRO_DRIVE_STEERING_SIGN = 1.0


def gyro_forward(target_angle, power, target_distance):
    """gyro_forward(目標角度, 出力%, 停止する前壁距離mm)。

    角度は最後のreset_angle("z")基準。前壁距離が目標以下で停止。
    例: gyro_forward(90, 30, 150)。powerは正の大きさ。
    """
    if target_angle is None or not math.isfinite(target_angle):
        raise ValueError("目標角度は有限の数値を指定してください")
    return _lidar_drive(target_distance, _power_magnitude(power), steering="gyro",
                        target_angle=target_angle, kp=GYRO_DRIVE_KP, kd=GYRO_DRIVE_KD,
                        max_steering=GYRO_DRIVE_MAX_STEERING,
                        steering_sign=GYRO_DRIVE_STEERING_SIGN)


def gyro_backward(target_angle, power, target_distance):
    """gyro_backward(目標角度, 出力%, 停止する前壁距離mm)。

    角度は最後のreset_angle("z")基準。前壁距離が目標以上で停止。
    例: gyro_backward(90, 30, 1200)。powerは正の大きさ。操舵反転は自動。
    """
    if target_angle is None or not math.isfinite(target_angle):
        raise ValueError("目標角度は有限の数値を指定してください")
    return _lidar_drive(target_distance, -_power_magnitude(power), steering="gyro",
                        target_angle=target_angle, kp=GYRO_DRIVE_KP, kd=GYRO_DRIVE_KD,
                        max_steering=GYRO_DRIVE_MAX_STEERING,
                        steering_sign=GYRO_DRIVE_STEERING_SIGN)


def _lidar_drive(target_distance, power, *, steering="side", target_angle=None,
                side="left", target_side_distance=None,
                kp=1.0, kd=None, max_steering=None, steering_sign=1.0,
                final_wall=False, slowdown_distance=None, slow_power=None,
                start_power=None, start_duration=0.4, ignore_seconds=0.0,
                confirm_samples=1, timeout=30.0):
    """距離指定走行の共通ループ。通常は直接呼ばない。

    final_wall: 前壁を法線90±40°から選択する。
    slowdown_distance / slow_power: 減速開始距離mm / 減速出力の大きさ。
    start_power / start_duration: 発進出力の大きさ / 発進出力を使う秒数。
    ignore_seconds / confirm_samples: 終了判定の待機秒数 / 連続確認回数。
    timeout: 全体の制限秒数。必要な壁が見えない間は停止する。
    """
    _positive(target_distance, "target_distance")
    _positive(timeout, "timeout")
    if not math.isfinite(power) or not 0 < abs(power) <= 100:
        raise ValueError("power は0以外の -100〜100 にしてください")
    if steering not in ("side", "gyro", "front"):
        raise ValueError("steering は side / gyro / front を指定してください")
    if side not in ("left", "right"):
        raise ValueError("side は left / right を指定してください")
    if target_side_distance is not None:
        _positive(target_side_distance, "target_side_distance")
    for value, name in ((kp, "kp"), (kd, "kd"), (ignore_seconds, "ignore_seconds"),
                        (start_duration, "start_duration")):
        if value is not None and (not math.isfinite(value) or value < 0):
            raise ValueError(f"{name} は有限の0以上の値にしてください")
    if max_steering is not None and (not math.isfinite(max_steering) or not 0 < max_steering <= 50):
        raise ValueError("max_steering は0より大きく50以下にしてください")
    if target_angle is not None and not math.isfinite(target_angle):
        raise ValueError("target_angle は有限値にしてください")
    if steering_sign not in (-1, 1):
        raise ValueError("steering_sign は1または-1にしてください")
    for value in (start_power, slow_power):
        if value is not None:
            _power_magnitude(value)
    if slowdown_distance is not None:
        _positive(slowdown_distance, "slowdown_distance")
    if not isinstance(confirm_samples, int) or confirm_samples < 1:
        raise ValueError("confirm_samples は1以上の整数にしてください")
    pd = _PD()
    with _movement():
        target = target_angle
        if target is None:
            target = _yaw() if steering == "gyro" else None
        side_target = target_side_distance
        started = time.monotonic()
        drive_started = None
        samples = 0
        while True:
            _check_cancel()
            elapsed = time.monotonic() - started
            if elapsed >= timeout:
                raise TimeoutError("LiDAR距離走行がタイムアウトしました")
            side_wall = None
            if steering == "side":
                wall, side_walls = read_front_and_side_walls(final_wall=final_wall)
                side_wall = side_walls[side]
            else:
                wall = read_front_wall(final_wall=final_wall)
            if wall is None:
                stop()
                set_angle(0)
                pd = _PD()
                samples = 0
                wait(CONTROL_INTERVAL)
                continue
            distance = float(wall["wall_distance"])
            reached = distance <= target_distance if power > 0 else distance >= target_distance
            samples = samples + 1 if reached and elapsed >= ignore_seconds else 0
            if samples >= confirm_samples:
                return distance
            if steering == "side" and side_wall is None:
                stop()
                set_angle(0)
                pd = _PD()
                samples = 0
                wait(CONTROL_INTERVAL)
                continue
            if drive_started is None:
                drive_started = time.monotonic()
            travel_sign = 1 if power > 0 else -1
            if steering == "side":
                side_distance = float(side_wall["wall_distance"])
                if side_target is None:
                    side_target = side_distance
                side_sign = 1 if side == "right" else -1
                angle = pd.update(side_distance - side_target, kp, kd,
                                  max_steering, side_sign * travel_sign)
            elif steering == "front":
                error = (float(wall["normal_angle"]) - target + 180) % 360 - 180
                angle = pd.update(error, kp, kd, max_steering,
                                  -travel_sign, FRONT_ANGLE_DEADBAND)
            else:
                angle = pd.update(target - _yaw(), kp, kd,
                                  max_steering, steering_sign * travel_sign)
            output = power
            if start_power is not None and time.monotonic() - drive_started < start_duration:
                output = travel_sign * abs(start_power)
            if slowdown_distance is not None and slow_power is not None:
                slowing = distance <= slowdown_distance if power > 0 else distance >= slowdown_distance
                if slowing:
                    output = travel_sign * abs(slow_power)
            set_angle(angle)
            dc_motor(output)
            wait(CONTROL_INTERVAL)


def prepare():
    _cancel.clear()


def start_lidar_viewer():
    """駐車と同じLiDARを5000番ポートで配信。Live Serverからも参照可能。"""
    global _viewer
    if _viewer is None:
        from lidar_api import LidarViewerServer
        _viewer = LidarViewerServer(_read_lidar_frame)
        _viewer.start()
        print("LiDAR表示: http://<Raspberry PiのIP>:5000")


def cancel():
    _cancel.set()


def shutdown():
    """モーター・ジャイロ・LiDARの終了処理。"""
    global _lidar, _viewer
    _cancel.set()
    try:
        if _hardware is not None:
            try:
                _hardware.stop()
            finally:
                _hardware.set_angle(0)
    finally:
        try:
            close_gyro()
        finally:
            try:
                if _viewer is not None:
                    _viewer.stop()
                    _viewer = None
            finally:
                with _lidar_lock:
                    if _lidar is not None:
                        _lidar.stop()
                        _lidar = None
