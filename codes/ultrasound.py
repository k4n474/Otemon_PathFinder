import threading
import time
from statistics import median

try:
    import pigpio
except ImportError:
    pigpio = None

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None


FRONT_TRIGGER_PIN = 20
FRONT_ECHO_PIN = 16
BACK_TRIGGER_PIN = 7
BACK_ECHO_PIN = 8

_sensor = None
_back_sensor = None
_pigpio_instance = None


def _get_pigpio():
    global _pigpio_instance
    if pigpio is None:
        return None
    if _pigpio_instance is None:
        pi = pigpio.pi()
        if pi.connected:
            _pigpio_instance = pi
        else:
            pi.stop()
            _pigpio_instance = False
    return _pigpio_instance if _pigpio_instance is not False else None


class HCSR04:
    SPEED_OF_SOUND_CM_PER_SEC = 34300

    def __init__(self, trigger_pin=20, echo_pin=16, timeout=0.025):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.timeout = timeout
        self._lock = threading.Lock()
        self._echo_event = threading.Event()
        self._rise_tick = None
        self._pulse_us = None
        self.pi = _get_pigpio()
        self._callback = None

        if self.pi is not None:
            self.pi.set_mode(self.trigger_pin, pigpio.OUTPUT)
            self.pi.set_mode(self.echo_pin, pigpio.INPUT)
            self.pi.set_pull_up_down(self.echo_pin, pigpio.PUD_DOWN)
            self.pi.write(self.trigger_pin, 0)
            self._callback = self.pi.callback(self.echo_pin, pigpio.EITHER_EDGE, self._echo_callback)
        else:
            if GPIO is None:
                raise RuntimeError("pigpio か RPi.GPIO のどちらかが必要です")
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.trigger_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trigger_pin, GPIO.LOW)
        time.sleep(0.05)

    def _echo_callback(self, gpio, level, tick):
        if level == 1:
            self._rise_tick = tick
        elif level == 0 and self._rise_tick is not None:
            self._pulse_us = pigpio.tickDiff(self._rise_tick, tick)
            self._echo_event.set()

    def measure_distance(self):
        """距離をcmで返す。測定できないときは None を返す。"""
        if self.pi is not None:
            return self._measure_distance_pigpio()
        return self._measure_distance_gpio()

    def _measure_distance_pigpio(self):
        with self._lock:
            self._echo_event.clear()
            self._rise_tick = None
            self._pulse_us = None

            self.pi.gpio_trigger(self.trigger_pin, 10, 1)
            if not self._echo_event.wait(self.timeout):
                return None

            if self._pulse_us is None:
                return None
            distance = (self._pulse_us / 1_000_000) * self.SPEED_OF_SOUND_CM_PER_SEC / 2
            return round(distance, 2)

    def _measure_distance_gpio(self):
        GPIO.output(self.trigger_pin, GPIO.LOW)
        time.sleep(0.000002)
        GPIO.output(self.trigger_pin, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self.trigger_pin, GPIO.LOW)

        start_wait = time.monotonic()
        while GPIO.input(self.echo_pin) == GPIO.LOW:
            if time.monotonic() - start_wait > self.timeout:
                return None

        pulse_start = time.monotonic()
        while GPIO.input(self.echo_pin) == GPIO.HIGH:
            if time.monotonic() - pulse_start > self.timeout:
                return None

        pulse_end = time.monotonic()
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * self.SPEED_OF_SOUND_CM_PER_SEC / 2
        return round(distance, 2)

    def cleanup(self):
        if self._callback is not None:
            self._callback.cancel()
            self._callback = None
        if self.pi is None and GPIO is not None:
            GPIO.cleanup((self.trigger_pin, self.echo_pin))


def _get_sensor():
    global _sensor
    if _sensor is None:
        _sensor = HCSR04(trigger_pin=FRONT_TRIGGER_PIN, echo_pin=FRONT_ECHO_PIN)
    return _sensor


def _get_back_sensor():
    global _back_sensor
    if _back_sensor is None:
        _back_sensor = HCSR04(trigger_pin=BACK_TRIGGER_PIN, echo_pin=BACK_ECHO_PIN)
    return _back_sensor


def init_sensors():
    """前側と後ろ側の超音波センサーを両方初期化する。"""
    _get_sensor()
    _get_back_sensor()


def cleanup_sensors():
    """前側と後ろ側の超音波センサーで使ったGPIOを解放する。"""
    global _sensor, _back_sensor, _pigpio_instance
    if _sensor is not None:
        _sensor.cleanup()
        _sensor = None
    if _back_sensor is not None:
        _back_sensor.cleanup()
        _back_sensor = None
    if _pigpio_instance not in (None, False):
        _pigpio_instance.stop()
        _pigpio_instance = None


def us_get():
    """今の距離をcmで返す。測定できないときは None を返す。"""
    return _get_sensor().measure_distance()


def us_back_get():
    """後ろ側の今の距離をcmで返す。測定できないときは None を返す。"""
    return _get_back_sensor().measure_distance()


def dis_get(samples=10, interval=0.015):
    """距離をsamples回読んで、成功した測定値の中央値をcmで返す。"""
    return _median_distance(_get_sensor(), samples, interval)


def dis_back_get(samples=10, interval=0.015):
    """後ろ側の距離をsamples回読んで、成功した測定値の中央値をcmで返す。"""
    return _median_distance(_get_back_sensor(), samples, interval)


def _median_distance(sensor, samples, interval):
    distances = []

    for index in range(samples):
        distance = sensor.measure_distance()
        if distance is not None:
            distances.append(distance)
        if index < samples - 1 and interval > 0:
            time.sleep(interval)

    if not distances:
        return None
    return round(median(distances), 2)


def main():
    try:
        init_sensors()
        while True:
            front_distance = us_get()
            back_distance = us_back_get()

            if front_distance is None:
                front_text = "前: 測定できません"
            else:
                front_text = f"前: {front_distance:.2f} cm"

            if back_distance is None:
                back_text = "後ろ: 測定できません"
            else:
                back_text = f"後ろ: {back_distance:.2f} cm"

            print(f"{front_text} / {back_text}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("終了します")
    finally:
        cleanup_sensors()


if __name__ == "__main__":
    main()
