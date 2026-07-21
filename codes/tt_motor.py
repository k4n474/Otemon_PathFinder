import RPi.GPIO as GPIO
import time

class tt_motor:
    DEFAULT_DUTY_CYCLE = 50
    FORWARD_DUTY_OFFSET = 0
    BACKWARD_DUTY_OFFSET = 30

    def __init__(self):
        self.AIN1 = 27
        self.AIN2 = 17
        self.PWMA = 12
        self.STBY = 25

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        GPIO.setup(self.AIN1, GPIO.OUT)
        GPIO.setup(self.AIN2, GPIO.OUT)
        GPIO.setup(self.PWMA, GPIO.OUT)
        GPIO.setup(self.STBY, GPIO.OUT)

        self.pwm = GPIO.PWM(self.PWMA, 1000)  # 1kHz
        self.pwm.start(0)
        self.last_duty_cycle = 0
        GPIO.output(self.STBY, GPIO.HIGH)

    def calc_motor_pwm(self, servo_angle, center=0, pwm_max=80, pwm_min=40, k=5):
        error = abs(servo_angle - center)   # まっすぐからどれだけズレたか
        pwm = pwm_max - k * error
        if pwm < pwm_min:
            pwm = pwm_min
        if pwm > pwm_max:
            pwm = pwm_max

        return int(pwm)

    def _normalize_duty_cycle(self, duty_cycle, offset=0):
        duty_cycle = max(0, min(100, int(duty_cycle)))
        return max(0, min(100, duty_cycle + offset))

    def _set_speed(self, duty_cycle):
        self.last_duty_cycle = duty_cycle
        if self.pwm is not None:
            self.pwm.ChangeDutyCycle(duty_cycle)

    def forward(self, duty_cycle=None):
        GPIO.output(self.STBY, GPIO.HIGH)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.HIGH)
        if duty_cycle is None:
            duty_cycle = self.DEFAULT_DUTY_CYCLE
        duty_cycle = self._normalize_duty_cycle(duty_cycle, self.FORWARD_DUTY_OFFSET)
        self._set_speed(duty_cycle)
        return duty_cycle

    def backward(self, duty_cycle=None):
        GPIO.output(self.STBY, GPIO.HIGH)
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.LOW)
        if duty_cycle is None:
            duty_cycle = self.DEFAULT_DUTY_CYCLE
        duty_cycle = self._normalize_duty_cycle(duty_cycle, self.BACKWARD_DUTY_OFFSET)
        self._set_speed(duty_cycle)
        return duty_cycle

    def stop(self):
        self._set_speed(0)
        GPIO.output(self.AIN1, GPIO.LOW)
        GPIO.output(self.AIN2, GPIO.LOW)

    def brake(self):
        GPIO.output(self.STBY, GPIO.HIGH)
        self._set_speed(100)
        GPIO.output(self.AIN1, GPIO.HIGH)
        GPIO.output(self.AIN2, GPIO.HIGH)

    def cleanup(self):
        if self.pwm is not None:
            self.pwm.stop()
            self.pwm = None
        GPIO.output(self.STBY, GPIO.LOW)
        GPIO.cleanup()
