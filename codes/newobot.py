from gpiozero import PWMOutputDevice
from time import sleep
from gpiozero import Servo

# DRV8871
motor_in1 = PWMOutputDevice(5, frequency=1000)
motor_in2 = PWMOutputDevice(6, frequency=1000)

servo = Servo(
    13,
    min_pulse_width=0.8 / 1000,
    max_pulse_width=2.2 / 1000
)


def dc_motor(speed):
    speed =speed / 100
    if speed > 0:
        motor_in1.value = speed
        motor_in2.value = 0
    else :
        motor_in1.value = 0
        motor_in2.value = speed * -1

def set_angle(angle):
    dif = 0
    angle += dif
    if angle > 50 + dif:
        angle = 50 + dif
    elif angle < -50 + dif:
        angle = -50 + dif
    angle = angle / 71.4
    servo.value = angle


def stop():
    motor_in1.value = 0
    motor_in2.value = 0

def cleanup():
    motor_in1.close()
    motor_in2.close()
