"""Control the drive motor and calibrated steering servo through GPIO."""

from gpiozero import PWMOutputDevice
from time import sleep
from gpiozero import Servo

# DRV8871 direction inputs use PWM duty cycles from 0.0 to 1.0.
motor_in1 = PWMOutputDevice(5, frequency=1000)
motor_in2 = PWMOutputDevice(6, frequency=1000)

servo = Servo(
    13,
    min_pulse_width=0.8 / 1000,
    max_pulse_width=2.2 / 1000
)


def dc_motor(speed):
    """Set signed motor power as a percentage: positive forward, negative reverse."""
    speed =speed / 100
    if speed > 0:
        motor_in1.value = speed
        motor_in2.value = 0
    else :
        motor_in1.value = 0
        motor_in2.value = speed * -1

def set_angle(angle):
    """Apply steering trim, limit the requested angle, and map it to servo travel."""
    # Compensate for the mechanical center offset before limiting travel.
    dif = 5
    angle += dif
    if angle > 50 + dif:
        angle = 50 + dif
    elif angle < -50 + dif:
        angle = -50 + dif
    # Different gains compensate for unequal left and right steering travel.
    angle = angle / 71.4
    if angle > 0:
        angle = angle * 1.4
    else:
        angle = angle * 2
    servo.value = angle


def stop():
    """Remove power from both motor inputs."""
    motor_in1.value = 0
    motor_in2.value = 0

def brake():
    """Apply electrical braking by driving both DRV8871 inputs high."""
    motor_in1.value = 1
    motor_in2.value = 1

def cleanup():
    """Release both motor PWM devices."""
    motor_in1.close()
    motor_in2.close()
