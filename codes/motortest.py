"""Manually test the motor by alternating forward and reverse every second."""

from gpiozero import PWMOutputDevice
from time import sleep
from gpiozero import Servo
from newobot import dc_motor, set_angle, stop, cleanup

# Previous direct DRV8871 control example; the active test uses newobot.
# # DRV8871
# motor_in1 = PWMOutputDevice(5, frequency=1000)
# motor_in2 = PWMOutputDevice(6, frequency=1000)

# def forward(speed=0.3):

#     motor_in1.value = speed

#     motor_in2.value = 0

# def backward(speed=0.3):

#     motor_in1.value = 0

#     motor_in2.value = speed

# def stop():
#     motor_in1.value = 0
#     motor_in2.value = 0

try:
    while True:
        
        while True:
            dc_motor(40)
            sleep(1)
            dc_motor(-40)
            sleep(1)


finally:
    # Stop the motor and release its GPIO devices even after Ctrl+C.
    stop()

    cleanup()
