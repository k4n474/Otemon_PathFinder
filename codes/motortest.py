from gpiozero import PWMOutputDevice
from time import sleep
from gpiozero import Servo
from newobot import dc_motor, set_angle, stop, cleanup

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

    # 30%の強さで1秒だけ回す
    set_angle(-50)

    dc_motor(-35)

    sleep(5)

    stop()

finally:

    stop()

    cleanup()