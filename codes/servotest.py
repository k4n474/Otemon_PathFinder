from gpiozero import AngularServo
from time import sleep

servo = AngularServo(
    13,
    min_angle=-90,
    max_angle=90
)

while True:
    servo.angle = -45
    sleep(1)

    servo.angle = 0
    sleep(1)

    servo.angle = 45
    sleep(1)