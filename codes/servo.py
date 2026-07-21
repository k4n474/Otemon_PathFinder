import pigpio
import time

class servo:
    def __init__(self, servo_pin=18):
        self.servo_pin = servo_pin
        self.pi = pigpio.pi()
        if not self.pi.connected:
            print("pigpiodに接続できてない")
            exit()

    def set_angle(self, sg_angle):
        pulse = 500 + ((sg_angle + 90) / 180.0) * 2000
        self.pi.set_servo_pulsewidth(self.servo_pin, pulse)

    def stop(self):
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        self.pi.stop()
