from servo import servo
from tt_motor import tt_motor

class robot:
    def __init__(self):
        self.sg = servo()
        self.tt = tt_motor()

    def forward(self, duty_cycle=None):
        self.tt.forward(duty_cycle)

    def backward(self, duty_cycle=None):
        self.tt.backward(duty_cycle)

    def backword(self, duty_cycle=None):
        self.backward(duty_cycle)

    def set_angle(self, angle):
        self.sg.set_angle(angle)

    def stop(self):
        self.tt.stop()

    def brake(self):
        self.tt.brake()

    def cleanup(self):
        self.sg.stop()
        self.tt.cleanup()
