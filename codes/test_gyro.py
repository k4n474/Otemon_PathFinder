import time
import unittest
from unittest.mock import patch

import gyro


class FakeSensor:
    def __init__(self, rate_z=0.0):
        self.rate_z = rate_z
        self.closed = False

    def initialize(self, enable_magnetometer=False):
        pass

    def calibrate_gyro(self):
        pass

    def read_accel(self):
        return {"x": 0.0, "y": 0.0, "z": 1.0}

    def read_motion(self):
        return self.read_accel(), {
            "x": 0.0, "y": 0.0, "z": self.rate_z
        }

    def close(self):
        self.closed = True


class GyroAngleReaderTest(unittest.TestCase):
    def make_reader(self, rate_z=0.0):
        with patch.object(gyro, "MPU9250", return_value=FakeSensor(rate_z)):
            return gyro.GyroAngleReader()

    def test_yaw_updates_without_get_angle_polling(self):
        reader = self.make_reader(rate_z=100.0)
        try:
            reader.initialize(calibrate=False)
            time.sleep(0.06)
            self.assertGreater(reader.get_angle("z"), 2.0)
        finally:
            reader.close()

    def test_reset_uses_current_angle_as_zero(self):
        reader = self.make_reader(rate_z=100.0)
        try:
            reader.initialize(calibrate=False)
            time.sleep(0.04)
            reader.reset_angle("z")
            self.assertAlmostEqual(reader.get_angle("z"), 0.0, delta=1.5)
        finally:
            reader.close()

    def test_close_is_idempotent(self):
        reader = self.make_reader()
        reader.initialize(calibrate=False)
        reader.close()
        reader.close()
        self.assertTrue(reader.sensor.closed)


if __name__ == "__main__":
    unittest.main()
