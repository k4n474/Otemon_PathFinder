#!/usr/bin/env python3
"""
Read angles from an MPU-9250/MPU-9255 style 9-axis IMU over Raspberry Pi I2C.

Wiring for Raspberry Pi I2C1:
  SDA -> GPIO2, physical pin 3
  SCL -> GPIO3, physical pin 5
  VCC -> 3.3V
  GND -> GND

Run:
  python3 gyro.py

If the smbus module is missing on Raspberry Pi OS:
  sudo apt install python3-smbus i2c-tools
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

try:
    from smbus2 import SMBus
except ImportError:
    try:
        from smbus import SMBus  # type: ignore
    except ImportError:
        SMBus = None  # type: ignore


MPU_ADDR = 0x68
AK8963_ADDR = 0x0C

PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_CONFIG_2 = 0x1D
INT_PIN_CFG = 0x37
WHO_AM_I = 0x75

ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

AK8963_WHO_AM_I = 0x00
AK8963_ST1 = 0x02
AK8963_XOUT_L = 0x03
AK8963_CNTL1 = 0x0A
AK8963_ASAX = 0x10

ACCEL_SCALE = 16384.0  # +/- 2g
GYRO_SCALE = 131.0  # +/- 250 deg/s
MAG_SCALE = 4912.0 / 32760.0  # uT/LSB at 16-bit output


@dataclass
class Angles:
    roll: float
    pitch: float
    yaw: float
    heading: float | None


class MPU9250:
    def __init__(self, bus_number: int = 1, address: int = MPU_ADDR) -> None:
        if SMBus is None:
            raise RuntimeError(
                "I2C library is not installed.\n"
                "On Raspberry Pi OS, run:\n"
                "  sudo apt update\n"
                "  sudo apt install python3-smbus i2c-tools"
            )
        self.bus = SMBus(bus_number)
        self.address = address
        self.gyro_bias = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.mag_adjust = (1.0, 1.0, 1.0)
        self.mag_available = False

    def write_byte(self, register: int, value: int) -> None:
        self.bus.write_byte_data(self.address, register, value)

    def read_byte(self, register: int) -> int:
        return self.bus.read_byte_data(self.address, register)

    def read_i2c_block(self, register: int, length: int) -> list[int]:
        return self.bus.read_i2c_block_data(self.address, register, length)

    @staticmethod
    def _to_signed(high: int, low: int) -> int:
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    @staticmethod
    def _to_signed_le(low: int, high: int) -> int:
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    def initialize(self, enable_magnetometer: bool = False) -> None:
        self.write_byte(PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        self.write_byte(SMPLRT_DIV, 0x07)
        self.write_byte(CONFIG, 0x03)
        self.write_byte(GYRO_CONFIG, 0x00)  # +/- 250 deg/s
        self.write_byte(ACCEL_CONFIG, 0x00)  # +/- 2g
        self.write_byte(ACCEL_CONFIG_2, 0x03)

        if enable_magnetometer:
            self.write_byte(INT_PIN_CFG, 0x02)
            time.sleep(0.05)
            self._initialize_magnetometer()

    def _initialize_magnetometer(self) -> None:
        try:
            whoami = self.bus.read_byte_data(AK8963_ADDR, AK8963_WHO_AM_I)
            if whoami != 0x48:
                return

            self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x00)
            time.sleep(0.02)
            self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x0F)
            time.sleep(0.02)
            asa = self.bus.read_i2c_block_data(AK8963_ADDR, AK8963_ASAX, 3)
            self.mag_adjust = tuple(((x - 128) / 256.0) + 1.0 for x in asa)

            self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x00)
            time.sleep(0.02)
            self.bus.write_byte_data(AK8963_ADDR, AK8963_CNTL1, 0x16)
            self.mag_available = True
        except OSError:
            self.mag_available = False

    def read_accel(self) -> dict[str, float]:
        data = self.read_i2c_block(ACCEL_XOUT_H, 6)
        return {
            "x": self._to_signed(data[0], data[1]) / ACCEL_SCALE,
            "y": self._to_signed(data[2], data[3]) / ACCEL_SCALE,
            "z": self._to_signed(data[4], data[5]) / ACCEL_SCALE,
        }

    def read_gyro(self) -> dict[str, float]:
        data = self.read_i2c_block(GYRO_XOUT_H, 6)
        raw = {
            "x": self._to_signed(data[0], data[1]) / GYRO_SCALE,
            "y": self._to_signed(data[2], data[3]) / GYRO_SCALE,
            "z": self._to_signed(data[4], data[5]) / GYRO_SCALE,
        }
        return {axis: raw[axis] - self.gyro_bias[axis] for axis in raw}

    def read_motion(self) -> tuple[dict[str, float], dict[str, float]]:
        data = self.read_i2c_block(ACCEL_XOUT_H, 14)
        accel = {
            "x": self._to_signed(data[0], data[1]) / ACCEL_SCALE,
            "y": self._to_signed(data[2], data[3]) / ACCEL_SCALE,
            "z": self._to_signed(data[4], data[5]) / ACCEL_SCALE,
        }
        raw_gyro = {
            "x": self._to_signed(data[8], data[9]) / GYRO_SCALE,
            "y": self._to_signed(data[10], data[11]) / GYRO_SCALE,
            "z": self._to_signed(data[12], data[13]) / GYRO_SCALE,
        }
        gyro = {axis: raw_gyro[axis] - self.gyro_bias[axis] for axis in raw_gyro}
        return accel, gyro

    def read_mag(self) -> dict[str, float] | None:
        if not self.mag_available:
            return None

        try:
            status = self.bus.read_byte_data(AK8963_ADDR, AK8963_ST1)
            if not status & 0x01:
                return None

            data = self.bus.read_i2c_block_data(AK8963_ADDR, AK8963_XOUT_L, 7)
            if data[6] & 0x08:
                return None

            mx = self._to_signed_le(data[0], data[1]) * MAG_SCALE * self.mag_adjust[0]
            my = self._to_signed_le(data[2], data[3]) * MAG_SCALE * self.mag_adjust[1]
            mz = self._to_signed_le(data[4], data[5]) * MAG_SCALE * self.mag_adjust[2]
            return {"x": mx, "y": my, "z": mz}
        except OSError:
            return None

    def calibrate_gyro(self, samples: int = 200) -> None:
        print("Calibrating gyro. Keep the sensor still...")
        total = {"x": 0.0, "y": 0.0, "z": 0.0}

        old_bias = self.gyro_bias
        self.gyro_bias = {"x": 0.0, "y": 0.0, "z": 0.0}

        for _ in range(samples):
            gyro = self.read_gyro()
            for axis in total:
                total[axis] += gyro[axis]
            time.sleep(0.005)

        self.gyro_bias = {
            "x": total["x"] / samples,
            "y": total["y"] / samples,
            "z": total["z"] / samples,
        }
        if old_bias != {"x": 0.0, "y": 0.0, "z": 0.0}:
            print("Replaced previous gyro calibration.")

    def close(self) -> None:
        self.bus.close()


def accel_angles(accel: dict[str, float]) -> tuple[float, float]:
    ax, ay, az = accel["x"], accel["y"], accel["z"]
    roll = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
    return roll, pitch


def magnetic_heading(mag: dict[str, float]) -> float:
    heading = math.degrees(math.atan2(mag["y"], mag["x"]))
    return heading + 360.0 if heading < 0 else heading


class GyroAngleReader:
    """Import-friendly angle reader.

    x -> roll, y -> pitch, z -> yaw
    """

    def __init__(
        self,
        bus_number: int = 1,
        address: int = MPU_ADDR,
        alpha: float = 0.98,
        enable_magnetometer: bool = False,
    ) -> None:
        self.sensor = MPU9250(bus_number=bus_number, address=address)
        self.alpha = alpha
        self.enable_magnetometer = enable_magnetometer
        self.angles = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.offsets = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_time = time.monotonic()
        self.initialized = False

    def initialize(self, calibrate: bool = True) -> None: 
        if self.initialized:
            return

        self.sensor.initialize(enable_magnetometer=self.enable_magnetometer)
        if calibrate:
            self.sensor.calibrate_gyro()

        accel = self.sensor.read_accel()
        roll, pitch = accel_angles(accel)
        self.angles = {"x": roll, "y": pitch, "z": 0.0}
        self.offsets = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_time = time.monotonic()
        self.initialized = True

    def update(self, axis: str | None = None) -> None:
        self.initialize()

        now = time.monotonic()
        dt = now - self.last_time
        self.last_time = now

        if axis == "z":
            gyro = self.sensor.read_gyro()
            self.angles["z"] += gyro["z"] * dt
            return

        accel, gyro = self.sensor.read_motion()
        acc_roll, acc_pitch = accel_angles(accel)

        self.angles["x"] = self.alpha * (self.angles["x"] + gyro["x"] * dt) + (1.0 - self.alpha) * acc_roll
        self.angles["y"] = self.alpha * (self.angles["y"] + gyro["y"] * dt) + (1.0 - self.alpha) * acc_pitch
        self.angles["z"] += gyro["z"] * dt

    def get_angle(self, axis: str) -> float:
        axis = normalize_axis(axis)
        self.update(axis)
        return self.angles[axis] - self.offsets[axis]

    def reset_angle(self, axis: str | None = None) -> None:
        if axis is None:
            self.update()
            self.offsets = self.angles.copy()
            return

        axis = normalize_axis(axis)
        self.update(axis)
        self.offsets[axis] = self.angles[axis]

    def close(self) -> None:
        self.sensor.close()


_default_reader: GyroAngleReader | None = None


def normalize_axis(axis: str) -> str:
    axis = axis.lower().strip()
    if axis not in ("x", "y", "z"):
        raise ValueError('axis must be "x", "y", or "z"')
    return axis


def get_reader() -> GyroAngleReader:
    global _default_reader
    if _default_reader is None:
        _default_reader = GyroAngleReader()
    return _default_reader


def get_angle(axis: str) -> float:
    """Return the current angle in degrees.

    Use x for roll, y for pitch, and z for yaw.
    Example:
      from gyro import get_angle
      print(get_angle("z"))
    """

    return get_reader().get_angle(axis)


def reset_angle(axis: str | None = None) -> None:
    """Make the current angle the zero point.

    If axis is omitted, x/y/z are all reset.
    Example:
      from gyro import reset_angle
      reset_angle()
      reset_angle("z")
    """

    get_reader().reset_angle(axis)


def close_gyro() -> None:
    global _default_reader
    if _default_reader is not None:
        _default_reader.close()
        _default_reader = None


def main() -> None:
    sensor = MPU9250(bus_number=1, address=MPU_ADDR)

    try:
        sensor.initialize(enable_magnetometer=True)
        whoami = sensor.read_byte(WHO_AM_I)
        print(f"MPU WHO_AM_I: 0x{whoami:02X}")
        print(f"Magnetometer: {'OK' if sensor.mag_available else 'not detected'}")

        sensor.calibrate_gyro()

        accel = sensor.read_accel()
        roll, pitch = accel_angles(accel)
        yaw = 0.0
        last_time = time.monotonic()
        alpha = 0.98

        print("Press Ctrl+C to stop.")
        while True:
            now = time.monotonic()
            dt = now - last_time
            last_time = now

            accel = sensor.read_accel()
            gyro = sensor.read_gyro()
            acc_roll, acc_pitch = accel_angles(accel)

            roll = alpha * (roll + gyro["x"] * dt) + (1.0 - alpha) * acc_roll
            pitch = alpha * (pitch + gyro["y"] * dt) + (1.0 - alpha) * acc_pitch
            yaw += gyro["z"] * dt

            mag = sensor.read_mag()
            heading = magnetic_heading(mag) if mag else None

            if heading is None:
                print(f"roll={roll:7.2f} deg  pitch={pitch:7.2f} deg  yaw={yaw:7.2f} deg")
            else:
                print(
                    f"roll={roll:7.2f} deg  pitch={pitch:7.2f} deg  "
                    f"yaw={yaw:7.2f} deg  heading={heading:7.2f} deg"
                )

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sensor.close()


if __name__ == "__main__":
    main()
