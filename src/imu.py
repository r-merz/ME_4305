"""BNO055 driver used by the Lab 0x07 IMU lane-change controller.

Wire the Adafruit breakout as: VIN->U5V, GND->GND, SCL/yellow->PB13, and
SDA/blue->PB14. These pins are the alternate I2C2 pair (AF4). Leave RST and
3Vo unconnected.
"""
import json
import math
import time
from pyb import I2C, Pin

BNO055_ADDRESS = 0x28
CHIP_ID_REGISTER = 0x00
EXPECTED_CHIP_ID = 0xA0
PAGE_ID_REGISTER = 0x07
CALIBRATION_STATUS_REGISTER = 0x35
UNIT_SELECTION_REGISTER = 0x3B
OPERATION_MODE_REGISTER = 0x3D
POWER_MODE_REGISTER = 0x3E
CALIBRATION_DATA_REGISTER = 0x55
CALIBRATION_DATA_LENGTH = 22
CONFIG_MODE = 0x00
IMUPLUS_MODE = 0x08


def make_i2c2_pb13_pb14(baudrate):
    """Return I2C2 after routing it from its default pins to PB13/PB14.

    The Nucleo firmware creates I2C2 on PB10/PB11 by default. PB10 is the
    left motor driver's nSLEEP pin, so it is released here before routing
    I2C2 to PB13 (SCL) and PB14 (SDA). main.py creates the motor objects only
    after this function, restoring PB10 as the motor-control output.
    """
    i2c = I2C(2, I2C.MASTER, baudrate=baudrate)
    Pin(Pin.cpu.B10, mode=Pin.ANALOG)
    Pin(Pin.cpu.B11, mode=Pin.ANALOG)
    Pin(Pin.cpu.B13, mode=Pin.ALT, alt=4)  # I2C2_SCL
    Pin(Pin.cpu.B14, mode=Pin.ALT, alt=4)  # I2C2_SDA
    return i2c


class BNO055:
    """Small BNO055 interface with persistent IMUPLUS calibration support."""
    def __init__(self, address=BNO055_ADDRESS, reset_pin=None, baudrate=100000):
        self.address = address
        self.reset_pin = reset_pin
        self.i2c = make_i2c2_pb13_pb14(baudrate)

    def _read(self, register, count=1):
        return self.i2c.mem_read(count, self.address, register)

    def _write(self, register, value):
        self.i2c.mem_write(bytes((value,)), self.address, register)

    def _write_block(self, register, values):
        self.i2c.mem_write(bytes(values), self.address, register)

    def set_mode(self, mode):
        self._write(OPERATION_MODE_REGISTER, mode)
        time.sleep_ms(25 if mode == CONFIG_MODE else 20)

    def reset(self):
        """Reset through RST when it is wired; otherwise allow power-up time."""
        if self.reset_pin is not None:
            self.reset_pin.low()
            time.sleep_ms(10)
            self.reset_pin.high()
        time.sleep_ms(700)

    def begin(self):
        """Confirm the chip, select degree/dps units, and enter IMUPLUS mode."""
        self.reset()
        if self._read(CHIP_ID_REGISTER)[0] != EXPECTED_CHIP_ID:
            raise OSError("BNO055 not found at I2C address 0x{:02X}".format(self.address))
        self.set_mode(CONFIG_MODE)
        self._write(PAGE_ID_REGISTER, 0x00)
        self._write(POWER_MODE_REGISTER, 0x00)
        self._write(UNIT_SELECTION_REGISTER, 0x00)  # Degrees and degrees/s.
        self.set_mode(IMUPLUS_MODE)

    @staticmethod
    def _signed_16(low, high):
        value = low | (high << 8)
        return value - 65536 if value & 0x8000 else value

    def euler_degrees(self):
        """Return (heading, roll, pitch), before mounting sign correction."""
        data = self._read(0x1A, 6)
        return tuple(self._signed_16(data[index], data[index + 1]) / 16.0
                     for index in (0, 2, 4))

    def heading_rad(self, yaw_sign=-1):
        """Return yaw in radians; choose yaw_sign so a left turn is positive."""
        heading = self._signed_16(*self._read(0x1A, 2)) / 16.0
        return yaw_sign * heading * math.pi / 180.0

    def yaw_rate_rad_s(self, yaw_sign=-1):
        """Return z gyro rate in rad/s; choose yaw_sign so a left turn is positive."""
        raw = self._signed_16(*self._read(0x18, 2)) / 16.0
        return yaw_sign * raw * math.pi / 180.0

    def calibration_status(self):
        """Return (system, gyro, accel, magnetometer), each from 0 through 3."""
        value = self._read(CALIBRATION_STATUS_REGISTER)[0]
        return ((value >> 6) & 3, (value >> 4) & 3,
                (value >> 2) & 3, value & 3)

    def calibration_profile(self):
        """Read the 22-byte profile. The device must be in config mode to do so."""
        self.set_mode(CONFIG_MODE)
        profile = list(self._read(CALIBRATION_DATA_REGISTER, CALIBRATION_DATA_LENGTH))
        self.set_mode(IMUPLUS_MODE)
        return profile

    def restore_calibration_profile(self, profile):
        """Restore a previously saved 22-byte profile and return to IMUPLUS."""
        if not isinstance(profile, list) or len(profile) != CALIBRATION_DATA_LENGTH:
            raise ValueError("BNO055 profile must contain exactly 22 bytes.")
        if any(not isinstance(value, int) or value < 0 or value > 255 for value in profile):
            raise ValueError("BNO055 profile contains an invalid byte.")
        self.set_mode(CONFIG_MODE)
        self._write_block(CALIBRATION_DATA_REGISTER, profile)
        self.set_mode(IMUPLUS_MODE)

    def save_profile(self, filename="imu_calibration.json", mounting="Z axis up", yaw_sign=-1):
        """Save calibration data plus mounting metadata to the Nucleo filesystem."""
        record = {"mode": "IMUPLUS", "mounting": mounting,
                  "yaw_sign": yaw_sign, "profile": self.calibration_profile()}
        with open(filename, "w") as file:
            json.dump(record, file)
        return record

    def load_profile(self, filename="imu_calibration.json"):
        """Load a saved profile, validate it, and return its metadata."""
        with open(filename, "r") as file:
            record = json.load(file)
        if record.get("mode") != "IMUPLUS":
            raise ValueError("Calibration file is not an IMUPLUS profile.")
        self.restore_calibration_profile(record.get("profile"))
        return record
