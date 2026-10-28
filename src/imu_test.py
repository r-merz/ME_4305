"""Standalone BNO055 bring-up test for I2C2 PB13/PB14.

Upload with imu.py; motors remain disabled. Wiring: VIN->U5V, GND->GND,
SCL/yellow->PB13, SDA/blue->PB14, and leave RST open.
"""
import time
from imu import BNO055


def main():
    imu = BNO055()
    imu.begin()
    print("BNO055 found. Move it slowly; Ctrl-C stops the test.")
    while True:
        heading, roll, pitch = imu.euler_degrees()
        yaw_rate = imu.yaw_rate_rad_s()
        system, gyro, accel, magnetometer = imu.calibration_status()
        print("heading={:7.2f} roll={:7.2f} pitch={:7.2f} deg zrate={:6.3f} rad/s | cal S/G/A/M={}/{}/{}/{}".format(
            heading, roll, pitch, yaw_rate, system, gyro, accel, magnetometer))
        time.sleep_ms(200)


if __name__ == "__main__":
    main()
