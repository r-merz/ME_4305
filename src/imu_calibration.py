"""Motors-off BNO055 IMUPLUS calibration helper for Lab 0x07 on I2C2.

Upload with imu.py. It writes imu_calibration.json only after gyro and accel
calibration status both reach 3.
"""
from imu import BNO055

MOUNTING = "BNO055 flat, +Z up, connector toward rear"
YAW_SIGN = -1


def main():
    imu = BNO055()
    imu.begin()
    print("Keep the IMU still for gyro calibration, then hold all six faces still.")
    print("Press Enter to read status. Ctrl-C aborts without saving.")
    while True:
        input()
        system, gyro, accel, magnetometer = imu.calibration_status()
        print("Calibration S/G/A/M = {}/{}/{}/{}".format(
            system, gyro, accel, magnetometer))
        if gyro == 3 and accel == 3:
            imu.save_profile(mounting=MOUNTING, yaw_sign=YAW_SIGN)
            print("Saved imu_calibration.json")
            return


if __name__ == "__main__":
    main()
