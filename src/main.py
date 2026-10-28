"""Lab 0x07 application: line following plus BNO055 lane change and stop bar."""
from pyb import Pin, Timer
import cotask
from closedloop import ClosedLoop
from encoder import Encoder
from imu import BNO055
from linesensor import LineSensor
from motor import MotorDriver
from taskline import TaskLine
from taskmotor import TaskMotor
from taskuser import TaskUser

BATTERY_VOLTAGE = 9.0  # Replace with a measured value before powered tests.
MOTOR_PERIOD_MS = 5
LINE_PERIOD_MS = 20
USER_PERIOD_MS = 10

# With the installed orientation, a physical left turn prints a negative raw
# gyro Z rate. Keep that convention so it agrees with this Romi's wheel-turn
# convention (lane_direction=-1 below is a physical left departure).
IMU_YAW_SIGN = 1
IMU_MOUNTING = "BNO055 flat, +Z up, connector toward rear"


def wheel_mailbox(label, system):
    return {"label": label, "system": system, "request": False, "active": False,
            "data_ready": False, "sample_count": 0, "completion": "IDLE", "data": None,
            "mode": "P", "duty_cycle": 20.0, "velocity_setpoint": 3.0, "kp": 0.70,
            "line_following": False, "line_setpoint_rad_s": 0.0,
            "measured_velocity_rad_s": 0.0}


def line_mailbox(system):
    return {"system": system, "request": False, "lane_request": False,
            "stop_requested": False, "active": False, "data_ready": False,
            "sample_count": 0, "completion": "IDLE", "data": None,
            # Established Lab 0x06 controller settings.
            "base_speed_rad_s": 6.0, "kp_line_rad_s_per_mm": 0.50,
            "max_wheel_speed_rad_s": 6.0,
            # Lab 0x07 lane-change defaults; tune only after basic checks pass.
            # This Romi's verified motor/IMU conventions require -1 for a
            # physical left lane departure; change to +1 for a right change.
            "lane_direction": -1,
            "lane_spacing_m": 0.200,
            "departure_delay_ms": 2500,
            "maneuver_timeout_ms": 12000,
            "turn_base_speed_rad_s": 4.0,
            # Conservative first lane-change turn; increase only after the
            # target line is acquired reliably.
            "yaw_rate_setpoint_rad_s": 0.35,
            "yaw_rate_kp": 1.5,
            "wheel_radius_m": 0.035,
            "white_total_threshold": 0.05,
            "target_line_total_threshold": 0.10,
            "all_black_weight_threshold": 0.70,
            "imu_yaw_sign": IMU_YAW_SIGN}


def prepare_sensor():
    sensor = LineSensor(samples=10, sample_rate_hz=5000)
    print("Line sensor: [l]oad existing calibration or [c]apture black then white.")
    if input().strip().lower() == "l":
        sensor.load_calibration()
        print("Loaded line_calibration.json")
    else:
        sensor.capture_calibration(min_span=100)
    return sensor


def capture_imu_calibration(imu):
    print("IMU calibration: keep it still for gyro, then hold each of six faces still.")
    print("Press Enter to view status; save only when gyro and accel are both 3.")
    while True:
        input()
        system, gyro, accel, magnetometer = imu.calibration_status()
        print("IMU calibration S/G/A/M = {}/{}/{}/{}".format(
            system, gyro, accel, magnetometer))
        if gyro == 3 and accel == 3:
            imu.save_profile(mounting=IMU_MOUNTING, yaw_sign=IMU_YAW_SIGN)
            print("Saved imu_calibration.json")
            return


def prepare_imu():
    # I2C2 alternate pins: PB13=SCL and PB14=SDA.
    # Leave the BNO055 RST pin open: the Adafruit breakout holds it high.
    # This avoids using an Arduino pin that may already have another board role.
    imu = BNO055()
    imu.begin()
    print("BNO055 found. [l]oad imu_calibration.json or [c]alibrate now.")
    if input().strip().lower() == "l":
        try:
            record = imu.load_profile()
            print("Loaded IMU profile: {}".format(record.get("mounting", "unknown mounting")))
            if record.get("yaw_sign", IMU_YAW_SIGN) != IMU_YAW_SIGN:
                print("Warning: calibration yaw sign differs from IMU_YAW_SIGN in main.py")
        except Exception as error:
            print("Could not load IMU profile:", error)
            print("Calibration is required before continuing.")
            capture_imu_calibration(imu)
    else:
        capture_imu_calibration(imu)
    return imu


def main():
    if BATTERY_VOLTAGE <= 0:
        raise ValueError("BATTERY_VOLTAGE must be positive.")
    # Configure the sensor and alternate I2C2 pins before constructing the
    # motors: BNO055 setup briefly releases I2C2's default PB10/PB11 pins.
    sensor = prepare_sensor()
    imu = prepare_imu()

    # Motors: left PWM D7/PA8, DIR PB5, nSLEEP PB10; right PWM D2/PA10,
    # DIR PB4, nSLEEP D3/PB3. Encoders: left PC6/PC7; right PB6/PB7.
    pwm_timer = Timer(1, freq=20_000)
    left_motor = MotorDriver(Pin.cpu.A8, Pin.cpu.B5, Pin.cpu.B10, pwm_timer, 1)
    right_motor = MotorDriver(Pin.cpu.A10, Pin.cpu.B4, Pin.cpu.B3, pwm_timer, 3)
    left_encoder = Encoder(Timer(3, prescaler=0, period=0xFFFF), Pin.cpu.C6, Pin.cpu.C7)
    right_encoder = Encoder(Timer(4, prescaler=0, period=0xFFFF), Pin.cpu.B6, Pin.cpu.B7)
    left_motor.disable()
    right_motor.disable()

    system = {"shutdown": False}
    left, right, line = wheel_mailbox("left", system), wheel_mailbox("right", system), line_mailbox(system)
    left_task = TaskMotor(left_motor, left_encoder, ClosedLoop(), left, BATTERY_VOLTAGE, MOTOR_PERIOD_MS)
    right_task = TaskMotor(right_motor, right_encoder, ClosedLoop(), right, BATTERY_VOLTAGE, MOTOR_PERIOD_MS)
    line_task = TaskLine(sensor, imu, line, left, right, LINE_PERIOD_MS)
    user_task = TaskUser(left, right, line, system)
    tasks = (cotask.Task(left_task.run(), name="Left motor", priority=3, period=MOTOR_PERIOD_MS, profile=True),
             cotask.Task(right_task.run(), name="Right motor", priority=3, period=MOTOR_PERIOD_MS, profile=True),
             cotask.Task(line_task.run(), name="Line", priority=2, period=LINE_PERIOD_MS, profile=True),
             cotask.Task(user_task.run(), name="User", priority=1, period=USER_PERIOD_MS, profile=True))
    for task in tasks:
        cotask.task_list.append(task)
    print("Ready. [f] Lab 6 line test; [g] Lab 7 left lane change; [x] stop; [e] exit.")
    try:
        while not system["shutdown"]:
            cotask.task_list.pri_sched()
    finally:
        left_task.disable_motor()
        right_task.disable_motor()
        print("Motors disabled.")


if __name__ == "__main__":
    main()
