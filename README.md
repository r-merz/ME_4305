# Romi Line Following and IMU Lane Change

MicroPython for an STM32 Nucleo-L476RG-based Romi. It retains the
Lab 6 line follower and adds the Lab 7 BNO055-assisted lane-change. 

## Behavior

`main.py` starts four cooperative tasks:

- Left and right wheel velocity-control tasks
- A line/IMU task that owns the QTR and BNO055
- A USB-VCP user-interface task

The Lab 7 maneuver follows the departure line, uses IMU yaw-rate feedback to
turn toward an adjacent line, estimates lateral displacement from encoder speed
and relative heading, reacquires the target line, and attempts to stop at its transverse
black bar.

### Resource ownership and interfaces

`TaskMotor` owns one motor driver and one encoder per wheel. `TaskLine` owns the
QTR line sensor and BNO055 IMU, then writes left/right wheel setpoints into
shared mailboxes. `TaskUser` owns the USB-VCP command interface. Wheel setpoints
and measured velocities use rad/s; line centroid uses mm; IMU yaw rate uses
rad/s; and the lane-change lateral estimate uses m. A positive/negative wheel
setpoint follows each verified wheel’s forward/reverse convention. For this
robot, lane direction `-1` commands the verified physical left departure.


## Verified hardware configuration

### BNO055 IMU

The IMU uses the alternate I2C2 pins so it does not conflict with the motor
driver enable signal on PB10.

| BNO055 connection | Nucleo connection |
| --- | --- |
| VIN | U5V |
| GND | GND |
| SCL / yellow | PB13 (I2C2_SCL, AF4) |
| SDA / blue | PB14 (I2C2_SDA, AF4) |
| RST / green | Leave disconnected |
| 3Vo | Leave disconnected |

Mount the IMU flat and rigidly, with its labeled/chip side upward and its
connector toward the rear of the chassis. The current robot convention has a
negative raw gyro-Z reading during a physical left turn; `main.py` and the
lane direction are configured together for that verified hardware behavior.

### Existing Romi signals

| Device | Signals |
| --- | --- |
| Left motor | PWM PA8, direction PB5, nSLEEP PB10 |
| Right motor | PWM PA10, direction PB4, nSLEEP PB3 |
| Left encoder | PC6, PC7 (Timer 3) |
| Right encoder | PB6, PB7 (Timer 4) |
| QTR-HD-13A | PA0, PA1, PA4, PB0, PC1, PC0, PA7, PA6, PA5, PC2, PC3, PC4, PC5 |

## Required firmware and dependency

This project requires the course MicroPython firmware for the STM32L476RG Nucleo.
That firmware provides `pyb`, `utime`, and `array`. It also requires the
course-supplied `cotask.py` scheduler module on the Nucleo filesystem; that
module is not duplicated in this repository.


## Setup

1. Copy the required `.py` files to the Nucleo MicroPython filesystem.
2. Keep `line_calibration.json` on the Nucleo, or capture a new line-sensor
   calibration at startup.
3. Run `imu_test.py` with motors disabled to confirm I2C communication and
   gyro output.
4. Run `imu_calibration.py` once with motors disabled. Keep the IMU still for
   gyro calibration, then hold it still on each of its six faces until gyro
   and accelerometer status are both `3`. This creates `imu_calibration.json`.
5. Start the application:

   ```python
   exec(open("main.py").read())
   ```

6. At startup, choose `l` to load both saved calibration files.

7. `line_calibration.json` and `imu_calibration.json` are specific to this
   Romi’s installed sensors and IMU mounting. Recalibrate them after changing
   the sensor height, replacing the line sensor or IMU, or remounting the IMU.

## USB commands

| Command | Action |
| --- | --- |
| `o` / `c` | Select open-loop / P velocity wheel-test mode |
| `d`, `s`, `p` | Set wheel duty cycle, velocity setpoint, or wheel Kp |
| `l` / `r` | Run left / right finite wheel test |
| `b` / `k` | Set line-follow base speed / line P gain |
| `f` | Run the Lab 6 line-following test |
| `g` | Run the Lab 7 left lane-change maneuver |
| `x` | Immediate stop |
| `h` | Print command menu |
| `e` | Disable motors and exit safely |

## Tuning notes

Current starting settings are in `line_mailbox()` in `main.py`:

- Line base speed: `6.0 rad/s`
- Line Kp: `0.50 (rad/s)/mm`
- Lane direction: `-1` (physical left for this build)
- Yaw-rate setpoint: `0.35 rad/s`
- Yaw-rate Kp: `1.5`

Change only one parameter between trials. Reduce `yaw_rate_kp` if the lane
turn oscillates; reduce the yaw-rate setpoint for a gentler departure.

## Safety

- Keep wheels elevated during wheel-sign and steering-sign checks.
- Use physical fences on the printed course during bench tests.
- Keep a hand near motor power and use `x` for an immediate software stop.
- Disconnect motor power during wiring, IMU installation, and calibration.

## Important files

- `main.py` - hardware creation, startup calibration, scheduler, settings
- `taskline.py` - Lab 6 line following and Lab 7 lane-change state machine
- `taskmotor.py` - wheel velocity control; wheel-test logs are allocated only
  when a wheel test is requested to preserve RAM for the IMU maneuver
- `imu.py` - BNO055 I2C2 driver and calibration profile support
- `imu_test.py` - motors-off IMU communication/sign test
- `imu_calibration.py` - motors-off BNO055 calibration helper
- `line_calibration.json` / `imu_calibration.json` - persistent calibration
  files; retain them on the Nucleo for repeatable startup


## Lab 8 checkpoint and verification

**Checkpoint:** `lab08-incremental-output` branch, merged into `main`.
The Lab 8 refactor commit is `f20715d`; the lazy log-allocation change is in
`src/taskline.py`.

### Change made

`TaskLine` previously allocated nine line/IMU logging arrays during startup,
even when no line test was running. This left insufficient RAM for
`TaskMotor` to allocate its wheel-test log.

Line/IMU logging arrays are now allocated only when a Lab 6 line test or Lab
7 lane-change run starts. This preserves RAM while the robot is idle or
performing a wheel test.

### Verification performed

- Startup: passed; line and IMU calibration profiles loaded and the command
  menu appeared.
- Left wheel test (`l`): passed after the change; the test completed and
  printed its data without a `MemoryError`.
- Lab 7 lane-change (`g`): lane-change motion ran without a `MemoryError`.
- Emergency stop (`x`): passed; the robot stopped and reported
  `TEST_ABORTED`.


### Reproducible verification procedure

1. Upload the repository `src` files and both calibration JSON files to the
   Nucleo. Connect through PuTTY at 115200 baud.
2. Run `exec(open("main.py").read())`, then enter `l` for both saved
   calibration prompts. Expected result: the command menu appears.
3. With the wheels raised, enter `l` to run the left-wheel test. Expected
   result: data prints and ends in `TEST_COMPLETE`, with no `MemoryError`.
4. Soft-reset the Nucleo, restart the application, and run `g` on the course.
   Expected result: lane-change motion executes without a `MemoryError`.
5. During a line or lane-change run, enter `x`. Expected result:
   `TEST_ABORTED` and both motors stop.

   
### Known limitation

During the post-change Lab 7 check, the robot completed the lane-change
motion but ended with `LINE_LOST` near the final stop bar instead of
`STOP_BAR`. This is retained as a separate line/stop-bar behavior issue and
was not changed as part of the Lab 8 memory refactor.