"""Owns one motor/encoder pair for Lab 0x05 tests and Lab 0x06 following."""
import utime
import gc
from array import array

WAIT, TEST, FOLLOW = range(3)


class TaskMotor:
    def __init__(self, motor, encoder, controller, mailbox, supply_voltage,
                 period_ms=5, test_duration_ms=2000):
        self.motor, self.encoder, self.controller = motor, encoder, controller
        self.mailbox, self.supply_voltage = mailbox, supply_voltage
        self.period_ms, self.test_duration_ms = period_ms, test_duration_ms
        self.max_samples = test_duration_ms // period_ms + 2

        # Do not reserve two large Lab 0x05 data logs at boot. The line/IMU
        # controller needs that RAM. These arrays are created only if the user
        # explicitly starts an individual wheel-response test.
        self.time = self.setpoint = self.velocity = None
        self.voltage = self.duty = None
        mailbox["data"] = None
        self.state = WAIT

    def _allocate_test_log(self):
        """Allocate the finite wheel-test log only when that test is requested."""
        gc.collect()
        self.time = array("f", [0] * self.max_samples)
        self.setpoint = array("f", [0] * self.max_samples)
        self.velocity = array("f", [0] * self.max_samples)
        self.voltage = array("f", [0] * self.max_samples)
        self.duty = array("f", [0] * self.max_samples)
        self.mailbox["data"] = {"time_ms": self.time, "setpoint_rad_s": self.setpoint,
                                "velocity_rad_s": self.velocity,
                                "voltage_limited_V": self.voltage,
                                "duty_percent": self.duty}

    def disable_motor(self):
        self.motor.set_effort(0)
        self.motor.disable()
        self.mailbox["active"] = False

    def _control(self, setpoint):
        self.encoder.update()
        measured = self.encoder.get_velocity()
        self.mailbox["measured_velocity_rad_s"] = measured
        _, volts = self.controller.update(setpoint, measured)
        duty = 100.0 * volts / self.supply_voltage
        self.motor.set_effort(duty)
        return measured, volts, duty

    def _start_test(self, now):
        self._allocate_test_log()
        self.mailbox["request"] = False
        self.mailbox["active"] = True
        self.mailbox["data_ready"] = False
        self.mailbox["completion"] = "RUNNING"
        self.index, self.started_ms = 0, now
        self.mode = self.mailbox["mode"]
        self.test_duty = self.mailbox["duty_cycle"]
        self.test_setpoint = self.mailbox["velocity_setpoint"]
        self.controller.kp = self.mailbox["kp"]
        self.controller.v_min, self.controller.v_max = -self.supply_voltage, self.supply_voltage
        self.controller.reset()
        self.encoder.zero()
        self.motor.enable()
        self.state = TEST

    def _finish_test(self, result):
        self.disable_motor()
        self.mailbox["sample_count"] = self.index
        self.mailbox["completion"] = result
        self.mailbox["data_ready"] = True
        self.state = WAIT

    def _start_following(self):
        self.controller.kp = self.mailbox["kp"]
        self.controller.v_min, self.controller.v_max = -self.supply_voltage, self.supply_voltage
        self.controller.reset()
        self.encoder.zero()
        self.mailbox["measured_velocity_rad_s"] = 0.0
        self.motor.enable()
        self.mailbox["active"] = True
        self.state = FOLLOW

    def run(self):
        while True:
            now = utime.ticks_ms()
            if self.mailbox["system"]["shutdown"]:
                self.disable_motor()
                self.state = WAIT
            elif self.state == WAIT:
                if self.mailbox["line_following"]:
                    self._start_following()
                elif self.mailbox["request"]:
                    self._start_test(now)
            elif self.state == FOLLOW:
                if not self.mailbox["line_following"]:
                    self.disable_motor()
                    self.state = WAIT
                else:
                    self._control(self.mailbox["line_setpoint_rad_s"])
            else:  # finite Lab 0x05 response capture
                elapsed = utime.ticks_diff(now, self.started_ms)
                if self.mode == "OPEN":
                    self.encoder.update()
                    velocity = self.encoder.get_velocity()
                    duty = self.test_duty
                    volts = duty * self.supply_voltage / 100.0
                    self.motor.set_effort(duty)
                    setpoint = 0.0
                else:
                    velocity, volts, duty = self._control(self.test_setpoint)
                    setpoint = self.test_setpoint
                if self.index < self.max_samples:
                    i = self.index
                    self.time[i], self.setpoint[i] = elapsed, setpoint
                    self.velocity[i], self.voltage[i], self.duty[i] = velocity, volts, duty
                    self.index += 1
                else:
                    self._finish_test("FAILED_BUFFER_FULL")
                if elapsed >= self.test_duration_ms:
                    self._finish_test("TEST_COMPLETE")
            yield 0
