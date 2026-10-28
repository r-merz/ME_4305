"""Line following and Lab 0x07 IMU-feedback lane-change state machine."""
import math
import utime
from array import array

WAIT, LINE_TEST, LANE_FOLLOW_START, LANE_DEPART, LANE_RETURN, LANE_FOLLOW_TARGET = range(6)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_angle(angle):
    """Wrap an angle to [-pi, pi)."""
    while angle >= math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class TaskLine:
    """Owns the line sensor and sends velocity setpoints to both wheel tasks."""
    def __init__(self, sensor, imu, line_mailbox, left_mailbox, right_mailbox,
                 period_ms=20, run_duration_ms=60000, log_period_ms=200):
        self.sensor, self.imu = sensor, imu
        self.box, self.left, self.right = line_mailbox, left_mailbox, right_mailbox
        self.period_ms = period_ms
        self.run_duration_ms, self.log_period_ms = run_duration_ms, log_period_ms
        self.max_samples = run_duration_ms // log_period_ms + 4
        self.time = array("f", [0] * self.max_samples)
        self.centroid = array("f", [0] * self.max_samples)
        self.weight = array("f", [0] * self.max_samples)
        self.left_sp = array("f", [0] * self.max_samples)
        self.right_sp = array("f", [0] * self.max_samples)
        self.psi = array("f", [0] * self.max_samples)
        self.yaw_rate = array("f", [0] * self.max_samples)
        self.lateral = array("f", [0] * self.max_samples)
        self.state_log = array("f", [0] * self.max_samples)
        self.box["data"] = {"time_ms": self.time, "centroid_mm": self.centroid,
                            "total_weight": self.weight,
                            "left_setpoint_rad_s": self.left_sp,
                            "right_setpoint_rad_s": self.right_sp,
                            "psi_rad": self.psi, "yaw_rate_rad_s": self.yaw_rate,
                            "lateral_estimate_m": self.lateral,
                            "state": self.state_log}
        self.state = WAIT

    def _stop_wheels(self):
        self.left["line_following"] = False
        self.right["line_following"] = False

    def _finish(self, completion):
        self._stop_wheels()
        self.box["sample_count"] = self.index
        self.box["completion"] = completion
        self.box["active"] = False
        self.box["data_ready"] = True
        self.state = WAIT

    def _snapshot_settings(self):
        self.base_speed = self.box["base_speed_rad_s"]
        self.line_kp = self.box["kp_line_rad_s_per_mm"]
        self.max_speed = self.box["max_wheel_speed_rad_s"]
        self.yaw_sign = self.box["imu_yaw_sign"]
        self.lane_direction = self.box["lane_direction"]
        self.lane_spacing = self.box["lane_spacing_m"]
        self.turn_speed = self.box["turn_base_speed_rad_s"]
        self.yaw_rate_setpoint = self.box["yaw_rate_setpoint_rad_s"]
        self.yaw_rate_kp = self.box["yaw_rate_kp"]
        self.wheel_radius = self.box["wheel_radius_m"]
        self.white_threshold = self.box["white_total_threshold"]
        self.target_threshold = self.box["target_line_total_threshold"]
        self.black_threshold = self.box["all_black_weight_threshold"]
        self.depart_delay_ms = self.box["departure_delay_ms"]
        self.maneuver_timeout_ms = self.box["maneuver_timeout_ms"]

    def _start(self, now, lane_change):
        if self.left["active"] or self.right["active"]:
            self.box["request"] = self.box["lane_request"] = False
            self.box["completion"] = "MOTORS_BUSY"
            self.box["data_ready"] = True
            return
        self.box["request"] = self.box["lane_request"] = False
        self.box["data_ready"] = False
        self.box["active"] = True
        self.box["completion"] = "RUNNING"
        self.index, self.started_ms = 0, now
        self.last_log_ms, self.last_update_ms = now - self.log_period_ms, now
        self.lateral_estimate, self.heading_start = 0.0, None
        self.white_cleared = False
        self._snapshot_settings()
        self.left["line_following"] = True
        self.right["line_following"] = True
        self.state = LANE_FOLLOW_START if lane_change else LINE_TEST

    def _set_wheels(self, left_speed, right_speed):
        self.current_left = clamp(left_speed, -self.max_speed, self.max_speed)
        self.current_right = clamp(right_speed, -self.max_speed, self.max_speed)
        self.left["line_setpoint_rad_s"] = self.current_left
        self.right["line_setpoint_rad_s"] = self.current_right

    def _line_control(self, centroid):
        correction = self.line_kp * centroid
        # This sign matches the established Lab 0x06 wiring: +centroid turns left.
        self._set_wheels(self.base_speed - correction, self.base_speed + correction)

    def _motion_estimate(self, now):
        heading = self.imu.heading_rad(self.yaw_sign)
        if self.heading_start is None:
            self.heading_start = heading
        self.current_psi = wrap_angle(heading - self.heading_start)
        self.current_yaw_rate = self.imu.yaw_rate_rad_s(self.yaw_sign)
        dt_s = utime.ticks_diff(now, self.last_update_ms) / 1000.0
        self.last_update_ms = now
        forward_speed = 0.5 * self.wheel_radius * (
            self.left["measured_velocity_rad_s"] + self.right["measured_velocity_rad_s"])
        self.lateral_estimate += forward_speed * self.current_psi * max(0.0, dt_s)

    def _yaw_control(self, signed_rate):
        correction = self.yaw_rate_kp * (signed_rate - self.current_yaw_rate)
        self._set_wheels(self.turn_speed - correction, self.turn_speed + correction)

    def _log(self, now, centroid, total_weight):
        if utime.ticks_diff(now, self.last_log_ms) < self.log_period_ms:
            return True
        if self.index >= self.max_samples:
            self._finish("FAILED_BUFFER_FULL")
            return False
        i = self.index
        self.time[i] = utime.ticks_diff(now, self.started_ms)
        self.centroid[i], self.weight[i] = centroid or 0.0, total_weight
        self.left_sp[i], self.right_sp[i] = self.current_left, self.current_right
        self.psi[i], self.yaw_rate[i] = self.current_psi, self.current_yaw_rate
        self.lateral[i], self.state_log[i] = self.lateral_estimate, self.state
        self.index += 1
        self.last_log_ms = now
        return True

    def _lane_step(self, now, centroid, total_weight):
        elapsed = utime.ticks_diff(now, self.started_ms)
        if self.state == LANE_FOLLOW_START:
            if centroid is None or total_weight < self.target_threshold:
                self._finish("LINE_LOST_BEFORE_DEPART")
            else:
                self._line_control(centroid)
                if elapsed >= self.depart_delay_ms:
                    self.heading_start = self.imu.heading_rad(self.yaw_sign)
                    self.last_update_ms = now
                    self.lateral_estimate = 0.0
                    self.state = LANE_DEPART
                    print("Lane maneuver: departing initial line.")
        elif self.state == LANE_DEPART:
            self._motion_estimate(now)
            if total_weight < self.white_threshold:
                self.white_cleared = True
            self._yaw_control(self.lane_direction * self.yaw_rate_setpoint)
            if abs(self.lateral_estimate) >= self.lane_spacing / 2.0:
                self.state = LANE_RETURN
                print("Lane maneuver: returning toward parallel line.")
            elif elapsed >= self.maneuver_timeout_ms:
                self._finish("MANEUVER_TIMEOUT")
        elif self.state == LANE_RETURN:
            self._motion_estimate(now)
            self._yaw_control(-self.lane_direction * self.yaw_rate_setpoint)
            if self.white_cleared and centroid is not None and total_weight >= self.target_threshold:
                self.state = LANE_FOLLOW_TARGET
                print("Lane maneuver: target line acquired.")
            elif elapsed >= self.maneuver_timeout_ms:
                self._finish("TARGET_LINE_NOT_FOUND")
        else:  # LANE_FOLLOW_TARGET
            self._motion_estimate(now)
            if self.sensor.all_black(self.black_threshold):
                self._finish("STOP_BAR")
            elif centroid is None or total_weight < self.target_threshold:
                self._finish("LINE_LOST")
            else:
                self._line_control(centroid)

    def run(self):
        while True:
            now = utime.ticks_ms()
            if self.box["system"]["shutdown"]:
                self._stop_wheels()
                self.state = WAIT
            elif self.state == WAIT:
                if self.box["lane_request"]:
                    self._start(now, True)
                elif self.box["request"]:
                    self._start(now, False)
            elif self.box["stop_requested"]:
                self.box["stop_requested"] = False
                self._finish("TEST_ABORTED")
            else:
                try:
                    centroid, total_weight = self.sensor.centroid_mm()
                    self.current_psi = 0.0
                    self.current_yaw_rate = 0.0
                    self.current_left = self.current_right = 0.0
                    if self.state == LINE_TEST:
                        if centroid is None or total_weight < self.target_threshold:
                            self._finish("LINE_LOST")
                        else:
                            self._line_control(centroid)
                            if utime.ticks_diff(now, self.started_ms) >= self.run_duration_ms:
                                self._finish("TEST_COMPLETE")
                    else:
                        self._lane_step(now, centroid, total_weight)
                    if self.state != WAIT:
                        self._log(now, centroid, total_weight)
                except Exception as error:
                    print("Line/IMU control error:", error)
                    self._finish("CONTROL_ERROR")
            yield 0
