"""Saturated proportional wheel-velocity controller."""


class ClosedLoop:
    def __init__(self, kp=0.0, v_min=-9.0, v_max=9.0):
        self.kp = kp
        self.v_min = v_min
        self.v_max = v_max

    def reset(self):
        """Provided so TaskMotor can reset every controller at mode changes."""

    def update(self, setpoint_rad_s, measurement_rad_s):
        voltage_unlimited = self.kp * (setpoint_rad_s - measurement_rad_s)
        voltage_limited = max(self.v_min, min(self.v_max, voltage_unlimited))
        return voltage_unlimited, voltage_limited



