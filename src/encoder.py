"""Timer quadrature encoder driver with rollover-safe position and velocity."""
from math import pi
from pyb import Timer
from time import ticks_diff, ticks_us

COUNTS_PER_WHEEL_REV = 1440
RAD_PER_COUNT = 2 * pi / COUNTS_PER_WHEEL_REV


class Encoder:
    def __init__(self, timer, channel_a_pin, channel_b_pin):
        timer.channel(1, Timer.ENC_AB, pin=channel_a_pin)
        timer.channel(2, Timer.ENC_AB, pin=channel_b_pin)
        self._timer = timer
        self._modulus = timer.period() + 1
        self._half_modulus = self._modulus // 2
        self._previous_count = timer.counter()
        self._previous_time_us = ticks_us()
        self._position_counts = 0
        self._velocity_rad_s = 0.0

    def update(self):
        now = ticks_us()
        count = self._timer.counter()
        delta = count - self._previous_count
        if delta > self._half_modulus:
            delta -= self._modulus
        elif delta < -self._half_modulus:
            delta += self._modulus
        dt_us = ticks_diff(now, self._previous_time_us)
        self._position_counts += delta
        self._velocity_rad_s = (delta * RAD_PER_COUNT * 1_000_000 / dt_us
                                if dt_us > 0 else 0.0)
        self._previous_count = count
        self._previous_time_us = now

    def get_position(self):
        return self._position_counts * RAD_PER_COUNT

    def get_velocity(self):
        return self._velocity_rad_s

    def zero(self):
        self._position_counts = 0
        self._velocity_rad_s = 0.0
        self._previous_count = self._timer.counter()
        self._previous_time_us = ticks_us()
