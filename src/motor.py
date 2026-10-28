"""DRV8838 PWM/direction motor driver for the Romi."""
from pyb import Pin, Timer


class MotorDriver:
    def __init__(self, pwm_pin, direction_pin, sleep_pin, timer, channel):
        self._direction = Pin(direction_pin, Pin.OUT_PP, value=0)
        self._sleep = Pin(sleep_pin, Pin.OUT_PP, value=0)
        self._pwm = timer.channel(channel, Timer.PWM, pin=pwm_pin,
                                  pulse_width_percent=0)

    def set_effort(self, effort_percent):
        effort_percent = max(-100.0, min(100.0, effort_percent))
        self._direction.value(1 if effort_percent < 0 else 0)
        self._pwm.pulse_width_percent(abs(effort_percent))

    def enable(self):
        self.set_effort(0)
        self._sleep.high()

    def disable(self):
        self._pwm.pulse_width_percent(0)
        self._sleep.low()
