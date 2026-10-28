"""Calibrated 13-channel Pololu QTR-HD-13A reflectance-array driver.

Pin order is QTR printed channels 1 through 13. It is a proposed wiring map,
not a substitute for checking the physical channel order with the bar-graph test.
PB1 is deliberately reserved for the required battery-voltage ADC input.
"""
from array import array
from pyb import ADC, Pin, Timer
from line_calibration import load, save

QTR_CHANNELS = tuple(range(1, 14))
# ADC-capable pins not used by the motors or corrected encoder pins.
# Order: QTR channels 1 through 13. PF3/PF4/PF5 are not exposed on this board.
QTR_PINS = (Pin.cpu.A0, Pin.cpu.A1, Pin.cpu.A4, Pin.cpu.B0,
            Pin.cpu.C1, Pin.cpu.C0, Pin.cpu.A7, Pin.cpu.A6,
            Pin.cpu.A5, Pin.cpu.C2, Pin.cpu.C3, Pin.cpu.C4,
            Pin.cpu.C5)
LATERAL_MM = tuple(-24 + 4 * index for index in range(13))


class LineSensor:
    def __init__(self, pins=QTR_PINS, channels=QTR_CHANNELS, samples=10,
                 sample_rate_hz=5000):
        if len(pins) != 13 or len(channels) != 13:
            raise ValueError("Exactly 13 sensor pins and channel labels are required.")
        if sample_rate_hz * len(pins) > 65000:
            raise ValueError("ADC aggregate rate must not exceed 65,000 readings/s.")
        self.channels = tuple(channels)
        self.adcs = tuple(ADC(pin) for pin in pins)
        # TIM3 and TIM4 are used by the corrected left/right encoder pairs.
        self.timer = Timer(5, freq=sample_rate_hz)
        self.buffers = tuple(array("H", [0] * samples) for _ in pins)
        self.readings = [0.0] * len(pins)
        self.weights = [0.0] * len(pins)
        self.black = None
        self.white = None
        self.min_span = None

    def read_averages(self):
        # False means an ADC acquisition overrun; do not control from stale data.
        if not ADC.read_timed_multi(self.adcs, self.buffers, self.timer):
            raise RuntimeError("ADC acquisition overrun")
        for index, buffer in enumerate(self.buffers):
            self.readings[index] = sum(buffer) / len(buffer)
        return self.readings

    def capture_calibration(self, filename="line_calibration.json", min_span=100):
        print("Place ALL sensors over BLACK, then press any key.")
        input()
        black = list(self.read_averages())
        print("Place ALL sensors over WHITE, then press any key.")
        input()
        white = list(self.read_averages())
        save(filename, self.channels, black, white, min_span)
        self.set_calibration(load(filename, self.channels))
        print("Saved {}.".format(filename))

    def load_calibration(self, filename="line_calibration.json"):
        self.set_calibration(load(filename, self.channels))

    def set_calibration(self, calibration):
        self.black = tuple(calibration["black"])
        self.white = tuple(calibration["white"])
        self.min_span = calibration["min_span"]

    def centroid_mm(self):
        if self.black is None:
            raise RuntimeError("Line sensor has not been calibrated.")
        readings = self.read_averages()
        total_weight = 0.0
        weighted_position = 0.0
        for index, reading in enumerate(readings):
            span = self.black[index] - self.white[index]
            if span < self.min_span:
                raise RuntimeError("Calibration span became invalid.")
            weight = max(0.0, min(1.0, (reading - self.white[index]) / span))
            self.weights[index] = weight
            total_weight += weight
            weighted_position += LATERAL_MM[index] * weight
        return (weighted_position / total_weight if total_weight > 0 else None,
                total_weight)

    def all_black(self, threshold=0.70): 
        return all(weight >= threshold for weight in self.weights)
