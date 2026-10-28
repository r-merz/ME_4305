"""JSON helpers for the QTR-HD-13A black/white calibration."""
import json


def save(filename, channels, black, white, min_span):
    with open(filename, "w") as file:
        json.dump({"channels": list(channels), "black": list(black),
                   "white": list(white), "min_span": min_span}, file)


def load(filename, expected_channels):
    with open(filename, "r") as file:
        calibration = json.load(file)
    required = ("channels", "black", "white", "min_span")
    if any(key not in calibration for key in required):
        raise ValueError("Calibration file is missing required entries.")
    if list(calibration["channels"]) != list(expected_channels):
        raise ValueError("Calibration channel order does not match this firmware.")
    if len(calibration["black"]) != len(expected_channels) or \
            len(calibration["white"]) != len(expected_channels):
        raise ValueError("Calibration length is not 13 channels.")
    if calibration["min_span"] <= 0:
        raise ValueError("min_span must be positive.")
    for black, white in zip(calibration["black"], calibration["white"]):
        if black - white < calibration["min_span"]:
            raise ValueError("Bad calibration: black must exceed white by min_span.")
    return calibration
