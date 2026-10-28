"""Nonblocking USB-VCP UI.  This task is the only VCP owner."""
import pyb
from pyb import UART 

class TaskUser:
    def __init__(self, left, right, line, system):
        self.left, self.right, self.line, self.system = left, right, line, system
        self.vcp = UART(2, 115_200)
        self.pending = None
        self.numeric = ""
        self.print_menu()

    def print_menu(self):
        print("\n[o] open-loop  [c] P velocity  [d] duty  [s] setpoint  [p] wheel Kp")
        print("[l]/[r] wheel test  [b] line base speed  [k] line Kp  [f] Lab 6 follow-line test")
        print("[g] Lab 7 left lane change [x] stop line following  [h] help  [e] safe exit")

    def _set_value(self, value):
        if self.pending == "d":
            self.left["duty_cycle"] = self.right["duty_cycle"] = value
        elif self.pending == "s":
            self.left["velocity_setpoint"] = self.right["velocity_setpoint"] = value
        elif self.pending == "p":
            self.left["kp"] = self.right["kp"] = value
        elif self.pending == "b":
            self.line["base_speed_rad_s"] = value
        elif self.pending == "k":
            self.line["kp_line_rad_s_per_mm"] = value
        print("Value accepted.")
        self.pending, self.numeric = None, ""

    def _emit_data(self, box):
        if not box["data_ready"]:
            return
        box["data_ready"] = False
        data, count = box["data"], box["sample_count"]
        print("# completion={}".format(box["completion"]))
        print("# mode={}".format(box.get("mode", "LINE")))
        headers = tuple(data.keys())
        print(",".join(headers))
        for index in range(count):
            print(",".join("{:.7g}".format(data[key][index]) for key in headers))
        print(box["completion"])

    def _command(self, character):
        prompts = {"d": "Enter duty cycle [%]:", "s": "Enter velocity setpoint [rad/s]:",
                   "p": "Enter wheel Kp [V/(rad/s)]:", "b": "Enter line base speed [rad/s]:",
                   "k": "Enter line Kp [(rad/s)/mm]:"}
        if character in prompts:
            self.pending, self.numeric = character, ""
            print(prompts[character])
        elif character == "o":
            self.left["mode"] = self.right["mode"] = "OPEN"
            print("Open-loop duty-cycle mode selected")
        elif character == "c":
            self.left["mode"] = self.right["mode"] = "P"
            print("P closed-loop velocity mode selected")
        elif character == "l" and not self.line["active"]:
            self.left["request"] = True
        elif character == "r" and not self.line["active"]:
            self.right["request"] = True
        elif character == "f":
            self.line["request"] = True
            print("Line-following test requested")
        elif character == "g": 
            if not self.line["active"]: 
                self.line["lane_request"] = True 
                print("Lane-changed maneuver requested")
        elif character == "x":
            self.line["stop_requested"] = True
        elif character == "h":
            self.print_menu()
        elif character == "e":
            self.line["stop_requested"] = True
            self.system["shutdown"] = True

    def run(self):
        while True:
            self._emit_data(self.left)
            self._emit_data(self.right)
            self._emit_data(self.line)
            if self.vcp.any():
                raw = self.vcp.read(1)
                if raw:
                    char = raw.decode().lower()
                    if self.pending:
                        if char in "\r\n":
                            try:
                                self._set_value(float(self.numeric))
                            except ValueError:
                                print("Invalid number; try again.")
                                self.numeric = ""
                        elif char in "0123456789.-+e":
                            self.numeric += char
                    else:
                        self._command(char)
            yield 0
