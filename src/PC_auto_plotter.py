"""Collect and plot Lab 0x05 wheel tests or Lab 0x06 line-following data."""
from datetime import datetime
from pathlib import Path
from time import monotonic
import matplotlib.pyplot as plt
from serial import Serial, SerialException

SERIAL_PORT = "/dev/cu.usbmodem2103"  # Change for your computer.
BAUDRATE, SERIAL_TIMEOUT = 115200, 0.25
OUTPUT_FOLDER = Path.home() / "Desktop" / "ME4305" / "Lab07_results"
TESTS = (("line", "LINE", 6.0, 0.5),)  # target, mode, base/setpoint, Kp
COMPLETE = "TEST_COMPLETE"
WHEEL_KP = 0.7 
FIRST_RESPONSE_TIMEOUT, BETWEEN_LINES_TIMEOUT = 90.0, 5.0


def read_line(port, timeout):
    started = monotonic()
    while monotonic() - started < timeout:
        line = port.readline()
        if line:
            return line.decode("utf-8", errors="replace").strip()
    raise TimeoutError("Timed out waiting for firmware output.")


def wait_for(port, text):
    while True:
        line = read_line(port, FIRST_RESPONSE_TIMEOUT)
        print("Firmware:", line)
        if text in line:
            return


def set_number(port, command, prompt, value):
    port.write(command.encode())
    wait_for(port, prompt)
    port.write((str(value) + "\r\n").encode())
    ##wait_for(port, "Value accepted.")


def start_test(port, target, mode, setpoint, kp):
    if mode == "OPEN":
        port.write(b"o")
        wait_for(port, "Open-loop duty-cycle mode selected")
        set_number(port, "d", "Enter duty cycle", setpoint)
        port.write(b"l" if target == "left" else b"r")
    elif mode == "P":
        port.write(b"c")
        wait_for(port, "P closed-loop velocity mode selected")
        set_number(port, "s", "Enter velocity setpoint", setpoint)
        set_number(port, "p", "Enter wheel Kp", kp)
        port.write(b"l" if target == "left" else b"r")
    elif mode == "LINE":
        set_number(port, "p", "Enter wheel Kp", WHEEL_KP)
        set_number(port, "b", "Enter line base speed", setpoint)
        set_number(port, "k", "Enter line Kp", kp)
        port.write(b"f")
        wait_for(port, "Line-following test requested")
    else:
        raise ValueError("Unsupported mode: " + mode)


def collect(port):
    comments, headers, columns = [], None, None
    last = monotonic()
    while True:
        raw = port.readline()
        if not raw:
            if monotonic() - last > (FIRST_RESPONSE_TIMEOUT if headers is None else BETWEEN_LINES_TIMEOUT):
                raise TimeoutError("Timed out before a complete dataset arrived.")
            continue
        last = monotonic()
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        if line in (COMPLETE, "TEST_ABORTED", "LINE_LOST", "SENSOR_ERROR", "MOTORS_BUSY", "FAILED_BUFFER_FULL"):
            if line != COMPLETE:
                raise RuntimeError("Firmware ended: " + line)
            break
        if line.startswith("#"):
            comments.append(line)
            continue
        values = [field.strip() for field in line.split(",")]
        if headers is None:
            if len(values) < 2:
                print("Firmware:", line)
                continue
            headers, columns = values, [[] for _ in values]
            continue
        if len(values) != len(headers):
            print("Skipped malformed row:", line)
            continue
        try:
            for column, value in zip(columns, values):
                column.append(float(value))
        except ValueError:
            print("Skipped nonnumeric row:", line)
    if headers is None or not columns[0]:
        raise ValueError("No data rows received.")
    return comments, headers, columns


def save_csv(path, comments, headers, columns):
    with path.open("w", encoding="utf-8", newline="") as file:
        file.write("\n".join(comments) + "\n")
        file.write(",".join(headers) + "\n")
        for row in zip(*columns):
            file.write(",".join("{:.8g}".format(value) for value in row) + "\n")


def plot(path, headers, columns, mode, setting, kp):
    data = dict(zip(headers, columns))
    time_s = [value / 1000.0 for value in data["time_ms"]]
    figure, axis = plt.subplots()
    if mode == "LINE":
        axis.plot(time_s, data["centroid_mm"], label="Line centroid")
        axis.axhline(0, color="black", linewidth=0.8, label="Center reference")
        axis.set_ylabel("Centroid [mm]")
        axis.set_title("Line-following centroid; base = {:.2f} rad/s, Kp,line = {:.3f} (rad/s)/mm".format(setting, kp))
    else:
        axis.plot(time_s, data["velocity_rad_s"], label="Measured velocity")
        axis.plot(time_s, data["setpoint_rad_s"], "--", label="Setpoint")
        axis.set_ylabel("Wheel velocity [rad/s]")
        axis.set_title("{} wheel {} response".format("Selected", mode))
    axis.set_xlabel("Time [s]")
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=200)
    plt.close(figure)


def main():
    OUTPUT_FOLDER.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    try:
        with Serial(SERIAL_PORT, BAUDRATE, timeout=SERIAL_TIMEOUT) as port:
            port.reset_input_buffer()
            for target, mode, setting, kp in TESTS:
                start_test(port, target, mode, setting, kp)
                comments, headers, columns = collect(port)
                base = "{}_{}_{}".format(stamp, target, mode.lower())
                csv_path, png_path = OUTPUT_FOLDER / (base + ".csv"), OUTPUT_FOLDER / (base + ".png")
                save_csv(csv_path, comments, headers, columns)
                plot(png_path, headers, columns, mode, setting, kp)
                print("Saved", csv_path, "and", png_path)
    except (SerialException, TimeoutError, RuntimeError, ValueError) as error:
        raise SystemExit("Collection stopped: {}".format(error))


if __name__ == "__main__":
    main()
