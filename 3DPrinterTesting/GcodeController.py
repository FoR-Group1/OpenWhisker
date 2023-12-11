import serial
from typing import Final
import threading
from time import sleep, time
import csv
import re


class GcodeController:
    MIN_SPEED: Final[int] = 1000
    MAX_SPEED: Final[int] = 15000
    READING_FREQUENCY: Final[int] = 50
    POS_DATA_REGEX = re.compile(r"\bX\b | \bY\b | \bZ\b | \bCount\b", flags=re.I | re.X)
    POS_MATCH = ["X", "Y", "Z", "Count", "X", "Y", "Z"]

    def __init__(self, port="/dev/ttyACM0"):
        print("-- initiating 3D Printer connection --")
        self.port = port
        self.serial = serial.Serial(self.port, 115200)
        self.x = 0
        self.y = 0
        self.z = 0
        self.goal_x = 0
        self.goal_y = 0
        self.goal_z = 0
        self.printer_status = ""

        self.start_serial_read_thread()
        self.start_position_reader_thread()

    def start_serial_read_thread(self):
        threading.Thread(target=self.read_serial).start()

    def start_position_reader_thread(self):
        threading.Thread(target=self.position).start()

    def read_serial(self):
        while True:
            if self.serial.in_waiting:
                with open("3dPrinterData.csv", mode="a") as csvfile:
                    fieldnames = ["timestamp", "gcode"]
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    data = self.gcode_parser()
                    if data:
                        writer.writerow(
                            {
                                "timestamp": time(),
                                "gcode": data,
                            }
                        )

    def gcode_parser(self):
        self.printer_status = self.serial.readline().decode().strip()

        if self.printer_status == "ok":
            return 0

        if self.POS_DATA_REGEX.findall(self.printer_status) == self.POS_MATCH:
            goal_coords, current_coords = self.printer_status.split("Count")

            for coord in goal_coords.strip().split(" "):
                axis, position = coord.split(":")
                if axis == "X":
                    self.goal_x = position
                if axis == "Y":
                    self.goal_y = position
                if axis == "Z":
                    self.goal_z = position

            for coord in current_coords.strip().replace(": ", ":").split(" "):
                axis, position = coord.split(":")
                if axis == "X":
                    self.x = position
                if axis == "Y":
                    self.y = position

        return self.printer_status

    def position(self):
        while True:
            if self.gcode_available_available:
                sleep(1 / self.READING_FREQUENCY)
                self.serial.write("M114:".encode())

    @property
    def gcode_available_available(self):
        if self.printer_status in [
            "Full RX Buffer",
            "echo:busy: paused for user",
            "echo:Unknown command",
            "Invalid M code",
        ]:
            return False
        return True

    def send_gcode(self, gcode):
        # if gcode.type() is array:
        self.serial.write(gcode.encode())

    @property
    def is_available(self):
        return NotImplementedError

    @property
    def beam_test(self):
        return "G1 X115: G1 X90: G1 X115: G1 X90:G1 X115: G1 X90:G1 X115: G1 X90:G1 X115: G1 X90:"

    @property
    def home_xy(self):
        return "G28 XY:"

    def prepare(self):
        self.send_gcode("G28 XY: G1 Y100 X20:")
        self.send_gcode(self.set_speed(2000))

    def set_speed(self, speed):
        if speed < self.MIN_SPEED or speed > self.MAX_SPEED:
            raise ValueError(
                f"Set a speed between {self.MIN_SPEED} and {self.MAX_SPEED}"
            )
        return f"G1 F{speed}:"


if __name__ == "__main__":
    gcode = GcodeController()
    gcode.start_serial_read_thread()
    gcode.start_position_reader_thread()
    gcode.prepare()
    gcode.set_speed(9000)
    gcode.send_gcode("G1 Y110 X100:")

    print("complete")
