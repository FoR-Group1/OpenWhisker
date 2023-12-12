import serial
from typing import Final
import threading
from time import sleep, time
import csv
import re


class GcodeController:
    # Constants
    PORT: Final[str] = "/dev/ttyACM2"  # printer connection port
    MIN_SPEED: Final[int] = 1000  # minimum allowed speed for the nozzle
    MAX_SPEED: Final[int] = 15000  # maximum allowed speed for the nozzle
    MIN_X: Final[int] = 0
    MAX_X: Final[int] = 250
    MIN_Y: Final[int] = 0
    MAX_Y: Final[int] = 210
    # MIN_Z: Final[int] = 0
    # MAX_Z: Final[int] = 250

    READING_FREQUENCY: Final[int] = 100  # loggin rate into 3dPrinterData.csv
    POS_DATA_REGEX = re.compile(r"\bX\b | \bY\b | \bZ\b | \bCount\b", flags=re.I | re.X)
    POS_MATCH = ["X", "Y", "Z", "Count", "X", "Y", "Z"]

    BEAM_START: Final[float] = -5.0  # relative to the nozzle x position
    BEAM_THICKNESS: Final[float] = 35.0

    WHISKER_X: Final[float] = 130.0  # set the x distance of the whisker

    def __init__(self, port=PORT):
        print("-- initiating 3D Printer connection --")
        self.port: str = port
        self.serial: serial.Serial = serial.Serial(self.port, 115200)

        # initialising printer position variables
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0
        self.goal_x: float = 0
        self.goal_y: float = 0
        self.goal_z: float = 0
        self.printer_speed: float = 2000

        # initialising the printer status
        self.printer_status = ""

        # begins threads to get and log positional data of the nozzle
        threading.Thread(target=self.read_and_store_serial).start()
        threading.Thread(target=self.get_position_loop).start()

    def read_and_store_serial(self):
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
    
    def get_position_loop(self) -> None:
        """
        Returns the current position of the print head
        """
        while True:
            if self.gcode_is_available:
                sleep(1 / self.READING_FREQUENCY)
                self.serial.write("M114:".encode())


    def gcode_parser(self):
        """
        Method to extact gcode information returned from the printer
        currently:
            - prevents outputting the 'ok' status as it was found to provide little value
            - extracts the goal position values to the class object
            - extracts the current position values to the class object
        returns and stores the printer_status (ie last readline)
        """
        self.printer_status = self.serial.readline().decode().strip()

        if self.printer_status == "ok":
            return 0

        if self.printer_status_is_position_data:
            goal_coords, current_coords = self.printer_status.split("Count")

            for coord in goal_coords.strip().split(" "):
                axis, position = coord.split(":")
                if axis == "X":
                    self.goal_x = float(position)
                if axis == "Y":
                    self.goal_y = float(position)
                if axis == "Z":
                    self.goal_z = float(position)

            for coord in current_coords.strip().replace(": ", ":").split(" "):
                axis, position = coord.split(":")
                if axis == "X":
                    self.x = float(position)
                if axis == "Y":
                    self.y = float(position)
                if axis == "Z":
                    self.z = float(position)

        return self.printer_status

       def gcode(self, x=None, y=None, z=None, f=None) -> str:
        """
        Method to prepare the gcode with some validation checks
        """
        x = self.x if x is None else x
        y = self.y if y is None else y
        z = self.z if z is None else z
        f = self.x if y is None else y

        self.raise_between_error("X", x, self.MIN_X, self.MAX_X)
        self.raise_between_error("Y", y, self.MIN_Y, self.MAX_Y)
        self.raise_between_error("Speed", f, self.MIN_SPEED, self.MAX_SPEED)

        return f"G1 X{x} Y{y} Z{z} F{f}"

    def raise_between_error(self, label, value, min, max):
        """
        validation check to make sure the the gcode is safe to output
        """
        if value < min or value > max:
            raise ValueError(f"Make sure {label} is between {min} and {max}")

        def send_gcode(self, gcode) -> None:
        """
        writes gcode to the printer
        """
        self.serial.write(gcode.encode())

    def set_prepare_position(self, *Y_POS: float) -> None:
        """
        Sets the position of beam for testing
        """
        gcode = f"G1 X{self.get_prepare_position_x}"
        self.gcode(self.get_prepare_position_x)
        if Y_POS is not None:
            gcode += f"Y {Y_POS}"
        gcode += ":"
        self.send_gcode(gcode)

    def prepare(self) -> None:
        self.send_gcode("G28 XY: G1 Y100 X20:")
        self.set_speed(self.printer_speed)

    def set_speed(self, speed) -> None:
        if speed < self.MIN_SPEED or speed > self.MAX_SPEED:
            raise ValueError(
                f"Set a speed between {self.MIN_SPEED} and {self.MAX_SPEED}"
            )
        self.printer_speed = speed
        self.send_gcode(f"G1 F{self.printer_speed}:")

    def beam_test_prepare(self) -> None:
        self.send_gcode("G1 X100 Y100:")

    def beam_test(self) -> None:
        self.beam_test_prepare()
        self.send_gcode(
            "G1 X115: G1 X90: G1 X115: G1 X90:G1 X115: G1 X90:G1 X115: G1 X90:G1 X115: G1 X90:"
        )

    @property
    def gcode_is_available(self) -> bool:
        """
        property to ensure that gcode isn't sent when it is blocked
        due to processing other things
        """
        if self.printer_status in [
            "Full RX Buffer",
            "echo:busy: paused for user",
            "echo:Unknown command",
            "Invalid M code",
            "Processing",
            "echo:busy: processing",
        ]:
            return False
        return True
    
    @property
    def printer_status_is_position_data(self) -> bool:
        ''' Uses Regex to determine if the current status is Positional data'''
        return self.POS_DATA_REGEX.findall(self.printer_status) == self.POS_MATCH

    @property
    def get_prepare_position_x(self) -> float:
        """
        Evaluates the preparation position of the contact point of rigid beam
        """
        return float(self.WHISKER_X - self.get_beam_contact_position_x)

    @property
    def get_beam_contact_position_x(self) -> float:
        """
        Evaluates the current position of the contact point of rigid beam
        """
        return float(self.BEAM_START + self.BEAM_THICKNESS)

    @property
    def home_xy(self) -> str:
        return "G28 XY:"


if __name__ == "__main__":
    gcode = GcodeController()
    gcode.start_serial_read_thread()
    gcode.start_position_reader_thread()
    gcode.prepare()
    gcode.set_speed(9000)
    gcode.send_gcode("G1 Y110 X100:")

    print("complete")
