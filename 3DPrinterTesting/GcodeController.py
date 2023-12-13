import serial
from typing import Final
import threading
from time import sleep, time
import csv
import re
import os
import queue


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

    BEAM_START: Final[float] = -13.20  # relative to the nozzle x position
    BEAM_THICKNESS: Final[float] = 16.20
    BEAM_END: Final[float] = BEAM_START + BEAM_THICKNESS

    WHISKER_X: Final[float] = 150.0  # set the x distance of the whisker
    WHISKER_TIP_Y: Final[float] = 40
    WHISKER_LENGTH_Y: Final[float] = 150

    LOG_FOLDER: Final[str] = os.path.dirname(os.path.abspath(__file__)) + "/log/"
    LOG_FILE: str = LOG_FOLDER + "log.csv"

    def __init__(self, port=PORT):
        print("-- initiating 3D Printer connection --")
        self.port: str = port
        self.serial: serial.Serial = serial.Serial(self.port, 115200)

        # initialising printer position variables
        self.x: float = 0
        self.y: float = 0
        self.z: float = 0

        self.prev_x: float = 0
        self.prev_y: float = 0
        self.prev_z: float = 0

        self.goal_x: float = 0
        self.goal_y: float = 0
        self.goal_z: float = 0

        self.printer_speed: float = 2000

        # initialising the printer status
        self.printer_status = ""
        self.checked_heading = False
        self.current_gcode = ""

        # initialise pritner configurations
        self.send_gcode("G21:")

        self.gcode_g1_queue = queue.Queue()

        # begins threads to get and log positional data of the nozzle
        self.store_data_thread = threading.Thread(
            target=self.read_and_store_serial
        ).start()
        self.position_thread = threading.Thread(target=self.get_position_loop).start()
        self.process_gcode_queue_thread = threading.Thread(
            target=self.process_gcode_g1_queue
        )

    def read_and_store_serial(self):
        fieldnames = [
            "timestamp",
            "x",
            "y",
            "z",
            "f",
            "goal_x",
            "goal_y",
            "goal_z",
        ]

        if not os.path.isfile(self.LOG_FILE) or os.path.getsize(self.LOG_FILE) == 0:
            # File does not exist or is empty, so write the header
            with open(self.LOG_FILE, mode="w", newline="") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

        with open(self.LOG_FILE, mode="r") as csvfile:
            if not csv.Sniffer().has_header(csvfile.read(1024)):
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

            while True:
                if self.serial.in_waiting:
                    with open(self.LOG_FILE, mode="a") as csvfile:
                        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                        writeable_data = self.gcode_parser()
                        if writeable_data:
                            writer.writerow(
                                {
                                    "timestamp": int(time() * 1000),
                                    "x": self.x,
                                    "y": self.y,
                                    "z": self.z,
                                    "f": self.printer_speed,
                                    "goal_x": self.goal_x,
                                    "goal_y": self.goal_y,
                                    "goal_z": self.goal_z,
                                }
                            )

    def get_position_loop(self) -> None:
        """
        Returns the current position of the print head
        """
        while True:
            if self.gcode_is_available:
                # if i want to sleep then use a timer callback
                sleep(1 / self.READING_FREQUENCY)  # freq is set by the printer
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
                    self.prev_x = self.x
                    self.x = float(position)
                if axis == "Y":
                    self.prev_y = self.y
                    self.y = float(position)
                if axis == "Z":
                    self.prev_z = self.z
                    self.z = float(position)

        return self.printer_status

    @property
    def at_goal(self) -> bool:
        x_at_goal = self.x == self.prev_x
        y_at_goal = self.y == self.prev_y
        z_at_goal = self.z == self.prev_z
        return x_at_goal and y_at_goal and z_at_goal

    def gcode(self, x=None, y=None, z=None, f=None) -> str:
        """
        Method to prepare the gcode with some validation checks
        """
        x = self.x if x is None else x
        y = self.y if y is None else y
        z = self.z if z is None else z
        f = self.printer_speed if f is None else f

        self.raise_between_error("X", x, self.MIN_X, self.MAX_X)
        self.raise_between_error("Y", y, self.MIN_Y, self.MAX_Y)
        self.raise_between_error("Speed", f, self.MIN_SPEED, self.MAX_SPEED)

        return f"G1 X{x} Y{y} Z{z} F{f}"

    # def progressive_gcode_move(self, x=None, y=None, z=None, f=None) -> str:
    #     return f"G1 X{x} Y{y} Z{z} F{f}"

    def raise_between_error(self, label, value, min, max):
        """
        validation check to make sure the the gcode is safe to output
        """
        if value <= min or value >= max:
            raise ValueError(f"Make sure {label} is between {min} and {max}")

    def send_gcode(self, gcode, bypass=False) -> None:
        """
        writes gcode to the printer
        """
        while True:
            # if movement, make sure last goal is reached first
            print("here")
            if "G1" in gcode and not self.at_goal and not bypass:
                self.gcode_g1_queue.put(gcode)
                if not self.process_gcode_queue_thread.is_alive():
                    self.process_gcode_queue_thread.start()
                break
            if self.gcode_is_available:
                self.serial.write(gcode.encode())
                break

    def process_gcode_g1_queue(self):
        while self.gcode_g1_queue.qsize() > 0:
            if self.at_goal:
                self.send_gcode(self.gcode_g1_queue.get(), True)

        # terminate queue

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
        f_gcode = f"G1 F{speed}:"
        self.printer_speed = speed
        self.send_gcode(f_gcode)

    def beam_test_prepare(self) -> None:
        self.send_gcode(self.gcode(x=self.get_prepare_position_x, y=100))

    def send_wait(self, m_sec) -> None:
        """this sucks, don't use it"""
        self.send_gcode(f"G4 P{m_sec}:")
        sleep(m_sec / 1000)

    def send_message(self, message) -> None:
        self.send_gcode(f"M117 {message}:")

    # def std_out_data_gcode(self, gcode):

    def increments_beam_test(
        self,
        total_x_distance: float = 10,
        total_y_distance: float = 10,
        increments_x: int = 5,
        increments_y: int = 0,
        pause_sec: int = 5,
    ) -> None:
        # -- prepare printer position for the test --
        controller = self
        inc_dist_x = total_x_distance / increments_x
        inc_dist_y = total_y_distance / increments_y
        starting_y_pos = 100  # TODO: figure out a standard constant for this value
        deflect_x_distance = inc_dist_x
        deflect_y_pos = starting_y_pos

        controller.send_message("Preparing for increment test")
        controller.beam_test_prepare()
        sleep(5)
        controller.send_message("Begin increment test")

        for _ in range(increments_y + 1):
            controller.send_gcode(controller.gcode(x=self.WHISKER_X))
            while not self.at_goal:
                sleep(1)
            controller.send_gcode(controller.gcode(y=starting_y_pos))
            controller.send_gcode(controller.gcode(x=self.WHISKER_X))
            for _ in range(increments_x + 1):
                controller.send_gcode(
                    controller.gcode(x=self.WHISKER_X + deflect_x_distance)
                )
                sleep(pause_sec)
                while not self.at_goal:
                    sleep(1)
                deflect_x_distance += inc_dist_x

            deflect_y_pos += starting_y_pos

        controller.send_message("Returning to start")
        controller.beam_test_prepare()
        controller.send_message("Increment test complete")
        sleep(5)

        controller.send_message("yo yo, what next?")

        # self.send_wait(pause_sec)
        # set bending location
        # read

    def beam_test(self, count=1, pause=2, x_deflection=None, y_distance=None) -> None:
        self.beam_test_prepare()
        gcode_prepare = self.gcode(x=self.get_prepare_position_x, y=y_distance)
        gcode_bend = self.gcode(x=x_deflection, y=y_distance)
        gcode = gcode_prepare + gcode_bend + f"G4 S{pause}:"
        for _ in range(count):
            gcode += gcode
        self.send_gcode(gcode)

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
        """Uses Regex to determine if the current status is Positional data"""
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


status = [
    "timestamp",
    "distance_on_shaft",
    "displacement from center",
    "do I think i'm touching whisker",
]
