import serial
from typing import Final
import threading
from time import sleep, time
import csv
import re
import os
import queue
import sys


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

    BEAM_EDGE_Y_RELATIVE_TO_NOZZLE: Final[float] = 56.75
    WHISKER_X: Final[float] = 150.0  # set the x distance of the whisker
    WHISKER_TIP_Y: Final[float] = 35
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

        self.fieldnames = [
            "timestamp",
            "x",
            "y",
            "z",
            "f",
            "goal_x",
            "goal_y",
            "goal_z",
        ]

        self.printer_speed: float = 2000

        # initialising the printer status
        self.printer_status = ""
        self.checked_heading = False
        self.previous_gcode = ""
        self.current_gcode = ""

        # initialise pritner configurations
        self.send_gcode("G21:")

        self.initialise_log_file()

        self.gcode_g1_queue = queue.Queue()

        # begins threads to get and log positional data of the nozzle
        # self.store_data_thread = threading.Thread(
        #     target=self.read_and_store_serial
        # ).start()
        self.position_thread = threading.Thread(target=self.get_position_loop).start()
        # self.process_gcode_queue_thread = threading.Thread(
        #     target=self.process_gcode_g1_queue
        # )

    def initialise_log_file(self):
        if not os.path.isfile(self.LOG_FILE) or os.path.getsize(self.LOG_FILE) == 0:
            # File does not exist or is empty, so write the header

            with open(self.LOG_FILE, mode="w", newline="") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writer.writeheader()

    def read_and_store_serial(self):
        # print("in read and store")
        while self.serial.in_waiting:
            # print("serial in waiting true")
            self.printer_status = self.serial.readline().decode().strip()
            with open(self.LOG_FILE, mode="a") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writeable_data = self.gcode_parser()
                # print("storing data")
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
            # print("in true get position loop")
            if self.current_gcode != self.previous_gcode or not self.at_goal:
                # print("in not at goal position loop")
                if self.gcode_is_available:
                    # print("gcode avaialbe in loop")
                    # if i want to sleep then use a timer callback
                    sleep(1 / self.READING_FREQUENCY)  # freq is set by the printer
                    self.send_gcode("M114:")
                self.read_and_store_serial()

    def gcode_parser(self):
        """
        Method to extact gcode information returned from the printer
        currently:
            - prevents outputting the 'ok' status as it was found to provide little value
            - extracts the goal position values to the class object
            - extracts the current position values to the class object
        returns and stores the printer_status (ie last readline)
        """
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
        gcode = "G1"
        if x is not None:
            self.raise_between_error("X", x, self.MIN_X, self.MAX_X)
            gcode += f" X{x}"

        if y is not None:
            self.raise_between_error("Y", y, self.MIN_Y, self.MAX_Y)
            gcode += f" Y{y}"

        if z is not None:
            self.raise_between_error("Z", z, self.MIN_X, self.MAX_X)
            gcode += f" Z{z}"

        if f is not None:
            self.raise_between_error("Speed", f, self.MIN_SPEED, self.MAX_SPEED)
            gcode += f" F{f}"

        gcode += ":"
        return gcode

    # def progressive_gcode_move(self, x=None, y=None, z=None, f=None) -> str:
    #     return f"G1 X{x} Y{y} Z{z} F{f}"

    def raise_between_error(self, label: str, value: float, min: float, max: float):
        """
        validation check to make sure the the gcode is safe to output
        """
        if value <= min or value >= max:
            raise ValueError(f"Make sure {label} is between {min} and {max}")

    def send_gcode(self, gcode, bypass=False) -> None:
        """
        writes gcode to the printer
        """
        self.previous_gcode = self.current_gcode
        self.current_gcode = gcode.encode()

        while True:
            # if movement, make sure last goal is reached first
            # print("here")
            # if "G1" in gcode and not self.at_goal and not bypass:
            #     self.gcode_g1_queue.put(gcode)
            #     if not self.process_gcode_queue_thread.is_alive():
            #         self.procesself.printer_status = self.serial.readline().decode().strip()s_gcode_queue_thread.start()
            #     break
            if self.gcode_is_available:
                self.serial.write(self.current_gcode)
                break

    # def process_gcode_g1_queue(self):
    #     while self.gcode_g1_queue.qsize() > 0:
    #         if self.at_goal:
    #             self.send_gcode(self.gcode_g1_queue.get(), True)

    # terminate queue

    def set_prepare_position(self, Y_POS=None) -> None:
        """
        Sets the position of beam for testing
        """
        # gcode = f"G1 X{self.get_prepare_position_x}"
        gcode = self.gcode(x=self.get_prepare_position_x, y=Y_POS)
        # self.gcode(self.get_prepare_position_x)
        # if Y_POS is not None:
        #     gcode += f"Y {Y_POS}"
        # gcode += ":"
        self.send_gcode(gcode)

    def prepare(self) -> None:
        self.send_gcode("G28 XY: G1 Y100 X20:")
        self.set_speed(self.printer_speed)

    def set_speed(self, speed) -> None:
        f_gcode = f"G1 F{speed}:"
        self.printer_speed = speed
        self.send_gcode(f_gcode)

    def beam_test_prepare(self) -> None:
        self.send_gcode(
            self.gcode(
                x=self.get_prepare_position_x - 5,
                y=self.WHISKER_TIP_Y + self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE + 5,
            )
        )

    def send_wait(self, m_sec) -> None:
        """this sucks, don't use it"""
        self.send_gcode(f"G4 P{m_sec}:")
        sleep(m_sec / 1000)

    def send_message(self, message) -> None:
        self.send_gcode(f"M117 {message}:")

    # def std_out_data_gcode(self, gcode):

    @property
    def beam_along_whisker(self) -> float:
        return self.y - self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE - self.WHISKER_TIP_Y

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
        starting_y_pos = self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE + self.WHISKER_TIP_Y + 5
        deflect_x_distance = 0

        controller.send_message("Preparing for increment test")
        controller.beam_test_prepare()
        sleep(5)
        controller.send_message("Begin increment test")
        print_to_stdout("Begin increment test")

        deflect_y_pos = starting_y_pos
        for _ in range(increments_y + 1):
            x_spacing = 10
            y_spacing = 10
            controller.send_gcode(
                controller.gcode(x=self.get_prepare_position_x - x_spacing)
            )
            while not self.at_goal:
                sleep(1)
            controller.send_gcode(controller.gcode(y=deflect_y_pos))
            while not self.at_goal:
                sleep(1)

            deflect_x_distance = inc_dist_x
            for _ in range(increments_x + 1):
                sdt_out_data = {}
                sdt_out_data["timestamp"] = time()
                sdt_out_data["distance_on_shaft"] = self.beam_along_whisker
                sdt_out_data["bend_distance"] = deflect_x_distance
                print_to_stdout(sdt_out_data)

                controller.send_gcode(
                    controller.gcode(
                        x=self.WHISKER_X + deflect_x_distance, y=deflect_y_pos
                    )
                )
                sleep(pause_sec)
                while not self.at_goal:
                    sleep(1)
                deflect_x_distance += inc_dist_x


            sdt_out_data = {}
            sdt_out_data["timestamp"] = time()
            sdt_out_data["distance_on_shaft"] = self.beam_along_whisker
            sdt_out_data["bend_distance"] = deflect_x_distance
            print_to_stdout(f"Maximum deflection:{sdt_out_data}")

            deflect_y_pos += inc_dist_y
            controller.send_gcode(controller.gcode(x=self.WHISKER_X - x_spacing))
            while not self.at_goal:
                sleep(1)
            controller.send_gcode(
                controller.gcode(x=self.WHISKER_X - x_spacing, y=deflect_y_pos)
            )
            while not self.at_goal:
                sleep(1)
            sleep(3)
        controller.send_message("Returning to start")
        controller.beam_test_prepare()
        controller.send_message("Increment test complete")
        sleep(5)
        controller.send_message("yo yo, what next?")

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


def print_to_stdout(*a):
    print(*a, file=sys.stdout)
    with open("test_data.txt", mode="a") as file:
        # writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
        file.write("\n" + str(*a))

    # sys.stdout.write(*a)
    # sys.stdout.flush()


# Y91 tip of whisker with ruler
if __name__ == "__main__":
    gcode = GcodeController()
    gcode.prepare()
    controller.increment_beam_test(6, 15, 2, 2, 3)

    print("complete")
