import serial
from typing import Final
import threading
from time import sleep, time
import csv
import re
import os
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
    BEAM_THICKNESS: Final[float] = 16.20  # ie thickeness of ruler
    BEAM_END: Final[float] = BEAM_START + BEAM_THICKNESS

    BEAM_EDGE_Y_RELATIVE_TO_NOZZLE: Final[float] = 56.75
    WHISKER_X: Final[float] = 150.0  # set the x distance of the whisker
    WHISKER_TIP_Y: Final[float] = 35  # from the base of the printer bed
    WHISKER_LENGTH_Y: Final[float] = 150
    WHISKER_TIP_TEST_POS: Final[float] = 5

    BEAM_TOUCH_WHISKER_LEFT = WHISKER_X - BEAM_END
    BEAM_TOUCH_WHISKER_RIGHT = WHISKER_X + BEAM_END
    X_PADDING = 20
    Y_PADDING = 20

    LOG_FOLDER: Final[str] = os.path.dirname(os.path.abspath(__file__)) + "/log/"
    LOG_FILE: str = LOG_FOLDER + "log.csv"

    FEEDBACK_CODES = {
        0: "SAFE",
        1: "HIGH MAGENETIC RESPONSE",
        2: "HIGH VOLTAGE RESPONSE",
        999: "UNSAFE",
    }

    # TODO: ADD THE VALID PRINTER MODES HERE
    PRINTER_MODES = {
        "RELATIVE": "something",
        "ORIGIN": "something else",
    }

    def __init__(self, port=PORT):
        print("-- initiating 3D Printer connection --")
        self.port: str = port
        self.serial: serial.Serial = serial.Serial(self.port, 115200)
        self.calibrated = False
        self.feedback_status = 0

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
        self.initialise_log_file()  # create log file if empty
        # begin collection position information
        self.position_thread = threading.Thread(target=self.get_position_loop).start()

    def initialise_log_file(self):
        if not os.path.isfile(self.LOG_FILE) or os.path.getsize(self.LOG_FILE) == 0:
            # File does not exist or is empty, so write the header

            with open(self.LOG_FILE, mode="w", newline="") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writer.writeheader()

    def read_and_store_serial(self):
        while self.serial.in_waiting:
            self.printer_status = self.serial.readline().decode().strip()
            with open(self.LOG_FILE, mode="a") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writeable_data = self.gcode_parser()
                if writeable_data:
                    writer.writerow(self.log_data)

    def get_position_loop(self) -> None:
        """
        Returns the current position of the print head
        """
        while True:
            if self.current_gcode != self.previous_gcode or not self.at_goal:
                if self.gcode_is_available:
                    # FIXME: if i want to sleep then use a timer callback
                    sleep(1 / self.READING_FREQUENCY)  # freq is set by the printer
                    self.send_gcode("M114:")
                self.read_and_store_serial()

    def beam_test_prepare(self, *direction: str) -> None:
        self.send_movement(
            y=self.WHISKER_TIP_Y + self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE - self.Y_PADDING
        )

        if self.left_of_whisker or self.inline_of_whisker or direction == "left":
            self.send_movement(
                x=self.BEAM_TOUCH_WHISKER_LEFT - self.X_PADDING,
            )
        elif self.right_of_whisker or direction == "right":
            self.send_movement(
                x=self.BEAM_TOUCH_WHISKER_RIGHT + self.X_PADDING,
            )
        else:
            raise ValueError("Preparation position unclear")
        self.send_movement(
            y=self.WHISKER_TIP_Y
            + self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE
            + self.WHISKER_TIP_TEST_POS
        )

    def increments_beam_test(
        self,
        total_x_distance: float = 10,
        total_y_distance: float = 10,
        increments_x: int = 1,
        increments_y: int = 1,
        pause_sec: int = 3,
    ) -> None:
        if right_of_whisker:
            raise ValueError("Test Cannot be performed on the right of the whisker")
        # -- prepare printer position for the test --
        inc_dist_x = (
            total_x_distance if increments_x == 0 else total_x_distance / increments_x
        )
        inc_dist_y = (
            total_y_distance if increments_y == 0 else total_y_distance / increments_y
        )
        starting_y_pos = (
            self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE
            + self.WHISKER_TIP_Y
            + self.WHISKER_TIP_TEST_POS
        )
        deflect_x_distance = 0

        self.send_message("Preparing for increment test")
        self.beam_test_prepare()
        sleep(5)
        self.send_message("Begin increment test")
        print_to_stdout("Begin increment test")

        deflect_y_pos = starting_y_pos
        for _ in range(increments_y + 1):
            self.send_movement(x=self.WHISKER_X - self.X_PADDING)
            self.send_movement(x=self.WHISKER_X - self.X_PADDING, y=deflect_y_pos)
            deflect_x_distance = inc_dist_x
            for _ in range(increments_x + 1):
                print_to_stdout(self.data_for_std_out)
                self.send_movement(
                    x=self.WHISKER_X + deflect_x_distance, y=deflect_y_pos
                )
                sleep(pause_sec)
                deflect_x_distance += inc_dist_x

            print_to_stdout(f"Maximum deflection:{self.data_for_std_out}")
            deflect_y_pos += inc_dist_y
            sleep(3)
        self.send_message("Returning to start")
        self.beam_test_prepare()
        self.send_message("Increment test complete")
        sleep(5)
        self.send_message("yo yo, what next?")

    def increments_beam_test_reverse(
        self,
        total_x_distance: float = 10,
        total_y_distance: float = 10,
        increments_x: int = 1,
        increments_y: int = 1,
        pause_sec: int = 3,
    ) -> None:
        if self.left_of_whisker:
            raise ValueError("Test Cannot be performed on the left of the whisker")
        # -- prepare printer position for the test --
        inc_dist_x = -(
            total_x_distance if increments_x == 0 else total_x_distance / increments_x
        )
        inc_dist_y = -(
            total_y_distance if increments_y == 0 else total_y_distance / increments_y
        )
        starting_y_pos = (
            self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE
            + self.WHISKER_TIP_Y
            + self.WHISKER_TIP_TEST_POS
        )
        deflect_x_distance = 0

        self.send_message("Preparing for reverse increment test")
        # if (self.left_of_whisker):
        self.beam_test_prepare()
        if self.left_of_whisker:
            self.u_motion_around_whisker()
        self.beam_test_prepare()
        sleep(5)

        self.send_message("Begin reverse increment test")
        print_to_stdout("Begin reverse increment test")

        deflect_y_pos = starting_y_pos
        for _ in range(increments_y + 1):
            self.send_movement(x=self.WHISKER_X + self.X_PADDING)
            self.send_movement(x=self.WHISKER_X + self.X_PADDING, y=deflect_y_pos)
            deflect_x_distance = inc_dist_x
            for _ in range(increments_x + 1):
                print_to_stdout(self.data_for_std_out)
                self.send_movement(
                    x=self.WHISKER_X + deflect_x_distance, y=deflect_y_pos
                )
                sleep(pause_sec)
                deflect_x_distance += inc_dist_x
            print_to_stdout(f"Maximum deflection:{self.data_for_std_out}")
            deflect_y_pos += inc_dist_y
            sleep(3)
        self.send_message("Returning to start")
        self.beam_test_prepare()
        self.send_message("Reverse increment test complete")
        sleep(5)
        self.send_message("yo yo, what next?")

    #########################################
    ############## UTILITY ##################
    #########################################

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

    def raise_between_error(self, label: str, value: float, min: float, max: float):
        """
        validation check to make sure the the gcode is safe to output
        """
        if value <= min or value >= max:
            raise ValueError(f"Make sure {label} is between {min} and {max}")

    #########################################
    ############## ACTIONS ##################
    #########################################

    def send_gcode(self, gcode) -> None:
        """
        writes gcode to the printer
        """
        self.previous_gcode = self.current_gcode
        self.current_gcode = gcode.encode()

        while True:
            if self.gcode_is_available:
                self.serial.write(self.current_gcode)
                break

    def set_speed(self, speed) -> None:
        """
        sets the speed of the printer
        """
        set_speed_gcode = self.gcode(f=speed)
        self.printer_speed = speed
        self.send_gcode(set_speed_gcode)

    def send_movement(self, x=None, y=None, z=None, f=None):
        if not self.calibrated:
            print_to_stdout("Please Calibrate/Prepare the Printer First")
            self.send_message("Please run prepare()")

        gcode = self.gcode(x, y, z, f)
        while not self.at_goal:
            sleep(1)
        self.send_gcode(gcode)

    def send_wait(self, m_sec) -> None:
        """this sucks, don't use it"""
        self.send_gcode(f"G4 P{m_sec}:")
        sleep(m_sec / 1000)

    def send_message(self, message) -> None:
        self.send_gcode(f"M117 {message}:")
        self.current_message = message

    def set_printer_mode(self, type) -> None:
        if type not in self.PRINTER_MODES:
            raise ValueError("This is not a valid printer mode type")
        self.send_gcode(f"")
        self.current_mode

    def prepare(self) -> None:
        """
        Calibrates X and Y axis on the 3D Printer
        """
        self.send_gcode("G21:")  # setting the printer to mm
        self.send_gcode("G28 XY:")  # homing X and Y of the printer
        self.set_speed(self.printer_speed)
        self.calibrated = True

    def progressive_gcode_move(
        self, input_goal_x=None, input_goal_y=None, input_goal_z=None, f=None
    ) -> str:
        """TODO: in progress"""
        # put printiner into relative mode
        self.send_gcode("TODO: USE RELATIVE MODE")
        origin_x = self.x
        origin_y = self.y
        origin_z = self.z

        # caculating the absolute distance values
        x_dist = input_goal_x - origin_x if input_goal_x else 0
        y_dist = input_goal_y - origin_y if input_goal_y else 0
        z_dist = input_goal_z - origin_z if input_goal_z else 0

        # TODO: calculate the vector to travel a 1mm in 3d space eg:
        step_count, [step_x, step_y, step_z] = self.calculate_distance_per_gcode(
            inc_dist, [x_dist, y_dist, z_dist]
        )

        for _ in range(step_count):
            if self.feedback_status != 0:
                break
            self.send_movement(step_x, step_y, step_z)

        # if the printer when into a non safe status
        if self.feedback_status != 0:
            # recovery action
            # TODO: get OUT of relative mode
            print("placeholder action")

    def calculate_distance_per_gcode(self, increment_distance = 1, x_dist, y_dist, z_dist):
        

        # FIXME: need to do actual calc, this is an exmaple
        return 50, [0.1, 0.6, 0.3]

    def u_motion_around_whisker(self):
        # make sure beam is in the correct position
        self.beam_test_prepare()
        last_y = self.y
        neg = -1 if self.right_of_whisker else 1  # for bi-directional usage

        # move beam away from the whisker in X
        self.send_movement(x=self.x - (neg * self.X_PADDING))
        # move beam beyon the whisker in Y
        self.send_movement(
            y=self.WHISKER_TIP_Y + self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE - self.Y_PADDING
        )
        # move beam past the whisker in X
        self.send_movement(
            x=self.WHISKER_X + neg * (self.BEAM_THICKNESS + self.X_PADDING)
        )
        self.send_movement(y=last_y)

    #########################################
    ############# PROPERTIES ################
    #########################################

    @property
    def data_for_std_out(self) -> dict:
        data = {}
        data["timestamp"] = time()
        data["distance_on_shaft"] = self.beam_from_whisker_tip_y
        data["bend_distance"] = self.x - self.WHISKER_X
        return data

    @property
    def beam_from_whisker_tip_x(self) -> float:
        return self.WHISKER_X - (self.x + self.BEAM_END)

    @property
    def beam_from_whisker_tip_y(self) -> float:
        return self.y - self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE - self.WHISKER_TIP_Y

    @property
    def left_of_whisker(self) -> bool:
        return self.beam_from_whisker_tip_x > 0

    @property
    def right_of_whisker(self) -> bool:
        return self.beam_from_whisker_tip_x < 0

    @property
    def inline_of_whisker(self) -> bool:
        return self.beam_from_whisker_tip_x == 0

    @property
    def beam_from_whisker_tip(self) -> tuple:
        beam_x = self.beam_from_whisker_tip_x
        beam_y = self.beam_from_whisker_tip_y
        return beam_x, beam_y

    @property
    def at_goal(self) -> bool:
        x_at_goal = self.x == self.prev_x
        y_at_goal = self.y == self.prev_y
        z_at_goal = self.z == self.prev_z
        return x_at_goal and y_at_goal and z_at_goal

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
    def log_data(self):
        """
        TODO: when file is initialised it might be a good idea to validate the headings
        match the log_data
        """
        log_data = {
            "timestamp": int(time() * 1000),
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "f": self.printer_speed,
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "goal_z": self.goal_z,
        }
        return log_data


def print_to_stdout(*a):
    print(*a, file=sys.stdout)
    with open("test_data.txt", mode="a") as file:
        file.write("\n" + str(*a))


# Y91 tip of whisker with ruler
if __name__ == "__main__":
    gcode = GcodeController()
    gcode.prepare()
    controller.increment_beam_test(6, 15, 2, 2, 3)

    print("complete")
