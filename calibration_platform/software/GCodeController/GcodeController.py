#! /usr/bin/env python3

import serial
from typing import Final
from time import sleep, time
import csv
import re
import os
import sys


class GcodeController:
    # Constants
    PORT: Final[str] = "/dev/ttyACM0"  # printer connection port
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

    X_PADDING = 20
    Y_PADDING = 20

    LOG_FOLDER: Final[str] = os.path.dirname(os.path.abspath(__file__)) + "/log/"
    LOG_FILE: str = LOG_FOLDER + "log.csv"

    def __init__(self, port=PORT):
        print("-- initiating 3D Printer connection --")
        self.port: str = port
        self.serial: serial.Serial = serial.Serial(self.port, 115200)
        self.calibrated = False

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

        # Wait for printer to be ready
        sleep(10)

        # begin collection position information
        # self.position_thread = threading.Thread(target=self.get_position_loop).start()

        # always do a reset on initialisation
        self.prepare()

        print_to_stdout("-- 3D Printer connection established --")

    def initialise_log_file(self):
        if not os.path.isfile(self.LOG_FILE) or os.path.getsize(self.LOG_FILE) == 0:
            # File does not exist or is empty, so write the header
            os.makedirs(self.LOG_FOLDER, exist_ok=True)

            with open(self.LOG_FILE, mode="w", newline="") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writer.writeheader()

    def read_and_store_serial(self):
        while self.serial.in_waiting:
            self.printer_status = self.serial.readline().decode().strip()
            with open(self.LOG_FILE, mode="a") as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.fieldnames)
                writeable_data = self._gcode_parser()
                if writeable_data:
                    writer.writerow(self.log_data)

    def update_position(self) -> None:
        """
        Method to update the current position of the print head
        """
        print_to_stdout("update_position")
        self._send_gcode("M114:")
        self.read_and_store_serial()

    def get_position_loop(self) -> None:
        """
        Returns the current position of the print head
        """
        while True:
            if self.current_gcode != self.previous_gcode or not self.at_goal:
                if self.gcode_is_available:
                    # FIXME: if i want to sleep then use a timer callback
                    sleep(1 / self.READING_FREQUENCY)  # freq is set by the printer
                    self._send_gcode("M114:")
                self.read_and_store_serial()

    def beam_test_prepare(self) -> None:
        self.send_movement(
            x=self.beam_from_whisker_tip_x - 10,
            y=self.WHISKER_TIP_Y + self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE + 5,
        )

    def increments_beam_test(
        self,
        total_x_distance: float = 10,
        total_y_distance: float = 10,
        increments_x: int = 1,
        increments_y: int = 1,
        pause_sec: int = 3,
    ) -> None:
        # -- prepare printer position for the test --
        controller = self
        inc_dist_x = (
            total_x_distance if increments_x == 0 else total_x_distance / increments_x
        )
        inc_dist_y = (
            total_y_distance if increments_y == 0 else total_y_distance / increments_y
        )
        starting_y_pos = self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE + self.WHISKER_TIP_Y + 5
        deflect_x_distance = 0

        controller.send_message("Preparing for increment test")
        controller.beam_test_prepare()
        sleep(5)
        controller.send_message("Begin increment test")
        print_to_stdout("Begin increment test")

        deflect_y_pos = starting_y_pos
        for i in range(increments_y + 1):
            self.send_movement(x=self.WHISKER_X - self.X_PADDING)
            self.send_movement(x=self.WHISKER_X - self.X_PADDING, y=deflect_y_pos)
            deflect_x_distance = inc_dist_x
            print_to_stdout(f"running test {i}: y = {deflect_y_pos}")
            for _ in range(increments_x + 1):
                print_to_stdout(self.data_for_std_out)
                self.send_movement(
                    x=self.WHISKER_X + deflect_x_distance, y=deflect_y_pos
                )
                sleep(pause_sec)
                deflect_x_distance += inc_dist_x

            print_to_stdout(f"Maximum deflection:{self.data_for_std_out}")
            deflect_y_pos += inc_dist_y

        # move away from the beam before returning to start position
        self.send_movement(x=self.WHISKER_X - self.X_PADDING, y=deflect_y_pos)

        # return to start position
        controller.send_message("Returning to start")
        controller.beam_test_prepare()

        controller.send_message("Increment test complete")
        sleep(5)
        controller.send_message("yo yo, what next?")

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

    def _gcode_parser(self):
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

    def _send_gcode(self, gcode) -> None:
        """
        Sends a G-code command to the printer.

        Parameters:
        gcode (string): The G-code command to send to the printer.

        """
        self.previous_gcode = self.current_gcode
        self.current_gcode = gcode.encode()

        while True:
            if self.gcode_is_available:
                self.serial.write(self.current_gcode)
                break

    def set_speed(self, speed) -> None:
        """
        Sets the speed of the printer.

        This function generates a G-code command to set the printer's speed to the specified value. 
        It updates the printer_speed attribute of the object and sends the G-code command to the printer.

        Parameters:
        speed (float): The speed to set the printer to. This should be a positive number representing
        the desired speed in mm/s.

        Returns:
        None
        """
        set_speed_gcode = self.gcode(f=speed)
        self.printer_speed = speed
        self._send_gcode(set_speed_gcode)

    def send_movement(self, x=None, y=None, z=None, f=None) -> None:
        """
        Sends a movement command to the printer.

        Parameters:
        x (float): The x-coordinate to move to. If None, the printer will not move along the x-axis.
        y (float): The y-coordinate to move to. If None, the printer will not move along the y-axis.
        z (float): The z-coordinate to move to. If None, the printer will not move along the z-axis.
        f (float): The speed for the movement. If None, the printer will use its current speed.

        Returns:
        None
        """
        if not self.calibrated:
            print_to_stdout("Please Calibrate/Prepare the Printer First")
            self.send_message("Please run prepare()")

        gcode = self.gcode(x, y, z, f)
        self._send_gcode(gcode)

        done = False
        while not done:
            self.update_position()
            done = self.at_goal

            if done:
                break
            print_to_stdout(
                f"waiting for goal {x}, {y}, {z}, current {self.x}, {self.y}, {self.z}"
            )
            sleep(0.05)

    def send_wait(self, m_sec) -> None:
        """this sucks, don't use it"""
        self._send_gcode(f"G4 P{m_sec}:")
        sleep(m_sec / 1000)

    def send_message(self, message) -> None:
        """
        Sends a message to the printer LCD.

        Parameters:
        message (string): Message to send to the printer display.
        
        Returns:
        None
        """
        self._send_gcode(f"M117 {message}:")

    def prepare(self) -> None:
        """
        Prepares the printer for use.

        This function sends a series of G-code commands to the printer to prepare it for use.
        It sets the printer to use millimeters as its unit of measurement, homes the X and Y axes, 
        and sets the printer speed to the default value.

        Returns:
        None
        """
        self._send_gcode("G21:")  # setting the printer to mm
        self._send_gcode("G28 XY:")  # homing X and Y of the printer
        self.set_speed(self.printer_speed)
        sleep(6)
        self.calibrated = True

    # def progressive_gcode_move(self, x=None, y=None, z=None, f=None) -> str:
    #     ''' TODO: in progress'''
    #     for i in range(x - self.x):
    #     while not se:
    #     return f"G1 X{x} Y{y} Z{z} F{f}"

    def u_motion_around_whisker(self):
        # make sure beam is in the correct position
        self.beam_test_prepare()
        last_y = self.y
        self.send_movement(x=self.x - self.X_PADDING)
        self.send_movement(y=self.WHISKER_TIP_Y - self.Y_PADDING)
        self.send_movement(x=self.WHISKER_X + self.BEAM_THICKNESS + self.X_PADDING)
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
        """
        Evaluates the preparation position of the contact point of rigid beam
        """
        return float(self.WHISKER_X - self.get_beam_contact_position_x)

    @property
    def beam_from_whisker_tip_y(self) -> float:
        return self.y - self.BEAM_EDGE_Y_RELATIVE_TO_NOZZLE - self.WHISKER_TIP_Y

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
    def get_beam_contact_position_x(self) -> float:
        """
        Evaluates the current position of the contact point of rigid beam
        """
        return float(self.BEAM_START + self.BEAM_THICKNESS)

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


def main(port="/dev/ttyACM0"):
    controller = GcodeController(port)
    controller.prepare()
    controller.increments_beam_test(
        total_x_distance=6,
        total_y_distance=100,
        increments_x=0,
        increments_y=10,
        pause_sec=1,
    )
    print("complete")


# Y91 tip of whisker with ruler
if __name__ == "__main__":
    import fire
    fire.Fire(main)
