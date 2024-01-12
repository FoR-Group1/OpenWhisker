#! /usr/bin/env python3

# Python libs
import rclpy
from rclpy.node import Node
import serial
import re
import math
import datetime
from collections import deque
from threading import Lock
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
import time
import threading

from sensor_msgs.msg import MagneticField
from whisker_interfaces.msg import MagneticFieldArray
import pickle

import numpy as np
from scipy.signal import butter, filtfilt


class WhiskerDriverNode(Node):
    def __init__(self):
        super().__init__("whisker_driver_node")

        self.declare_parameter(
            "serial_device",
            "/dev/ttyACM0",
        )
        self.declare_parameter(
            "baud_rate",
            115200,
        )
        self.declare_parameter(
            "contact_detection_rate_hz",
            10.0,
        )

        self.declare_parameter(
            "whisker_model_path",
            "does/not/exist",
        )

        serial_device = (
            self.get_parameter("serial_device").get_parameter_value().string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.get_logger().info(f"Opening serial device: {serial_device} {baud_rate}")
        self.ser = serial.Serial(
            serial_device,
            baud_rate,
            timeout=1,
        )

        # sensor data serial output regex
        self.pattern = re.compile(r"^\d\d([a-f0-9]{4})([a-f0-9]{4})([a-f0-9]{4})")

        self.magnetometer_reading_publisher = self.create_publisher(
            MagneticFieldArray, "magnetometer_reading", 10
        )
        self.buffer = deque(maxlen=1000)
        self.buffer_lock = Lock()

        period = (
            1
            / self.get_parameter("contact_detection_rate_hz")
            .get_parameter_value()
            .double_value
        )
        self.contact_detector_timer = self.create_timer(
            period, self.contact_detector_callback
        )

        self.contact_detection_publisher = self.create_publisher(
            PoseStamped, "detected_contact", 10
        )

        self.serial_data_thread = threading.Thread(
            target=self.serial_data_thread_function
        )
        self.serial_data_thread.start()
        self.serial_data_thread_shutdown = False

        self.publish_queue = deque(maxlen=1000)
        self.publish_queue_lock = Lock()

        self.params = []
        with open(
            self.get_parameter("whisker_model_path").get_parameter_value().string_value,
            "rb",
        ) as fd:
            self.params = pickle.load(fd)

        # de-noise filter params
        cutoff = 2  # desired cutoff frequency of the filter, Hz
        fs = 850  # sample rate, Hz
        order = 2  # quadratic
        nyq = 0.5 * fs  # Nyquist Frequency
        normal_cutoff = cutoff / nyq
        # Get the filter coefficients of a low pass filter
        b, a = butter(order, normal_cutoff, btype="low", analog=False)
        self.filter_params = [b, a]

        self.SHAFT_LENGTH = 150

    def whisker_model(self, x):
        return (
            self.params[0] * (x**3)
            + self.params[1] * (x**2)
            + self.params[2] * x
            + self.params[3]
        )

    def serial_data_thread_function(self):
        self.get_logger().info("serial_data_thread_function")
        while not self.serial_data_thread_shutdown:
            line = self.ser.readline().decode("utf-8")
            # self.get_logger().info("line" + line)

            stamp = time.time()
            match = self.pattern.search(line)
            if match is not None:
                x, y, z = match.groups()
                # self.get_logger().info(f"{x} {y} {z}")
                msg = MagneticField()
                # msg.header.frame_id = "magnetometer"
                msg.header.stamp.sec = math.floor(stamp)
                msg.header.stamp.nanosec = round(stamp % 1 * 1e9)
                msg.magnetic_field.x = float(int(x, 16))
                msg.magnetic_field.y = float(int(y, 16))
                msg.magnetic_field.z = float(int(z, 16))
                with self.publish_queue_lock:
                    self.publish_queue.append(msg)

                with self.buffer_lock:
                    self.buffer.append(msg)

    def butter_lowpass_filter(self, data):
        return filtfilt(self.filter_params[0], self.filter_params[1], data)

    def contact_detector_callback(self):
        self.get_logger().info(f"contact_detector_callback {len(self.publish_queue)}")

        # publish raw magnetometer readings
        with self.publish_queue_lock:
            mfa = MagneticFieldArray()
            for mf in self.publish_queue:
                mfa.magnetic_field_array.append(mf)
            self.magnetometer_reading_publisher.publish(mfa)
            self.publish_queue.clear()

        buffer_copy = None
        with self.buffer_lock:
            buffer_copy = deepcopy(self.buffer)

        if len(buffer_copy) > 200:
            t_data = [
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                for msg in buffer_copy
            ]

            # de-noise
            y_filtered = self.butter_lowpass_filter(
                [msg.magnetic_field.y for msg in buffer_copy]
            )

            # calculate first derivative
            y_grad = np.gradient(y_filtered)

            # find an episode in the data where there is a rising edge followed by a falling edge
            episodes = []
            open_episode = True
            episode_start = 0
            steady_state = False
            for idx, pt in enumerate(y_grad):
                # first detect a steady state
                if idx > 5 and np.abs(np.average(y_grad[idx - 5 : idx])) < 5:
                    steady_state = True

                # then detect a large change from steady state
                if steady_state:
                    if open_episode:
                        if pt < -10:
                            # found start of episode
                            open_episode = False
                            steady_state = False
                            episode_start = t_data[idx]
                        else:
                            continue
                    else:
                        if pt > 10:
                            # found end of episode
                            open_episode = True
                            episodes.append((episode_start, t_data[idx]))
                            break
                        else:
                            continue

            if len(episodes) > 0:
                start, end = episodes[0]
                self.get_logger().info(f"contact detected {start} {end}")
                # Find the max gradient within the episode
                episode_grad_data = [
                    (grad, t)
                    for grad, t in zip(y_grad, t_data)
                    if t >= start and t <= end
                ]
                min_index = np.argmin([grad for grad, t in episode_grad_data])
                min_grad, min_grad_t = episode_grad_data[min_index]

                shaft_distance_from_tip = self.whisker_model(min_grad)

                # publish contact detection
                msg = PoseStamped()
                msg.header.frame_id = "whisker_base_link"
                msg.header.stamp.sec = math.floor(min_grad_t)
                msg.header.stamp.nanosec = round(min_grad_t % 1 * 1e9)
                msg.pose.position.x = 0.0
                msg.pose.position.y = self.SHAFT_LENGTH - shaft_distance_from_tip
                msg.pose.position.z = 0.0

                self.contact_detection_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    whisker_driver_node = WhiskerDriverNode()
    rclpy.spin(whisker_driver_node)
    whisker_driver_node.serial_data_thread_shutdown = True
    whisker_driver_node.serial_data_thread.join()
    whisker_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
