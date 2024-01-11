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

class WiskerDriverNode(Node):
    def __init__(self):
        super().__init__("wisker_driver_node")

        self.declare_parameter(
            "serial_device",
            "/dev/ttyACM0",
        )
        self.declare_parameter(
            "baud_rate",
            115200,
        )
        self.declare_parameter(
            "classifier_rate_hz",
            10.0,
        )

        serial_device = self.get_parameter("serial_device").get_parameter_value().string_value
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.get_logger().info(f"Opening serial device: {serial_device} {baud_rate}")
        self.ser = serial.Serial(
            serial_device,
            baud_rate,
            timeout=1,
        )

        # sensor data serial output regex
        self.pattern = re.compile(
            r"^\d\d([a-f0-9]{4})([a-f0-9]{4})([a-f0-9]{4})"
        )

        self.magnetometer_reading_publisher = self.create_publisher(
            MagneticFieldArray, "magnetometer_reading", 10
        )
        self.buffer = deque(maxlen=1000)
        self.buffer_lock = Lock()

        period = 1/self.get_parameter("classifier_rate_hz").get_parameter_value().double_value
        self.classifier_timer = self.create_timer(period, self.classifier_callback)
        # self.classifier = Classifier()

        self.serial_data_thread = threading.Thread(target=self.serial_data_thread_function)
        self.serial_data_thread.start()
        self.serial_data_thread_shutdown = False

        self.publish_queue = deque(maxlen=1000)
        self.publish_queue_lock = Lock()

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

    def classifier_callback(self):
        self.get_logger().info(f"classifier_callback {len(self.publish_queue)}")

        with self.publish_queue_lock:
            mfa = MagneticFieldArray()
            for mf in self.publish_queue:
                mfa.magnetic_field_array.append(mf)
            self.magnetometer_reading_publisher.publish(mfa)
            self.publish_queue.clear()

        # buffer_copy = None
        # with self.buffer_lock:
        #     buffer_copy = deepcopy(self.buffer)
        
        # d = self.classifier.update(buffer_copy)

        



def main(args=None):
    rclpy.init(args=args)
    waypoints_mission_node = WiskerDriverNode()
    rclpy.spin(waypoints_mission_node)
    waypoints_mission_node.serial_data_thread_shutdown = True
    waypoints_mission_node.serial_data_thread.join()
    waypoints_mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()