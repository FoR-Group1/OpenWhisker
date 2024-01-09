#! /usr/bin/env python3

# Python libs
import rclpy
from rclpy.node import Node
import serial
import re
import math
from sensor_msgs.msg import MagneticField
import datetime
from collections import deque
from threading import Lock
from copy import deepcopy
from geometry_msgs.msg import PoseStamped()

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
            10,
        )
        self.ser = serial.Serial(
            self.get_parameter("serial_device"),
            self.get_parameter("baud_rate"),
            timeout=1,
        )

        # sensor data serial output regex
        self.pattern = re.compile(
            r"(19\d\d|20\d\d)-(0[1-9]|1[0-2])-(0[1-9]|[12]\d|3[01])T(\d\d):(\d\d):(\d\d.\d+).+SensorSample { x: (\d+), y: (\d+), z: (\d+)"
        )

        self.magnetometer_reading_publisher = self.create_publisher(
            MagneticField, "magnetometer_reading", 10
        )
        self.buffer = deque(maxlen=1000)
        self.buffer_lock = Lock()

        self.classifier_timer = self.create_timer(1/self.get_parameter("classifier_rate_hz"), self.classifier_callback)
        # self.classifier = Classifier()


    def serial_data_thread(self):
        while True:
            line = self.ser.readline().decode("utf-8")
            match = self.pattern.search(line)
            if match is not None:
                YY, MM, DD, h, m, s, x, y, z = match.groups()
                dt = datetime.datetime(
                    int(YY),
                    int(MM),
                    int(DD),
                    int(h),
                    int(m),
                    math.floor(float(s)),
                    math.floor(float(s) % 1 * 1e6),
                )

                msg = MagneticField()
                msg.header.frame_id = "magnetometer"
                msg.header.stamp.sec = math.floor(dt.timestamp())
                msg.header.stamp.nanosec = dt.timestamp() % 1 * 1e9
                msg.magnetic_field.x = int(x)
                msg.magnetic_field.y = int(y)
                msg.magnetic_field.z = int(z)
                self.magnetometer_reading_publisher.publish(msg)

                with self.buffer_lock:
                    self.buffer.append(msg)

    def classifier_callback(self):
        buffer_copy = None
        with self.buffer_lock:
            buffer_copy = deepcopy(self.buffer)
        
        # d = self.classifier.update(buffer_copy)

        



def main(args=None):
    rclpy.init(args=args)
    waypoints_mission_node = WiskerDriverNode()
    rclpy.spin(waypoints_mission_node)
    waypoints_mission_node.close_serial()
    waypoints_mission_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
