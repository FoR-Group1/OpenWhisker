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
from geometry_msgs.msg import PoseStamped
from printer_driver_node.GcodeController import GcodeController
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from printer_driver_node.utils import quaternion_from_euler
from whisker_interfaces.srv import IncrementsBeamTest
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class PrinterDriverNode(Node):
    def __init__(self):
        super().__init__("printer_driver_node")

        self.declare_parameter(
            "serial_device",
            "/dev/ttyACM0",
        )

        self.declare_parameter(
            "location_polling_rate_hz",
            10.0,
        )

        serial_device = (
            self.get_parameter("serial_device").get_parameter_value().string_value
        )
        self.get_logger().info(f"Opening serial device: {serial_device}")
        self.controller = GcodeController(port=serial_device)
        self.get_logger().info("Printer ready")

        self.location_poll_cbg = MutuallyExclusiveCallbackGroup()
        self.location_poll_timer = self.create_timer(
            1
            / self.get_parameter("location_polling_rate_hz")
            .get_parameter_value()
            .double_value,
            self.location_poll_timer_callback,
            callback_group=self.location_poll_cbg,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "printer_base_link"
        tf.child_frame_id = "whisker_base_link"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, 0)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        self.static_tf_broadcaster.sendTransform(tf)

        self.increments_beam_test_service_cbg = MutuallyExclusiveCallbackGroup()
        self.increments_beam_test_service = self.create_service(
            IncrementsBeamTest,
            "increments_beam_test",
            self.increments_beam_test_callback,
            callback_group=self.increments_beam_test_service_cbg,
        )

    def location_poll_timer_callback(self):
        # ps = PoseStamped()
        # ps.pose.position.x = float(self.controller.x)
        # ps.pose.position.y = float(self.controller.y)
        # ps.pose.position.z = float(self.controller.z)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "printer_head_link"
        t.child_frame_id = "printer_base_link"  # defined as the origin of the printer
        t.transform.translation.x = float(self.controller.x)
        t.transform.translation.y = float(self.controller.y)
        t.transform.translation.z = float(self.controller.z)
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def increments_beam_test_callback(self, request, response):
        self.controller.increments_beam_test(
            total_x_distance=request.total_x_distance,
            total_y_distance=request.total_y_distance,
            increments_x=request.increments_x,
            increments_y=request.increments_y,
            pause_sec=request.pause_sec,
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    printer_driver_node = PrinterDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(printer_driver_node)
    executor.spin()
    printer_driver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
