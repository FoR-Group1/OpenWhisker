from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import numpy
import cv2

class VisualCalibrator(Node):

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    aruco_params = cv2.aruco.DetectorParameters_create()
    camera_info = None
    aruco_marker_side_length = 0.0425  # TODO: make this a ROS param

    def __init__(self):
        super().__init__('visual_calibrator')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            "/image_raw", 
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera_info",
            self.camera_info_callback,
            10
        )

    def camera_info_callback(self, data):
        self.camera_info = data

    def image_callback(self, data):
        cv2.namedWindow("Image window")
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters = self.aruco_params)
        print("Detected %d markers" % len(corners))

        labelled_image = cv_image.copy()
        cv2.aruco.drawDetectedMarkers(labelled_image, corners, ids)

        if self.camera_info is not None and ids is not None:
            mtx = numpy.reshape(numpy.array(self.camera_info.k), (3, 3))
            dst = numpy.array([self.camera_info.d])

            # mtx = numpy.array([[0, 0, 0], [0, 0, 0], [0, 0, 0]], dtype=float)
            # dst = numpy.array([[0, 0, 0, 0, 0]], dtype=float)

            # print(mtx)
            # print(dst)

            rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.aruco_marker_side_length,
                mtx,
                dst
            )

            for i, id_ in enumerate(ids, 0):

                cv2.aruco.drawAxis(
                    labelled_image,
                    mtx,
                    dst,
                    rvecs[i],
                    tvecs[i],
                    0.05
                )

        cv2.imshow("Image window", labelled_image)
        cv2.waitKey(ord("q"))

def main(args=None):
    rclpy.init(args = args)
    p = VisualCalibrator()
    rclpy.spin(p)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    p.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()