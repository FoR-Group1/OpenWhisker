import cv2
import numpy

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
for i in range(24, 28):
    tag = numpy.zeros((500, 500, 1), dtype="uint8")
    cv2.aruco.drawMarker(dictionary, i, 500, tag, 1)
    cv2.imwrite("%d.png" % i, tag)
