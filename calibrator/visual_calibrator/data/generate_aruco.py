import cv2
import numpy
import sys

SIZE = int(sys.argv[1])

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
for i in range(24, 28):
    tag = numpy.zeros((SIZE, SIZE, 1), dtype="uint8")
    cv2.aruco.drawMarker(dictionary, i, SIZE, tag, 1)
    cv2.imwrite("%d_%ipx.png" % (i, SIZE), tag)
