# Visual Calibrator

We originally planned to use a raspberry pi camera for this but it has a too slow framerate and adds a lot of latency. Instead I'm now using a cheap random usb webcam and `usb_cam` to make this into ROS messages directly. These cameras need calibration which I will do shortly.

## Calibration 

The camera was calibrated using [the guide at nav2 docs](https://navigation.ros.org/tutorials/docs/camera_calibration.html). It should be recalibrated
if the settings or the physical camera is changed. A printed template was made from [calib.io](https://calib.io/). Comment out the existing `camera_info.yml` then (changing parameters as appropriate):

```
ros2 run camera_calibration cameracalibrator --pattern charuco --size 8x11 --square 0.015 --charuco_marker_size 0.011 --aruco_dict 4x4_250 --ros-args -r image:=/image_raw -p camera:=/test_camera
```

![camera calibration](https://i.imgur.com/9He7T2n.png)

Not all points were recorded on my shitty 640x480 webcam. I think this leads the calibration to be quite bad. That's why I'm looking at using an Xbox Kinect instead

TODO:
- Increase the resolution- I currently am only running at 640x480 when it should be capable of 1280x720 (See `v4l2-ctl --device=0 --list-formats-ext` and `guvcview /dev/video0`. Then calibrate again.

## Pose Estimation

![pose estimation](https://i.imgur.com/93Hd6W4.png)

Pose estimation with aruco images. The 3D printer has 4 aruco images in the corners of the print space. This is the best I could get with the shitty camera.