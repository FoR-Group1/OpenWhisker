# Visual Calibrator

We originally planned to use a raspberry pi camera for this but it has a too slow framerate and adds a lot of latency. Instead I'm now using a cheap random usb webcam and `usb_cam` to make this into ROS messages directly. These cameras need calibration which I will do shortly.

TODO:
- Increase the resolution- I currently am only running at 640x480 when it should be capable of 1280x720 (See `v4l2-ctl --device=0 --list-formats-ext` and `guvcview /dev/video0`. Then calibrate again.
