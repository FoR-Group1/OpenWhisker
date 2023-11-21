We are currently using a Raspberry Pi and a Raspberry Pi camera which pushes the camera to a simple websocket with [pi-h264-to-browser](https://github.com/dans98/pi-h264-to-browser), then we can read this camera feed easily on another laptop easily with something like [ros2_ipcamera](https://github.com/surfertas/ros2_ipcamera). We will see in time if the resolution/framerate is good enough.

## Raspberry Pi

Follow the instructions in the `pi-h264-to-browser` repo and set the configuration:

```python
serverPort = 8000

camera = PiCamera(sensor_mode=2, resolution='1024x768', framerate=20)
camera.video_denoise = False

recordingOptions = {
    'format' : 'h264', 
    'quality' : 20, 
    'profile' : 'high', 
    'level' : '4.2', 
    'intra_period' : 15, 
    'intra_refresh' : 'both', 
    'inline_headers' : True, 
    'sps_timing' : True
}

focusPeakingColor = '1.0, 0.0, 0.0, 1.0'
focusPeakingthreshold = 0.055

centerColor = '255, 0, 0, 1.0'
centerThickness = 2

gridColor = '255, 0, 0, 1.0'
gridThickness = 2
```

20 FPS seems to be the highest framerate it works at. Also there is a fair bit of latency which potentially might be an issue
