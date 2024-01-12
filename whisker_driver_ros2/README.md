# Whisker Sensor Driver for ROS2

The driver has the following main components:

- **whisker_driver_node** Interfaces with the whisker sensor micro-controller via a serial port.
Publishes data on the topic `/magnetometer_reading`.
- **printer_driver_node** Interfaces with the 3d printer, drive it to go through a calibration sequence
upon a service call.
- **whisker_interfaces** Message and service definitions for the drivers.

## Getting Started

To gather calibration data, run the following commands:

### Install Dependencies

```sh
> scripts/install_deps.sh
```

### Build

```sh
> colcon build && . install/setup.bash
```

### Run

Launch whisker sensor driver node

```sh
> ros2 run whisker_driver_node whisker_driver_node --ros-args -p serial_device:=/dev/ttyACM0 -p whisker_model_path:=$(pwd)/whisker_driver_ros2/scripts/whisker_model.pkl
```

The whisker model is a parametric model to predict contact point along the whisker from sensor reading.
To fit a new whisker model, see [Data Analysis](#data-analysis) section.

Launch 3d printer driver node

```sh
> ros2 run printer_driver_node printer_driver_node --ros-args -p serial_device:=/dev/ttyACM1
```

Start recording data on all topics into a rosbag

```sh
> ros2 bag record -s mcap --all
```

Issue a service call to 3d printer to start the calibration routine

```sh
> ros2 service call /increments_beam_test whisker_interfaces/IncrementsBeamTest "{total_x_distance: 6, total_y_distance: 100, increments_x: 0, increments_y
: 15, pause_sec: 0}"
```

## Data Analysis

An example on how to read the sensor data from the mcap file and fit a new whisker model can be found [here](scripts/data_analysis.ipynb).
