# Whisker Sensor Calibration Platform

<img src=docs/figures/3d_printer_overview.jpg  width="250">

## Bill of Materials

| Item                    | Quantity | URL |
| ----------------------- | -------- | --- |
| Prusa i3 3D printer     | 1        |     |
| Whisker Sensor Mount    | 1        |     |
| Whisker Sensor Adaptor  | 1        |     |
| End Effector Mount      | 5        |     |
| M4 Screws               | 4        |     |
| Ruler                   | 1        |     |
| Rubber Sheet (optional) |          |     |

## Assembly Instructions

1. Printing the

   - End Effector Mount
   - Whisker Sensor Adaptor
   - End Effector Mount

2. Attach the Whisker Sensor Adaptor to the rear end of the printer base (optionally use a sheet to rubber between the adaptor and the printer base for increased resistance and protection from the screws)

3. Attachment of the End Effector Mount and Beam
   - Remove the screws from the extruder motor
   - Attach the End Effector Mount
   - Re-attach the screws through the End Effector Mount holes
   - Place the Ruler into the slot
   ![Image](docs/figures/beam_mount.png | width=100)

4. Measurements:
   It is important to take note of a few measurements to allow effective testing.
   - `BEAM_THICKNESS`: width of the ruler (or any alternative beam used)
   - `WHISKER_X`: x position of the whisker on the printing bed
   - `BEAM_EDGE_Y_RELATIVE_TO_NOZZLE`: y distance of the beam relative to the nozzle
   - `BEAM_START`: x distance of the right-most edge of the beam relative to the nozzle
   - `WHISKER_LENGTH_Y`: length of the whisker being evaluated
   - `WHISKER_TIP_Y`: y distance of the beam at the edge tip of the whisker

<img src=docs/figures/printer_measurements_1.png  width="250">
<img src=docs/figures/printer_measurements_2.png  width="250">
<img src=docs/figures/printer_measurements_3.png  width="250">

## Software Instructions

To calibration the sensor for radial contact distance inference, run the following commands in the [whisker_driver_ros2](software/whisker_driver_ros2/) folder.

### Install Dependencies

Install ROS2 environment following [this](https://docs.ros.org/en/foxy/Installation.html) instruction. Then run:

```sh
> scripts/install_deps.sh
```

### Build

```sh
> colcon build && . install/setup.bash
```

### Determine Serial Port Name

1. Connect the 3D printer to the host computer via a serial connection (i.e. using the USB interface in this case)
2. Identify the serial port of the 3D printer on the computer. By default, it is set to locate the serial port at `/dev/ttyACM0`.
Check for the new port connection by running: `ls /dev/tty*`
3. Now connect the whisker sensor to another USB port. Run again `ls /dev/tty*` and note down the newly appeared port. It is likely to be `/dev/ttyACM1`.

The port names are then used in the following ros2 commands.

### Data collection

Launch whisker sensor driver node

```sh
> ros2 run whisker_driver_node whisker_driver_node --ros-args -p serial_device:=/dev/ttyACM0 -p whisker_model_path:=$(pwd)/whisker_driver_ros2/scripts/whisker_model.pkl
```

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

**WARNING**: It is recommended to test the controller WITHOUT the whisker in place first, to understand the behaviors!!

### Fitting a parametric model

The whisker model is a parametric model to predict contact point along the whisker from sensor reading.
To fit a new whisker model, open and execute [this](software/whisker_driver_ros2/scripts/data_analysis.ipynb) jupyter notebook.
A file named `whisker_model.pkl` is produced from the script.

### Radial Contact Distance Inference

To estimate contact distance in real time, launch `whisker_driver_node`:

```sh
> ros2 run whisker_driver_node whisker_driver_node --ros-args -p serial_device:=/dev/ttyACM0 -p whisker_model_path:=$(pwd)/whisker_driver_ros2/scripts/whisker_model.pkl
```

This will load the newly fitted `whisker_model.pkl` and use it to infer any contact made on the whisker.

**_NOTE:_** Please see software [documentation](calibration_platform/software/whisker_driver_ros2/README.md) for further details about the software structure and how to adapt the code for different types of calibrations.
