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

## Running GCodeController directly

A python interface is written to control the 3D printer. It can be invoked independently of ROS2 and can be useful in certain applications. GcodeController class can be imported to customize testing configurations. Every time the class is initialized, the `prepare()` method must be called to ensure the printer is aware of its `X` and `Y` origins.

The controller is comprised of a set of Controller Action, Test Action and Properties.

#### Class Properties
These are useful to allow introspection into the positions if the beam as well as the state of the testing rig.

|||
|---|---|
|x | x position, relative to origin, of the printer nozzle in mm |
|y | y position, relative to origin, of the printer nozzle in mm |
|z | z position, relative to origin, of the printer nozzle in mm |
|printer_speed | speed of the printer nozzle in in mm/min |
|current_gcode  | the gcode currently in execution |
| beam_from_whisker_tip | the x and y distance of the beam edge to the whisker tip |
| get_beam_contact_position_x  | the x position, relative to nozzle, of the beam edge that will apply a force to the whisker  |
| at_goal | if the current position of x is at the intended goal position from the previous gcode command |


#### Controller Actions 
Set of actions in order to safely control the printer without generating conflicts in commands. This improves safety compared to sending raw gcode commands
|||
|---|---|
| set_speed | sets the printer speed|
| send_movement | safely sends movements to the printer for |
| send_message | displays a message on the printer screen to indicate the state|
| prepare | zeroing calibration command in XY|

### Test Actions
Test actions are a combination of the different controller actions to allow a consistent testing procedure.  

#### increments_beam_test
The `increments_beam_test` is the default action that is currently deployed. Below are the input parameters described for it.

|||
|---|---|
|total_x_distance| maximum distance to bend the whisker|
|total_y_distance| maximum distance along the whisker from the tip to bend|
|increments_x|The number of steps the beam will wait at before reaching total_x_distance,<br>Eg:<br>If total_x_distance = 9 and increments_x = 0:<br>the beam will only pause at 9mm<br><br>If total_x_distance = 9 and increments_x = 2:<br>the beam will pause at 3mm, then 6mm then 9mm|
|increments_y| The number of steps the beam will wait at before reaching total_y_distance, similar to increments_x, but along the y axis of the whisker|
|pause_sec| time to pause at each incremental position|


<img src=../../docs/figures/increments_beam_test.png width=500>

#### Custom test configurations
If you would like to create a custom test configuration, it is suggested to be done by making use of the controller actions described above. This could look like something along the lines of:

Example Usage

```python
from GcodeController import GcodeController

port = "/dev/ttyACM0"
controller = GcodeController(port)

# Homing the printer
controller.prepare()

# Sending Custom Configurations
send_message("starting progress...")
controller.send_movement(10, 50)
sleep(2)
controller.send_movement(150, 50)
send_message("...slowing down...")
controller.set_speed(1000)
controller.send_movement(100, 50)
send_message("...speeing up...")
controller.set_speed(2000)
controller.send_movement(200, 50)
send_message("...complete!")
sleep(3)
send_message("Home time! :)")
controller.prepare()
```

Actions
