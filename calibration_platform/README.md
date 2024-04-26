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
Requirements
```
pip install pyserial
pip install fire
```

1. Connect the 3D printer to you computer via a serial connection (i.e. using the USB interface in this case)
2. Identify the serial port of the 3D printer on the computer. By default, it is set to locate the serial port at `/dev/ttyACM0`. 
Check for the new port connection at: 
```
ls /dev/tty*
```

3. Begin the calibration process
Once all the prior configurations have been set up. You may now run `GcodeController.py`. By default, this will initially calibrate the printer. And then will begin a routine of bending the whisker along its shaft. This is defined in the `main()` function.

**It is recommended to test the controller WITHOUT the whisker in place, to observe the behaviours!!**


#### Using the controller
Alternatively, the GcodeController can be imported to customise testing configurations. Every time the class is initialised, the `prepare()` method must be called to ensure the printer is aware of its `X` and `Y` origins. 

Custom test configurations can be made using a combination of the methods:
   - `prepare()`
   - `send_movement(..)`
   - `set_speed(..)`
   - `send_message(..)`

Example Usage
```python
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

