# Whisker Sensor Calibration Platform

![Image](docs/figures/3d_printer_overview.jpg)

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


<img src=docs/figures/printer_measurements_1.png  width="250"><img src=docs/figures/printer_measurements_2.png  width="250"><img src=docs/figures/printer_measurements_3.png  width="250">


## Software Instructions
- 
