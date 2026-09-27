; Heater wiring check - run with the serial monitor open.
; Must be the ONLY .gcode file on the SD card. Do not add blank lines.
; Step 1: HOTEND channel (GPIO7) to 200 C. Watch which heater gets warm.
M109 S200
; Step 2: BED channel (GPIO6) to 60 C. Watch which heater gets warm.
M190 S60
