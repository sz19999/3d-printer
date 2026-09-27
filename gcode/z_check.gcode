; Z zero check: homes, then parks the nozzle at X80 Y80 Z0 (cold, no heating).
; Slide a sheet of paper under the nozzle: light drag = Z0 is on the bed = OK to print.
; A visible gap (about 3 mm) = Z0 is above the bed; the cube would print in the air.
; Must be the ONLY .gcode file on the SD card. Do not add blank lines.
G90
G28
G0 Z5 F300
G0 X80 Y80 F3000
G0 Z0 F300
