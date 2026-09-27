; Extruder calibration: asks for exactly 100 mm of filament.
; Before running: mark the filament 200 mm above the extruder inlet.
; After: pushed = 200 - (mark-to-inlet). 100 = correct. About 180 = E steps/mm too high (1.8x).
; The hotend stays at 200 C when this ends: press Abort on the display to turn it off.
; (No M104 S0 here: the firmware has no M400, so it would cut the heat mid-extrusion.)
; Must be the ONLY .gcode file on the SD card. Do not add blank lines.
G90
M109 S200
G1 E100 F60
