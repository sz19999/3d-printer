; PID autotune - hotend then bed. Gains are saved to flash automatically.
; Must be the ONLY .gcode file on the SD card. Do not add blank lines.
; Hotend at 200 C (about 5-10 min)
M106 S0
M303 E0 S200
; Bed at 60 C (about 20-40 min)
M303 E-1 S60
M104 S0
M140 S0
