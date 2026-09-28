; Part-cooling fan test (GPIO15): full, half, low, off - about 10 s each.
; Each slow Z move is a timer; M400 waits for it to finish before the next fan change.
; Expect: fan off at start, then full, half, low, off. No heating.
; Needs the firmware with M400 support. Must be the ONLY .gcode file on the SD card.
G90
M107
G28
M400
; Full speed (S255)
M106 S255
G1 Z10 F60
M400
; Half speed (S128)
M106 S128
G1 Z20 F60
M400
; Low speed (S64) - some fans stall this low; that is OK
M106 S64
G1 Z30 F60
M400
; Off
M107
G1 Z40 F60
M400
