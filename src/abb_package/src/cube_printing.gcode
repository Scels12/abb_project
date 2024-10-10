; Start G-code for printing a 3 cm cube
M190 S60 ; Set bed temperature to 60 degrees Celsius
M109 S200 ; Set extruder temperature to 200 degrees Celsius
G28 ; Home the printer
G0 X0 Y0 Z0.3 ; Move to the start position

; Extrude a layer of filament
G1 X3 Y0 Z0.3 E1
G1 X3 Y3 Z0.3 E1
G1 X0 Y3 Z0.3 E1
G1 X0 Y0 Z0.3 E1

; Move to the next layer position
G0 Z0.6

; Extrude another layer of filament
G1 X3 Y0 Z0.6 E1
G1 X3 Y3 Z0.6 E1
G1 X0 Y3 Z0.6 E1
G1 X0 Y0 Z0.6 E1

; Repeat the above steps until the cube is complete

; End G-code
M104 S0 ; Turn off temperature
G28 X0 ; Home X axis
M84 ; Disable motors
