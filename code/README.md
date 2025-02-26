## Bug notes
When driving Arduino boards, use USB 2.0 hubs instead of 3.0 hubs for maximum compatibility

## Latest Update

Feb 25, 2025:
Sam: Added I2C inter-board communication test code and reconfigured platformio.ini. WILL NOT RECOMMEND MERGING THIS BRANCH UNLESS YOU HAVE TO. And make sure you keep a copy of your own platformio.ini if you're running platformIO code. 
Enzo: integrated motor control code in the FSM and test code for all 4 motors separately.

Feb 24, 2025:

Sam: Added code for IR and ultrasonic sensing into the master code file (test_line_sensor.ino).
Enzo: added FSM implementation with line sensor signals, working on motor control functions
