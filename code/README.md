## Bug notes
When driving Arduino boards, use USB 2.0 hubs instead of 3.0 hubs for maximum compatibility

## Latest Update

Mar 7, 2025:

Fixed the ultrasonic sensor range issue (see comment in core.cpp, line 307). Implemented line following correction but commented out (see case statement in core.cpp, line 145-160 and 180-195). Implemented motor speed adjustment in peripheral.cpp without success (see peripheral.cpp, line 90). 

If you would like to revert the changes to get a working version before our experimentation, commit hash is 690b6b4 (full hash 690b6b45a4b425a36bfee0e5647bee5e484e7ed0).

Feb 27, 2025:

Sam: Integrated I2C inter-board communication and reconfigured platformio.ini. Now the core board should be able to send commands to the peripheral board to drive motors, and the peripheral board should be able to send back flags

Feb 25, 2025:

Sam: Added I2C inter-board communication test code and reconfigured platformio.ini. Modified code to be split into core_code and peripheral_code, each containing a header and implementation file. WILL NOT RECOMMEND MERGING THIS BRANCH UNLESS YOU HAVE TO. And make sure you keep a copy of your own platformio.ini if you're running platformIO code. 
Enzo: integrated motor control code in the FSM and test code for all 4 motors separately.

Feb 24, 2025:

Sam: Added code for IR and ultrasonic sensing into the master code file (test_line_sensor.ino).
Enzo: added FSM implementation with line sensor signals, working on motor control functions
