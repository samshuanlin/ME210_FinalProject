## Meeting Notes

**Feb 22, 2024**
Members: Sam C., Archer D., Thomas H., Enzo A.

Discussion items:
1. Confirmed and submitted the schematic, FSM, design sketches and test results.
2. Thomas and Sam: tested the IR sensor and confirmed that the IR phototransistor can pick up rising edges from the IR beacon. Need to revise circuit.
3. Archer: worked on the motor bracket and later (on Feb 23) finished the motor chassis plate.
4. Enzo: integrated the line sensor and confirmed line sensor detection functionality in the proposed setting.

Action items:
1. Integrating the robot chassis.
2. Revise IR sensor circuit.
3. Program basic FSM for line sensor network with motor to confirm functionality.
4. Obtain batteries for initial motor testing.

**Feb 18, 2024**

Members: Sam C., Enzo A.

Discussion items:
1. We discussed the operation procedure further and settled on the idea of using 4 line sensors in the center of the robot. This allows for line tracking without having to turn. Sam found a proof of concept video: https://www.youtube.com/watch?v=ZKfnxVbddTw
2. We talked about the use of a 9DOF sensor (specifically focusing on the magnetometer). Sam needs to find precise ways to calibrate the magnetometer and whether there are ways to do it more efficiently.
3. Sam got an ultrasonic sensor, Enzo purchased one, and Thomas got one from a friend taking ME 210 a while ago. We need 1 more ultrasonic sensor.

Action items: 
1. Finalize magnetometer calibration procedure by Feb 19.
2. Change ultrasonic sensor implementatino to be driven by timer interrupts by Feb 19.
3. Start circuit designs with consideration of Arduino pin usage. Finish by Feb 20. 
4. Build an IR receiver circuit, preferably similar to Lab1 so it can be a digital reading. Finish by Feb 21.