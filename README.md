## Meeting Notes

**Feb 25, 2025 Afternoon-Evening**
Members: Sam C., Archer D., Enzo A.
Discussion items/worked on: 
1. Assembled motor drivers and crimped battery wires. Tested the motor driver functionality with Enzo's test code, no issues with software. Need to redo direction of the mecanums.
2. Sam looked into inter-board I2C communication and found ways to send messages. Also found a way to drive 2 Arduinos at once.

Action items:
1. Re-organize code to split into core and peripheral boards.
2. Servo integration with gate & TPU.

**Feb 25, 2025 Morning**
Members: Sam C., Enzo A., Thomas H.,

Discussion items/Worked on:
1. Finalizing the motor operation code and integrating it into the FSM.
2. Revamped IR sensor circuit to include pre-gain filter and a greater gain (~240x now) to pick up the signals from the furthest point of interest.
3. 

Action items:
1. Test out IR sensor operation.
2. Figure out two-board communication and organize code from there.
3. Servo integration with gate & TPU.
4. Battery connector crimping and wiring.

**Feb 24, 2025**
Members: Sam C., Archer D., Thomas H., Enzo A.

Discussion items/Worked on:
1. Working version of the IR phototransistor network complete with code support.
2. Chassis, motor and wheel assembly complete.
3. FSM implementation for pantry-pot loop complete with line sensor input reading.
4. Integrated IR and ultrasonic sensor code together.

Action items:
1. I2C inter-board communication.
2. Integrate motor control code.
3. Organize code to have separate classes/files for different functions.
4. Test current ball track and servo gate.
5. Start working on TPU arm.

**Feb 22, 2025**
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

**Feb 18, 2025**

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