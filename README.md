## Meeting Notes

**March 2, 2025**
Members: Sam C., Archer D.

Discussion items/worked on:
1. Troubleshooting the IR sensor. The issue turned out to be the PWM signal, as unplugging it 
from the circuit eliminates the interference. It could be because the small resistances in the
shared wires and possibly the protoboard lines cause current jumps due to the PWM. We are isolating the power and ground connections. We also added unity buffers between the photoresistor output and highpass input to stop interference between the two.

Action items:
1. We still need to fix the bugs from last time (servo bugs, stopping before ignition, timer for pot pushing).

**Feb 28, 2025**
Members: Thomas H., Enzo A.

Discussion items/worked on:
1. robot functions well ( 80\% ish success rate?) from leaving starting zone until pushing pot.
2. robot goes back on track fine. 

Action items:
1. It is likely that we have to implement a timer to push the pot since we cannot get the ultrasonic sensor to work.
2. The west ultrasonic sensor should be used to sense when we should stop before igniting the burner, the current method stops way too late and risks the robot bumping into the pot handle.
3.  IR sensor needs to be retested. The issue ONLY happens when the power is on. I suggest we first test it with the computer power arduino, make sure it works. Then we test it still with our laptop, but in the vicinity of the robot, turn on the switch, and see what happens. I also think that it might be good if the ultrasonic sensor is not sharing a ground with the rest of the circuit.
4. To fix everything servo related, Thomas needs to talk to Sam to understand how the boards are communicating.
5. To sum up, in order to beat the brick, the following transitions needs to be fixed.
	(1) IR scanning and leaving the start zone.
	(2) timer for pot pushing
	(3) stopping before ignition
	(4) servo bugs


**Feb 27, 2025**
Members: Sam C., Archer D., Thomas H., Enzo A.

Discussion items/worked on:
1. Tested robot driving mechanisms and can confirm that normal driving and loading works. Dumping probably works.
2. Tested robot IR sensing mechanism, pretty inconsistent, sometimes arbitrary. Need improvement.
3. Assembled robot double-decker prototype and finished circuitry for everything except the TPU and gate servo motors.

Action items:
1. Subsystem checkoff due tomorrow.
2. Sensor testing.
3. Improvement of IR mechanism.
4. Re-design and design of robot second level, switch holder (in the robot), ultrasonic sensor holder, etc.

**Feb 25, 2025 Afternoon-Evening Testing Bot**

Members: Thomas H., Enzo A.

Discussion items/worked on: 
1. Tested robot on the field.
2. reinstalled buck converter, installed switch

Action items:
1. Tape Sensor needs to be more off the ground (0.5 in)
2. Tape sensor needs to be in the right direction.

**Feb 25, 2025 Afternoon-Evening**

Members: Sam C., Archer D., Enzo A.

Discussion items/worked on: 
1. Assembled motor drivers and crimped battery wires. Tested the motor driver functionality with Enzo's test code, no issues with software. Need to redo direction of the mecanums.
2. Sam looked into inter-board I2C communication and found ways to send messages. Also found a way to drive 2 Arduinos at once.
3. Tested slope and gate with servo code, dimensions for slope finalized. Finished gate redsign to clamp/ screw down servo.

Action items:
1. Re-organize code to split into core and peripheral boards.
2. Servo integration with gate & TPU.
3. Print and test new gate tomorrow.
4. Thomas needs to talk to Archer about interfacing the slope with the chassis.

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