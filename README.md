## Meeting Notes

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