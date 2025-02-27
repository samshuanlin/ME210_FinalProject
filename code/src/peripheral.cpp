/*
 *  This file contains methods that will be run by the peripheral Arduino board.
 *  The responsibilities of the peripheral board include:
 *  1. Receive signals from the core board and process them
 *  2. Drive DC and servo motors at intended times.
 */
 
 #include <peripheral_code.h>

 // I2C interrupt handler
 void receiveEvent(int bytes) {
     incoming_cmd = (uint8_t)Wire.read();
 }
 
 /*---------------Robot Main Functions----------------*/
 void setup(void) {
     Serial.begin(9600);
     while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
     // ------- Clears the Serial Monitor --------
     for (int i = 0; i < 50; i++) {
          Serial.println();
     }
 
     // pin mode setup
     pinMode(GATE_SERVO_PIN, OUTPUT);
     pinMode(MOTOR_1_IN1_PIN, OUTPUT);
     pinMode(MOTOR_1_IN2_PIN, OUTPUT);
     pinMode(MOTOR_2_IN3_PIN, OUTPUT);
     pinMode(MOTOR_2_IN4_PIN, OUTPUT);
     pinMode(MOTOR_3_IN1_PIN, OUTPUT);
     pinMode(MOTOR_3_IN2_PIN, OUTPUT);
     pinMode(MOTOR_4_IN3_PIN, OUTPUT);
     pinMode(MOTOR_4_IN4_PIN, OUTPUT);
     pinMode(MOTOR_SPEED_PIN,OUTPUT);
     
     // servo setup
     gateServo.attach(GATE_SERVO_PIN);
 
     // I2C peripheral device setup
     Wire.begin();
     // Attach a function to trigger when something is received.
     Wire.onReceive(receiveEvent);
 }
  
   
 void loop() {
     // turn off any done flags
     load_done_flag = 0;
     dump_done_flag = 0;
     if (cur_cmd != incoming_cmd) stop();   // stop before executing other command if the new cmd is not the current cmd
     cur_cmd = incoming_cmd;    // store what the currently running command is
     if (incoming_cmd == STOP_CMD) {
         stop();
     } else if (incoming_cmd == DRIVE_NORTH_CMD) {
         driveNorth();
     } else if (incoming_cmd == DRIVE_EAST_CMD) {
         driveEast();
     } else if (incoming_cmd == DRIVE_WEST_CMD) {
         driveWest();
     } else if (incoming_cmd == DRIVE_SOUTH_CMD) {
         driveSouth();
     } else if (incoming_cmd == LOADING_CMD) {
         load();
     } else if (incoming_cmd == DUMPING_CMD) {
         dump();
     }
 
     analogWrite(MOTOR_SPEED_PIN, mtrSpeed);
 }
  
  
  /*----------------Module Functions--------------------------*/
 
 void dump(void) {
     gateServo.write(100);
     delay(dumpingDuration);
     gateServo.write(0);
     delay(dumpingDuration);
     dump_done_flag = 1;
 }
  
 void load(void) {
    driveSouth();
    delay(loading_driving_delay);
    stop();
    delay(loading_staying_delay);
    driveNorth();
    delay(loading_driving_delay);
    load_done_flag = 1;
 }
  
 void driveNorth(void) {
    setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR);
 }
  
 void driveEast(void) {
    setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR);
    setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR);
    setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR);
 }
  
 void driveSouth(void) {
    setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR);
    setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR);
    setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR);
    setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, BACKWARD_DIR);
 }
  
 void driveWest(void) {
    setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR);
    setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, BACKWARD_DIR);
 }
  
 void driveTurnRound(void) {
    // We turn CCW
    setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR);
    setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, BACKWARD_DIR);
 }
  
 void drivePivot(void) {
    // We pivot around the wheel number 4 (MTR 4 = Nort-West)
    setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR);
    setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR);
 } 
  
 void stop(void) {
    Serial.println("");
 }
  
 // Shortcut function that sets the direction of a given motor to forward / backward
 void setMotorDirection(int in1, int in2, int dir) {
    if (dir == FORWARD_DIR) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else if (dir == BACKWARD_DIR) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
 }