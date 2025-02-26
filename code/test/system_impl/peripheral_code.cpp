/*
 *  This file contains methods that will be run by the peripheral Arduino board.
 *  The responsibilities of the peripheral board include:
 *  1. Receive signals from the core board and process them
 *  2. Drive DC and servo motors at intended times.
 */
 
#include <peripheral_code.h>

/*---------------Robot Main Functions----------------*/
void setup(void) {
    Serial.begin(9600);
    while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
    // ------- Clears the Serial Monitor --------
    for (int i = 0; i < 50; i++) {
         Serial.println();
    }

    pinMode(MOTOR_1_IN1_PIN, OUTPUT);
    pinMode(MOTOR_1_IN2_PIN, OUTPUT);
    pinMode(MOTOR_2_IN3_PIN, OUTPUT);
    pinMode(MOTOR_2_IN4_PIN, OUTPUT);
    pinMode(MOTOR_3_IN1_PIN, OUTPUT);
    pinMode(MOTOR_3_IN2_PIN, OUTPUT);
    pinMode(MOTOR_4_IN3_PIN, OUTPUT);
    pinMode(MOTOR_4_IN4_PIN, OUTPUT);
    pinMode(MOTOR_SPEED_PIN,OUTPUT);
 
    gateServo.attach(GATE_SERVO_PIN);
}
 
  
void loop() {
    switch (state) {
        case LEAVING_SZ_1:
            driveNorth();
            break;
        case LEAVING_SZ_2:
            driveNorth();
            break;
        case PIVOTING:
            drivePivot();
            break;
        case GOING_TO_CW_1:
            driveEast();
            break;
        case GOING_TO_CW_2:
            driveNorth();
            break;
        case MOVING_POT:
            driveWest();
            break;
        case GOING_BACK_ON_TRACK:
            driveSouth();
            break;
        case GOING_TO_BTN_i:
            driveWest();
            break;
        case IGNITING_BTN:
            stop();
            break;
        case LEAVING_FROM_BTN_i:
            driveNorth();
            break;
        case DUMPING:
            Dump();
            break;
        case GOING_TO_PANTRY_1:
            driveSouth();
            break;
        case GOING_TO_PANTRY_2:
            driveEast();
            break;
        case GOING_TO_PANTRY_3:
            driveSouth();
            break;
        case LOADING:
            Load();
            break;
        case GOING_TO_BURNER_1:
            driveNorth();
            break;
        case GOING_TO_BURNER_2:
            driveWest();
            break;
        case GOING_TO_BURNER_3:
            driveNorth();
            break;
        case GOING_TO_BTN_f:
            driveSouth();
            break;
        case TURNING_OFF_BURNER:
            stop();
            break;
        case LEAVING_FROM_BTN_f:
            driveNorth();
            break;
        case DELIVERING:
            driveEast();
            break;
        case CELEBRATING:
            stop();
            break;
        }
 
     displayState();
     analogWrite(MOTOR_SPEED_PIN, mtrSpeed);
 
     //displayLineSensors();
  }
 
 
 /*----------------Module Functions--------------------------*/

 void Dump(void) {
   gateServo.write(100);
   delay(dumpingDuration);
   gateServo.write(0);
   delay(dumpingDuration);
   state = GOING_TO_PANTRY_1;
 }
 
 void Load(void) {
   driveSouth();
   delay(loading_driving_delay);
   stop();
   delay(loading_staying_delay);
   driveNorth();
   delay(loading_driving_delay);
   state = GOING_TO_BURNER_1;
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
 
 
 
 
 
 
 
 
 