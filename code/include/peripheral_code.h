/*
*  This is a pseudo header file for the core_code implementation. It contains enum, macro, and pin definitions,
*  as well as neceessary library dependencies for core_code.cpp to function properly.
*/

#include <Arduino.h>
#include <Servo.h>

/*---------------Module Defines-----------------------------*/

#define GATE_SERVO_PIN      9

#define MOTOR_1_IN1_PIN     2 // This is M3 on Kicad
#define MOTOR_1_IN2_PIN     4 // This is M3 on Kicad
#define MOTOR_2_IN3_PIN     5 // This is M4 on Kicad
#define MOTOR_2_IN4_PIN     6 // This is M4 on Kicad
#define MOTOR_3_IN1_PIN     7 // This is M5 on Kicad
#define MOTOR_3_IN2_PIN     8 // This is M5 on Kicad
#define MOTOR_4_IN3_PIN     12 // This is M6 on Kicad
#define MOTOR_4_IN4_PIN     13 // This is M6 on Kicad
#define MOTOR_SPEED_PIN     3

#define FORWARD_DIR         1
#define BACKWARD_DIR        -1

/*---------------Module Function Prototypes-----------------*/
// Blocking functions for timed states
void Dump(void);
void Load(void);

// Motor control functions
void driveNorth(void);
void driveEast(void);
void driveSouth(void);
void driveWest(void);
void driveTurnRound(void);
void drivePivot(void);
void stop(void);

// Loading-related variables (servo)
Servo gateServo;  // create servo object to control a servo
int gateServoPos = 0;    // variable to store the servo position
int dumpingDuration = 1000; // milliseconds

// Motor-related variables
int loading_driving_delay = 1000; // number of milliseconds the robot will drive toward and from the loading position
int loading_staying_delay = 500; // number of milliseconds the robot will stay during loading
int mtrSpeed = 50;