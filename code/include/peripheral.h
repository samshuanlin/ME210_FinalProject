/*
*  This is a pseudo header file for the core_code implementation. It contains enum, macro, and pin definitions,
*  as well as neceessary library dependencies for core_code.cpp to function properly.
*/

#include <Arduino.h>
#include <PWMServo.h>
#include <Wire.h>

/*---------------Module Defines-----------------------------*/

#define GATE_SERVO_PIN      9
#define IGNITER_SERVO_PIN   10

#define MOTOR_1_IN1_PIN     2 // This is M3 on Kicad
#define MOTOR_1_IN2_PIN     4 // This is M3 on Kicad
#define MOTOR_2_IN3_PIN     A1 // This is M4 on Kicad
#define MOTOR_2_IN4_PIN     A2 // This is M4 on Kicad
#define MOTOR_3_IN1_PIN     7 // This is M5 on Kicad
#define MOTOR_3_IN2_PIN     8 // This is M5 on Kicad
#define MOTOR_4_IN3_PIN     12 // This is M6 on Kicad
#define MOTOR_4_IN4_PIN     13 // This is M6 on Kicad
#define MOTOR_SPEED_PIN     3

#define FORWARD_DIR         1
#define BACKWARD_DIR        -1
#define OFF                 0

// Command codes
#define STOP_CMD        (uint8_t)0
#define DRIVE_NORTH_CMD (uint8_t)1
#define DRIVE_EAST_CMD  (uint8_t)2
#define DRIVE_WEST_CMD  (uint8_t)3
#define DRIVE_SOUTH_CMD (uint8_t)4
#define DRIVE_TURNAROUND_CCW_CMD (uint8_t)5
#define DRIVE_PIVOT_CMD (uint8_t)6
#define LOADING_CMD     (uint8_t)7
uint8_t load_done_flag = 0;
#define DUMPING_CMD     (uint8_t)8
uint8_t dump_done_flag = 0;
#define IGNITION_CMD    (uint8_t)9
#define DRIVE_TURNAROUND_CW_CMD (uint8_t)10
#define CELEBRATION_CMD   (uint8_t)11
#define DISIGNITION_CMD   (uint8_t)12
uint8_t ignition_done_flag = 0;
uint8_t cur_cmd = 100;    // no one is using this, so set as initial value
uint8_t incoming_cmd = 255;

// I2C Peripheral Address
#define PERIPHERAL_ADDR 0x09

/*---------------Module Function Prototypes-----------------*/
// Blocking functions for timed states
void dump(void);
void load(void);

// Motor control functions
void setMotorDirection(int in1, int in2, int dir, int motor);
void stop(void);
void driveNorth(void);
void driveEast(void);
void driveSouth(void);
void driveWest(void);
void driveTurnAroundCW(void);
void drivePivot(void);
void ignition(void);
void celebration(void);
void driveTurnAroundCCW(void);
void disignition(void);

// I2C interrupt handlers
void receiveEvent(int bytes);
void requestEvent(int bytes);

// Loading-related variables (servo)
PWMServo gateServo;  // create servo object to control a servo
int gateServoPos = 90;    // variable to store the servo position
int dumpingDuration = 500; // milliseconds

// Igniting-related variables (servo)
PWMServo igniterServo;  // create servo object to control a servo
int igniterServoPos = 0;    // variable to store the servo position

// Motor-related variables
int loading_driving_delay = 1000; // number of milliseconds the robot will drive toward and from the loading position
int loading_staying_delay = 1000; // number of milliseconds the robot will stay during loading
int mtrSpeed = 255;