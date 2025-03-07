/*
 *  This is a pseudo header file for the core_code implementation. It contains enum, macro, and pin definitions,
 *  as well as neceessary library dependencies for core_code.cpp to function properly.
 */

#include <Arduino.h>

/*----------------Timer Interrupt Defines and Variables-------------*/
#define USE_TIMER_1     true

#include <TimerInterrupt.hpp>        
#include <ISR_Timer.hpp> 
#include <TimerInterrupt.h>
#include <ISR_Timer.h>

/*----------------I2C Defines----------------*/
#include <Wire.h>

 /*---------------Module Defines-----------------------------*/

// Line sensor pins. Analog input, so analog pins used
#define LINE_SENSOR_N_PIN   A0
#define LINE_SENSOR_E_PIN   A1
#define LINE_SENSOR_S_PIN   A2
#define LINE_SENSOR_W_PIN   A3

// IR sensor pins
#define IR_RX_PIN_1     2     // Interrupt-capable
#define IR_RX_PIN_2     3     // Interrupt-capable

// Ultrasonic sensor pins
#define US_1_TRIG       9     // PWM pin
#define US_1_ECHO       4     // regular IO pin
#define US_2_TRIG       10    // PWM pin
#define US_2_ECHO       7     // regular IO pin

// I2C Peripheral Address
#define PERIPHERAL_ADDR 0x09

/*---------------Module Function Prototypes-----------------*/
// Checking for global events, specifically driving the FSM in an infinite loop based on events.
void checkGlobalEvents(void);

// Recurring function to check the front and left distance parameters from the ultrasonic sensor.
uint8_t checkDistance1(void);
uint8_t checkDistance2(void);

// Line sensor event detection and response functions
unsigned char TestForChangeInTape_1(void);
void RespToChangeInTape_1(void);
unsigned char TestForChangeInTape_2(void);
void RespToChangeInTape_2(void);
unsigned char TestForChangeInTape_3(void);
void RespToChangeInTape_3(void);
unsigned char TestForChangeInTape_4(void);
void RespToChangeInTape_4(void);

// IR receiver interrupt handlers
void ir1_handler();
void ir2_handler();
// IR receiver reading average array, for "debouncing"
int ir1_debouncing_array[10] = {};
int ir2_debouncing_array[10] = {};
int ir1_arr_idx = 0;
int ir2_arr_idx = 0;

// IR receiver sensing event detection and response functions
uint8_t TestForBeaconSensing(void);
void RespToBeaconSensing(void);

// Ultrasonic sensoor event detection and response functions
uint8_t TestForFrontWall(void);
void RespToFrontWall(void);
uint8_t TestForLeftWall(void);
void RespToLeftWall(void);
uint8_t TestForTriggerTimerExpired(void);
void RespToTriggerTimerExpired(void);

// State display function for testing
void displayState(void);

// Commands & Command Codes
#define STOP_CMD        (uint8_t)0
void stopCmd(void);

#define DRIVE_NORTH_CMD (uint8_t)1
void driveNorthCmd(void);

#define DRIVE_EAST_CMD  (uint8_t)2
void driveEastCmd(void);

#define DRIVE_WEST_CMD  (uint8_t)3
void driveWestCmd(void);

#define DRIVE_SOUTH_CMD (uint8_t)4
void driveSouthCmd(void);

#define DRIVE_TURNAROUND_CMD (uint8_t)5
void driveTurnAroundCmd(void);

#define DRIVE_PIVOT_CMD (uint8_t)6
void drivePivotCmd(void);

#define LOADING_CMD     (uint8_t)7
void loadCmd(void);

#define DUMPING_CMD     (uint8_t)8
void dumpCmd(void);

#define IGNITION_CMD    (uint8_t)9
void ignitionCmd(void);

/*---------------State Definitions--------------------------*/
const char* stateNames[] = {
  "SPINNING_NOODLE", "SCANNING", "LEAVING_SZ_1", "LEAVING_SZ_2","LEAVING_SZ_3", "PIVOTING", "GOING_TO_CW_1", "GOING_TO_CW_2",
  "MOVING_POT", "GOING_BACK_ON_TRACK", "GOING_TO_BTN_i", "STOPPING_FOR_IGNITION", "IGNITING_BTN",
  "LEAVING_FROM_BTN_i", "DUMPING", "GOING_TO_PANTRY_1", "GOING_TO_PANTRY_2",
  "GOING_TO_PANTRY_3", "LOADING", "GOING_TO_BURNER_1", "GOING_TO_BURNER_2",
  "GOING_TO_BURNER_3", "GOING_TO_BTN_f", "TURNING_OFF_BURNER",
  "LEAVING_FROM_BTN_f", "DELIVERING", "CELEBRATING"
};

typedef enum {
SPINNING_NOODLE, SCANNING, LEAVING_SZ_1, LEAVING_SZ_2, LEAVING_SZ_3, PIVOTING, GOING_TO_CW_1, GOING_TO_CW_2,
MOVING_POT, GOING_BACK_ON_TRACK, GOING_TO_BTN_i, STOPPING_FOR_IGNITION,  IGNITING_BTN,
LEAVING_FROM_BTN_i, DUMPING, 
GOING_TO_PANTRY_1, GOING_TO_PANTRY_2, GOING_TO_PANTRY_3, LOADING,
GOING_TO_BURNER_1, GOING_TO_BURNER_2, GOING_TO_BURNER_3,
GOING_TO_BTN_f, TURNING_OFF_BURNER, LEAVING_FROM_BTN_f,
DELIVERING, CELEBRATING, NUM_STATES 
} States_t;

/*---------------Module Variables---------------------------*/
// State variables
States_t state;
States_t initialState = GOING_TO_BTN_i;

// Line sensor variables
int thrLine = 300; // depend on sensing
// previous detects
int line1;
int line2;
int line3;
int line4;
// current detects
int current_line1;
int current_line2;
int current_line3;
int current_line4;

// IR sensor variables
int ir_1_status = 0;
int ir_2_status = 0;
long duration1, distance1, duration2, distance2;

// Ultrasonic sensor variables
int timerTrigger = 10; // milliseconds
int currentMillis;
int startMillis;
int us1; // distance sensed by the ultrasonic sensor 1
int us2; // distance sensed by the ultrasonic sensor 1
int thr_us1 = 3; // cm, front
int thr_us2 = 17; // cm, left
int us_score;
int thr_us_score = 16;
int buffer_value_1;
int buffer_value_2;
int buffer_value_3;


// Timer variables
int delay_ignoring_tape_in_sz = 500;
int timer_moving_pot = 7500;
int delay_going_against_kitchen = 800;
int delay_rotation_to_45_orientation = 900;
int delay_to_enter_loading_zone = 1300;
int delay_to_leave_loading_zone = 1100;
int delay_back_to_kitchen = 1000;
int dumping_duration = 500;