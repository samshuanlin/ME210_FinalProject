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
#define US_TRIG         9     // PWM pin
#define US_1_ECHO       4     // regular IO pin
#define US_2_ECHO       7     // regular IO pin

/*---------------Module Function Prototypes-----------------*/
// Checking for global events, specifically driving the FSM in an infinite loop based on events.
void checkGlobalEvents(void);

// Recurring function to check the front and left distance parameters from the ultrasonic sensor.
void checkDistance(void);

// Line sensor event detection and response functions
unsigned char TestForChangeInTape_1(void);
void RespToChangeInTape_1(void);
unsigned char TestForChangeInTape_2(void);
void RespToChangeInTape_2(void);
unsigned char TestForChangeInTape_3(void);
void RespToChangeInTape_3(void);
unsigned char TestForChangeInTape_4(void);
void RespToChangeInTape_4(void);

/*
// IR receiver interrupt handlers
void ir1_handler();
void ir2_handler();
*/

// IR receiver sensing event detection and response functions
uint8_t TestForBeaconSensing(void);
void RespToBeaconSensing(void);

// Ultrasonic sensoor event detection and response functions
uint8_t TestForFrontWall(void);
void RespToFrontWall(void);
uint8_t TestForLeftWall(void);
void RespToLeftWall(void);

// State display function for testing
void displayState(void);

// Motor Command Sensing Functions
void handleDump(void);

/*---------------State Definitions--------------------------*/
const char* stateNames[] = {
    "SPINNING_NOODLE", "SCANNING", "LEAVING_SZ_1", "LEAVING_SZ_2", "GOING_TO_CW_1", "GOING_TO_CW_2",
    "MOVING_POT", "GOING_BACK_ON_TRACK", "GOING_TO_BTN_i", "IGNITING_BTN",
    "LEAVING_FROM_BTN_i", "DUMPING", "GOING_TO_PANTRY_1", "GOING_TO_PANTRY_2",
    "GOING_TO_PANTRY_3", "LOADING", "GOING_TO_BURNER_1", "GOING_TO_BURNER_2",
    "GOING_TO_BURNER_3", "GOING_TO_BTN_f", "TURNING_OFF_BURNER",
    "LEAVING_FROM_BTN_f", "DELIVERING", "CELEBRATING"
};

typedef enum {
  SPINNING_NOODLE, SCANNING, LEAVING_SZ_1, LEAVING_SZ_2, GOING_TO_CW_1, GOING_TO_CW_2,
  MOVING_POT, GOING_BACK_ON_TRACK, GOING_TO_BTN_i, IGNITING_BTN,
  LEAVING_FROM_BTN_i, DUMPING, 
  GOING_TO_PANTRY_1, GOING_TO_PANTRY_2, GOING_TO_PANTRY_3, LOADING,
  GOING_TO_BURNER_1, GOING_TO_BURNER_2, GOING_TO_BURNER_3,
  GOING_TO_BTN_f, TURNING_OFF_BURNER, LEAVING_FROM_BTN_f,
  DELIVERING, CELEBRATING, NUM_STATES
} States_t;

/*---------------Module Variables---------------------------*/
// State variables
States_t state;
States_t initialState = DUMPING;

// Line sensor variables
float thrLine = 200.0;
int line1;
int line2;
int line3;
int line4;
int current_line1;
int current_line2;
int current_line3;
int current_line4;

// IR sensor variables
int ir_1_status = 0;
int ir_2_status = 0;

// Ultrasonic sensor variables
long duration1, distance1, duration2, distance2;    // long for 64 bit storage
long prev_dist1, prev_dist2;
long front_dist_threshold = 5;
long left_dist_threshold = 50; 
long hysteresis_threshold = 5;    // may need different threshold for different distances