#include <Arduino.h>
#include <Servo.h>

/*----------------Timer Interrupt Defines and Variables-------------*/
#define USE_TIMER_1     true
#define FREQUENCY       3300    // to be measured in the states

#include <TimerInterrupt.hpp>        
#include <ISR_Timer.hpp> 
#include <TimerInterrupt.h>
#include <ISR_Timer.h>

 /*---------------Module Defines-----------------------------*/

// Line sensor pins
#define LINE_SENSOR_N_PIN   A0
#define LINE_SENSOR_E_PIN   A1
#define LINE_SENSOR_S_PIN   A2
#define LINE_SENSOR_W_PIN   A3

// Servo pins
#define GATE_SERVO_PIN      9

// IR sensor pins
#define IR_RX_PIN_1     2
#define IR_RX_PIN_2     3

// Ultrasonic sensor pins
#define US_TRIG         9     // PWM pin
#define US_1_ECHO       4     // regular IO pin
#define US_2_ECHO       7     // regular IO pin

/*---------------Module Function Prototypes-----------------*/
void checkGlobalEvents(void);
unsigned char TestForChangeInTape_1(void);
void RespToChangeInTape_1(void);
unsigned char TestForChangeInTape_2(void);
void RespToChangeInTape_2(void);
unsigned char TestForChangeInTape_3(void);
void RespToChangeInTape_3(void);
unsigned char TestForChangeInTape_4(void);
void RespToChangeInTape_4(void);

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

// Servo variables
Servo gateServo;  // create servo object to control a servo
int gateServoPos = 0;    // variable to store the servo position
int dumpingDuration = 1000; // milliseconds

// IR sensor variables
int ir_1_status = 0;
int ir_2_status = 0;

// Ultrasonic sensor variables
long duration1, distance1, duration2, distance2;    // long for 64 bit storage
long prev_dist1, prev_dist2;
long front_dist_threshold = 5;
long left_dist_threshold = 50; 
long hysteresis_threshold = 5;    // may need different threshold for different distances

/*---------------Robot Main Functions----------------*/
void setup(void) {
   Serial.begin(9600);
   while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
   // ------- Clears the Serial Monitor --------
   for (int i = 0; i < 50; i++) {
        Serial.println();
    }
   
   state = initialState;

   // pin setup for line sensors
   pinMode(LINE_SENSOR_N_PIN, INPUT);
   pinMode(LINE_SENSOR_E_PIN, INPUT);
   pinMode(LINE_SENSOR_S_PIN, INPUT);
   pinMode(LINE_SENSOR_W_PIN, INPUT);

   gateServo.attach(GATE_SERVO_PIN);

   // pin setup for IR sensors
   pinMode(IR_RX_PIN_1, INPUT);
   pinMode(IR_RX_PIN_2, INPUT);

   // digital pin interrupt setup for IR sensors
   attachInterrupt(digitalPinToInterrupt(IR_RX_PIN_1), ir1_Handler, RISING);
   attachInterrupt(digitalPinToInterrupt(IR_RX_PIN_2), ir2_Handler, RISING);

   // ultrasonic sensor pin setup
   pinMode(US_TRIG, OUTPUT);
   pinMode(US_1_ECHO, INPUT);
   pinMode(US_2_ECHO, INPUT);
 }

 
 void loop() {
    checkGlobalEvents();
    checkFrontDistance();
    switch (state) {
      case DUMPING:
        handleDump();
        break;
    }

    displayState();

    //displayLineSensors();
 }

/*----------------ISRs---------------*/


/*----------------Module Functions--------------------------*/

void checkGlobalEvents(void) {
  if (TestForChangeInTape_1()) RespToChangeInTape_1();
  if (TestForChangeInTape_2()) RespToChangeInTape_2();
  if (TestForChangeInTape_3()) RespToChangeInTape_3();
  if (TestForChangeInTape_4()) RespToChangeInTape_4();
  if (TestForBeaconSensing()) RespToBeaconSensing();
  if (TestForFrontWall()) RespToFrontWall();
  if (TestForLeftWall()) RespToLeftWall();
} 

void ir1_handler(void *) {
  ir_1_status = 1;  // fired at every pin interrupt, set ir_1_status to be 1, to be turned of by TestForBeaconSensing
}

void ir2_handler(void *) {
  ir_2_status = 1;  // fired at every pin interrupt, set ir_2_status to be 1, to be turned of by TestForBeaconSensing
}

uint8_t TestForBeaconSensing(void) {
  if (ir_1_status || ir_2_status) {   // use OR logic to allow for greater coverage
    ir_1_status = 0;
    ir_2_status = 0;
    return 1;
  }
  return 0;
}

void checkFrontDistance(void) {
  analogWrite(US_TRIG, 128);  // 50% duty cycle, 490Hz frequency
  unsigned long timeout = 3000L;
  // US_1 is the front-facing ultrasonic sensor
  duration1 = pulseIn(US_1_ECHO, HIGH, timeout);    // pulse in us. if returning 0, means no feedback received
  distance1 = duration1 * 10 / 2 / 291;   // duration (us) / 2 / 29.1 (us / cm) (speed is the speed of light)
                                          // additional 10 multiplied to prevent decimal numbers
  // US_2 is the left-facing ultrasonic sensor
  duration2 = pulseIn(US_2_ECHO, HIGH, timeout);
  distance2 = duration2 * 10 / 2 / 291;
  // note that this is done in a superloop, so will cause delays for 6 ms maximum
}

uint8_t TestForFrontWall(void) {
  if (distance1 >= front_dist_threshold + hysteresis_threshold && prev_dist1 < front_dist_threshold + hysteresis_threshold) {
    prev_dist1 = distance1;
    return true;
  }
  prev_dist1 = distance1;
  return false;
}

void RespToFrontWall(void) {
  switch (state) {
    case GOING_TO_CW_2:
      state = MOVING_POT;
      break;
    case LEAVING_FROM_BTN_i:
      state = MOVING_POT;
      break;
      case GOING_TO_BURNER_3:
        state = DUMPING;
      break;
    case LEAVING_FROM_BTN_f:
      state = DELIVERING;
      break;
  }
}

uint8_t TestForLeftWall(void) {
  if (distance2 >= left_dist_threshold + hysteresis_threshold && prev_dist2 < left_dist_threshold + hysteresis_threshold) {
    prev_dist2 = distance2;
    return true;
  }
  prev_dist2 = distance2;
  return false;
}

void RespToLeftWall(void) {
  state = GOING_BACK_ON_TRACK;
}

void RespToBeaconSensing(void) {
  state = LEAVING_SZ_1; // only state it can enter is leaving starting zone 1. It should stop spinning and go in the determined direction.
}

uint8_t TestForChangeInTape_1(void) {
  current_line1 = analogRead(LINE_SENSOR_N_PIN) > thrLine;
  return current_line1 != line1;
}

void RespToChangeInTape_1() {
  switch (state) {
    case GOING_TO_CW_1:
      if(current_line1 == 1 && line3 == 1) {
        state = GOING_TO_CW_2;
      }
      break;
    case GOING_TO_BTN_i:
      if(current_line1 == 1 && line3 == 1) {
        state = IGNITING_BTN;
      }
      break;
    case GOING_TO_PANTRY_2:
      if (current_line1 == 1 && line3 == 1) {
        state = GOING_TO_PANTRY_3;
      }
      break;
    case GOING_TO_BURNER_2:
      if (current_line1 == 1 && line3 == 1) {
        state = GOING_TO_BURNER_3;
      }
      break;
  }
  line1 = current_line1;
}

uint8_t TestForChangeInTape_2(void) {
  current_line2 = analogRead(LINE_SENSOR_E_PIN) > thrLine;
  return current_line2 != line2;
}

void RespToChangeInTape_2() {
  switch (state) {
    case LEAVING_SZ_1:
      if (current_line2 == 0){
        state = LEAVING_SZ_2;
      }
      break;
    case LEAVING_SZ_2:
      if (current_line2 == 1 && line4 == 1) {
        state = GOING_TO_CW_1;
      }
      break;
    case GOING_BACK_ON_TRACK:
      if (current_line2 == 1 && line4 == 1) {
        state = GOING_TO_BTN_i;
      }
      break;
    case GOING_TO_PANTRY_1:
      if (current_line2 == 1) {
        state = GOING_TO_PANTRY_2;
      }
      break;
    case GOING_TO_PANTRY_3:
      if (current_line2 == 1 && line4 == 1) {
        state = LOADING;
      }
      break;
    case GOING_TO_BTN_f:
      if (current_line2 == 1) {
        state = TURNING_OFF_BURNER;
      }
      break;
    case DELIVERING:
      if (current_line2 == 1 && line4 == 1) {
          state = CELEBRATING;
        }
      break;

  }
  line2 = current_line2;
}

uint8_t TestForChangeInTape_3(void) {
  current_line3 = analogRead(LINE_SENSOR_S_PIN) > thrLine;
  return current_line3 != line3;
}

void RespToChangeInTape_3() {
  switch (state) {
    case GOING_TO_CW_1:
      if(current_line3 == 1 && line1 == 1) {
        state = GOING_TO_CW_2;
      }
      break;
    case GOING_TO_BTN_i:
      if(current_line3 == 1 && line1 == 1) {
        state = IGNITING_BTN;
      }
      break;
    case GOING_TO_PANTRY_2:
      if (current_line3 == 1 && line1 == 1) {
        state = GOING_TO_PANTRY_3;
      }
      break;
    case GOING_TO_BURNER_2:
      if (current_line3 == 1 && line1 == 1) {
        state = GOING_TO_BURNER_3;
      }
      break;
  }
  line3 = current_line3;
}

uint8_t TestForChangeInTape_4(void) {
  current_line4 = analogRead(LINE_SENSOR_W_PIN) > thrLine;
  return current_line4 != line4;
}

void RespToChangeInTape_4() {
  switch (state) {
    case LEAVING_SZ_2:
      if (current_line4 == 1 && line2 == 1) {
        state = GOING_TO_CW_1;
      }
      break;
    case GOING_BACK_ON_TRACK:
      if (current_line4 == 1 && line2 == 1) {
        state = GOING_TO_BTN_i;
      }
      break;
    case GOING_TO_PANTRY_3:
      if (current_line4 == 1 && line2 == 1) {
        state = LOADING;
      }
      break;
    case GOING_TO_BURNER_1:
      if (current_line4 == 1) {
        state = GOING_TO_BURNER_2;
      }
      break;
    case DELIVERING:
      if (current_line4 == 1 && line2 == 1) {
          state = CELEBRATING;
        }
      break;
  }
  line4 = current_line4;
}

void displayLineSensors(void) {
  Serial.print("(");
  Serial.print(line1);
  Serial.print(",");
  Serial.print(line2);
  Serial.print(",");
  Serial.print(line3);
  Serial.print(",");
  Serial.print(line4);
  Serial.println(")");
}

const char* getStateName(States_t state) {
    if (state >= 0 && state < NUM_STATES) {
        return stateNames[state];
    }
    return "UNKNOWN";
}

void displayState(void){
  static const char* previousState = "";
    const char* stateName = getStateName(state);

    // Print the state name only if it has changed
    if (strcmp(stateName, previousState) != 0) {
        Serial.println(stateName);
        previousState = stateName;
    }
}

void handleDump(void) {
  gateServo.write(100);
  delay(dumpingDuration);
  gateServo.write(0);
  delay(dumpingDuration);
  state = GOING_TO_PANTRY_1;
}

