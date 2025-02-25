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

#define GATE_SERVO_PIN      9

// IR sensor pins
#define IR_RX_PIN_1     2
#define IR_RX_PIN_2     3

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
States_t state;
States_t initialState = DUMPING;
float thrLine = 200.0;
int line1;
int line2;
int line3;
int line4;
int current_line1;
int current_line2;
int current_line3;
int current_line4;
Servo gateServo;  // create servo object to control a servo
int gateServoPos = 0;    // variable to store the servo position
int dumpingDuration = 1000; // milliseconds

/*---------------Robot Main Functions----------------*/
void setup(void) {
   Serial.begin(9600);
   while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
   // ------- Clears the Serial Monitor --------
   for (int i = 0; i < 50; i++) {
        Serial.println();
    }
   
   state = initialState;

   pinMode(LINE_SENSOR_N_PIN, INPUT);
   pinMode(LINE_SENSOR_E_PIN, INPUT);
   pinMode(LINE_SENSOR_S_PIN, INPUT);
   pinMode(LINE_SENSOR_W_PIN, INPUT);

   gateServo.attach(GATE_SERVO_PIN);
 }

 
 void loop() {
    checkGlobalEvents();
    switch (state) {
      case DUMPING:
        handleDump();
        break;
    }

    displayState();

    //displayLineSensors();
 }


/*----------------Module Functions--------------------------*/

void checkGlobalEvents(void) {
  if (TestForChangeInTape_1()) RespToChangeInTape_1();
  if (TestForChangeInTape_2()) RespToChangeInTape_2();
  if (TestForChangeInTape_3()) RespToChangeInTape_3();
  if (TestForChangeInTape_4()) RespToChangeInTape_4();
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




