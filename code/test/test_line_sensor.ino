#include <Arduino.h>

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


/*---------------State Definitions--------------------------*/
typedef enum {
  GOING_TO_PANTRY_1, GOING_TO_PANTRY_2, GOING_TO_PANTRY_3, LOADING
} States_t;

/*---------------Module Variables---------------------------*/
States_t state;
float thrLine = 200.0;
int line1;
int line2;
int line3;
int line4;
int current_line1;
int current_line2;
int current_line3;
int current_line4;

/*---------------Robot Main Functions----------------*/
void setup(void) {
   Serial.begin(9600);
   while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
   
   state = GOING_TO_PANTRY_1;

   // pin declarations for line sensor pin
   pinMode(LINE_SENSOR_N_PIN, INPUT);
   pinMode(LINE_SENSOR_E_PIN, INPUT);
   pinMode(LINE_SENSOR_S_PIN, INPUT);
   pinMode(LINE_SENSOR_W_PIN, INPUT);

   // pin declaration for IR sensor pin
   pinMode(IR_RX_PIN_1, INPUT);
   pinMode(IR_RX_PIN_2, INPUT);
 }

 
 void loop() {
    checkGlobalEvents();
    switch (state) {
      case GOING_TO_PANTRY_1:
        break;
      case GOING_TO_PANTRY_2:
        break;
      case GOING_TO_PANTRY_3:
        break;
      case LOADING:
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
    case GOING_TO_PANTRY_2:
      if (current_line1 == 1 && line3 == 1) {
        state = GOING_TO_PANTRY_3;
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
    case GOING_TO_PANTRY_1:
      if (current_line2 == 1) {
        state = GOING_TO_PANTRY_2;
      }
      break;
    case GOING_TO_PANTRY_3:
      if (current_line2 == 1 && line4 == 1) {
        state = LOADING;
      }
  }
  line2 = current_line2;
}

uint8_t TestForChangeInTape_3(void) {
  current_line3 = analogRead(LINE_SENSOR_S_PIN) > thrLine;
  return current_line3 != line3;
}

void RespToChangeInTape_3() {
  switch (state) {
    case GOING_TO_PANTRY_2:
      if (current_line3 == 1 && line1 == 1) {
        state = GOING_TO_PANTRY_3;
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
    case GOING_TO_PANTRY_3:
      if (current_line4 == 1 && line2 == 1) {
        state = LOADING;
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

void displayState(void){
  Serial.println(state);
}




