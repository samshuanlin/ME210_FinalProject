#include <Arduino.h>
#include <Servo.h>

 /*---------------Module Defines-----------------------------*/

#define LINE_SENSOR_N_PIN   A1
#define LINE_SENSOR_E_PIN   A2
#define LINE_SENSOR_S_PIN   A3
#define LINE_SENSOR_W_PIN   A4
#define GATE_SERVO_PIN      9
#define USE_TIMER_1         true
#define IR_SIGNAL_PIN       3

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
void detectRisingEdge(void);


/*---------------State Definitions--------------------------*/
typedef enum {
  GOING_TO_PANTRY_1, GOING_TO_PANTRY_2, GOING_TO_PANTRY_3, LOADING, DUMPING
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
Servo gateServo;  // create servo object to control a servo
int gateServoPos = 0;    // variable to store the servo position

/*---------------Robot Main Functions----------------*/
void setup(void) {
  Serial.begin(9600);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  gateServo.attach(GATE_SERVO_PIN);

  state = GOING_TO_PANTRY_1;

  pinMode(LINE_SENSOR_N_PIN, INPUT);
  pinMode(LINE_SENSOR_E_PIN, INPUT);
  pinMode(LINE_SENSOR_S_PIN, INPUT);
  pinMode(LINE_SENSOR_W_PIN, INPUT);
  // pinMode(IR_SIGNAL_PIN, INPUT);      // IR Pin
  // attachInterrupt(digitalPinToInterrupt(IR_SIGNAL_PIN), detectRisingEdge, RISING);

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
      case DUMPING:
        handleDump(void);
        break;
      default:
        Serial.println("Default State");
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

// void handleDump(void) {
//   gateServo.write(100);
//   delay(5000);
//   gateServo.write(0);
//   delay(5000);
// }

// void detectRisingEdge() {
//   Serial.println("light");
// }


