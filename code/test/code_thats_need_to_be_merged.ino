#include <Arduino.h>
#include <Servo.h>

 /*---------------Module Defines-----------------------------*/

#define LINE_SENSOR_N_PIN   A0
#define LINE_SENSOR_E_PIN   A1
#define LINE_SENSOR_S_PIN   A2
#define LINE_SENSOR_W_PIN   A3

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

#define TRIGGER_PIN         9 // Trigger pin for US1 and US4
#define ECHO_PIN_1          10 // Echo pin for US1
#define ECHO_PIN_4          11 // Echo pin for US4

#define FORWARD_DIR         1
#define BACKWARD_DIR        -1
#define OFF                 0


/*---------------Module Function Prototypes-----------------*/
// Event-related functions
void checkGlobalEvents(void);
unsigned char TestForChangeInTape_1(void);
void RespToChangeInTape_1(void);
unsigned char TestForChangeInTape_2(void);
void RespToChangeInTape_2(void);
unsigned char TestForChangeInTape_3(void);
void RespToChangeInTape_3(void);
unsigned char TestForChangeInTape_4(void);
void RespToChangeInTape_4(void);

// Blocking functions
void Dump(void);
void Load(void);

// Motor control
void driveNorth(void);
void driveEast(void);
void driveSouth(void);
void driveWest(void);
void driveTurnRound(void);
void drivePivot(void);
void stop(void);


/*---------------State Definitions--------------------------*/
const char* stateNames[] = {
    "SPINNING_NOODLE", "SCANNING", "LEAVING_SZ_1", "LEAVING_SZ_2", "PIVOTING", "GOING_TO_CW_1", "GOING_TO_CW_2",
    "MOVING_POT", "GOING_BACK_ON_TRACK", "GOING_TO_BTN_i", "IGNITING_BTN",
    "LEAVING_FROM_BTN_i", "DUMPING", "GOING_TO_PANTRY_1", "GOING_TO_PANTRY_2",
    "GOING_TO_PANTRY_3", "LOADING", "GOING_TO_BURNER_1", "GOING_TO_BURNER_2",
    "GOING_TO_BURNER_3", "GOING_TO_BTN_f", "TURNING_OFF_BURNER",
    "LEAVING_FROM_BTN_f", "DELIVERING", "CELEBRATING"
};

typedef enum {
  SPINNING_NOODLE, SCANNING, LEAVING_SZ_1, LEAVING_SZ_2, PIVOTING, GOING_TO_CW_1, GOING_TO_CW_2,
  MOVING_POT, GOING_BACK_ON_TRACK, GOING_TO_BTN_i, IGNITING_BTN,
  LEAVING_FROM_BTN_i, DUMPING, 
  GOING_TO_PANTRY_1, GOING_TO_PANTRY_2, GOING_TO_PANTRY_3, LOADING,
  GOING_TO_BURNER_1, GOING_TO_BURNER_2, GOING_TO_BURNER_3,
  GOING_TO_BTN_f, TURNING_OFF_BURNER, LEAVING_FROM_BTN_f,
  DELIVERING, CELEBRATING, NUM_STATES
} States_t;

/*---------------Module Variables---------------------------*/
States_t state;
States_t initialState = GOING_TO_CW_1;
float thrLine = 50.0;
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
int loading_driving_delay = 1000; // number of milliseconds the robot will drive toward and from the loading position
int loading_staying_delay = 500; // number of milliseconds the robot will stay during loading
int mtrSpeed = 150;
int timerTrigger = 10; // milliseconds
int currentMillis;
int startMillis;
int us1; // distance sensed by the ultrasonic sensor 1
int us4; // distance sensed by the ultrasonic sensor 1
int thr_us1 = 4; // cm
int thr_us4 = 20; // cm

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

   pinMode(MOTOR_1_IN1_PIN, OUTPUT);
   pinMode(MOTOR_1_IN2_PIN, OUTPUT);
   pinMode(MOTOR_2_IN3_PIN, OUTPUT);
   pinMode(MOTOR_2_IN4_PIN, OUTPUT);
   pinMode(MOTOR_3_IN1_PIN, OUTPUT);
   pinMode(MOTOR_3_IN2_PIN, OUTPUT);
   pinMode(MOTOR_4_IN3_PIN, OUTPUT);
   pinMode(MOTOR_4_IN4_PIN, OUTPUT);
   pinMode(MOTOR_SPEED_PIN,OUTPUT);

   pinMode(TRIGGER_PIN, OUTPUT);
   pinMode(ECHO_PIN_1, INPUT);
   pinMode(ECHO_PIN_4, INPUT);
   gateServo.attach(GATE_SERVO_PIN);
 }

 
 void loop() {
    checkGlobalEvents();
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
        if (TestForUS1DetectsWall()) RespToUS1DetectsWall();
        break;
      case MOVING_POT:
        driveWest();
        if (TestForUS4DetectsWall()) RespToUS4DetectsWall();
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

    currentMillis = millis();
    //displayLineSen
 }


/*----------------Module Functions--------------------------*/

void checkGlobalEvents(void) {
  if (TestForChangeInTape_1()) RespToChangeInTape_1();
  if (TestForChangeInTape_2()) RespToChangeInTape_2();
  if (TestForChangeInTape_3()) RespToChangeInTape_3();
  if (TestForChangeInTape_4()) RespToChangeInTape_4();
  if (TestForTriggerTimerExpired()) RespToTriggerTimerExpired();
} 

uint8_t TestForUS1DetectsWall(void) {
  return us1 < thr_us1 && us1 > 0;
}

void RespToUS1DetectsWall() {
  if (state == GOING_TO_CW_2) {
    state = MOVING_POT;
    startMillis = millis();
  }
  else if (state == GOING_TO_BURNER_3) {
    state = DUMPING;
  }
  else if (state == LEAVING_FROM_BTN_f) {
    state = DELIVERING;
  }
}

uint8_t TestForUS4DetectsWall(void) {
  return us4 < thr_us4 && us4 > 0;
}

void RespToUS4DetectsWall() {
  if (state == MOVING_POT) {
    state = GOING_BACK_ON_TRACK;
  }
}

uint8_t TestForTriggerTimerExpired(void) {
  return currentMillis - startMillis > timerTrigger;
}

void RespToTriggerTimerExpired() {
  // we echo only when we need because the pulseIn function takes time
  if (state == GOING_TO_CW_2 || state == GOING_TO_BURNER_3 || state == LEAVING_FROM_BTN_f) {
    us1 = pulseIn(ECHO_PIN_1, DEC);
    us1 = (us1/2) / 29.1;
    Serial.println(us4);
  }
  else if (state == MOVING_POT) {
    us4 = pulseIn(ECHO_PIN_4, DEC);
    us4 = (us4/2) / 29.1;
    Serial.println(us4);
  }
  
  
  startMillis = millis();
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
        startMillis = millis();
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
    case PIVOTING:
      if (current_line2 == 1) {
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
        startMillis = millis();
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
    case LEAVING_SZ_1:
      if (current_line4 == 0) {
        state = LEAVING_SZ_2;
      }
      break;
    case LEAVING_SZ_2:
      if (current_line4 == 1) {
        state = PIVOTING;
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
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, FORWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR, 4);
}

void driveEast(void) {
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, FORWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, BACKWARD_DIR, 4);
}

void driveSouth(void) {
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, BACKWARD_DIR, 4);
}

void driveWest(void) {
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR, 4);
}

void driveTurnRound(void) {
  // We turn CW
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR, 4);
}

void drivePivot(void) {
  // We pivot around the wheel number 4 (MTR 4 = Nort-West)
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, OFF, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, FORWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, OFF, 4);
} 

void stop(void) {
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, OFF, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, OFF, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, OFF, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, OFF, 4);
}



// Shortcut function that sets the direction of a given motor to forward / backward
void setMotorDirection(int in1, int in2, int dir, int motor) {
  if (dir == FORWARD_DIR) {
    if (motor == 1 || motor == 4) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
  }
  else if (dir == BACKWARD_DIR) {
    if (motor == 1 || motor == 4) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
  }
  else if (dir == OFF) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}








