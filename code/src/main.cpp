/* 
 *  This file contains implementations that will be run on the core Arduino board (MCU).
 *  The responsibilities of the core board include:
 *  1. Handle FSM state transitions based on ultrasonic sensor, IR receiver, and line sensor inputs.
 *  2. Send commands to the peripheral board to drive any motors or do any actuation.
 */

 #include <core_code.h>

 /*---------------Interrupt Handlers------------------*/
 void ir1_handler(void) {
     ir_1_status = 1;  // fired at every pin interrupt, set ir_1_status to be 1, to be turned of by TestForBeaconSensing
 }
   
 void ir2_handler(void) {
     ir_2_status = 1;  // fired at every pin interrupt, set ir_2_status to be 1, to be turned of by TestForBeaconSensing
 }  
 
 /*---------------Robot Main Functions----------------*/
 void setup() {
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
 
    // pin setup for IR sensors
    pinMode(IR_RX_PIN_1, INPUT);
    pinMode(IR_RX_PIN_2, INPUT);
 
    // digital pin interrupt setup for IR sensors
    attachInterrupt(digitalPinToInterrupt(IR_RX_PIN_1), ir1_handler, RISING);
    attachInterrupt(digitalPinToInterrupt(IR_RX_PIN_2), ir2_handler, RISING);
 
    // ultrasonic sensor pin setup
    pinMode(US_TRIG, OUTPUT);
    pinMode(US_1_ECHO, INPUT);
    pinMode(US_2_ECHO, INPUT);
 
    // I2C core board setup
    Wire.begin(); 
  }
 
  
  void loop() {
     checkGlobalEvents();
     checkDistance();
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
   if (TestForBeaconSensing()) RespToBeaconSensing();
   if (TestForFrontWall()) RespToFrontWall();
   if (TestForLeftWall()) RespToLeftWall();
 } 
 
 uint8_t TestForBeaconSensing(void) {
   if (ir_1_status || ir_2_status) {   // use OR logic to allow for greater coverage
     ir_1_status = 0;
     ir_2_status = 0;
     return 1;
   }
   return 0;
 }
 
 void RespToBeaconSensing(void) {
   state = LEAVING_SZ_1; // only state it can enter is leaving starting zone 1. It should stop spinning and go in the determined direction.
 }
 
 void checkDistance(void) {
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
  return;
 }