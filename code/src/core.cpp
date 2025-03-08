/*
 *  This file contains implementations that will be run on the core Arduino board (MCU).
 *  The responsibilities of the core board include:
 *  1. Handle FSM state transitions based on ultrasonic sensor, IR receiver, and line sensor inputs.
 *  2. Send commands to the peripheral board to drive any motors or do any actuation.
 */

#include <core.h>



/*---------------Interrupt Handlers------------------*/
void ir1_handler(void)
{
  ir_1_status = 1; // fired at every pin interrupt, set ir_1_status to be 1, to be turned off by TestForBeaconSensing
  // ir1_debouncing_array[ir1_arr_idx] = ir_1_status;

}
void ir2_handler(void)
{
  ir_2_status = 1; // fired at every pin interrupt, set ir_2_status to be 1, to be turned off by TestForBeaconSensing
  // ir2_debouncing_array[ir2_arr_idx] = ir_2_status;
}

/*---------------Robot Main Functions----------------*/

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  // ------- Clears the Serial Monitor --------
  for (int i = 0; i < 50; i++)
  {
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
  attachInterrupt(digitalPinToInterrupt(IR_RX_PIN_1), ir1_handler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(IR_RX_PIN_2), ir2_handler, CHANGE);

  // ultrasonic sensor pin setup
  pinMode(US_1_TRIG, OUTPUT);
  pinMode(US_2_TRIG, OUTPUT);
  pinMode(US_1_ECHO, INPUT);
  pinMode(US_2_ECHO, INPUT);


  // I2C core board setup
  Wire.begin();
  startMillis = millis();

}

void loop()
{

  checkGlobalEvents();
  displayState();
  currentMillis = millis();

  // us1 = checkDistance1();
  // us2 = checkDistance2();



  // Serial.println(line3);


  switch (state)
  {

  case SCANNING:
    us1 = checkDistance1();
    us2 = checkDistance2();
    driveTurnAroundCWCmd();
    break;
  case LEAVING_SZ_1:
    driveSouthCmd();
    delay(delay_going_against_kitchen);
    state = LEAVING_SZ_2;
    break;
  case LEAVING_SZ_2:
    driveTurnAroundCWCmd();
    delay(delay_rotation_to_45_orientation);
    state = LEAVING_SZ_3;
    break;
  case LEAVING_SZ_3:
    driveNorthCmd();
    if (TestForChangeInTape_4() && current_line4 == 1)
    {
      state = PIVOTING;
    }
    break;
  case PIVOTING:
    drivePivotCmd();
    if (TestForChangeInTape_3() && current_line3 == 0)
    {
      state = GOING_TO_CW_1;
    }
    break;
  case GOING_TO_CW_1:
    driveEastCmd();
    if (TestForChangeInTape_1() && current_line1 == 1 && line3 == 1) {
      state = FIRST_LOADING;
      startMillis = millis();
    } 
    if (TestForChangeInTape_3() && current_line3 == 1 && line1 == 1)
    {
      state = FIRST_LOADING;
      startMillis = millis();
    }
    break;
  case FIRST_LOADING:
    driveSouthCmd();
    delay(delay_to_enter_loading_zone);
    state = GOING_TO_CW_2;
    break;

  case GOING_TO_CW_2:
    driveNorthCmd();
    us1 = checkDistance1();
    if (TestForFrontWall()) {
      state = MOVING_POT;
      startMillis = millis();
    }
    break;
  case MOVING_POT:
    driveWestCmd();
    delay(timer_moving_pot);
    state = GOING_BACK_ON_TRACK;
    // if (TestForLeftWall())
    //   RespToLeftWall();
    break;
  case GOING_BACK_ON_TRACK:
    driveSouthCmd();
    if (TestForChangeInTape_2()) {
      RespToChangeInTape();
      if (current_line2 == 1 || line4 == 1) {
        state = GOING_TO_BTN_i;
      }
    }
    if (TestForChangeInTape_4()) {
      RespToChangeInTape();
      if (current_line4 == 1 || line2 == 1)
      {
        state = GOING_TO_BTN_i;
      }
    }
    break;
  case GOING_TO_BTN_i:
    us2 = checkDistance2();
    driveWestCmd();
    if (TestForLeftWall())
      state = STOPPING_FOR_IGNITION;
    break;
  case STOPPING_FOR_IGNITION:
    stopCmd();
    state = IGNITING_BTN;
    break;
  case IGNITING_BTN:
    ignitionCmd();
    state = LEAVING_FROM_BTN_i;
    break;
  case LEAVING_FROM_BTN_i:
    driveNorthCmd();
    us1 = checkDistance1();
    if (TestForFrontWall())
      state = DUMPING;
    break;
  case DUMPING:
    dumpCmd();
    delay(dumping_duration);
    state = GOING_TO_PANTRY_1;
    break;
  case GOING_TO_PANTRY_1:
    driveSouthCmd();
    if (TestForChangeInTape_2() && current_line2 == 1) {
      state = GOING_TO_PANTRY_2;
    }
    break;
  case GOING_TO_PANTRY_2:
    driveEastCmd();
    if (TestForChangeInTape_1()) {
      RespToChangeInTape();
      if (current_line1 == 1 && line3 == 1) {
        state = GOING_TO_PANTRY_3;
      }
    } 
    if (TestForChangeInTape_3()) {
      RespToChangeInTape();
      if (current_line3 == 1 && line1 == 1) {
        state = GOING_TO_PANTRY_3;
      }
    }
    /*
    // SE direction adjustment
    if (current_line1 && !line2 && !line3) {  // went off the line
      driveTurnAroundCCWCmd();
      delay(adjust1_duration);
      driveNorthCmd();
      delay(adjust2_duration);
    }
    // NE direction adjustment
    if (current_line3 && !line2 && !line1) {  // went off the line
      driveTurnAroundCWCmd();
      delay(adjust1_duration);
      driveSouthCmd();
      delay(adjust2_duration);
    }
      */
    break;
  case GOING_TO_PANTRY_3:
    loadCmd();
    driveSouthCmd();
    delay(delay_to_enter_loading_zone);
    state = LOADING;
    break;
  case LOADING:
    state = GOING_TO_BURNER_1;
    break;
  case GOING_TO_BURNER_1:
    driveNorthCmd();
    delay(delay_to_leave_loading_zone);
    driveWestCmd();
    delay(delay_back_to_kitchen);
    state = GOING_TO_BURNER_2;
  case GOING_TO_BURNER_2:
    us2 = checkDistance2();
    driveWestCmd();
    if (TestForLeftWall())
      state = GOING_TO_BURNER_3;
    /*
    // SW direction adjustment
    if (current_line1 && !line4 && !line3) {  // went off the line
      driveTurnAroundCCWCmd();
      delay(adjust1_duration);
      driveSouthCmd();
      delay(adjust2_duration);
    }
    // NW direction adjustment
    if (current_line3 && !line4 && !line1) {  // went off the line
      driveTurnAroundCWCmd();
      delay(adjust1_duration);
      driveNorthCmd();
      delay(adjust2_duration);
    }
      */
    break;
  case GOING_TO_BURNER_3:
    us1 = checkDistance1();
    driveNorthCmd();
    if (TestForFrontWall())
      state = DUMPING;
    break;
  case GOING_TO_BTN_f:
    driveSouthCmd();
    if (TestForChangeInTape_2() && current_line2 == 1)
    {
      state = TURNING_OFF_BURNER;
    }
    break;
  case TURNING_OFF_BURNER:
    stopCmd();
    break;
  case LEAVING_FROM_BTN_f:
    driveNorthCmd();
    if (TestForFrontWall())
      state = DELIVERING; 
    break;
  case DELIVERING:
    driveEastCmd();
    if (TestForChangeInTape_2() && current_line2 == 1 && line4 == 1)
    {
      state = CELEBRATING;
    } 
    if (TestForChangeInTape_4() && current_line4 == 1 && line2 == 1)
    {
      state = CELEBRATING;
    }
    break;
  case CELEBRATING:
    stopCmd();
    break;
  }

}

/*----------------Module Functions--------------------------*/

void checkGlobalEvents(void)
{
  if (TestForChangeInTape_1()) RespToChangeInTape_1();
  if (TestForChangeInTape_2()) RespToChangeInTape_2();
  if (TestForChangeInTape_3()) RespToChangeInTape_3();
  if (TestForChangeInTape_4()) RespToChangeInTape_4();
  RespToChangeInTape();

  if (state == SCANNING)
  {
    if (us1 == 0 || us2 == 0) {
      us_score = -1;
    } else
    {
      us_score = us1 + us2;
    }

    if (us_score > 0 && us_score < thr_us_score) {
      state = LEAVING_SZ_1;
    }
  
  }

  buffer_value_3 = buffer_value_2;
  buffer_value_2 = buffer_value_1;
  buffer_value_1 = distance2;
}
 
unsigned long checkDistance1(void)
{
  // ultrasonic sensors have to be implemented like this to not have errors.
  // see this website for details: https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/#how-the-hc-sr04-ultrasonic-distance-sensor-works
  // specifically, the pulse to send as HIGH is not custom.
  digitalWrite(US_1_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_1_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_1_TRIG, LOW);
  // unsigned long timeout = 3000L;k
  // US_1 is the front-facing ultrasonic sensor
  duration1 = pulseIn(US_1_ECHO, HIGH); // pulse in us. if returning 0, means no feedback received
  distance1 = duration1 * 10 / 2 / 291;          // duration (us) / 2 / 29.1 (us / cm) (speed is the speed of light)
                                                 // additional 10 multiplied to prevent decimal numbers
                                                 
  // Serial.print("Distance 1: ");
  // Serial.print(distance1);
  return distance1;
}

unsigned long checkDistance2(void)
{
  //analogWrite(US_2_TRIG, 200); // 50% duty cycle, 490Hz frequency
  digitalWrite(US_2_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_2_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_2_TRIG, LOW);
  // unsigned long timeout = 3000L;
  // US_2 is the left-facing ultrasonic sensor
  duration2 = pulseIn(US_2_ECHO, HIGH);
  distance2 = duration2 * 10 / 2 / 291;
  // note that this is done in a superloop, so will cause delays for 6 ms maximum
  Serial.print(", Distance 2: ");
  Serial.println(distance2);
  
  return distance2;
}


uint8_t TestForFrontWall(void)
{
  // Serial.println(us1);
  return us1 < thr_us1 && us1 > 0;
}

uint8_t TestForLeftWall(void)
{
  return us2 < thr_us2 && us2 > 0;
}


// Ultrasonic sensor time trigger control
uint8_t TestForTriggerTimerExpired(void)
{
  
  return currentMillis - startMillis > timerTrigger;
}

void RespToTriggerTimerExpired()
{
  // we echo only when we need because the pulseIn function takes time
  if (state == GOING_TO_CW_2 || state == GOING_TO_BURNER_3 || state == LEAVING_FROM_BTN_f)
  {
    us1 = pulseIn(US_1_ECHO, DEC);
    us1 = (us1 / 2) / 29.1;
    // Serial.println(us2); // for testing. TODO
  }
  else if (state == MOVING_POT)
  {
    us2 = pulseIn(US_2_ECHO, DEC);
    us2 = (us2 / 2) / 29.1;
    Serial.println(us2); // for testing. TODO
  }

  startMillis = millis();
}

uint8_t TestForChangeInTape_1(void)
{
  current_line1 = analogRead(LINE_SENSOR_N_PIN) > thrLine;
  return current_line1 != line1;
}

void RespToChangeInTape_1()
{
  line1 = current_line1;
}

uint8_t TestForChangeInTape_2(void)
{
  current_line2 = analogRead(LINE_SENSOR_E_PIN) > thrLine;
  Serial.print(", Line 2 Value: ");
  Serial.print(analogRead(LINE_SENSOR_E_PIN));
  return current_line2 != line2;
}

void RespToChangeInTape_2()
{
  line2 = current_line2;
}

uint8_t TestForChangeInTape_3(void)
{
  current_line3 = analogRead(LINE_SENSOR_S_PIN) > thrLine;
  return current_line3 != line3;
}

void RespToChangeInTape_3()
{
  line3 = current_line3;
}

uint8_t TestForChangeInTape_4(void)
{
  current_line4 = analogRead(LINE_SENSOR_W_PIN) > thrLine;
  //Serial.print(", Line 4 Value: ");
  //Serial.println(analogRead(LINE_SENSOR_W_PIN));
  return current_line4 != line4;
}

void RespToChangeInTape_4()
{
  line4 = current_line4;
}

void RespToChangeInTape() {
  line1 = current_line1;
  line2 = current_line2; 
  line3 = current_line3;
  line4 = current_line4;
}

void displayLineSensors(void)
{
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

const char *getStateName(States_t state)
{
  if (state >= 0 || state < NUM_STATES)
  {
    return stateNames[state];
  }
  return "UNKNOWN";
}

void displayState(void)
{
  static const char *previousState = "";
  const char *stateName = getStateName(state);

  // Print the state name only if it has changed
  if (strcmp(stateName, previousState) != 0)
  {
    Serial.println(stateName);
    previousState = stateName;
  }
}

// Motor command sending functions
void stopCmd()
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(STOP_CMD);
  Wire.endTransmission();
}

void driveNorthCmd()
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_NORTH_CMD);
  Wire.endTransmission();
}

void driveEastCmd()
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_EAST_CMD);
  Wire.endTransmission();
}

void driveWestCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_WEST_CMD);
  Wire.endTransmission();
}

void driveSouthCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_SOUTH_CMD);
  Wire.endTransmission();
}

void drivePivotCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_PIVOT_CMD);
  Wire.endTransmission();
}

void driveTurnAroundCWCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_TURNAROUND_CW_CMD);
  Wire.endTransmission();
}

void loadCmd(void)
{
  // Send command
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(LOADING_CMD);
  Wire.endTransmission();

  // Wait until done
  uint8_t inp;
  do
  {
    Wire.requestFrom(PERIPHERAL_ADDR, sizeof(uint8_t)); // request from peripheral
    inp = Wire.read();
  } while (inp != 1); // while the done flag is not raised, keep waiting

}

void dumpCmd(void)
{
  // Send command
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DUMPING_CMD);
  Wire.endTransmission();

  // Wait until done
  uint8_t inp;
  do
  {
    Wire.requestFrom(PERIPHERAL_ADDR, sizeof(uint8_t)); // request from peripheral
    inp = Wire.read();
  } while (inp != 1); // while the done flag is not raised, keep waiting

}

void ignitionCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(IGNITION_CMD);
  Wire.endTransmission();
  uint8_t inp;
  do
  {
    Wire.requestFrom(PERIPHERAL_ADDR, sizeof(uint8_t)); // request from peripheral
    inp = Wire.read();
  } while (inp != 1); // while the done flag is not raised, keep waiting

  // only return control after done
  if (state == IGNITING_BTN)
  {
    state = LEAVING_FROM_BTN_i; // set new state
    // startMillis = millis();
  }
  if (state == TURNING_OFF_BURNER)
  {
    state = LEAVING_FROM_BTN_f; // set new state
  }
}

void driveTurnAroundCCWCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_TURNAROUND_CCW_CMD);
  Wire.endTransmission();
}
