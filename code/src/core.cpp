/*
 *  This file contains implementations that will be run on the core Arduino board (MCU).
 *  The responsibilities of the core board include:
 *  1. Handle FSM state transitions based on ultrasonic sensor, IR receiver, and line sensor inputs.
 *  2. Send commands to the peripheral board to drive any motors or do any actuation.
 */

#include <core.h>

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

  switch (state)
  {

  case SPINNING_NOODLE:
    ignitionCmd();
    delay(500);
    disignitionCmd();
    delay(500);
    state = SCANNING;
    break;
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
    if (TestForChangeInTape_4() && current_line4 == 1)
    {
      state = PIVOTING;
    }
    driveNorthCmd();
    break;
  case PIVOTING:
    drivePivotCmd();
    if (TestForChangeInTape_2() && current_line2 == 1)
    {
      state = GOING_TO_CW_1;
    }
    break;
  case GOING_TO_CW_1:
    driveEastCmd();
    if (TestForChangeInTape_1() && current_line1 == 1 && line3 == 1) {
      state = FIRST_LOADING;
    } 
    if (TestForChangeInTape_3() && current_line3 == 1 && line1 == 1)
    {
      state = FIRST_LOADING;
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
    break;
  case GOING_BACK_ON_TRACK:
    driveSouthCmd();
    if (TestForChangeInTape_2()) {
      RespToChangeInTape();
      if (current_line2 == 1 && line4 == 1) {
        state = GOING_TO_BTN_i;
      }
    }
    if (TestForChangeInTape_4()) {
      RespToChangeInTape();
      if (current_line4 == 1 && line2 == 1)
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
    if (refill_counter == max_refill_number) {
      state = GOING_TO_BTN_f;
    }
    else {
      state = GOING_TO_PANTRY_1;
    }
    break;
  case GOING_TO_PANTRY_1:
    driveSouthCmd();
    if (TestForChangeInTape_2() && current_line2 == 1) {
      state = GOING_TO_PANTRY_2;
      driveEastCmd();
      delay(1000);
    }
    break;
  case GOING_TO_PANTRY_2:
    if (TestForChangeInTape_1()) {
      if (current_line1 == 1 && line3 == 1) {
        driveTurnAroundCWCmd();
        delay(100);
        state = GOING_TO_PANTRY_3;
      }
       RespToChangeInTape();
    } 
    if (TestForChangeInTape_3()) {
      if (current_line3 == 1 && line1 == 1) {
        driveTurnAroundCWCmd();
        delay(100);
        state = GOING_TO_PANTRY_3;
      }
       RespToChangeInTape();
    }
    break;
  case GOING_TO_PANTRY_3:
    loadCmd();
    driveSouthCmd();
    delay(delay_to_enter_loading_zone);
    state = LOADING;
    break;
  case LOADING:
    refill_counter++;
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
    break;
  case GOING_TO_BURNER_3:
    us1 = checkDistance1();
    driveNorthCmd();
    if (TestForFrontWall())
      state = DUMPING;
    break;
  case GOING_TO_BTN_f:
    disignitionCmd();
    delay(1000);
    driveSouthCmd();
    delay(1000);
    state = TURNING_OFF_BURNER;
    break;
  case TURNING_OFF_BURNER:
    ignitionCmd();
    delay(500);
    break;
  case LEAVING_FROM_BTN_f:
    driveNorthCmd();
    delay(1500);
    state = DELIVERING;
    break;
  case DELIVERING:
    driveEastCmd();
    delay(6000);
    state = CELEBRATING;
    break;
  case CELEBRATING:
    stopCmd();
    dumpCmd();
    delay(500);
    loadCmd();
    delay(500);
    dumpCmd();
    delay(500);
    loadCmd();
    delay(500);
    state = ENDING;
    break;
  case ENDING:
    stopCmd();
    break;
  }

}

/*----------------Module Functions--------------------------*/

void checkGlobalEvents(void)
{
  RespToChangeInTape();   // update all line sensor variables

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
  // implementation and formula for ultrasonic sensor should be specified as follows
  digitalWrite(US_1_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_1_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_1_TRIG, LOW);
  duration1 = pulseIn(US_1_ECHO, HIGH); // pulse in us. if returning 0, means no feedback received
  distance1 = duration1 * 10 / 2 / 291;          // duration (us) / 2 / 29.1 (us / cm) (speed is the speed of light)
                                                 // additional 10 multiplied to prevent decimal numbers
  return distance1;
}

unsigned long checkDistance2(void)
{
  digitalWrite(US_2_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_2_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_2_TRIG, LOW);
  duration2 = pulseIn(US_2_ECHO, HIGH);
  distance2 = duration2 * 10 / 2 / 291;
  
  return distance2;
}


uint8_t TestForFrontWall(void)
{
  return us1 == thr_us1;
}

uint8_t TestForLeftWall(void)
{
  return us2 == thr_us2;
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

void driveTurnAroundCCWCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_TURNAROUND_CCW_CMD);
  Wire.endTransmission();
}

void loadCmd(void)
{
  // Send command
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(LOADING_CMD);
  Wire.endTransmission();
}

void dumpCmd(void)
{
  // Send command
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DUMPING_CMD);
  Wire.endTransmission();
}

void ignitionCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(IGNITION_CMD);
  Wire.endTransmission();
  uint8_t inp;
}

void disignitionCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DISIGNITION_CMD);
  Wire.endTransmission();
}
