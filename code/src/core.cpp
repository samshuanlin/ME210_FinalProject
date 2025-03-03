/*
 *  This file contains implementations that will be run on the core Arduino board (MCU).
 *  The responsibilities of the core board include:
 *  1. Handle FSM state transitions based on ultrasonic sensor, IR receiver, and line sensor inputs.
 *  2. Send commands to the peripheral board to drive any motors or do any actuation.
 */

#include <core.h>


int timer_moving_pot = 8000;



/*---------------Interrupt Handlers------------------*/
void ir1_handler(void)
{
  ir_1_status = 1; // fired at every pin interrupt, set ir_1_status to be 1, to be turned off by TestForBeaconSensing
  Serial.println("Rise in IR 1 detected!");
}

void ir2_handler(void)
{
  ir_2_status = 1; // fired at every pin interrupt, set ir_2_status to be 1, to be turned off by TestForBeaconSensing
  Serial.println("Rise in IR 2 detected!");
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

  // ultrasonic sensor pin setup
  pinMode(US_TRIG, OUTPUT);
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
  switch (state)
  {
  case SCANNING:
    driveTurnAroundCmd();
    break;
  case LEAVING_SZ_1:
    driveNorthCmd();
    break;
  case LEAVING_SZ_2:
    driveNorthCmd();
    break;
  case PIVOTING:
    drivePivotCmd();
    break;
  case GOING_TO_CW_1:
    driveEastCmd();
    break;
  case GOING_TO_CW_2:
    driveNorthCmd();
    if (TestForFrontWall())
      RespToFrontWall();
    break;
  case MOVING_POT:
    driveWestCmd();

    // if (TestForLeftWall())
    //   RespToLeftWall();
    break;
  case GOING_BACK_ON_TRACK:
    driveSouthCmd();
    break;
  case GOING_TO_BTN_i:
    driveWestCmd();
    break;
  case STOPPING_FOR_IGNITION:
    stopCmd();
    // startMillis = millis();
    // state = IGNITING_BTN;
    // delay(1000);
    break;
  case IGNITING_BTN:
    ignitionCmd();
    break;
  case LEAVING_FROM_BTN_i:
    driveNorthCmd();
    if (TestForLeftWall())
      RespToLeftWall();
    break;
  case DUMPING:
    dumpCmd();
    break;
  case GOING_TO_PANTRY_1:
    driveSouthCmd();
    break;
  case GOING_TO_PANTRY_2:
    driveEastCmd();
    break;
  case GOING_TO_PANTRY_3:
    driveSouthCmd();
    break;
  case LOADING:
    loadCmd();
    break;
  case GOING_TO_BURNER_1:
    driveNorthCmd();
    break;
  case GOING_TO_BURNER_2:
    driveWestCmd();
    break;
  case GOING_TO_BURNER_3:
    driveNorthCmd();
    if (TestForFrontWall())
      RespToFrontWall();
    break;
  case GOING_TO_BTN_f:
    driveSouthCmd();
    break;
  case TURNING_OFF_BURNER:
    stopCmd();
    break;
  case LEAVING_FROM_BTN_f:
    driveNorthCmd();
    if (TestForFrontWall())
      RespToFrontWall();
    break;
  case DELIVERING:
    driveEastCmd();
    break;
  case CELEBRATING:
    stopCmd();
    break;
  }

  
  
  
  

  us1 = checkDistance1();
  us2 = checkDistance2();

  // Serial.println(analogRead(IR_RX_PIN_2));


  // Serial.println(us2);

  // displayLineSensors();
  // Serial.println(analogRead(LINE_SENSOR_E_PIN));



}

/*----------------Module Functions--------------------------*/

void checkGlobalEvents(void)
{
  if (TestForChangeInTape_1()) RespToChangeInTape_1();
  if (TestForChangeInTape_2()) RespToChangeInTape_2();
  if (TestForChangeInTape_3()) RespToChangeInTape_3();
  if (TestForChangeInTape_4()) RespToChangeInTape_4();

  if (state == SCANNING)
  {
    if (TestForBeaconSensing())
      RespToBeaconSensing();
  }

  if (TestForFrontWall()) RespToFrontWall();
  if (TestForLeftWall()) RespToLeftWall();
  // if (TestForTriggerTimerExpired()) RespToTriggerTimerExpired();

  if (currentMillis - startMillis > timer_moving_pot)
  {
    if (state == MOVING_POT)
    {
      state = GOING_BACK_ON_TRACK;
    }
  }

}

uint8_t TestForBeaconSensing(void)
{
  // digital pin interrupt setup for IR sensors
  attachInterrupt(digitalPinToInterrupt(IR_RX_PIN_1), ir1_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(IR_RX_PIN_2), ir2_handler, RISING);

  if (ir_1_status && ir_2_status)
  { // use OR logic to allow for greater coverage
    ir_1_status = 0;
    ir_2_status = 0;
    return 1;
  }
  return 0;
}

void RespToBeaconSensing(void)
{
  // detach interrupts since we don't need them anymore
  detachInterrupt(digitalPinToInterrupt(IR_RX_PIN_1));
  detachInterrupt(digitalPinToInterrupt(IR_RX_PIN_2));
  state = LEAVING_SZ_1; // only state it can enter is leaving starting zone 1. It should stop spinning and go in the determined direction.
}

uint8_t checkDistance1(void)
{
  analogWrite(US_TRIG, 128); // 50% duty cycle, 490Hz frequency
  unsigned long timeout = 3000L;
  // US_1 is the front-facing ultrasonic sensor
  duration1 = pulseIn(US_1_ECHO, HIGH, timeout); // pulse in us. if returning 0, means no feedback received
  distance1 = duration1 * 10 / 2 / 291;          // duration (us) / 2 / 29.1 (us / cm) (speed is the speed of light)
                                                 // additional 10 multiplied to prevent decimal numbers
  return distance1;
}

uint8_t checkDistance2(void)
{
  analogWrite(US_TRIG, 128); // 50% duty cycle, 490Hz frequency
  unsigned long timeout = 3000L;
  // US_2 is the left-facing ultrasonic sensor
  duration2 = pulseIn(US_2_ECHO, HIGH, timeout);
  distance2 = duration2 * 10 / 2 / 291;
  // note that this is done in a superloop, so will cause delays for 6 ms maximum
  return distance2;
}


uint8_t TestForFrontWall(void)
{
  // Serial.println(us1);
  return us1 < thr_us1 && us1 > 0;
}

void RespToFrontWall(void)
{
  if (state == GOING_TO_CW_2)
  {
    state = MOVING_POT;
    startMillis = millis();
  }
  else if (state == LEAVING_FROM_BTN_i)
  {
    state = DUMPING;
  }
  else if (state == GOING_TO_BURNER_3)
  {
    state = DUMPING;
  }
  else if (state == LEAVING_FROM_BTN_f)
  {
    state = DELIVERING;
  }
}

uint8_t TestForLeftWall(void)
{
  return us2 < thr_us2 && us2 > 0;
}

void RespToLeftWall(void)
{
  if (state == GOING_TO_BTN_i)
  {
    state = STOPPING_FOR_IGNITION;
  }
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
  switch (state)
  {
  case GOING_TO_CW_1:
    if (current_line1 == 1 && line3 == 1)
    {
      state = GOING_TO_CW_2;
      startMillis = millis();
    }
    break;
  // case GOING_TO_BTN_i:
  //   if (current_line1 == 1 && line3 == 1)
  //   {
  //     state = STOPPING_FOR_IGNITION;
  //   }
  //   break;
  case GOING_TO_PANTRY_2:
    if (current_line1 == 1 && line3 == 1)
    {
      state = GOING_TO_PANTRY_3;
    }
    break;
  case GOING_TO_BURNER_2:
    if (current_line1 == 1 && line3 == 1)
    {
      state = GOING_TO_BURNER_3;
      startMillis = millis();
    }
    break;
  }
  line1 = current_line1;
}

uint8_t TestForChangeInTape_2(void)
{
  current_line2 = analogRead(LINE_SENSOR_E_PIN) > thrLine;
  return current_line2 != line2;
}

void RespToChangeInTape_2()
{
  switch (state)
  {
  // case PIVOTING:
  //   if (current_line2 == 0)
  //   {
  //     state = GOING_TO_CW_1;
  //   }
  //   break;
  case GOING_BACK_ON_TRACK:
    if (current_line2 == 1 && line4 == 1)
    {
      state = GOING_TO_BTN_i;
    }
    break;
  case GOING_TO_PANTRY_1:
    if (current_line2 == 1)
    {
      state = GOING_TO_PANTRY_2;
    }
    break;
  case GOING_TO_PANTRY_3:
    if (current_line2 == 1 && line4 == 1)
    {
      state = LOADING;
    }
    break;
  case GOING_TO_BTN_f:
    if (current_line2 == 1)
    {
      state = TURNING_OFF_BURNER;
    }
    break;
  case DELIVERING:
    if (current_line2 == 1 && line4 == 1)
    {
      state = CELEBRATING;
    }
    break;
  }
  line2 = current_line2;
}

uint8_t TestForChangeInTape_3(void)
{
  current_line3 = analogRead(LINE_SENSOR_S_PIN) > thrLine;
  return current_line3 != line3;
}

void RespToChangeInTape_3()
{
  switch (state)
  {
  case PIVOTING:
    if (current_line3 == 0)
    {
      state = GOING_TO_CW_1;
    }
    break;
  case GOING_TO_CW_1:
    if (current_line3 == 1 && line1 == 1)
    {
      state = GOING_TO_CW_2;
      startMillis = millis();
    }
    break;
  // case GOING_TO_BTN_i:
  //   if (current_line3 == 1 && line1 == 1)
  //   {
  //     state = STOPPING_FOR_IGNITION;
  //   }
  //   break;
  case GOING_TO_PANTRY_2:
    if (current_line3 == 1 && line1 == 1)
    {
      state = GOING_TO_PANTRY_3;
    }
    break;
  case GOING_TO_BURNER_2:
    if (current_line3 == 1 && line1 == 1)
    {
      state = GOING_TO_BURNER_3;
      startMillis = millis();
    }
    break;
  }
  line3 = current_line3;
}

uint8_t TestForChangeInTape_4(void)
{
  current_line4 = analogRead(LINE_SENSOR_W_PIN) > thrLine;
  return current_line4 != line4;
}

void RespToChangeInTape_4()
{
  switch (state)
  {
  case LEAVING_SZ_1:
    if (current_line4 == 0)
    {
      state = LEAVING_SZ_2;
    }
    break;
  case LEAVING_SZ_2:
    if (current_line4 == 1)
    {
      state = PIVOTING;
    }
    break;
  case GOING_BACK_ON_TRACK:
    if (current_line4 == 1 && line2 == 1)
    {
      state = GOING_TO_BTN_i;
    }
    break;
  case GOING_TO_PANTRY_3:
    if (current_line4 == 1 && line2 == 1)
    {
      state = LOADING;
    }
    break;
  case GOING_TO_BURNER_1:
    if (current_line4 == 1)
    {
      state = GOING_TO_BURNER_2;
    }
    break;
  case DELIVERING:
    if (current_line4 == 1 && line2 == 1)
    {
      state = CELEBRATING;
    }
    break;
  }
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

  if (state == STOPPING_FOR_IGNITION)
  {
    delay(500);
    state = IGNITING_BTN;
  } 
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

void driveTurnAroundCmd(void)
{
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DRIVE_TURNAROUND_CMD);
  Wire.endTransmission();
  // delay(2000);
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

  // only return control after done
  state = GOING_TO_BURNER_1;
}

void dumpCmd(void)
{
  Serial.println("Dumping...");
  // Send command
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(DUMPING_CMD);
  Wire.endTransmission();

  // delay(5000);

  // Wait until done
  uint8_t inp;
  do
  {
    Wire.requestFrom(PERIPHERAL_ADDR, sizeof(uint8_t)); // request from peripheral
    inp = Wire.read();
  } while (inp != 1); // while the done flag is not raised, keep waiting

  // only return control after done
  state = GOING_TO_PANTRY_1; // set new state
}

void ignitionCmd(void)
{
  // Serial.println("Igniting...");
  // Send command
  Wire.beginTransmission(PERIPHERAL_ADDR);
  Wire.write(IGNITION_CMD);
  Wire.endTransmission();
  // Serial.println("Ignition command sent!");
  delay(1000);
  // Wait until done
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
    startMillis = millis();
  }
  else if (state == TURNING_OFF_BURNER)
  {
    state = LEAVING_FROM_BTN_f; // set new state
  }
}