/*
 *  This file contains methods that will be run by the peripheral Arduino board.
 *  The responsibilities of the peripheral board include:
 *  1. Receive signals from the core board and process them
 *  2. Drive DC and servo motors at intended times.
 */

#include <peripheral.h>

// I2C interrupt handler
void receiveEvent(int bytes)
{
  incoming_cmd = (uint8_t)Wire.read();
}

/*---------------Robot Main Functions----------------*/
void setup(void)
{
  Serial.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  // ------- Clears the Serial Monitor --------
  for (int i = 0; i < 50; i++)
  {
    Serial.println();
  }

  // pin mode setup
  // pinMode(GATE_SERVO_PIN, OUTPUT);
  pinMode(MOTOR_1_IN1_PIN, OUTPUT);
  pinMode(MOTOR_1_IN2_PIN, OUTPUT);
  pinMode(MOTOR_2_IN3_PIN, OUTPUT);
  pinMode(MOTOR_2_IN4_PIN, OUTPUT);
  pinMode(MOTOR_3_IN1_PIN, OUTPUT);
  pinMode(MOTOR_3_IN2_PIN, OUTPUT);
  pinMode(MOTOR_4_IN3_PIN, OUTPUT);
  pinMode(MOTOR_4_IN4_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_PIN, OUTPUT);

  // servo setup
  gateServo.attach(GATE_SERVO_PIN);
  igniterServo.attach(IGNITER_SERVO_PIN);

  gateServo.write(100);

  // I2C peripheral device setup
  Wire.begin(PERIPHERAL_ADDR);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
}

void loop()
{
  if (cur_cmd != incoming_cmd)
    stop();

  cur_cmd = incoming_cmd; // store what the currently running command is
  
  analogWrite(MOTOR_SPEED_PIN, mtrSpeed);
  analogWrite(MOTOR_SPEED_PIN_2, mtrSpeed);

  if (incoming_cmd == STOP_CMD)
  {
    stop();
  }
  else if (incoming_cmd == DRIVE_NORTH_CMD)
  {
    driveNorth();
  }
  else if (incoming_cmd == DRIVE_EAST_CMD)
  {
    driveEast();
  }
  else if (incoming_cmd == DRIVE_WEST_CMD)
  {
    driveWest();
  }
  else if (incoming_cmd == DRIVE_SOUTH_CMD)
  {
    driveSouth();
  }
  else if (incoming_cmd == LOADING_CMD)
  {
    load();
  }
  else if (incoming_cmd == DUMPING_CMD)
  {
    dump();
  }
  else if (incoming_cmd == DRIVE_TURNAROUND_CW_CMD)
  {
    driveTurnAroundCW();
  }
  else if (incoming_cmd == DRIVE_PIVOT_CMD)
  {
    drivePivot();
  }
  else if (incoming_cmd == IGNITION_CMD)
  {
    ignition();
  }
  else if (incoming_cmd == DRIVE_TURNAROUND_CCW_CMD)
  {
    driveTurnAroundCCW();
  }
  else if (incoming_cmd == DISIGNITION_CMD)
  {
    disignition();
  }
}
/*----------------Module Functions--------------------------*/

// Shortcut function that sets the direction of a given motor to forward / backward
void setMotorDirection(int in1, int in2, int dir, int motor)
{
  if (dir == FORWARD_DIR)
  {
    if (motor == 1 || motor == 4)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
  }
  else if (dir == BACKWARD_DIR)
  {
    if (motor == 1 || motor == 4)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    else
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
  }
  else if (dir == OFF)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void dump(void)
{
  gateServo.write(70+50);
}

void load(void)
{
  gateServo.write(100);
}

void ignition(void)
{
  igniterServo.write(0);
}

void disignition(void)
{
  igniterServo.write(100);
}

void driveNorth(void)
{
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR, 4);
}

void driveEast(void)
{
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, FORWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR, 4);
}

void driveSouth(void)
{
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, FORWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, BACKWARD_DIR, 4);
}

void driveWest(void)
{
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, BACKWARD_DIR, 4);
}

void driveTurnAroundCW(void)
{
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, FORWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, BACKWARD_DIR, 4);
}

void driveTurnAroundCCW(void)
{
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR, 4);
}

void drivePivot(void)
{
  // We pivot around the wheel number 4 (MTR 4 = Nort-West)
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, OFF, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, OFF, 4);
}

void stop(void)
{
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, OFF, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, OFF, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, OFF, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, OFF, 4);
}
