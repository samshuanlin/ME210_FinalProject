/*
 *  This file contains methods that will be run by the peripheral Arduino board.
 *  The responsibilities of the peripheral board include:
 *  1. Receive signals from the core board and process them
 *  2. Drive DC and servo motors at intended times.
 */

#include <peripheral.h>

// I2C interrupt handlers
void receiveEvent(int bytes)
{
  incoming_cmd = (uint8_t)Wire.read();
  Serial.print("Incoming command: ");
  Serial.println(incoming_cmd);
}

void requestEvent()
{
  if (cur_cmd == LOADING_CMD)
  {
    Wire.write(load_done_flag);
    // Serial.println(load_done_flag);
    if (load_done_flag == 1)
      load_done_flag = 0;
  }
  else if (cur_cmd == DUMPING_CMD)
  {
    Wire.write(dump_done_flag);
    if (dump_done_flag == 1)
      dump_done_flag = 0;
  }
  else if (cur_cmd == IGNITION_CMD) 
  {
    Wire.write(ignition_done_flag);
    if (ignition_done_flag == 1)
      ignition_done_flag = 0;
  }
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

  // I2C peripheral device setup
  Wire.begin(PERIPHERAL_ADDR);
  // Attach a function to trigger when something is received.
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}

void loop()
{

  // turn off any done flags
  // Serial.print("cur_cmd: ");
  // Serial.println(cur_cmd);
  // Serial.print("incoming_cmd: ");
  // Serial.println(incoming_cmd);
  if (cur_cmd != incoming_cmd)
    stop();
    // Serial.println("--------------");             // stop before executing other command if the new cmd is not the current cmd
  cur_cmd = incoming_cmd; // store what the currently running command is
  analogWrite(MOTOR_SPEED_PIN, mtrSpeed);

  Serial.print("Current command: ");
  Serial.println(cur_cmd);

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
  else if (incoming_cmd == DRIVE_TURNAROUND_CMD)
  {
    driveTurnAround();
  }
  else if (incoming_cmd == DRIVE_PIVOT_CMD)
  {
    drivePivot();
  }
  else if (incoming_cmd == IGNITION_CMD)
  {
    ignition();
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
    Serial.println("Stopping motor...");
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void dump(void)
{
  gateServo.write(90+50);
  delay(dumpingDuration);
  gateServo.write(90);
  // delay(dumpingDuration);
  // igniterServo.write(0);
  dump_done_flag = 1;
}

void load(void)
{
  Serial.println("Driving south!");
  driveSouth();
  delay(loading_driving_delay);
  Serial.println("Stopping!");
  stop();
  delay(loading_staying_delay);
  Serial.println("Driving north!");
  driveNorth();
  delay(loading_driving_delay);
  load_done_flag = 1;
}

void ignition(void)
{
  igniterServo.write(0);
  delay(200);
  // igniterServo.write(100);

  // delay(dumpingDuration);
  // gateServo.write(0);
  // delay(dumpingDuration);
  ignition_done_flag = 1;
}

void driveNorth(void)
{
  Serial.println("north...");
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, BACKWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, BACKWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR, 4);
}

void driveEast(void)
{
  Serial.println("east...");
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
void driveTurnAround(void)
{
  Serial.println("Turning around...");
  // We turn CCW
  // mtrSpeed = 80;
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, FORWARD_DIR, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, FORWARD_DIR, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, BACKWARD_DIR, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, FORWARD_DIR, 4);
  // mtrSpeed = 150;
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
  Serial.println("Stopping...");
  setMotorDirection(MOTOR_1_IN1_PIN, MOTOR_1_IN2_PIN, OFF, 1);
  setMotorDirection(MOTOR_2_IN3_PIN, MOTOR_2_IN4_PIN, OFF, 2);
  setMotorDirection(MOTOR_3_IN1_PIN, MOTOR_3_IN2_PIN, OFF, 3);
  setMotorDirection(MOTOR_4_IN3_PIN, MOTOR_4_IN4_PIN, OFF, 4);
}
