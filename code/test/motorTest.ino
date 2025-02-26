#define MOTOR_1_IN1_PIN     2 // This is M3 on Kicad
#define MOTOR_1_IN2_PIN     4 // This is M3 on Kicad
#define MOTOR_SPEED_PIN     3

int mtrSpeed = 100;

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTOR_1_IN1_PIN, OUTPUT);
  pinMode(MOTOR_1_IN2_PIN, OUTPUT);
  pinMode(MOTOR_SPEED_PIN,OUTPUT);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(MOTOR_SPEED_PIN, mtrSpeed);
  digitalWrite(MOTOR_1_IN1_PIN, LOW);
  digitalWrite(MOTOR_1_IN2_PIN, HIGH);
}
