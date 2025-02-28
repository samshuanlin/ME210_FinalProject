#define MOTOR_1_IN1_PIN     2 // This is M3 on Kicad
#define MOTOR_1_IN2_PIN     4 // This is M3 on Kicad
#define MOTOR_2_IN3_PIN     5 // This is M4 on Kicad
#define MOTOR_2_IN4_PIN     6 // This is M4 on Kicad
#define MOTOR_3_IN1_PIN     7 // This is M5 on Kicad
#define MOTOR_3_IN2_PIN     8 // This is M5 on Kicad
#define MOTOR_4_IN3_PIN     12 // This is M6 on Kicad
#define MOTOR_4_IN4_PIN     13 // This is M6 on Kicad
#define MOTOR_SPEED_PIN     3

int mtrSpeed = 150;

void setup() {
  // put your setup code here, to run once:
   pinMode(MOTOR_1_IN1_PIN, OUTPUT);
   pinMode(MOTOR_1_IN2_PIN, OUTPUT);
   pinMode(MOTOR_2_IN3_PIN, OUTPUT);
   pinMode(MOTOR_2_IN4_PIN, OUTPUT);
   pinMode(MOTOR_3_IN1_PIN, OUTPUT);
   pinMode(MOTOR_3_IN2_PIN, OUTPUT);
   pinMode(MOTOR_4_IN3_PIN, OUTPUT);
   pinMode(MOTOR_4_IN4_PIN, OUTPUT);
   pinMode(MOTOR_SPEED_PIN,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(MOTOR_SPEED_PIN, mtrSpeed);
  digitalWrite(MOTOR_1_IN1_PIN, LOW);
  digitalWrite(MOTOR_1_IN2_PIN, HIGH);
  digitalWrite(MOTOR_2_IN3_PIN, HIGH);
  digitalWrite(MOTOR_2_IN4_PIN, LOW);
  digitalWrite(MOTOR_3_IN1_PIN, LOW);
  digitalWrite(MOTOR_3_IN2_PIN, HIGH);
  digitalWrite(MOTOR_4_IN3_PIN, LOW);
  digitalWrite(MOTOR_4_IN4_PIN, HIGH);
  delay(3000);
  // digitalWrite(MOTOR_1_IN1_PIN, HIGH);
  // digitalWrite(MOTOR_1_IN2_PIN, LOW);
  // digitalWrite(MOTOR_2_IN3_PIN, HIGH);
  // digitalWrite(MOTOR_2_IN4_PIN, LOW);
  // digitalWrite(MOTOR_3_IN1_PIN, HIGH);
  // digitalWrite(MOTOR_3_IN2_PIN, LOW);
  // digitalWrite(MOTOR_4_IN3_PIN, HIGH);
  // digitalWrite(MOTOR_4_IN4_PIN, LOW);
  // delay(100);
  digitalWrite(MOTOR_1_IN1_PIN, LOW);
  digitalWrite(MOTOR_1_IN2_PIN, LOW);
  digitalWrite(MOTOR_2_IN3_PIN, LOW);
  digitalWrite(MOTOR_2_IN4_PIN, LOW);
  digitalWrite(MOTOR_3_IN1_PIN, LOW);
  digitalWrite(MOTOR_3_IN2_PIN, LOW);
  digitalWrite(MOTOR_4_IN3_PIN, LOW);
  digitalWrite(MOTOR_4_IN4_PIN, LOW);
  delay(3000);


}
