#include <Arduino.h>

#define INTERVAL        1000
#define USE_TIMER_1     true
// #define USE_TIMER_2     true
#define FREQUENCY       900
#define PIN_SIGNAL_IN   2
#define PIN_SIGNAL_IN_2 3
#define CALCULATE_PIN   1

#include <TimerInterrupt.hpp>        
#include <ISR_Timer.hpp> 
#include <TimerInterrupt.h>
#include <ISR_Timer.h>

/*---------------Module Function Prototypes-----------------*/
void handleTimer(void);
void CountFallingEdges(void);
void calcFreq(void);


int Pin = 13;
int state = 0;
int analogPin = A0;
int reading = 0;
volatile long counter = 0;
int readInPin = A2;

template<typename TArg> bool setInterval(unsigned long interval, void (*callback)(TArg), TArg params, unsigned long duration = 0);
template<typename TArg> bool attachInterrupt(float frequency, void (*callback)(TArg), TArg params, unsigned long duration = 0);
bool setFrequency(float frequency, timer_callback callback, unsigned long duration = 0);
template<typename TArg> bool attachInterruptInterval(unsigned long interval, void (*callback)(TArg), TArg params, unsigned long duration = 0);


void handleTimer() {
  int state = digitalRead(Pin);
  if (state == HIGH) {
    digitalWrite(Pin, LOW);
  }
  if (state == LOW) {
    digitalWrite(Pin, HIGH);
  }
}

void CountFallingEdges() {
  counter+=1;
}

static int ir_sensor_1_status = 0;

void detectRisingEdge() {
  ir_sensor_1_status = 1;
}

static int ir_sensor_2_status = 0;
void detectRisingEdge2() {
  ir_sensor_2_status = 1;
}

void calcFreq() {
  // Serial.println(counter);
  counter = 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  pinMode(Pin, OUTPUT);
  pinMode(PIN_SIGNAL_IN, INPUT);
  ITimer1.init();
  // ITimer2.init();
  ITimer1.setFrequency(FREQUENCY*2, handleTimer);
  // ITimer2.attachInterruptInterval(1000, calcFreq);
  attachInterrupt(digitalPinToInterrupt(PIN_SIGNAL_IN), detectRisingEdge, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_SIGNAL_IN_2), detectRisingEdge2, RISING);

}

void loop() {
  // int reading = analogRead(readInPin);
  // int reading = digitalRead(PIN_SIGNAL_IN);
  // Serial.println(reading);
  // Serial.println(reading);
  // put your main code here
  // reading = analogRead(analogPin);
  // int val = map(reading, 0, 1023, 0, 5);
  // Serial.println(val);
  Serial.print(ir_sensor_1_status);
  Serial.print(", ");
  Serial.println(ir_sensor_2_status);
  if (ir_sensor_1_status || ir_sensor_2_status) {
    Serial.println("light");
    ir_sensor_1_status = 0;
    ir_sensor_2_status = 0;
  }

}
