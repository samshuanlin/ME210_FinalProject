#define INTERVAL        1000
#define USE_TIMER_1     true
// #define USE_TIMER_2     true
#define FREQUENCY       3300
#define PIN_SIGNAL_IN   3
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

void detectRisingEdge() {
  Serial.println("light");
}

void calcFreq() {
  // Serial.println(counter);
  counter = 0;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial);
  pinMode(Pin, OUTPUT);
  pinMode(PIN_SIGNAL_IN, INPUT);
  ITimer1.init();
  // ITimer2.init();
  ITimer1.setFrequency(FREQUENCY*2, handleTimer);
  // ITimer2.attachInterruptInterval(1000, calcFreq);
  attachInterrupt(digitalPinToInterrupt(PIN_SIGNAL_IN), detectRisingEdge, RISING);

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

}
