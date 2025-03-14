// code adapted from this version: https://www.instructables.com/Simple-Arduino-and-HC-SR04-Example/
/*
 *  The way this code works is very simple. A trigger pulse is sent out, the duration between signal sending
 *  and echo receive is calculated, and the distance is just the duration divided by the speed of the signal.
 *  Speed can be further calibrated.
 */

#include <Arduino.h>

#define trigPin 13
#define echoPin 12

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
    long duration, distance;
    digitalWrite(trigPin, LOW);  // Added this line
    delayMicroseconds(2); // Added this line
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); // Added this line
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2) / 29.1;
    Serial.println(distance, DEC);
    if (distance >= 200 || distance <= 0){
        Serial.println("Out of range");
    }
    else {
        Serial.print(distance);
        Serial.println(" cm");
    }
    delay(500);
}