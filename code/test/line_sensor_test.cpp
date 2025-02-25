#include <Arduino.h>

/* 
 * Feb 20, 2025, 10-bit ADC reading
 * Values on top of black tape: close to 1000
 * Values at wooden table: close to 25
 * Ideal threshold: 500
 */

#define LINE_SENSOR_PIN   A0

void setup(void) {
   Serial.begin(115200);
   while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
   
   Serial.println("Line sensor test");

   pinMode(LINE_SENSOR_PIN, INPUT);
 }
 
 void loop() {
   Serial.println(analogRead(LINE_SENSOR_PIN));
   delay(500);
 }