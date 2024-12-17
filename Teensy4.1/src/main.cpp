/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

void setup()
{
  Serial.begin(115200);       // Start serial communication at 115200 baud rate
  delay(100); // Wait for serial port to connect
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  // wait for a quarter second
  delay(250);
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);
   // wait for a quarter second
  delay(250);
  // print serial debug message
  Serial.println("TJ has great eyes!");
}