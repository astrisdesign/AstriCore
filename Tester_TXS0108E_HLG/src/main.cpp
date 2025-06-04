#include <Arduino.h>

// Assign LED pins based on netlist (Net-(U1-D1-A), Net-(U1-D2-A), etc.)
constexpr int LED1_PIN = 23; // Net-(U1-D1-A)
constexpr int LED2_PIN = 22; // Net-(U1-D2-A)
constexpr int LED3_PIN = 1;  // Net-(U1-D3-A)
constexpr int LED4_PIN = 3;  // Net-(U1-D4-A)

void setup() {
  // Set LED pins as outputs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);

  // Turn all LEDs on
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  digitalWrite(LED3_PIN, HIGH);
  digitalWrite(LED4_PIN, HIGH);
}

void loop() {
  // TBD
}