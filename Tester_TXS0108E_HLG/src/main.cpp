#include <Arduino.h>
// include arduino JSON

// LED pins
constexpr int LED_PINS[] = {23, 22, 1, 3};  // D23, D22, TX0, RX0

// TXS0108E OE pins
constexpr int TXS1_OE = 15;  // Net-(TXS1-INIT/ENA)
constexpr int TXS2_OE = 35;  // Net-(TXS2-INIT/ENA)

// TXS0108E A-side pins (connected to ESP32)
// TXS1 A-side pins
constexpr int TXS1_A_PINS[] = {13, 12, 14, 27, 26, 25, 33, 32};  // A1-A8
// TXS2 A-side pins  
constexpr int TXS2_A_PINS[] = {21, 19, 18, 5, 17, 16, 4, 2};     // A1-A8

// Define test_diagnostic as an empty string

void setup() {
  // --------------- SETUP ----------------- //
  // Disable TXS chips immediately (they need to be low until VA and VB are power stable)
  pinMode(TXS1_OE, OUTPUT);
  pinMode(TXS2_OE, OUTPUT);
  digitalWrite(TXS1_OE, LOW);
  digitalWrite(TXS2_OE, LOW);

  // Initialize all TXS1 A-side pins to LOW
  for(int pin : TXS1_A_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Initialize all TXS2 A-side pins to LOW
  for(int pin : TXS2_A_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Initialize LEDs LOW
  for(int pin : LED_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // -------------- Test 1: Steady High, Steady Low, 1->2 ---------------- //
  // TXS2 A pins change mode to INPUT
  // for each TXS1 A pin:
  //    write high
  //    delay 10ms
  //    If corresponding TXS2 A pin goes high, test_diagnostic += "A(n)+_PASS"
  //    else test_diagnostic += "A(n)+_FAIL"
  // for each TXS1 A pin:
  //    same idea but write low

}

void loop() {
  // format test_diagnostic into JSON: "{"msg":"<true payload here>"}""
  // the payload is updated per sequential tests above during setup

}