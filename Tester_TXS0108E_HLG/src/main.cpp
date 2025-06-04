#include <Arduino.h>

// LED pins
constexpr int LED_PINS[] = {22, 23, 3};  // D22, D23, RX0 (Note that RX0 disables serial send)

// TXS0108E OE pins
constexpr int TXS1_OE = 15;  // Net-(TXS1-INIT/ENA)
constexpr int TXS2_OE = 35;  // Net-(TXS2-INIT/ENA)

// TXS0108E A-side pins (connected to ESP32)
// TXS1 A-side pins
constexpr int TXS1_A_PINS[] = {13, 12, 14, 27, 26, 25, 33, 32};  // A1-A8
// TXS2 A-side pins  
constexpr int TXS2_A_PINS[] = {21, 19, 18, 5, 17, 16, 4, 2};     // A1-A8

String test_diagnostic = "";

void setup() {
  // begin serial for output
  Serial.begin(115200);
  delay(100);

  // Disable TXS chips immediately (they need to be low until VA and VB are power stable)
  pinMode(TXS1_OE, OUTPUT);
  pinMode(TXS2_OE, OUTPUT);
  digitalWrite(TXS1_OE, LOW);
  digitalWrite(TXS2_OE, LOW);

  // Initialize all TXS1 A-side pins to LOW
  for (int pin : TXS1_A_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Initialize all TXS2 A-side pins to LOW, then set as inputs for testing
  for (int pin : TXS2_A_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  for (int pin : TXS2_A_PINS) {
    pinMode(pin, INPUT);
  }

  // Initialize LEDs LOW
  for (int pin : LED_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Initialize the TXS1s
  delay(500);
  digitalWrite(TXS1_OE, HIGH);
  digitalWrite(TXS2_OE, HIGH);
  delay(50);

  // -------------- Test 1: Steady High, Steady Low, 1->2 ---------------- //

  // Test TXS1-->TXS2 in the 'high' direction
  for (uint8_t i = 0; i < 8; i++) {
    int outPin = TXS1_A_PINS[i];
    int inPin = TXS2_A_PINS[i];
    digitalWrite(outPin, HIGH);
    delay(10);
    if (digitalRead(inPin) == HIGH) {
      test_diagnostic += "A" + String(i + 1) + "+_PASS ";
    } else {
      test_diagnostic += "A" + String(i + 1) + "+_FAIL ";
    }
  }

  // Test TXS1-->TXS2 in the 'low' direction
  for (uint8_t i = 0; i < 8; i++) {
    int outPin = TXS1_A_PINS[i];
    int inPin = TXS2_A_PINS[i];
    digitalWrite(outPin, LOW);
    delay(10);
    if (digitalRead(inPin) == LOW) {
      test_diagnostic += "A" + String(i + 1) + "-_PASS ";
    } else {
      test_diagnostic += "A" + String(i + 1) + "-_FAIL ";
    }
  }
}

void loop() {
  String payload = "{\"msg\":\"" + test_diagnostic + "\"}"; // format test_diagnostic into JSON: {"msg":"<payload>"}
  
  if (test_diagnostic.endsWith(" ")) { // remove trailing space (if present)
    test_diagnostic.remove(test_diagnostic.length() - 1);
  }
  Serial.println(payload);

  for (int pin : LED_PINS) {
    digitalWrite(pin, !digitalRead(pin));
  }

  // Toggle all TXS pins


  delay(250);// Pause before re-sending diagnostic
}
