#include <Arduino.h>

// LED pins
constexpr int LED_PINS[] = {22, 23, 2};  // D22, D23, onboard LED (Note that TX0 and RX0 disable USB serial)

// TXS0108E OE pin
constexpr int TXS_OE = 15;  // Net-(TXS2-INIT/ENA)

// TXS0108E A-side pins (connected to ESP32)
// TXSL A-side pins
constexpr int TXSL_A_PINS[] = {13, 12, 14, 27, 26, 25, 33, 32};  // A1-A8 goes top to bottom
// TXSR A-side pins  
constexpr int TXSR_A_PINS[] = {2, 4, 16, 17, 5, 18, 19, 21};     // A1-A8 goes bottom to top

String test_diagnostic = "";

void setup() {
  // begin serial for output
  Serial.begin(115200);
  delay(100);

  // Keep OE pin as INPUT during reset/boot so GPIO2 (the strapping pin)
  // isn’t forced low. This lets the ESP32 finish its normal boot.
  pinMode(TXS_OE, INPUT);
  delay(250);  // wait for the ESP32 to exit reset

  // Now that boot is done, drive OE low to disable both TXS chips
  pinMode(TXS_OE, OUTPUT);
  digitalWrite(TXS_OE, LOW);

  // Initialize all TXS1 A-side pins to LOW
  for (int pin : TXSL_A_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Initialize all TXS2 A-side pins to LOW, then set as inputs for testing
  for (int pin : TXSR_A_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  for (int pin : TXSR_A_PINS) {
    pinMode(pin, INPUT_PULLDOWN);
  }

  // Initialize LEDs LOW
  for (int pin : LED_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Initialize the TXS1s
  delay(250);
  digitalWrite(TXS_OE, HIGH);
  delay(50);

  // -------------- Test 1: Steady High, Steady Low, 1->2 ---------------- //

  // Test TXS1-->TXS2 in the 'high' direction
  for (int pin : TXSR_A_PINS) {
    pinMode(pin, INPUT_PULLUP); // INPUT_PULLDOWN fights TXSR driving high and can confuse drive direction
  }

  for (uint8_t i = 0; i < 8; i++) {
    int outPin = TXSL_A_PINS[i];
    int inPin = TXSR_A_PINS[i];
    digitalWrite(outPin, HIGH);
    delay(10);
    if (digitalRead(inPin) == HIGH) {
      test_diagnostic += "A" + String(i + 1) + "+_PASS ";
    } else {
      test_diagnostic += "A" + String(i + 1) + "+_FAIL ";
    }
    delay(10);
  }

  // Test TXS1-->TXS2 in the 'low' direction
  for (int pin : TXSR_A_PINS) {
    pinMode(pin, INPUT_PULLDOWN); // INPUT_PULLDOWN fights TXSR driving high and can confuse drive direction
  }
  for (uint8_t i = 0; i < 8; i++) {
    int outPin = TXSL_A_PINS[i];
    int inPin = TXSR_A_PINS[i];
    digitalWrite(outPin, LOW);
    delay(10);
    if (digitalRead(inPin) == LOW) {
      test_diagnostic += "A" + String(i + 1) + "-_PASS ";
    } else {
      test_diagnostic += "A" + String(i + 1) + "-_FAIL ";
    }
    delay(10);
  }

  // Test 2: TBD
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

  // Toggle all TXS1 pins
  for (int pin : TXSL_A_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, !digitalRead(pin));
  }

  delay(1500); // Pause before re-sending diagnostic
}
