#include <Arduino.h>
#include <array>
#include <algorithm>

// LED pins
constexpr int LED_PINS[] = {22, 23, 2};  // D22, D23, onboard LED (Note that TX0 and RX0 disable USB serial)

// TXS0108E OE pin
constexpr int TXS_OE = 15;  // Net-(TXS2-INIT/ENA)

// TXS0108E A-side pins (connected to ESP32)
// TXSL A-side pins
constexpr int TXSL_A_PINS[] = {13, 12, 14, 27, 26, 25, 33, 32};  // A1-A8 goes top to bottom
// TXSR A-side pins  
constexpr int TXSR_A_PINS[] = {2, 4, 16, 17, 5, 18, 19, 21};     // A1-A8 goes bottom to top

// Per-channel test result tracking
constexpr int NUM_CHANNELS = 8;
std::array<bool, NUM_CHANNELS> test1_high_pass = {false};
std::array<bool, NUM_CHANNELS> test1_low_pass  = {false};
// Add more arrays if you add more tests

String test_results = "Failed Pin Numbers: ";
String output_message = "{\"msg\":\"";

String make_fail_msg(const std::array<bool, NUM_CHANNELS>& test, const char* label) {
  String msg = String(label) + ":";
  bool any_failed = false;
  for (int i = 0; i < NUM_CHANNELS; ++i) {
    if (!test[i]) {
      if (any_failed) msg += ",";
      msg += String(i+1);
      any_failed = true;
    }
  }
  if (!any_failed) msg += "None";
  return msg;
}

void setup() {
  // begin serial for output
  Serial.begin(115200);
  delay(100);

  // Keep OE pin as INPUT during reset/boot so GPIO2 (the strapping pin)
  // isn't forced low. This lets the ESP32 finish its normal boot.
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

  // -------------- Test 1: Left to Right Steady High and Low ---------------- //

  // Test TXSL-->TXSR in the 'high' direction
  for (int pin : TXSR_A_PINS) {
    pinMode(pin, INPUT_PULLUP); // INPUT_PULLDOWN fights TXSR driving high and can confuse drive direction
  }

  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    int outPin = TXSL_A_PINS[i];
    int inPin = TXSR_A_PINS[i];
    digitalWrite(outPin, HIGH);
    delay(10);
    test1_high_pass[i] = (digitalRead(inPin) == HIGH);
    delay(10);
  }

  // Test TXSL-->TXSR in the 'low' direction
  for (int pin : TXSR_A_PINS) {
    pinMode(pin, INPUT_PULLDOWN); // INPUT_PULLDOWN fights TXSR driving high and can confuse drive direction
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    int outPin = TXSL_A_PINS[i];
    int inPin = TXSR_A_PINS[i];
    digitalWrite(outPin, LOW);
    delay(10);
    test1_low_pass[i] = (digitalRead(inPin) == LOW);
    delay(10);
  }

  // Test 2: TBD

  // Aggregate results and set status LEDs
  bool all_passed = std::all_of(test1_high_pass.begin(), test1_high_pass.end(), [](bool v){ return v; }) &&
                    std::all_of(test1_low_pass.begin(),  test1_low_pass.end(),  [](bool v){ return v; });

  if (all_passed) {
    digitalWrite(23, HIGH); // D23 high if all tests pass
    digitalWrite(22, LOW);
  } else {
    digitalWrite(23, LOW);
    digitalWrite(22, HIGH); // D22 high if any test fails
  }

  pinMode(2, OUTPUT); // Re-enable onboard LED (it's dual-purposed as one of the channel tests)
  digitalWrite(2, LOW); // onboard LED initially off

  test_results += make_fail_msg(test1_high_pass, "T1H") + ", ";
  test_results += make_fail_msg(test1_low_pass,  "T1L");
  output_message += test_results;
  output_message +=  "\"}";
}

void loop() {
  Serial.println(output_message);

  digitalWrite(2, !digitalRead(2)); // Toggle onboard LED

  // Toggle all TXSL pins for o-scope probing through circuit
  for (int pin : TXSL_A_PINS) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, !digitalRead(pin));
  }

  delay(1500); // Pause before re-sending diagnostic
}
