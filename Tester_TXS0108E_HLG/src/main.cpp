#include <Arduino.h>
#include <array>
#include <algorithm>
#include <HardwareSerial.h>   // for Serial1/Serial2

//––– UART-based channel test helper –––
struct HardwareSerialTest {
  int tx_pin_txsl;     // ESP32 pin connected to TXSL A-side
  int rx_pin_txsr;     // ESP32 pin connected to TXSR A-side
  HardwareSerial* serial_port;
  String test_name;
};

// forward-declare the serial test function
String run_hardware_serial_tests(int tx1, int rx1, int ch1, int tx2, int rx2, int ch2);

// LED pins
constexpr int LED_PINS[] = {22, 23, 2};  // D22, D23, onboard LED (Note that TX0 and RX0 disable USB serial)

// TXS0108E OE pin
constexpr int TXS_OE = 15;  // Net-(TXS2-INIT/ENA)

// TXS0108E A-side pins (connected to ESP32)
// TXSL A-side pins
constexpr int TXSL_A_PINS[] = {13, 12, 14, 27, 26, 25, 33, 32};  // A1-A8 goes top to bottom
// TXSR A-side pins  
constexpr int TXSR_A_PINS[] = {2, 4, 16, 17, 5, 18, 19, 21};     // A1-A8 goes bottom to top

// Record UART subtest pass/fail for channel 1 and 2
bool test_uart_ch1 = false;
bool test_uart_ch2 = false;

// Per-channel test result tracking
constexpr int NUM_CHANNELS = 8;
std::array<bool, NUM_CHANNELS> test1_high_pass = {false};
std::array<bool, NUM_CHANNELS> test1_low_pass  = {false};
std::array<bool, NUM_CHANNELS> test2_high_pass = {false};
std::array<bool, NUM_CHANNELS> test2_low_pass  = {false};
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

//–––––– Run two UART‐driven channel tests at once, report any failed channel numbers ––––––
String run_hardware_serial_tests(int tx1, int rx1, int ch1, int tx2, int rx2, int ch2) {
  // 1) Begin each UART with the given pins at 9600 baud
  Serial1.begin(9600, SERIAL_8N1, rx1, tx1);
  Serial2.begin(9600, SERIAL_8N1, rx2, tx2);

  // 2) Send a short ASCII test string on each UART
  Serial1.print("TEST_"); Serial1.print(tx1);  // embed pin in payload if you like
  Serial2.print("TEST_"); Serial2.print(tx2);
  delay(50);

  // 3) Collect incoming bytes (with a 100 ms timeout)
  String resp1 = "";
  String resp2 = "";
  unsigned long start = millis();
  while (millis() - start < 100) {
    if (Serial1.available()) resp1 += (char)Serial1.read();
    if (Serial2.available()) resp2 += (char)Serial2.read();
  }

  // 4) Print raw hex or ASCII to USB‐Serial for debug (unchanged)
  Serial.print("UART "); Serial.print(tx1);
  Serial.println(" → raw: " + resp1);
  Serial.print("UART "); Serial.print(tx2);
  Serial.println(" → raw: " + resp2);

  // 5) Build a list of any failed channel numbers:
  std::array<String,2> failedList = { "", "" };
  int failCount = 0;
  if (resp1.length() == 0) failedList[failCount++] = String(ch1);
  if (resp2.length() == 0) failedList[failCount++] = String(ch2);

  // 6) Teardown both UARTs so pins can be reused
  Serial1.end();
  Serial2.end();

  // 7) Return "None" if none failed, else join the failed channel(s) with ", "
  if (failCount == 0) {
    return String("None");
  } else if (failCount == 1) {
    return failedList[0];
  } else {
    return failedList[0] + ", " + failedList[1];
  }
}
//–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

// Left to right uart test
bool uart_channel_test(int idx) {
  int tx = TXSL_A_PINS[idx];
  int rx = TXSR_A_PINS[idx];
  Serial1.begin(9600, SERIAL_8N1, rx, tx);
  Serial1.print("TEST_"); Serial1.print(tx);
  delay(50);
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < 100) {
    if (Serial1.available()) resp += (char)Serial1.read();
  }
  Serial.print("UART "); Serial.print(tx);
  Serial.println(" → raw: " + resp);
  Serial1.end();
  return resp.length() > 0;
}

// right to left uart test
bool uart_channel_test_reverse(int idx) {
  int tx = TXSR_A_PINS[idx];
  int rx = TXSL_A_PINS[idx];
  Serial1.begin(9600, SERIAL_8N1, rx, tx);
  Serial1.print("TEST_"); Serial1.print(tx);
  delay(50);
  String resp = "";
  unsigned long start = millis();
  while (millis() - start < 100) {
    if (Serial1.available()) resp += (char)Serial1.read();
  }
  Serial.print("UART_REV "); Serial.print(tx);
  Serial.println(" → raw: " + resp);
  Serial1.end();
  return resp.length() > 0;
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

  // -------------- Test 2: Right to Left Steady High and Low ---------------- //
  
  // Test TXSR-->TXSL in the 'high' direction
  for (int pin : TXSL_A_PINS) {
    pinMode(pin, INPUT_PULLUP);   // Let TXSR drive high into TXSL
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    int outPin = TXSR_A_PINS[i];
    int inPin  = TXSL_A_PINS[i];
    pinMode(outPin, OUTPUT);
    digitalWrite(outPin, HIGH);
    delay(10);
    test2_high_pass[i] = (digitalRead(inPin) == HIGH);
    delay(10);
  }

  // Test TXSR-->TXSL in the 'low' direction
  for (int pin : TXSL_A_PINS) {
    pinMode(pin, INPUT_PULLDOWN); // Let TXSR drive low into TXSL
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
    int outPin = TXSR_A_PINS[i];
    int inPin  = TXSL_A_PINS[i];
    digitalWrite(outPin, LOW);
    delay(10);
    test2_low_pass[i] = (digitalRead(inPin) == LOW);
    delay(10);
  }

  //––––––– UART-based subtest: Test 3 (All Channels) ––––––
  std::array<bool, NUM_CHANNELS> test3_uart_pass = {false};
  for (int i = 0; i < NUM_CHANNELS; ++i)
      test3_uart_pass[i] = uart_channel_test(i);
  bool test3_pass = std::all_of(test3_uart_pass.begin(), test3_uart_pass.end(), [](bool v){ return v; });

  //––––––– UART-based subtest: Test 4 (Reverse, All Channels) ––––––
  std::array<bool, NUM_CHANNELS> test4_uart_pass = {false};
  for (int i = 0; i < NUM_CHANNELS; ++i)
      test4_uart_pass[i] = uart_channel_test_reverse(i);
  bool test4_pass = std::all_of(test4_uart_pass.begin(), test4_uart_pass.end(), [](bool v){ return v; });

  //––––––––––––––––––––––––––––––––––––––––––––––––––––––––––

  // Aggregate results and set status LEDs
  bool all_passed = std::all_of(test1_high_pass.begin(), test1_high_pass.end(), [](bool v){ return v; }) &&
                    std::all_of(test1_low_pass.begin(),  test1_low_pass.end(),  [](bool v){ return v; }) &&
                    std::all_of(test2_high_pass.begin(), test2_high_pass.end(), [](bool v){ return v; }) &&
                    std::all_of(test2_low_pass.begin(),  test2_low_pass.end(),  [](bool v){ return v; }) &&
                    test3_pass && test4_pass;

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
  test_results += make_fail_msg(test1_low_pass,  "T1L") + ", ";
  test_results += make_fail_msg(test2_high_pass, "T2H") + ", ";
  test_results += make_fail_msg(test2_low_pass,  "T2L") + ", ";
  test_results += make_fail_msg(test3_uart_pass, "T3") + ", ";
  test_results += make_fail_msg(test4_uart_pass, "T4");
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
