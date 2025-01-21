/*
 * Serial port troubleshooting
 */

#include <Arduino.h>

void setup() {
    Serial.begin(115200);
    delay(200);

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    Serial.print("CommsThread");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(400);
}