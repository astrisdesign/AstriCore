/*
 * THIS SPINS THE MOTOR
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <Teensy41_Pinout.h>

// Motor 1 pin definition
const int pulse1 = pin33;
const int dir1 = pin34;
const int enable1 = pin35;

void ControlTask() {
    while(true) {
        Serial.println("ControlTask");
        digitalWrite(LED_BUILTIN, HIGH); // blink on before pulse
        digitalWrite(pulse1, HIGH);
        threads.delay(1);
        digitalWrite(pulse1, LOW);
        digitalWrite(LED_BUILTIN, LOW); // blink off after pulse
        threads.delay(1);
    }
}

void SensorTask() {
    while(true) {
        Serial.println("SensorTask");
        threads.delay(499);
    }
}

void CommsTask() {
    while(true) {
        Serial.println("CommsTask");
        threads.delay(997);
    }
}

void setup() {
    Serial.begin(115200);

    // Pin definitions
    pinMode(LED_BUILTIN, OUTPUT);  // LED pin is output
    pinMode(pulse1, OUTPUT);
    pinMode(dir1, OUTPUT);
    pinMode(enable1, OUTPUT);

    digitalWrite(pulse1, LOW); // Start pulse low
    digitalWrite(dir1, LOW); // Start with known dir
    digitalWrite(enable1, LOW); // enable DM860T 

    delay(200); // Wait for DM860T to enable

    threads.addThread(ControlTask);
    threads.addThread(SensorTask);
    threads.addThread(CommsTask);
}

void loop() {
    threads.yield();
}