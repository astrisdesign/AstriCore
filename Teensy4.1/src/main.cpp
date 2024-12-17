/*
 * Multithreading test version 1
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <Teensy41_Pinout.h>

void ControlTask() {
    while(true) {
        Serial.println("ControlTask");
        digitalWrite(LED_BUILTIN, HIGH); // blink on and off while waiting for a quarter second-ish
        threads.delay(127);
        // turn the LED off by making the voltage LOW
        digitalWrite(LED_BUILTIN, LOW);
        // wait for a quarter second
        threads.delay(127);
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
    pinMode(LED_BUILTIN, OUTPUT);  // LED pin is output
    delay(10); // Wait for serial port to connect
    threads.addThread(ControlTask);
    threads.addThread(SensorTask);
    threads.addThread(CommsTask);
}

void loop() {
    threads.yield();
}