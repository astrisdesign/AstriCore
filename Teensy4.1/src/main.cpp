/*
 * Teensy4.1 motor driver control and load cell reading.
 * Sample load cell frequently.
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <HX711.h>
#include <Teensy41_Pinout.h>
#include "PulsePairSteppers.h"

//    #---------- Motor Driver Setup ----------#
volatile float speed = 1000; // TEMPORARY - delete when done with testing. ------------------------------------ # INITIAL SPEED SETTING #
volatile float targetSpeed = -speed; // TEMPORARY - delete when done with testing.
Threads::Mutex motorMutex;
PulsePairSteppers steppers(pin33, pin34, pin31, pin35, pin32);

//    #---------- Load Cell Setup ------------#
const int LC1_SCK_PIN = 0;
const int LC1_DAT_PIN = 1;
HX711 loadCell1;
volatile int32_t loadReading1 = 0;
Threads::Mutex loadMutex;

void ControlThread() {
    int lastSpeed = 0;    // Cache for the last set speed
    int currentSpeed = 0; // Cache current target speed
    while(true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // TEMPORARY LED blinks for testing

        {
        Threads::Scope lock(motorMutex); // motorMutex prevents race accessing targetSpeed
        currentSpeed = targetSpeed;
        }

        if (currentSpeed != lastSpeed) { // Only update the velocity if it has changed
            steppers.setVelocity(currentSpeed);
            lastSpeed = currentSpeed;
        }

        threads.yield();
    }
}

void SensorThread() {

    loadCell1.set_scale(1000.f);
    loadCell1.tare();

    while(true) {
        {
            Threads::Scope lock(loadMutex);
            loadReading1 = loadCell1.read();
        }
        threads.delay(142);
    }
}

void CommsThread() { // TEMPORARY CONTENTS - will become the USB serial comm thread.
    while(true) {
        {
            Threads::Scope lock(loadMutex);
            Serial.println(loadReading1); // serial print
        }
        threads.delay(100);
    }
}

void setup() {
    Serial.begin(115200);
    delay(200); // Short delay for DM860T startup and serial init
    
    pinMode(LED_BUILTIN, OUTPUT); // Enable Builtin LED flash

    loadCell1.begin(LC1_DAT_PIN, LC1_SCK_PIN); // load cell object  

    steppers.enable(); // DM860T pins low (enable motors)
    
    threads.addThread(ControlThread);
    threads.addThread(SensorThread);
    threads.addThread(CommsThread);
}

void loop() {
    threads.yield();
}