/*
 * Control firmware for Astris UniTest. Hardcoded motor velocity setpoints.
 * First version with PulsePairSteppers moved to include.
 */

#include <Arduino.h>
#include <TeensyThreads.h>
#include <Teensy41_Pinout.h>
#include "PulsePairSteppers.h"

// Define PulsePairSteppers motor control object
volatile float speed = 1000; // TEMPORARY - init to 0 when done with testing. ------------------------------------ # INITIAL SPEED SETTING #
volatile float targetSpeed = -speed;
Threads::Mutex motorMutex;
PulsePairSteppers steppers(pin33, pin34, pin31, pin35, pin32);

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
    while(true) {
        Serial.println("SensorThread");
        threads.delay(499);
    }
}

void CommsThread() {
    while(true) {
        Serial.println("CommsThread");

        for (int i = 0; i < 2; i++) {
        motorMutex.lock();
        targetSpeed = -speed;  // reverse A
        motorMutex.unlock();
        threads.delay(300);

        motorMutex.lock();
        targetSpeed = 0;      // pause B
        motorMutex.unlock();
        threads.delay(400);

        motorMutex.lock();
        targetSpeed = speed; // fwd C
        motorMutex.unlock();
        threads.delay(300);

        motorMutex.lock();
        targetSpeed = 0;      // pause D
        motorMutex.unlock();
        threads.delay(400);
        }

        for (int i = 0; i < 1; i++) {
            motorMutex.lock();
            targetSpeed = 70 * speed;  // forward E
            motorMutex.unlock();
            threads.delay(2000);

        for (int i = 0; i < 6; i++) {
            motorMutex.lock();
            targetSpeed = 100 * speed;  // forward F
            motorMutex.unlock();
            threads.delay(262);

            motorMutex.lock();
            targetSpeed = -100 * speed;      // reverse G
            motorMutex.unlock();
            threads.delay(238);
        }
        }

        motorMutex.lock();
        targetSpeed = -100 * speed; // reverse H
        motorMutex.unlock();
        threads.delay(2000);

        motorMutex.lock();
        targetSpeed = 0;      // pause I
        motorMutex.unlock();
        threads.delay(2000);

    }
}

void setup() {
    Serial.begin(115200);
    
    // Simplify pin setup - only LED needed as class handles other pins
    pinMode(LED_BUILTIN, OUTPUT);
    
    delay(200); // Short delay for DM860T startup

    steppers.enable(); // DM860T pins low (enable motors)
    
    threads.addThread(ControlThread);
    threads.addThread(SensorThread);
    threads.addThread(CommsThread);
}

void loop() {
    threads.yield();
}